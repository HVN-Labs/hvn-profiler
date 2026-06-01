//! v0.8.0 — runtime template discovery + the picker registry.
//!
//! The picker surfaces three classes of template:
//! 1. **Bundled** — compiled into the binary via `include_str!`
//!    (see [`crate::bundled`]).
//! 2. **User** — `*.json` files in the platform-specific user directory:
//!    - Windows: `%LOCALAPPDATA%\hvn-profiler\templates\`
//!    - Linux:   `~/.config/hvn-profiler/templates/`
//!    - macOS:   `~/Library/Application Support/hvn-profiler/templates/`
//! 3. **Current** — the `--template <path>` value the CLI was launched with,
//!    surfaced as "current" so the user can flip back to it after switching.
//!
//! Bundled templates are read-only (Save shows "Save as..." instead).
//! User templates accept Save (Ctrl+S overwrite) and Save-as.
//!
//! Everything in this module is pure I/O (no GUI), so it is unit-testable
//! against a `TempDir`.

use std::path::{Path, PathBuf};

use crate::bundled::{self, BundledTemplate};

/// Where a template came from. Drives the picker's Save vs Save-as menu.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TemplateOrigin {
    /// Compiled into the binary. Read-only.
    Bundled,
    /// On disk under the user templates directory. Save overwrites the file.
    User { path: PathBuf },
    /// Loaded from `--template <path>` (not in the user directory).
    /// Treated as user-savable (since we have a path) but tagged distinctly
    /// so the picker can show it as "current" / "external".
    Cli { path: PathBuf },
}

impl TemplateOrigin {
    /// `true` if Save (overwrite) is allowed without prompting for a path.
    pub fn is_savable_in_place(&self) -> bool {
        matches!(self, TemplateOrigin::User { .. } | TemplateOrigin::Cli { .. })
    }

    /// The on-disk path, if any.
    pub fn path(&self) -> Option<&Path> {
        match self {
            TemplateOrigin::Bundled => None,
            TemplateOrigin::User { path } | TemplateOrigin::Cli { path } => Some(path),
        }
    }
}

/// One entry in the picker dropdown.
#[derive(Debug, Clone)]
pub struct TemplateEntry {
    /// Display name (the JSON `name` field, falling back to the file stem).
    pub name: String,
    pub origin: TemplateOrigin,
}

impl TemplateEntry {
    /// Short suffix used by the dropdown — `"(bundled)"`, `"(user)"`, `"(cli)"`.
    pub fn origin_label(&self) -> &'static str {
        match self.origin {
            TemplateOrigin::Bundled => "bundled",
            TemplateOrigin::User { .. } => "user",
            TemplateOrigin::Cli { .. } => "cli",
        }
    }
}

/// Compute the platform-specific user templates directory.
///
/// - Windows: `%LOCALAPPDATA%\hvn-profiler\templates\`
/// - Linux:   `~/.config/hvn-profiler/templates/`
/// - macOS:   `~/Library/Application Support/hvn-profiler/templates/`
///
/// Falls back to `<cwd>/hvn-profiler-templates` when no platform dir is found
/// (e.g. headless CI with no `$HOME`).
pub fn user_templates_dir() -> PathBuf {
    let base = if cfg!(windows) {
        dirs::data_local_dir()
    } else {
        // `config_dir()` is `~/.config` on Linux and
        // `~/Library/Application Support` on macOS — matches our spec.
        dirs::config_dir()
    };
    base.unwrap_or_else(|| PathBuf::from("."))
        .join("hvn-profiler")
        .join("templates")
}

/// Ensure the user templates directory exists, creating it if needed. Returns
/// the path either way (the directory may already be present).
pub fn ensure_user_templates_dir() -> std::io::Result<PathBuf> {
    let dir = user_templates_dir();
    if !dir.exists() {
        std::fs::create_dir_all(&dir)?;
    }
    Ok(dir)
}

/// Scan `dir` for `*.json` user templates. Each matched file is opened and
/// its `name` field read; on parse failure we still surface the file with
/// the filename stem as the display name (so the user can see broken
/// templates in the picker and fix them).
///
/// Non-`.json` files are silently ignored. Subdirectories are not recursed.
pub fn scan_user_templates(dir: &Path) -> Vec<TemplateEntry> {
    let mut out = Vec::new();
    let rd = match std::fs::read_dir(dir) {
        Ok(rd) => rd,
        Err(_) => return out,
    };
    for entry in rd.flatten() {
        let path = entry.path();
        if path.is_file() && path.extension().is_some_and(|e| e == "json") {
            let name = read_template_name(&path).unwrap_or_else(|| {
                path.file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("unnamed")
                    .to_string()
            });
            out.push(TemplateEntry {
                name,
                origin: TemplateOrigin::User { path },
            });
        }
    }
    // Stable, predictable ordering for the dropdown.
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

/// Read just the top-level `name` field from a template file. Cheap enough
/// to call for every `.json` in the directory.
fn read_template_name(path: &Path) -> Option<String> {
    let text = std::fs::read_to_string(path).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;
    v.get("name").and_then(|n| n.as_str()).map(str::to_string)
}

/// Build the full picker registry: bundled + scanned user + the CLI value
/// (deduplicated by `(name, origin-path)` so a user template named the same
/// as a bundled one doesn't appear twice).
pub fn discover(cli_template: Option<&Path>) -> Vec<TemplateEntry> {
    let mut out: Vec<TemplateEntry> = Vec::new();

    // Bundled first so the dropdown leads with "hvn-default" / "real-drone".
    for b in bundled::BUNDLED {
        out.push(TemplateEntry {
            name: b.name.to_string(),
            origin: TemplateOrigin::Bundled,
        });
    }

    // User templates.
    let dir = user_templates_dir();
    for e in scan_user_templates(&dir) {
        out.push(e);
    }

    // CLI template — only add if it's outside the user dir (otherwise the
    // scan already picked it up under TemplateOrigin::User).
    if let Some(p) = cli_template {
        let abs = std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
        let already_present = out.iter().any(|e| {
            e.origin.path().is_some_and(|q| {
                std::fs::canonicalize(q).unwrap_or_else(|_| q.to_path_buf()) == abs
            })
        });
        if !already_present {
            let name = read_template_name(p).unwrap_or_else(|| {
                p.file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("current")
                    .to_string()
            });
            out.push(TemplateEntry {
                name,
                origin: TemplateOrigin::Cli {
                    path: p.to_path_buf(),
                },
            });
        }
    }

    out
}

/// Load the raw JSON text for an entry — handles bundled vs file-on-disk.
pub fn load_entry_json(entry: &TemplateEntry) -> std::io::Result<String> {
    match &entry.origin {
        TemplateOrigin::Bundled => {
            // Look up by name in the bundled registry.
            let b = bundled::by_name(&entry.name).ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("bundled template '{}' not in registry", entry.name),
                )
            })?;
            Ok(b.json.to_string())
        }
        TemplateOrigin::User { path } | TemplateOrigin::Cli { path } => {
            std::fs::read_to_string(path)
        }
    }
}

/// Borrow a bundled entry's JSON, if `entry` is bundled.
pub fn bundled_json(entry: &TemplateEntry) -> Option<&'static BundledTemplate> {
    match entry.origin {
        TemplateOrigin::Bundled => bundled::by_name(&entry.name),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a temp directory with two minimal JSON templates and one non-JSON
    /// file; assert discovery returns the two templates in alphabetical order.
    #[test]
    fn scan_picks_up_two_templates_alphabetical() {
        let tmp = tempfile::tempdir().expect("tempdir");
        std::fs::write(
            tmp.path().join("zeta.json"),
            r#"{"name":"zeta-layout","grid":{"rows":1,"cols":1}}"#,
        )
        .unwrap();
        std::fs::write(
            tmp.path().join("alpha.json"),
            r#"{"name":"alpha-layout","grid":{"rows":1,"cols":1}}"#,
        )
        .unwrap();
        // A non-JSON file must be ignored.
        std::fs::write(tmp.path().join("README.md"), "not a template").unwrap();

        let found = scan_user_templates(tmp.path());
        let names: Vec<&str> = found.iter().map(|e| e.name.as_str()).collect();
        assert_eq!(names, vec!["alpha-layout", "zeta-layout"]);
        for e in &found {
            assert!(matches!(e.origin, TemplateOrigin::User { .. }));
        }
    }

    #[test]
    fn fallback_to_filestem_on_broken_json() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(tmp.path().join("broken.json"), "{not valid json").unwrap();
        let found = scan_user_templates(tmp.path());
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].name, "broken");
    }

    #[test]
    fn user_dir_lives_under_hvn_profiler_templates() {
        let dir = user_templates_dir();
        let s = dir.to_string_lossy();
        assert!(s.ends_with(format!("{sep}hvn-profiler{sep}templates", sep = std::path::MAIN_SEPARATOR).as_str()),
                "expected dir to end with hvn-profiler/templates, got {s}");
    }

    #[test]
    fn origin_savability_distinguishes_bundled_vs_disk() {
        let bundled = TemplateOrigin::Bundled;
        let user = TemplateOrigin::User {
            path: PathBuf::from("/tmp/x.json"),
        };
        let cli = TemplateOrigin::Cli {
            path: PathBuf::from("/tmp/y.json"),
        };
        assert!(!bundled.is_savable_in_place());
        assert!(user.is_savable_in_place());
        assert!(cli.is_savable_in_place());
        assert!(bundled.path().is_none());
        assert_eq!(user.path().unwrap(), Path::new("/tmp/x.json"));
        assert_eq!(cli.path().unwrap(), Path::new("/tmp/y.json"));
    }
}
