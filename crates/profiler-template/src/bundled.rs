//! v0.8.0 — bundled template registry.
//!
//! Templates that ship with the binary (no user setup required). The JSON
//! text is embedded at compile time via `include_str!` so a release tarball
//! doesn't need to carry the `templates/` directory alongside the binary.
//!
//! ## Why these two?
//!
//! - `hvn-default` — the SITL-default 7x3 grid (truth/gps/ekf/dr trails). Used
//!   by every SITL smoke test.
//! - `real-drone`  — the same grid with DT-only panels (wind, mag interference,
//!   truth trails) stripped, for HIL / real-airframe sessions where DT ground
//!   truth isn't available.
//!
//! Both are also present at the repo root in `templates/` for editing /
//! diffing; the constants below are kept in sync by `include_str!`.

/// One bundled template descriptor: `(name, raw_json)`.
pub struct BundledTemplate {
    /// Display name used by the picker dropdown (matches the JSON's `name`).
    pub name: &'static str,
    /// JSON source — already validated to deserialise into [`crate::Template`]
    /// by the build's unit tests.
    pub json: &'static str,
}

/// Raw JSON for the SITL-default template (bundled).
pub const HVN_DEFAULT_JSON: &str =
    include_str!("../../../templates/hvn-default.json");

/// Raw JSON for the real-drone template (bundled).
pub const REAL_DRONE_JSON: &str =
    include_str!("../../../templates/real-drone.json");

/// All bundled templates the picker should surface, in display order.
pub const BUNDLED: &[BundledTemplate] = &[
    BundledTemplate {
        name: "hvn-default",
        json: HVN_DEFAULT_JSON,
    },
    BundledTemplate {
        name: "real-drone",
        json: REAL_DRONE_JSON,
    },
];

/// Look up a bundled template by name (the value of its JSON `name` field).
pub fn by_name(name: &str) -> Option<&'static BundledTemplate> {
    BUNDLED.iter().find(|t| t.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Template;

    #[test]
    fn all_bundled_parse() {
        for t in BUNDLED {
            let parsed = Template::from_str(t.json)
                .unwrap_or_else(|e| panic!("bundled template '{}' failed to parse: {e}", t.name));
            assert_eq!(parsed.name, t.name, "JSON name must match registry name");
        }
    }

    #[test]
    fn by_name_finds_each_bundled() {
        assert!(by_name("hvn-default").is_some());
        assert!(by_name("real-drone").is_some());
        assert!(by_name("does-not-exist").is_none());
    }

    #[test]
    fn bundled_registry_has_at_least_two_entries() {
        assert!(BUNDLED.len() >= 2);
    }
}
