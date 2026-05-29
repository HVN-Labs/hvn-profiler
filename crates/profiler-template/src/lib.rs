//! profiler-template — JSON template loader.
//!
//! Templates describe the panel layout, trace bindings (channel -> color,
//! axis, units), and decoration (titles, descriptions). v0.0.1 ships the
//! struct definitions and a loader stub so downstream crates can be written
//! against a stable type even though the renderer ignores templates today.
//!
//! Multi-panel rendering lands in v0.2.0.

use std::path::Path;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

/// Top-level template document.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Template {
    /// Human-readable template name (shown in the title bar).
    #[serde(default)]
    pub name: String,
    /// One entry per visual panel.
    #[serde(default)]
    pub panels: Vec<Panel>,
}

/// A single plot panel.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Panel {
    pub title: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub y_unit: String,
    #[serde(default)]
    pub traces: Vec<Trace>,
}

/// A single trace inside a panel.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Trace {
    /// Source channel name, e.g. `"ATT.Roll"`.
    pub channel: String,
    /// Optional display label (defaults to `channel`).
    #[serde(default)]
    pub label: String,
    /// `#RRGGBB` colour string. Renderer falls back to a palette if empty.
    #[serde(default)]
    pub color: String,
}

/// Load a template from a JSON file on disk.
pub fn load(path: impl AsRef<Path>) -> Result<Template> {
    let p = path.as_ref();
    let text = std::fs::read_to_string(p)
        .with_context(|| format!("reading template {}", p.display()))?;
    let tpl: Template = serde_json::from_str(&text)
        .with_context(|| format!("parsing template {}", p.display()))?;
    Ok(tpl)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_minimal() {
        let json = r#"{
            "name": "demo",
            "panels": [
                {
                    "title": "Attitude",
                    "y_unit": "deg",
                    "traces": [{ "channel": "ATT.Roll" }]
                }
            ]
        }"#;
        let t: Template = serde_json::from_str(json).unwrap();
        assert_eq!(t.name, "demo");
        assert_eq!(t.panels.len(), 1);
        assert_eq!(t.panels[0].traces[0].channel, "ATT.Roll");
    }
}
