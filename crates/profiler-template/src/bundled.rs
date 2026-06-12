//! v0.8.0 — bundled template registry.
//!
//! Templates that ship with the binary (no user setup required). The JSON
//! text is embedded at compile time via `include_str!` so a release tarball
//! doesn't need to carry the `templates/` directory alongside the binary.
//!
//! ## Why these three?
//!
//! - `tutorial`    — v0.14.0 welcome layout (3x3) for first-time users. Mixes
//!   live plots (altitude, attitude, battery) with `info_text` instructional
//!   panels. **Bundled default**: when no `--template` is passed on the CLI,
//!   this is what loads.
//! - `hvn-default` — the SITL-default 7x3 grid (truth/gps/ekf/dr trails). Used
//!   by every SITL smoke test. Still available in the picker for power users.
//! - `real-drone`  — the same grid with DT-only panels (wind, mag interference,
//!   truth trails) stripped, for HIL / real-airframe sessions where DT ground
//!   truth isn't available.
//!
//! All three are also present at the repo root in `templates/` for editing /
//! diffing; the constants below are kept in sync by `include_str!`.

/// One bundled template descriptor: `(name, raw_json)`.
pub struct BundledTemplate {
    /// Display name used by the picker dropdown (matches the JSON's `name`).
    pub name: &'static str,
    /// JSON source — already validated to deserialise into [`crate::Template`]
    /// by the build's unit tests.
    pub json: &'static str,
}

/// v0.14.0 — Raw JSON for the tutorial template (bundled). First-run default.
pub const TUTORIAL_JSON: &str =
    include_str!("../../../templates/tutorial.json");

/// Raw JSON for the SITL-default template (bundled).
pub const HVN_DEFAULT_JSON: &str =
    include_str!("../../../templates/hvn-default.json");

/// Raw JSON for the real-drone template (bundled).
pub const REAL_DRONE_JSON: &str =
    include_str!("../../../templates/real-drone.json");

/// v0.16.9 — Raw JSON for the mag-debug template (bundled). A 2x2 magnetometer
/// / EKF debug layout: |MAG| total field + mx/my/mz (both scaled gauss->mGauss
/// so a clean earth field reads ~490 and calibration-inflated drones ~640-660),
/// EKF compass variance, and attitude in degrees. Built to investigate the
/// calibration-vs-ambient-field mismatch class of incidents.
pub const MAG_DEBUG_JSON: &str =
    include_str!("../../../templates/mag-debug.json");

/// v0.14.0 — Name of the implicit-default bundled template when no
/// `--template` is supplied and no recently-used user template exists.
/// Always present in [`BUNDLED`] and at index 0.
pub const DEFAULT_BUNDLED_NAME: &str = "tutorial";

/// All bundled templates the picker should surface, in display order.
///
/// v0.14.0: `tutorial` is the first entry and the implicit default — the CLI
/// loads it when no `--template` flag is given. `hvn-default` (full 7x3 +
/// 3D view) remains available in the picker for power users.
pub const BUNDLED: &[BundledTemplate] = &[
    BundledTemplate {
        name: "tutorial",
        json: TUTORIAL_JSON,
    },
    BundledTemplate {
        name: "hvn-default",
        json: HVN_DEFAULT_JSON,
    },
    BundledTemplate {
        name: "real-drone",
        json: REAL_DRONE_JSON,
    },
    BundledTemplate {
        name: "mag-debug",
        json: MAG_DEBUG_JSON,
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
        assert!(by_name("tutorial").is_some());
        assert!(by_name("hvn-default").is_some());
        assert!(by_name("real-drone").is_some());
        assert!(by_name("mag-debug").is_some());
        assert!(by_name("does-not-exist").is_none());
    }

    #[test]
    fn bundled_registry_has_at_least_two_entries() {
        assert!(BUNDLED.len() >= 2);
    }

    #[test]
    fn tutorial_is_first_and_is_default() {
        assert_eq!(BUNDLED[0].name, "tutorial");
        assert_eq!(DEFAULT_BUNDLED_NAME, "tutorial");
    }
}
