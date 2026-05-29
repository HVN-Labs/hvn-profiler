//! profiler-render — egui_plot wrappers + GPU-friendly trace storage.
//!
//! v0.0.1 is a placeholder. The real surface area lands in v0.1.0:
//! - Ring-buffer–backed trace storage with configurable retention.
//! - Decimation for zoomed-out views (LTTB or min/max bucketing).
//! - Per-panel layout helpers (matches the JSON template schema).
//!
//! Today this crate just exposes a version constant so the CLI links against
//! something non-empty and the workspace dependency graph is exercised.

/// Build-time crate version, for logging from the CLI.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Sanity check used by the CLI smoke test.
pub fn hello() -> &'static str {
    "profiler-render v0.0.1 (placeholder)"
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_is_versioned() {
        assert!(hello().contains("v0.0.1"));
    }
}
