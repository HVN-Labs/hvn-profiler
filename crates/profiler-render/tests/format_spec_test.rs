//! v0.5.0 integration test — Python-style format-spec parser.
//!
//! Covers the spec mini-language declared in v0.5.0 plus a few edge cases:
//! - `{:.1f}`, `{:+.2f}`, `{:.3e}` numeric bodies
//! - literal suffix text concatenation (`°`, ` m/s`, etc.)
//! - empty / unparseable specs fall back to Rust's `{:.2}` default

use profiler_render::format_value_pub as fv;

#[test]
fn fixed_precision_rounds() {
    assert_eq!(fv("{:.1f}", 1.234), "1.2");
    assert_eq!(fv("{:.2f}", 1.234), "1.23");
    assert_eq!(fv("{:.0f}", 1.6), "2");
}

#[test]
fn sign_flag_forces_plus() {
    // Positive value gets an explicit `+`.
    assert_eq!(fv("{:+.2f}", 1.5), "+1.50");
    // Negative is unchanged (sign already there).
    assert_eq!(fv("{:+.2f}", -0.5), "-0.50");
}

#[test]
fn scientific_notation() {
    // `{:.3e}` of 1234.5 → "1.234e3" (Rust uses lowercase `e`).
    assert_eq!(fv("{:.3e}", 1234.5), "1.234e3");
    // With sign flag.
    assert_eq!(fv("{:+.2e}", 0.01), "+1.00e-2");
}

#[test]
fn literal_suffix_concatenated() {
    // SITL templates use trailing units like ° and m/s.
    assert_eq!(fv("{:+.1f}°", 12.34), "+12.3°");
    assert_eq!(fv("{:.2f} m/s", 3.42159), "3.42 m/s");
    assert_eq!(fv("{:.3e} m", 1234.5), "1.234e3 m");
}

#[test]
fn empty_spec_uses_default() {
    // Empty spec hits the `{:.2}` Rust default.
    assert_eq!(fv("", 1.23456), "1.23");
}

#[test]
fn unbraced_spec_kept_verbatim() {
    // No `{...}` at all → returned unchanged (caller handed us a literal).
    assert_eq!(fv("static label", 42.0), "static label");
}

#[test]
fn missing_close_brace_does_not_panic() {
    // Graceful degradation: incomplete spec → prefix + remainder, no crash.
    let s = fv("{:.1f m", 1.0);
    assert!(!s.is_empty());
}
