//! v0.7.0 — analytical / statistical checks on the signal-generator engine.
//!
//! The engine is deterministic for sine / square / sawtooth (closed-form),
//! and statistically constrained for random-walk / white-noise. We check
//! both with generous tolerances so the test stays green across compilers
//! / float modes.

use profiler_render::{Generator, Waveform};

#[test]
fn sine_matches_analytical_at_key_phases() {
    let mut g = Generator::new("gps.sigma_p");
    g.waveform = Waveform::Sine;
    g.amplitude = 0.5;
    g.period_s = 2.0;
    g.centre = 1.0;
    g.start(0);

    // t = 0  → centre
    let v0 = g.tick(0).expect("running gen yields a value");
    assert!((v0 - 1.0).abs() < 1e-9, "v0={v0}, want 1.0");

    // t = period/4 = 500 ms  → centre + amp = 1.5
    let v1 = g.tick(500).expect("running gen yields a value");
    assert!((v1 - 1.5).abs() < 1e-6, "v1={v1}, want 1.5");

    // t = period/2 = 1000 ms → centre (sin = 0)
    let v2 = g.tick(1000).expect("running gen yields a value");
    assert!((v2 - 1.0).abs() < 1e-6, "v2={v2}, want 1.0");

    // t = 3*period/4 = 1500 ms → centre - amp = 0.5
    let v3 = g.tick(1500).expect("running gen yields a value");
    assert!((v3 - 0.5).abs() < 1e-6, "v3={v3}, want 0.5");
}

#[test]
fn square_alternates_around_centre() {
    let mut g = Generator::new("imu.b_a.x");
    g.waveform = Waveform::Square;
    g.amplitude = 1.0;
    g.period_s = 1.0;
    g.centre = 0.0;
    g.start(0);

    // First half-cycle: sin(2π t/period) > 0 → +amp.
    let v_a = g.tick(100).expect("running");
    let v_b = g.tick(400).expect("running");
    assert!((v_a - 1.0).abs() < 1e-9, "v_a={v_a}");
    assert!((v_b - 1.0).abs() < 1e-9, "v_b={v_b}");

    // Second half-cycle: sin negative → -amp.
    let v_c = g.tick(600).expect("running");
    let v_d = g.tick(900).expect("running");
    assert!((v_c + 1.0).abs() < 1e-9, "v_c={v_c}");
    assert!((v_d + 1.0).abs() < 1e-9, "v_d={v_d}");
}

#[test]
fn sawtooth_ramps_from_minus_one_to_plus_one() {
    let mut g = Generator::new("gps.sigma_p");
    g.waveform = Waveform::Sawtooth;
    g.amplitude = 1.0;
    g.period_s = 2.0;
    g.centre = 0.0;
    g.start(0);

    // At t = 0 phase = 0 → -amp.
    let v0 = g.tick(0).expect("running");
    assert!((v0 + 1.0).abs() < 1e-9);

    // At t = period/2 = 1000 ms, phase = 0.5 → 0.
    let v_mid = g.tick(1000).expect("running");
    assert!(v_mid.abs() < 1e-6, "v_mid={v_mid}");

    // Just before wraparound (t = period - epsilon) → ~+amp.
    let v_pre = g.tick(1990).expect("running");
    assert!(v_pre > 0.95, "v_pre={v_pre}");
}

#[test]
fn white_noise_mean_is_close_to_centre() {
    let mut g = Generator::new("imu.sigma_a_n");
    g.waveform = Waveform::WhiteNoise;
    g.amplitude = 0.05;
    g.centre = 0.01;
    g.start(0);

    let n = 2000;
    let mut sum = 0.0;
    let mut sum_sq = 0.0;
    for i in 1..=n {
        let v = g.tick((i * 50) as u64).expect("running");
        let d = v - g.centre;
        sum += d;
        sum_sq += d * d;
    }
    let mean = g.centre + sum / n as f64;
    let var = sum_sq / n as f64;
    let stddev = var.sqrt();

    // Mean within ~5 standard errors of centre. SE = amp / sqrt(n).
    let se = g.amplitude / (n as f64).sqrt();
    assert!(
        (mean - g.centre).abs() < 5.0 * se,
        "mean={mean}, centre={}, 5*SE={}",
        g.centre,
        5.0 * se
    );

    // Std-dev within 25% of `amp`. xorshift Box–Muller is good enough.
    assert!(
        (stddev / g.amplitude - 1.0).abs() < 0.25,
        "stddev={stddev}, amp={}",
        g.amplitude
    );
}

#[test]
fn random_walk_std_grows_with_sqrt_t() {
    // For a Wiener process with step variance σ² · dt:
    //   Var(X_n) = n · σ² · dt   → stddev(X_n) ∝ sqrt(n)
    // The engine clamps to ±5·amp so we keep `amp` tiny and `n_short` /
    // `n_long` well clear of the clamp horizon. With amp=0.01 and dt=0.05:
    //   σ_step = amp·sqrt(dt) ≈ 0.002236; clamp at ±0.05.
    //   std(n=50)  ≈ sqrt(50)  · σ_step ≈ 0.0158
    //   std(n=200) ≈ sqrt(200) · σ_step ≈ 0.0316
    //   ratio ≈ 2.0  — clamp untouched.

    fn mk_with_seed(idx: u64) -> Generator {
        let mut g = Generator::new(format!("rw_seed_{idx}"));
        g.waveform = Waveform::RandomWalk;
        g.amplitude = 0.01;
        g.centre = 0.0;
        g
    }

    fn final_value(seed_idx: u64, n_ticks: u64) -> f64 {
        let mut g = mk_with_seed(seed_idx);
        g.start(0);
        let mut v = 0.0;
        for i in 1..=n_ticks {
            v = g.tick(i * 50).unwrap_or(v);
        }
        v
    }

    let runs = 400u64;
    let n_short = 50u64;
    let n_long = 200u64;

    let mut sumsq_short = 0.0;
    let mut sumsq_long = 0.0;
    for k in 0..runs {
        let v_s = final_value(k, n_short);
        sumsq_short += v_s * v_s;
        let v_l = final_value(k + 10_000, n_long);
        sumsq_long += v_l * v_l;
    }
    let std_short = (sumsq_short / runs as f64).sqrt();
    let std_long = (sumsq_long / runs as f64).sqrt();

    let ratio = std_long / std_short;
    let expected = (n_long as f64 / n_short as f64).sqrt(); // = 2.0
    assert!(
        (ratio - expected).abs() < 0.5,
        "random walk std ratio = {ratio}, expected ≈ {expected} (sqrt-t scaling). \
         std_short={std_short}, std_long={std_long}",
    );
}
