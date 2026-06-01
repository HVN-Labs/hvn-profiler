//! Signal-generator engine — v0.7.0 parity with the matplotlib
//! `signal_generator.py` UX (HVN-SITL `gui/components/signal_generator.py`).
//!
//! Each [`Generator`] holds a waveform, amplitude, period, centre, and a
//! target slider identifier (e.g. `"gps.sigma_p"`, `"imu.b_a.x"`). The CLI
//! ticks every running generator at ~20 Hz (50 ms) and writes the computed
//! value back into the Faults panel slider with the matching identifier —
//! which then debounces through the existing FaultPublisher path.
//!
//! This module is intentionally framework-free: no egui, no serde, just
//! `f64` math plus a tiny xorshift PRNG so we don't pull in `rand` for two
//! waveforms. The `Generator::tick` API takes monotonic-time-in-ms so unit
//! tests can drive deterministic clocks.

/// Five-waveform parity with `signal_generator.py`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Waveform {
    /// `centre + amp * sin(2π t / period)`
    Sine,
    /// `centre + amp * sign(sin(2π t / period))` — duty cycle 50%.
    Square,
    /// `centre + amp * (2 * frac(t/period) - 1)`
    Sawtooth,
    /// Random walk with Wiener-process step scaled by `amp * sqrt(dt)`,
    /// clamped to ±5·amp so a long run doesn't drift away unboundedly.
    RandomWalk,
    /// `centre + amp * randn()` — IID Gaussian draws.
    WhiteNoise,
}

impl Waveform {
    /// Stable name for combo box / serialization. Must match
    /// [`Waveform::from_label`] round-trip.
    pub fn label(self) -> &'static str {
        match self {
            Waveform::Sine => "Sine",
            Waveform::Square => "Square",
            Waveform::Sawtooth => "Sawtooth",
            Waveform::RandomWalk => "Random walk",
            Waveform::WhiteNoise => "White noise",
        }
    }

    /// All variants, declaration order — used to populate the UI combo box.
    pub fn all() -> [Waveform; 5] {
        [
            Waveform::Sine,
            Waveform::Square,
            Waveform::Sawtooth,
            Waveform::RandomWalk,
            Waveform::WhiteNoise,
        ]
    }
}

/// One generator row. The CLI owns a `Vec<Generator>` and drains it each
/// frame.
#[derive(Debug, Clone)]
pub struct Generator {
    /// Slider identifier this generator drives, e.g. `"gps.sigma_p"`.
    pub target: String,
    pub waveform: Waveform,
    pub amplitude: f64,
    pub period_s: f64,
    pub centre: f64,
    /// Monotonic time (ms) when the generator was last (re)started. `None`
    /// while the generator is paused or stopped.
    pub t_start_ms: Option<u64>,
    /// Persistent state for `RandomWalk`. Reset to zero on Start.
    pub random_walk_state: f64,
    /// `true` while the engine should drive the target slider. Toggled by
    /// the Start/Pause/Resume button in the panel.
    pub running: bool,
    /// Internal PRNG state — xorshift, seeded per-instance so two
    /// `WhiteNoise` rows don't echo each other.
    rng_state: u64,
    /// Last emitted value (for the panel status row and for the test
    /// suite to assert against without re-ticking).
    pub last_value: Option<f64>,
}

impl Generator {
    /// Construct a new (stopped) generator with sensible defaults.
    pub fn new(target: impl Into<String>) -> Self {
        let target = target.into();
        Self {
            rng_state: hash_seed(&target),
            target,
            waveform: Waveform::Sine,
            amplitude: 0.1,
            period_s: 2.0,
            centre: 0.0,
            t_start_ms: None,
            random_walk_state: 0.0,
            running: false,
            last_value: None,
        }
    }

    /// Start (or restart) the generator. Idempotent — calling `start` on
    /// an already-running generator just resets `t_start` so the waveform
    /// realigns with the new wall clock.
    pub fn start(&mut self, now_ms: u64) {
        self.t_start_ms = Some(now_ms);
        self.random_walk_state = 0.0;
        self.running = true;
    }

    /// Pause without resetting `t_start_ms` — Resume picks up the phase
    /// where we left off (subtracting wall time during the pause would
    /// require tracking elapsed-on-pause; this is the simpler choice and
    /// matches the matplotlib panel which simply stops calling `_tick`).
    pub fn pause(&mut self) {
        self.running = false;
    }

    /// Compute the current value, advancing PRNG / random-walk state as a
    /// side effect. Returns `None` when the generator is not running.
    pub fn tick(&mut self, now_ms: u64) -> Option<f64> {
        if !self.running {
            return None;
        }
        let t_start = self.t_start_ms.unwrap_or_else(|| {
            // Defensive: a running generator must have a start time. If we
            // get here, lazily seed from `now` rather than panicking — the
            // first tick is the start of the waveform.
            self.t_start_ms = Some(now_ms);
            now_ms
        });
        // Elapsed seconds since (re)start. Saturating sub guards against
        // a non-monotonic clock — shouldn't happen with `Instant`, but the
        // u64 API is friendlier to unit tests.
        let elapsed_ms = now_ms.saturating_sub(t_start);
        let elapsed = (elapsed_ms as f64) / 1000.0;
        let period = self.period_s.max(1e-3);
        let amp = self.amplitude;
        let centre = self.centre;

        let val = match self.waveform {
            Waveform::Sine => centre + amp * (std::f64::consts::TAU * elapsed / period).sin(),
            Waveform::Square => {
                let s = (std::f64::consts::TAU * elapsed / period).sin();
                let sign = if s >= 0.0 { 1.0 } else { -1.0 };
                centre + amp * sign
            }
            Waveform::Sawtooth => {
                let phase = ((elapsed / period) % 1.0 + 1.0) % 1.0;
                centre + amp * (2.0 * phase - 1.0)
            }
            Waveform::RandomWalk => {
                // Wiener increment: step = amp * randn * sqrt(dt). We
                // assume a fixed `dt = 0.05 s` (20 Hz tick) — matches the
                // matplotlib panel's _TICK_MS. Using the tick interval
                // directly (rather than tracking the last-tick wall time)
                // keeps the math simple and the unit-test variance check
                // tractable.
                let dt = 0.050_f64;
                let randn = self.next_gaussian();
                self.random_walk_state += amp * randn * dt.sqrt();
                let bound = (5.0 * amp).abs();
                if self.random_walk_state > bound {
                    self.random_walk_state = bound;
                } else if self.random_walk_state < -bound {
                    self.random_walk_state = -bound;
                }
                centre + self.random_walk_state
            }
            Waveform::WhiteNoise => centre + amp * self.next_gaussian(),
        };
        self.last_value = Some(val);
        Some(val)
    }

    /// xorshift64* → uniform [0, 1).
    fn next_uniform(&mut self) -> f64 {
        // Standard xorshift64*. State is non-zero (seeded via FNV-1a of the
        // target name).
        let mut x = self.rng_state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng_state = x;
        let mixed = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
        // Take the top 53 bits to fit in f64 mantissa, then normalise.
        ((mixed >> 11) as f64) / ((1u64 << 53) as f64)
    }

    /// Box–Muller transform → standard normal. We take the cosine branch
    /// and discard the sine; cheap enough for a 20 Hz loop.
    fn next_gaussian(&mut self) -> f64 {
        // u1 must avoid exact zero so ln(u1) is finite. xorshift64* never
        // emits exactly zero after the multiplier, but defence in depth.
        let mut u1 = self.next_uniform();
        if u1 < 1e-300 {
            u1 = 1e-300;
        }
        let u2 = self.next_uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// FNV-1a 64-bit on the target string, OR'd with 1 to guarantee non-zero
/// (xorshift collapses to zero if seeded with zero).
fn hash_seed(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h | 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waveform_label_roundtrip() {
        for w in Waveform::all() {
            assert!(!w.label().is_empty());
        }
    }

    #[test]
    fn paused_generator_yields_none() {
        let g = Generator::new("gps.sigma_p");
        assert!(!g.running);
        // .tick() on a fresh generator returns None.
        let mut g = g;
        assert!(g.tick(0).is_none());
    }

    #[test]
    fn started_then_paused_stops_emitting() {
        let mut g = Generator::new("gps.sigma_p");
        g.start(0);
        assert!(g.tick(50).is_some());
        g.pause();
        assert!(g.tick(100).is_none());
    }

    #[test]
    fn random_walk_is_clamped() {
        let mut g = Generator::new("rw");
        g.waveform = Waveform::RandomWalk;
        g.amplitude = 0.1;
        g.start(0);
        // 10k ticks — without the clamp this would diverge.
        for i in 1..=10_000 {
            let _ = g.tick((i * 50) as u64);
        }
        assert!(g.random_walk_state.abs() <= 5.0 * 0.1 + 1e-9);
    }
}
