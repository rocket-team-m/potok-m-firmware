// SPDX-License-Identifier: GPL-3.0-or-later
//! Typed pitot/static airdata seam for the runtime Mach shield.
//!
//! `Interlocks::evaluate_shield` gates on a Mach estimate the caller derives
//! from pitot dynamic pressure and barometric static pressure. This module is
//! that caller: it reads both through the [`Airdata`] trait (no pins, no HAL,
//! no wire field), derives subsonic Mach from the isentropic Pitot relation
//! with deterministic `f32` arithmetic — no `libm` — validates the readings
//! and the result, and returns a typed `ShieldSample`. A ratio outside the
//! subsonic domain is refused, never clamped to M = 1.
//!
//! A missing driver is a counted [`Unavailable`] reason, never a fabricated
//! zero that would look like healthy airspeed evidence to the shield.

use potok_m_firmware::control::ShieldSample;

/// Ratio of specific heats for air: `a^2 = GAMMA * p / rho`. The simulator's
/// atmosphere uses the same value, so this Mach matches its `speed / a`.
const GAMMA: f32 = 1.4;
/// `(GAMMA - 1) / 2` for air.
const B: f32 = (GAMMA - 1.0) * 0.5;

/// Why the isentropic Mach solve could not produce a sample.
///
/// Kept separate from [`Unavailable`] so the math seam can name a supersonic
/// ratio without pretending the reading was non-finite. The sampler maps each
/// case to a counted fallback below.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MachError {
    /// A pressure, their ratio, or the derived Mach is not finite.
    NonFinite,
    /// A pressure is zero or negative — impossible for a pitot/static pair.
    NonPositive,
    /// `q_c/p` is at or above the M = 1 Pitot limit. The subsonic relation has
    /// no solution here; this is refused loudly, never clamped.
    AboveSubsonicLimit,
}

/// The board-facing airdata seam. No pin or peripheral is claimed here: the
/// schematic-bound `HwBoard` returns `None` until its drivers are wired.
pub trait Airdata {
    /// Pitot differential (dynamic) pressure in Pa, `None` with no driver.
    fn pitot_dp_pa(&self) -> Option<f32>;
    /// Barometric static pressure in Pa, `None` with no driver.
    fn static_pa(&self) -> Option<f32>;
}

/// A host-only source with neither driver wired: the mock/fallback case. The
/// firmware's `HwBoard` implements `Airdata` directly rather than using this.
#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, Default)]
pub struct NoAirdata;

#[cfg(not(target_os = "none"))]
impl Airdata for NoAirdata {
    fn pitot_dp_pa(&self) -> Option<f32> {
        None
    }
    fn static_pa(&self) -> Option<f32> {
        None
    }
}

/// Why a sample could not be produced. Tallied, never dropped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unavailable {
    /// A pressure reading has no driver yet.
    NoDriver,
    /// A pressure reading or the derived Mach is NaN/infinite.
    NonFinite,
    /// A pressure reading or the derived Mach is not positive.
    NonPositive,
}

/// Per-reason tally of unavailable samples. Never cleared, so telemetry can
/// tell a missing driver from a bad sensor without a tracer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UnavailableCounts {
    pub no_driver: u32,
    pub non_finite: u32,
    pub non_positive: u32,
    /// Samples refused because `q_c/p` is outside the subsonic Pitot domain.
    pub above_subsonic: u32,
}

impl UnavailableCounts {
    pub fn note(&mut self, reason: Unavailable) {
        match reason {
            Unavailable::NoDriver => self.no_driver += 1,
            Unavailable::NonFinite => self.non_finite += 1,
            Unavailable::NonPositive => self.non_positive += 1,
        }
    }

    pub fn note_above_subsonic(&mut self) {
        self.above_subsonic += 1;
    }
}

/// Reads airdata once per tick, derives Mach, and counts every fallback.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShieldSampler {
    unavailable: UnavailableCounts,
}

impl ShieldSampler {
    pub const fn new() -> Self {
        Self {
            unavailable: UnavailableCounts {
                no_driver: 0,
                non_finite: 0,
                non_positive: 0,
                above_subsonic: 0,
            },
        }
    }

    /// One tick's sample. On any failure the result is exactly
    /// `ShieldSample::UNAVAILABLE` — the shield vetoes with `PitotInvalid` —
    /// and the reason is counted here, never hidden as a healthy zero.
    pub fn sample<S: Airdata>(&mut self, source: &S) -> ShieldSample {
        match Self::derive(source) {
            Ok(sample) => sample,
            Err(DeriveError::Unavailable(reason)) => {
                self.unavailable.note(reason);
                ShieldSample::UNAVAILABLE
            }
            Err(DeriveError::AboveSubsonic) => {
                self.unavailable.note_above_subsonic();
                ShieldSample::UNAVAILABLE
            }
        }
    }

    pub const fn unavailable_counts(&self) -> UnavailableCounts {
        self.unavailable
    }

    fn derive<S: Airdata>(source: &S) -> Result<ShieldSample, DeriveError> {
        let (Some(dp), Some(p)) = (source.pitot_dp_pa(), source.static_pa()) else {
            return Err(DeriveError::Unavailable(Unavailable::NoDriver));
        };
        match mach_from_pitot(dp, p) {
            Ok(mach) => Ok(ShieldSample {
                pitot_dp: dp as i16,
                mach_est: mach,
            }),
            Err(MachError::NonFinite) => Err(DeriveError::Unavailable(Unavailable::NonFinite)),
            Err(MachError::NonPositive) => Err(DeriveError::Unavailable(Unavailable::NonPositive)),
            Err(MachError::AboveSubsonicLimit) => Err(DeriveError::AboveSubsonic),
        }
    }
}

/// Internal split so an out-of-domain ratio gets its own counter without
/// masquerading as a non-finite sensor reading.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DeriveError {
    Unavailable(Unavailable),
    AboveSubsonic,
}

/// `(1 + B*M^2)^7` with `u = 1 + B*M^2`, by three stable multiplications.
/// Positive `u` keeps the map monotone, so the bisection below cannot
/// oscillate on rounding. Raising both sides of the Pitot relation to the
/// second power turns the fractional exponent `GAMMA/(GAMMA-1) = 3.5` into
/// the integer exponent 7: no `powf`, no `exp`, no `libm`.
const fn pow7(u: f32) -> f32 {
    let u2 = u * u;
    let u3 = u2 * u;
    let u6 = u3 * u3;
    u6 * u
}

/// `(1 + B)^7` — the squared impact-pressure ratio at M = 1.
const SUBSONIC_SQ_LIMIT: f32 = pow7(1.0 + B);

/// Bisection sweeps are a fixed count, so the result is bit-reproducible:
/// no convergence branch and no host-device round-trip. 32 halvings of
/// `[0, 1]` resolve `M^2` far below one f32 ULP.
const BISECTION_STEPS: u32 = 32;

/// Solve the subsonic isentropic Pitot relation for Mach:
///
/// `q_c/p = (1 + (GAMMA-1)/2 * M^2)^(GAMMA/(GAMMA-1)) - 1`,
///
/// with impact pressure `q_c = p_t - p = dp` (Anderson, *Fundamentals of
/// Aerodynamics*, compressible Pitot formula). Squaring gives an equivalent
/// monotone equation in `x = M^2`:
///
/// `(1 + B*x)^7 = (1 + dp/p)^2`,
///
/// solved by fixed-count bisection on `x ∈ [0, 1]` and one Newton-refined
/// bit-trick square root. Ratios at or above the M = 1 limit are refused with
/// [`MachError::AboveSubsonicLimit`] — there is no subsonic solution, so the
/// caller must not be handed a clamped M = 1 sample.
pub fn mach_from_pitot(dp: f32, p: f32) -> Result<f32, MachError> {
    if !dp.is_finite() || !p.is_finite() {
        return Err(MachError::NonFinite);
    }
    if dp <= 0.0 || p <= 0.0 {
        return Err(MachError::NonPositive);
    }

    let r = dp / p;
    let target = (1.0 + r) * (1.0 + r);
    if !target.is_finite() || target >= SUBSONIC_SQ_LIMIT {
        return Err(MachError::AboveSubsonicLimit);
    }

    let mut lo = 0.0f32;
    let mut hi = 1.0f32;
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        if pow7(1.0 + B * mid) < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let mach = sqrt_approx(0.5 * (lo + hi));
    if !mach.is_finite() || mach <= 0.0 {
        return Err(MachError::NonFinite);
    }
    Ok(mach)
}

/// Bit-trick seed plus four Newton steps: deterministic and `libm`-free. The
/// seed halves the exponent via `f32::from_bits((bits >> 1) + 0x1fc0_0000)`;
/// four steps refine it well below one f32 ULP over the airdata range.
fn sqrt_approx(x: f32) -> f32 {
    let mut g = f32::from_bits((x.to_bits() >> 1) + 0x1fc0_0000);
    for _ in 0..4 {
        g = 0.5 * (g + x / g);
    }
    g
}
