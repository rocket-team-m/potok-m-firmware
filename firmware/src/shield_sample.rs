// SPDX-License-Identifier: GPL-3.0-or-later
//! Typed pitot/static airdata seam for the runtime Mach shield.
//!
//! `Interlocks::evaluate_shield` gates on a Mach estimate the caller derives
//! from pitot dynamic pressure and barometric static pressure. This module is
//! that caller: it reads both through the [`Airdata`] trait (no pins, no HAL,
//! no wire field), derives Mach with deterministic `f32` arithmetic — no
//! `libm` — validates the readings and the result, and returns a typed
//! `ShieldSample`.
//!
//! A missing driver is a counted [`Unavailable`] reason, never a fabricated
//! zero that would look like healthy airspeed evidence to the shield.

use potok_m_firmware::control::ShieldSample;

/// Ratio of specific heats for air: `a^2 = GAMMA * p / rho`. The simulator's
/// atmosphere uses the same value, so this Mach matches its `speed / a`.
const GAMMA: f32 = 1.4;

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
}

impl UnavailableCounts {
    pub fn note(&mut self, reason: Unavailable) {
        match reason {
            Unavailable::NoDriver => self.no_driver += 1,
            Unavailable::NonFinite => self.non_finite += 1,
            Unavailable::NonPositive => self.non_positive += 1,
        }
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
            },
        }
    }

    /// One tick's sample. On any failure the result is exactly
    /// `ShieldSample::UNAVAILABLE` — the shield vetoes with `PitotInvalid` —
    /// and the reason is counted here, never hidden as a healthy zero.
    pub fn sample<S: Airdata>(&mut self, source: &S) -> ShieldSample {
        match Self::derive(source) {
            Ok(sample) => sample,
            Err(reason) => {
                self.unavailable.note(reason);
                ShieldSample::UNAVAILABLE
            }
        }
    }

    pub const fn unavailable_counts(&self) -> UnavailableCounts {
        self.unavailable
    }

    fn derive<S: Airdata>(source: &S) -> Result<ShieldSample, Unavailable> {
        let (Some(dp), Some(p)) = (source.pitot_dp_pa(), source.static_pa()) else {
            return Err(Unavailable::NoDriver);
        };
        if !dp.is_finite() || !p.is_finite() {
            return Err(Unavailable::NonFinite);
        }
        if dp <= 0.0 || p <= 0.0 {
            return Err(Unavailable::NonPositive);
        }
        // q = rho*V^2/2 and a^2 = GAMMA*p/rho give M^2 = 2q/(GAMMA*p).
        // This slightly overestimates the isentropic Mach, which is on the
        // conservative side of a ceiling gate.
        let mach = sqrt_approx((2.0 * dp) / (GAMMA * p));
        if !mach.is_finite() {
            return Err(Unavailable::NonFinite);
        }
        if mach <= 0.0 {
            return Err(Unavailable::NonPositive);
        }
        Ok(ShieldSample {
            pitot_dp: dp as i16,
            mach_est: mach,
        })
    }
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
