// SPDX-License-Identifier: GPL-3.0-or-later
//! Fixed-size sensor calibration table: per-axis bias and scale, interpolated
//! across temperature compensation points.
//!
//! `no_std`, no allocation, no floats, no `libm`. All arithmetic is integer
//! fixed point with `i64` intermediates, so a host test and the
//! `thumbv7em-none-eabihf` firmware execute the same operations and get
//! bit-identical results. There is no unchecked constructor: every table is
//! validated, and an invalid one is a hard error (LOUD), never a silent clamp.
//!
//! Scales:
//! - temperature: `TEMP_ONE` units per degree Celsius (Q8);
//! - bias: `BIAS_ONE` units per raw sensor count (Q12);
//! - scale: `SCALE_ONE` units per unit multiplier (Q12), `1..=MAX_SCALE`.
//!
//! Correction applied per axis: `out = round((raw - bias) * scale)`, with
//! `bias` and `scale` linearly interpolated at the requested temperature and
//! half-away-from-zero rounding back to raw counts.

/// Number of calibrated axes (x, y, z).
pub const AXES: usize = 3;
/// Fixed capacity of temperature compensation points.
pub const MAX_TEMP_POINTS: usize = 8;

/// Fractional bits of temperature (°C).
pub const TEMP_SHIFT: i32 = 8;
/// Temperature units per °C.
pub const TEMP_ONE: i32 = 1 << TEMP_SHIFT;
/// Fractional bits of bias (raw counts).
pub const BIAS_SHIFT: i32 = 12;
/// Bias units per raw count.
pub const BIAS_ONE: i32 = 1 << BIAS_SHIFT;
/// Fractional bits of the scale multiplier.
pub const SCALE_SHIFT: i32 = 12;
/// Scale units per unit multiplier.
pub const SCALE_ONE: i32 = 1 << SCALE_SHIFT;
/// Largest accepted scale multiplier (16.0 in Q12). Bounds `apply`'s product.
pub const MAX_SCALE: i32 = 16 * SCALE_ONE;
/// Largest accepted absolute raw count. Bounds `apply`'s product.
pub const MAX_RAW_ABS: i32 = 1 << 20;

/// Fractional bits of the interpolation blend factor.
const FRAC_SHIFT: i32 = 16;
const FRAC_ONE: i64 = 1 << FRAC_SHIFT;
/// `raw * BIAS_ONE` times `scale_q12` is divided by this to get raw counts.
const OUT_DEN: i64 = (BIAS_ONE as i64) * (SCALE_ONE as i64);
const OUT_HALF: i64 = OUT_DEN / 2;

/// Why a table or a reading was refused. Every variant is a LOUD rejection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationError {
    /// No compensation points were supplied.
    NoPoints,
    /// More than [`MAX_TEMP_POINTS`] points were supplied.
    TooManyPoints,
    /// Temperatures are not strictly increasing; segments would be ambiguous.
    TemperaturesNotIncreasing,
    /// A scale is zero, negative, or above [`MAX_SCALE`].
    ScaleOutOfRange,
    /// A raw count is outside `±MAX_RAW_ABS`.
    RawOutOfRange,
}

/// Per-axis bias (Q12 counts) and scale (Q12 multiplier).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisCal {
    pub bias_q12: i32,
    pub scale_q12: i32,
}

/// One temperature compensation point, holding all axes at `temp_q8`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TempPoint {
    pub temp_q8: i32,
    pub axes: [AxisCal; AXES],
}

const EMPTY_POINT: TempPoint = TempPoint {
    temp_q8: 0,
    axes: [AxisCal {
        bias_q12: 0,
        scale_q12: SCALE_ONE,
    }; AXES],
};

/// A validated, immutable calibration table sized at compile time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CalibrationTable {
    points: [TempPoint; MAX_TEMP_POINTS],
    len: usize,
}

impl CalibrationTable {
    /// Validate and copy `points` into fixed storage. `const` so a static
    /// table is checked at compile time; never returns an unchecked table.
    pub const fn new(points: &[TempPoint]) -> Result<Self, CalibrationError> {
        let len = points.len();
        if len == 0 {
            return Err(CalibrationError::NoPoints);
        }
        if len > MAX_TEMP_POINTS {
            return Err(CalibrationError::TooManyPoints);
        }

        let mut stored = [EMPTY_POINT; MAX_TEMP_POINTS];
        let mut i = 0;
        while i < len {
            let p = points[i];
            if i > 0 && p.temp_q8 <= points[i - 1].temp_q8 {
                return Err(CalibrationError::TemperaturesNotIncreasing);
            }
            let mut a = 0;
            while a < AXES {
                let scale = p.axes[a].scale_q12 as u32;
                if scale == 0 || scale > MAX_SCALE as u32 {
                    return Err(CalibrationError::ScaleOutOfRange);
                }
                a += 1;
            }
            stored[i] = p;
            i += 1;
        }
        Ok(Self {
            points: stored,
            len,
        })
    }

    /// Number of stored compensation points (always `1..=MAX_TEMP_POINTS`).
    pub const fn point_count(&self) -> usize {
        self.len
    }

    /// Return a stored point by index. An out-of-range index panics (LOUD).
    pub const fn point(&self, index: usize) -> TempPoint {
        self.points[index]
    }

    /// Segment for `temp_q8`: `(lo, hi, blend)` with `blend` in
    /// `0..=FRAC_ONE`. Outside the table the blend is zero at the nearest end,
    /// which clamps to that endpoint's parameters.
    const fn segment(&self, temp_q8: i32) -> (usize, usize, i64) {
        let last = self.len - 1;
        if temp_q8 <= self.points[0].temp_q8 {
            return (0, 0, 0);
        }
        let mut i = 0;
        while i < last && temp_q8 > self.points[i + 1].temp_q8 {
            i += 1;
        }
        if i == last {
            return (last, last, 0);
        }
        let t0 = self.points[i].temp_q8 as i64;
        let t1 = self.points[i + 1].temp_q8 as i64;
        let blend = ((temp_q8 as i64 - t0) * FRAC_ONE) / (t1 - t0);
        (i, i + 1, blend)
    }

    /// Correct `raw` counts at `temp_q8`. Returns raw counts, rounded
    /// half-away-from-zero. Refuses out-of-range readings; never wraps.
    pub const fn apply(
        &self,
        raw: [i32; AXES],
        temp_q8: i32,
    ) -> Result<[i32; AXES], CalibrationError> {
        let mut a = 0;
        while a < AXES {
            if raw[a].unsigned_abs() > MAX_RAW_ABS as u32 {
                return Err(CalibrationError::RawOutOfRange);
            }
            a += 1;
        }

        let (lo, hi, blend) = self.segment(temp_q8);
        let mut out = [0i32; AXES];
        a = 0;
        while a < AXES {
            let bias = lerp_q12(
                self.points[lo].axes[a].bias_q12,
                self.points[hi].axes[a].bias_q12,
                blend,
            );
            let scale = lerp_q12(
                self.points[lo].axes[a].scale_q12,
                self.points[hi].axes[a].scale_q12,
                blend,
            );
            let num = ((raw[a] as i64 * BIAS_ONE as i64) - bias as i64) * scale as i64;
            let rounded = if num >= 0 {
                (num + OUT_HALF) / OUT_DEN
            } else {
                (num - OUT_HALF) / OUT_DEN
            };
            out[a] = rounded as i32;
            a += 1;
        }
        Ok(out)
    }
}

/// Linear interpolation of a Q12 value; truncation is the defined rounding.
const fn lerp_q12(a: i32, b: i32, blend: i64) -> i32 {
    (a as i64 + ((b as i64 - a as i64) * blend) / FRAC_ONE) as i32
}
