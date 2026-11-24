// SPDX-License-Identifier: GPL-3.0-or-later
//! Fixed-point explicit complementary filter on SO(3), after
//! Mahony, Hamel & Pflimlin, *Nonlinear Complementary Filters on the Special
//! Orthogonal Group*, IEEE TAC 53(5), 2008, DOI 10.1109/TAC.2008.923738
//! (author manuscript HAL hal-00488376v1). Equation numbers below refer to
//! that paper; appendix numbers are given where the manuscript states the
//! same result in quaternion form.
//!
//! The filter fuses a body-frame gyroscope with gravity (accelerometer) and
//! optional north (magnetometer) reference directions. It carries a unit
//! quaternion, an online gyro-bias estimate and positive gains. All arithmetic
//! is integer fixed point: no floats, no `libm`, no allocation, so the same
//! source builds for `thumbv7em-none-eabihf` and for host tests.
//!
//! Conventions and the equations they implement (z-up world, NED-free body):
//! - [`Quat`] is Hamilton scalar-first and rotates body vectors into world;
//!   `R(q)` is that body-to-world rotation. `q̇ = ½ q ⊗ p(Ω)` for a body-frame
//!   rate `Ω` is Eq. 45 (Eq. 48b in the quaternion appendix).
//! - World up is +e3 and world north is +e1. The accelerometer input is
//!   specific force, i.e. "up" in body frame at rest: `v_a = a/|a| ≈ Rᵀe3`.
//!   The paper writes `v_a ≈ −Rᵀe3` (Eq. 29) because its inertial e3 points
//!   down along gravity; that is the same physical vector under the opposite
//!   world-axis label.
//! - `v̂_a = R̂ᵀe3` and `v̂_n = R̂ᵀe1` are the estimated body-frame reference
//!   directions (Eq. 29); the code forms them as rows 3 and 1 of `R(q)`.
//! - The innovation is `ω_mes = Σ kᵢ (vᵢ × v̂ᵢ)` (Eq. 32c): measured cross
//!   estimated. The per-reference weights `kᵢ > 0` are Eq. 30/31.
//! - The gyro model is `Ω_y = Ω + b` (Eq. 11b) with `b` a constant bias.
//!   The estimate `b̂` is SUBTRACTED from `Ω_y` and integrated with a NEGATIVE
//!   sign: `Ω_c = Ω_y − b̂ + kP·ω_mes` and `ḃ̂ = −kI·ω_mes` (Eq. 32a/32b).
//!   Those two signs are what make the Lyapunov derivative
//!   `V̇ = −kP‖Pa(R̃M)‖²` (Eq. 33) negative semi-definite. Reversing either
//!   inverts the feedback: the attitude can still level while `b̂` diverges.
//! - One discrete step applies `q ← normalize(q ⊗ [1, Ω_c·dt/2])`, the
//!   first-order form of Eq. 48b (cf. Eq. 45). Renormalizing each step is a
//!   projection back onto the unit-quaternion sphere, not a paper equation.
//! - Gains: `kp`/`ki` are `kP·w_a` and `kI·w_a` for gravity, `kp_mag`/
//!   `ki_mag` the same for north, where `w_i > 0` are the relative confidence
//!   weights of Eq. 30/31. Every gain must be positive; the default has
//!   `w_m/w_a = 0.5`.
//! - The manuscript is internally inconsistent by a factor of two: Eq. 32c
//!   defines `ω_mes` with `kᵢ`, while Eq. 34 and Eq. 48a print `kᵢ/2`. This
//!   code implements Eq. 32c verbatim and absorbs the factor in the positive
//!   per-reference weights; no gain is retuned to compensate a sign.
//!
//! Scales:
//! - unit quaternions / unit vectors: `2^UNIT_SHIFT` per unit;
//! - angular rates and bias: `2^RATE_SHIFT` per rad/s;
//! - time step: `2^DT_SHIFT` per second;
//! - gains: `2^GAIN_SHIFT` per (rad/s per unit error) and per (rad/s^2).

/// Fractional bits of unit quaternions and unit vectors (Q30).
pub const UNIT_SHIFT: i32 = 30;
/// Fixed scale of unit quaternions and unit vectors.
pub const UNIT: i64 = 1 << UNIT_SHIFT;
/// Fractional bits of angular rates (Q16 rad/s).
pub const RATE_SHIFT: i32 = 16;
/// Fixed scale of angular rates and gyro bias.
pub const RATE: i64 = 1 << RATE_SHIFT;
/// Fractional bits of the time step (Q20 seconds).
pub const DT_SHIFT: i32 = 20;
/// Fixed scale of the time step.
pub const DT: i64 = 1 << DT_SHIFT;
/// Fractional bits of the gains (Q8).
pub const GAIN_SHIFT: i32 = 8;
/// Fixed scale of the gains.
pub const GAIN: i64 = 1 << GAIN_SHIFT;
/// Fractional bits of the internal bias accumulator. Q16 would quantize a
/// small integral increment to zero and freeze the estimate.
pub const BIAS_SHIFT: i32 = 32;

/// Convert a time step in microseconds to the filter's Q20-seconds form.
pub const fn dt_q20_from_us(us: u32) -> i32 {
    ((us as i64 * DT) / 1_000_000) as i32
}

/// A fixed-point vector in Q30 (see [`UNIT`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Vec3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0, y: 0, z: 0 };

    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// Squared length in Q60 units (`UNIT^2` means unit length).
    pub fn norm_squared(self) -> i64 {
        let x = self.x as i64;
        let y = self.y as i64;
        let z = self.z as i64;
        x * x + y * y + z * z
    }

    /// Scale to unit length; `None` for the zero vector.
    pub fn normalized(self) -> Option<Self> {
        let n2 = self.norm_squared();
        if n2 == 0 {
            return None;
        }
        let n = isqrt(n2 as u64) as i64;
        Some(Self {
            x: ((self.x as i64 * UNIT) / n) as i32,
            y: ((self.y as i64 * UNIT) / n) as i32,
            z: ((self.z as i64 * UNIT) / n) as i32,
        })
    }
}

/// Hamilton scalar-first unit quaternion in Q30 (see [`UNIT`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Quat {
    pub w: i32,
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Quat {
    pub const IDENTITY: Self = Self {
        w: UNIT as i32,
        x: 0,
        y: 0,
        z: 0,
    };

    pub const fn new(w: i32, x: i32, y: i32, z: i32) -> Self {
        Self { w, x, y, z }
    }

    /// Squared norm in Q60 units (`UNIT^2` means unit length).
    pub fn norm_squared(self) -> i64 {
        let w = self.w as i64;
        let x = self.x as i64;
        let y = self.y as i64;
        let z = self.z as i64;
        w * w + x * x + y * y + z * z
    }

    /// Renormalize to unit length; `None` for the zero quaternion.
    pub fn normalized(self) -> Option<Self> {
        let n2 = self.norm_squared();
        if n2 == 0 {
            return None;
        }
        let n = isqrt(n2 as u64) as i64;
        Some(Self {
            w: ((self.w as i64 * UNIT) / n) as i32,
            x: ((self.x as i64 * UNIT) / n) as i32,
            y: ((self.y as i64 * UNIT) / n) as i32,
            z: ((self.z as i64 * UNIT) / n) as i32,
        })
    }
}

/// Mahony gains in Q8: `kp` is rad/s per unit error, `ki` is rad/s^2.
/// Magnetometer entries apply the same units to the north reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gains {
    pub kp: i32,
    pub ki: i32,
    pub kp_mag: i32,
    pub ki_mag: i32,
}

impl Gains {
    /// kp = 2.0, ki = 0.5, kp_mag = 1.0, ki_mag = 0.25.
    pub const DEFAULT: Self = Self {
        kp: 2 * GAIN as i32,
        ki: GAIN as i32 / 2,
        kp_mag: GAIN as i32,
        ki_mag: GAIN as i32 / 4,
    };

    pub const fn new(kp: i32, ki: i32, kp_mag: i32, ki_mag: i32) -> Self {
        Self {
            kp,
            ki,
            kp_mag,
            ki_mag,
        }
    }
}

impl Default for Gains {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Reason a filter step was refused. No silent fallbacks: a degenerate
/// reference is reported, never interpreted as "no correction needed".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterError {
    ZeroDt,
    DegenerateAccel,
    DegenerateMag,
    DegenerateQuaternion,
}

/// Mahony-style SO(3) filter state: quaternion, gyro bias and gains.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mahony {
    q: Quat,
    /// Bias accumulator in Q32 rad/s; `bias()` exposes the Q16 view.
    bias_x: i64,
    bias_y: i64,
    bias_z: i64,
    gains: Gains,
}

impl Mahony {
    /// Identity attitude, zero bias, given gains.
    pub const fn new(gains: Gains) -> Self {
        Self {
            q: Quat::IDENTITY,
            bias_x: 0,
            bias_y: 0,
            bias_z: 0,
            gains,
        }
    }

    pub const fn quaternion(&self) -> Quat {
        self.q
    }

    /// Estimate `b̂` of the bias added to the gyro in Q16 rad/s (Eq. 11b,
    /// Eq. 32b). Converges to the true bias `b`; subtract it from raw gyro.
    pub const fn bias(&self) -> Vec3 {
        Vec3::new(
            (self.bias_x >> (BIAS_SHIFT - RATE_SHIFT)) as i32,
            (self.bias_y >> (BIAS_SHIFT - RATE_SHIFT)) as i32,
            (self.bias_z >> (BIAS_SHIFT - RATE_SHIFT)) as i32,
        )
    }

    pub const fn gains(&self) -> Gains {
        self.gains
    }

    pub fn set_gains(&mut self, gains: Gains) {
        self.gains = gains;
    }

    /// Back to identity attitude and zero bias; gains are kept.
    pub fn reset(&mut self) {
        self.q = Quat::IDENTITY;
        self.bias_x = 0;
        self.bias_y = 0;
        self.bias_z = 0;
    }

    /// One filter step.
    ///
    /// `gyro` is body rate in Q16 rad/s; `accel` / `mag` are body-frame
    /// reference directions (normalized internally, `None` disables that
    /// reference and freezes the corresponding bias terms); `dt_us` is the
    /// true elapsed time in microseconds.
    pub fn update(
        &mut self,
        gyro: Vec3,
        accel: Option<Vec3>,
        mag: Option<Vec3>,
        dt_us: u32,
    ) -> Result<(), FilterError> {
        let dt = dt_q20_from_us(dt_us);
        if dt <= 0 {
            return Err(FilterError::ZeroDt);
        }

        let q = self.q;
        // Estimated up and north in body frame: rows 3 and 1 of R(q).
        let up = Vec3::new(
            (twice(q.x, q.z) - twice(q.w, q.y)) as i32,
            (twice(q.y, q.z) + twice(q.w, q.x)) as i32,
            (UNIT - twice(q.x, q.x) - twice(q.y, q.y)) as i32,
        );
        let north = Vec3::new(
            (UNIT - twice(q.y, q.y) - twice(q.z, q.z)) as i32,
            (twice(q.x, q.y) - twice(q.w, q.z)) as i32,
            (twice(q.x, q.z) + twice(q.w, q.y)) as i32,
        );

        let e_acc = match accel {
            Some(a) => {
                let a = a.normalized().ok_or(FilterError::DegenerateAccel)?;
                Some(cross(a, up))
            }
            None => None,
        };
        let e_mag = match mag {
            Some(m) => {
                let m = m.normalized().ok_or(FilterError::DegenerateMag)?;
                Some(cross(m, north))
            }
            None => None,
        };

        let g = self.gains;
        let dt_i = dt as i64;

        // Eq. 32b: ḃ̂ = −kI·ω_mes. Shift 26 converts Q8 * Q30 * Q20 into
        // Q32 rad/s. The minus is the paper's sign; it is what drives b̂ to
        // the bias b present in Ω_y = Ω + b (Eq. 11b).
        let bias_shift = GAIN_SHIFT + UNIT_SHIFT + DT_SHIFT - BIAS_SHIFT;
        let mut bx = self.bias_x;
        let mut by = self.bias_y;
        let mut bz = self.bias_z;
        if let Some(e) = e_acc {
            bx -= (g.ki as i64 * e.x as i64 * dt_i) >> bias_shift;
            by -= (g.ki as i64 * e.y as i64 * dt_i) >> bias_shift;
            bz -= (g.ki as i64 * e.z as i64 * dt_i) >> bias_shift;
        }
        if let Some(e) = e_mag {
            bx -= (g.ki_mag as i64 * e.x as i64 * dt_i) >> bias_shift;
            by -= (g.ki_mag as i64 * e.y as i64 * dt_i) >> bias_shift;
            bz -= (g.ki_mag as i64 * e.z as i64 * dt_i) >> bias_shift;
        }

        // Eq. 32a: Ω_c = Ω_y − b̂ + kP·ω_mes. Shift 22 converts Q8 * Q30
        // into Q16 rad/s; shift 16 turns the Q32 accumulator into Q16.
        let rate_shift = BIAS_SHIFT - RATE_SHIFT;
        let mut wx = gyro.x as i64 - (bx >> rate_shift);
        let mut wy = gyro.y as i64 - (by >> rate_shift);
        let mut wz = gyro.z as i64 - (bz >> rate_shift);
        if let Some(e) = e_acc {
            wx += (g.kp as i64 * e.x as i64) >> 22;
            wy += (g.kp as i64 * e.y as i64) >> 22;
            wz += (g.kp as i64 * e.z as i64) >> 22;
        }
        if let Some(e) = e_mag {
            wx += (g.kp_mag as i64 * e.x as i64) >> 22;
            wy += (g.kp_mag as i64 * e.y as i64) >> 22;
            wz += (g.kp_mag as i64 * e.z as i64) >> 22;
        }

        // Eq. 48b, first order: dq ~ [1, Ω_c·dt/2]. Shift 7 converts
        // (Q16 rad/s * Q20 s) into Q30 half-angles.
        let dq = Quat::new(
            UNIT as i32,
            ((wx * dt_i) >> 7) as i32,
            ((wy * dt_i) >> 7) as i32,
            ((wz * dt_i) >> 7) as i32,
        );
        let next = mul(q, dq)
            .normalized()
            .ok_or(FilterError::DegenerateQuaternion)?;

        self.q = next;
        self.bias_x = bx;
        self.bias_y = by;
        self.bias_z = bz;
        Ok(())
    }
}

impl Default for Mahony {
    fn default() -> Self {
        Self::new(Gains::DEFAULT)
    }
}

/// 2*a*b in Q30, widened so the intermediate cannot overflow.
fn twice(a: i32, b: i32) -> i64 {
    (a as i64 * b as i64) >> (UNIT_SHIFT - 1)
}

/// Cross product of two Q30 unit vectors, back in Q30.
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: ((a.y as i64 * b.z as i64 - a.z as i64 * b.y as i64) >> UNIT_SHIFT) as i32,
        y: ((a.z as i64 * b.x as i64 - a.x as i64 * b.z as i64) >> UNIT_SHIFT) as i32,
        z: ((a.x as i64 * b.y as i64 - a.y as i64 * b.x as i64) >> UNIT_SHIFT) as i32,
    }
}

/// Hamilton quaternion product of two Q30 quaternions, back in Q30.
fn mul(a: Quat, b: Quat) -> Quat {
    let (aw, ax, ay, az) = (a.w as i64, a.x as i64, a.y as i64, a.z as i64);
    let (bw, bx, by, bz) = (b.w as i64, b.x as i64, b.y as i64, b.z as i64);
    Quat {
        w: ((aw * bw - ax * bx - ay * by - az * bz) >> UNIT_SHIFT) as i32,
        x: ((aw * bx + ax * bw + ay * bz - az * by) >> UNIT_SHIFT) as i32,
        y: ((aw * by - ax * bz + ay * bw + az * bx) >> UNIT_SHIFT) as i32,
        z: ((aw * bz + ax * by - ay * bx + az * bw) >> UNIT_SHIFT) as i32,
    }
}

/// Integer square root by Newton iteration; `isqrt(0) = 0`.
fn isqrt(n: u64) -> u64 {
    if n < 2 {
        return n;
    }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}
