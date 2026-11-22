// SPDX-License-Identifier: GPL-3.0-or-later
//! Airframe PID: one axis, one tick, no I/O and no allocation.
//!
//! The control law is written once over [`PidNum`] and runs on two backends:
//! [`Fx`] (Q16.16 in `i32`, the fixed-point path for the flight controller) and
//! `f32` (the host-test path). No divisions, `libm` calls or float intrinsics
//! are used in [`Pid::update`]; gains are pre-scaled to per-step units at
//! construction. Both paths use plain add/sub/mul so a host test sees the
//! same sequence of operations the firmware runs.
//!
//! Safety properties, in the order they are applied each tick:
//! - derivative on **measurement**, never on error: a setpoint step cannot
//!   produce a derivative kick;
//! - integral state is stored in output units and clamped to `i_limit`;
//! - clamping anti-windup: the integrator freezes when the output is
//!   saturated and the next integration step would push it further out;
//! - bumpless setpoint: a setpoint change preloads the integrator so the
//!   output is continuous across the change;
//! - output saturation to `out_limit`, with a `saturated` flag for counters.
//!
//! Output is in the same raw units as [`Command`]: 0.001 rad per count, bounded
//! by [`MAX_CORRECTION_RAW`]. [`command_from_raw`] is the only bridge to the
//! existing control type; this module has no pins, sensors or actuators.

use crate::{Command, MAX_CORRECTION_RAW};

/// Default actuator bound, shared with the firmware's code veto.
pub const DEFAULT_OUT_LIMIT_RAW: i32 = MAX_CORRECTION_RAW as i32;
/// Tick period of the existing 500 Hz control loop (`control.rs`).
pub const TICK_500HZ_S: f32 = 0.002;

/// Minimal arithmetic the control law needs. Implemented for `f32` and [`Fx`].
pub trait PidNum: Copy + PartialOrd {
    fn zero() -> Self;
    fn one() -> Self;
    /// Round back to raw units (half away from zero).
    fn to_raw(self) -> i32;
    fn add(self, rhs: Self) -> Self;
    fn sub(self, rhs: Self) -> Self;
    fn mul(self, rhs: Self) -> Self;
    fn neg(self) -> Self;
    fn clamped(self, lo: Self, hi: Self) -> Self;
}

impl PidNum for f32 {
    fn zero() -> Self {
        0.0
    }
    fn one() -> Self {
        1.0
    }
    // `round()` lives in std; this is its half-away-from-zero equivalent in core.
    fn to_raw(self) -> i32 {
        if self >= 0.0 {
            (self + 0.5) as i32
        } else {
            (self - 0.5) as i32
        }
    }
    fn add(self, rhs: Self) -> Self {
        self + rhs
    }
    fn sub(self, rhs: Self) -> Self {
        self - rhs
    }
    fn mul(self, rhs: Self) -> Self {
        self * rhs
    }
    fn neg(self) -> Self {
        -self
    }
    fn clamped(self, lo: Self, hi: Self) -> Self {
        if self < lo {
            lo
        } else if self > hi {
            hi
        } else {
            self
        }
    }
}

/// Q16.16 fixed point in an `i32`: 16 integer bits, 16 fractional bits.
/// Products use an `i64` intermediate and saturate instead of wrapping.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Fx(i32);

impl Fx {
    pub fn from_raw(raw: i32) -> Self {
        let scaled = (raw as i64) << 16;
        Self(scaled.clamp(i32::MIN as i64, i32::MAX as i64) as i32)
    }

    pub fn to_raw(self) -> i32 {
        round_shift(self.0, 16)
    }
}

/// Arithmetic shift right with round-half-away-from-zero, symmetric on sign.
fn round_shift(value: i32, bits: u32) -> i32 {
    let half = 1i64 << (bits - 1);
    let v = value as i64;
    let rounded = if v >= 0 {
        (v + half) >> bits
    } else {
        -(((-v) + half) >> bits)
    };
    rounded.clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

/// Q16.16 product with the same symmetric rounding rule as `round_shift`.
fn mul_q16(a: i32, b: i32) -> i32 {
    let v = (a as i64) * (b as i64);
    let rounded = if v >= 0 {
        (v + (1i64 << 15)) >> 16
    } else {
        -(((-v) + (1i64 << 15)) >> 16)
    };
    rounded.clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

impl PidNum for Fx {
    fn zero() -> Self {
        Self(0)
    }
    fn one() -> Self {
        Self(1 << 16)
    }
    fn to_raw(self) -> i32 {
        Fx::to_raw(self)
    }
    fn add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }
    fn sub(self, rhs: Self) -> Self {
        Self(self.0.saturating_sub(rhs.0))
    }
    fn mul(self, rhs: Self) -> Self {
        Self(mul_q16(self.0, rhs.0))
    }
    fn neg(self) -> Self {
        Self(self.0.saturating_neg())
    }
    fn clamped(self, lo: Self, hi: Self) -> Self {
        Self(self.0.clamp(lo.0, hi.0))
    }
}

/// Gains are **per step**, not per second: `ki_dt = Ki * dt`, `kd_dt = Kd / dt`.
/// Keeping them pre-scaled means [`Pid::update`] needs no division.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidConfig<T> {
    /// Output raw counts per raw count of error.
    pub kp: T,
    /// Integral increment per step per raw count of error.
    pub ki_dt: T,
    /// Output raw counts per raw count of measurement change in one step.
    pub kd_dt: T,
    /// Output saturation, in raw counts (positive).
    pub out_limit: T,
    /// Max integral contribution, in output raw counts (non-negative).
    pub i_limit: T,
    /// Derivative low-pass coefficient in `(0, 1]`; `1` disables filtering.
    pub deriv_alpha: T,
}

impl PidConfig<f32> {
    /// Convert continuous gains (per second) at a fixed tick period to the
    /// per-step form the controller consumes.
    pub fn from_si(
        kp: f32,
        ki: f32,
        kd: f32,
        dt: f32,
        out_limit: f32,
        i_limit: f32,
        deriv_alpha: f32,
    ) -> Self {
        Self {
            kp,
            ki_dt: ki * dt,
            kd_dt: kd / dt,
            out_limit,
            i_limit,
            deriv_alpha,
        }
    }
}

/// One axis of the stabilization loop. All state is plain `T`; no heap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid<T: PidNum> {
    cfg: PidConfig<T>,
    integral: T,
    prev_measurement: T,
    prev_setpoint: T,
    filtered_delta: T,
    last_output: T,
    primed: bool,
    saturated: bool,
}

impl<T: PidNum> Pid<T> {
    /// Loud refusal of a nonsense configuration: this is a safety boundary,
    /// not a case for a silent fallback.
    pub fn new(cfg: PidConfig<T>) -> Self {
        assert!(cfg.kp >= T::zero(), "pid: kp must be non-negative");
        assert!(cfg.ki_dt >= T::zero(), "pid: ki_dt must be non-negative");
        assert!(cfg.kd_dt >= T::zero(), "pid: kd_dt must be non-negative");
        assert!(cfg.out_limit > T::zero(), "pid: out_limit must be positive");
        assert!(
            cfg.i_limit >= T::zero(),
            "pid: i_limit must be non-negative"
        );
        assert!(
            cfg.deriv_alpha > T::zero() && cfg.deriv_alpha <= T::one(),
            "pid: deriv_alpha must be in (0, 1]"
        );
        Self {
            cfg,
            integral: T::zero(),
            prev_measurement: T::zero(),
            prev_setpoint: T::zero(),
            filtered_delta: T::zero(),
            last_output: T::zero(),
            primed: false,
            saturated: false,
        }
    }

    /// Drop all accumulated state. The next tick behaves like the first one.
    pub fn reset(&mut self) {
        self.integral = T::zero();
        self.prev_measurement = T::zero();
        self.prev_setpoint = T::zero();
        self.filtered_delta = T::zero();
        self.last_output = T::zero();
        self.primed = false;
        self.saturated = false;
    }

    /// Advance one tick and return the saturated command in raw units.
    pub fn update(&mut self, setpoint: T, measurement: T) -> T {
        let delta = measurement.sub(self.prev_measurement);
        self.prev_measurement = measurement;
        self.filtered_delta = self
            .filtered_delta
            .add(self.cfg.deriv_alpha.mul(delta.sub(self.filtered_delta)));
        let d = self.cfg.kd_dt.mul(self.filtered_delta).neg();

        let error = setpoint.sub(measurement);
        let p = self.cfg.kp.mul(error);
        let limit = self.cfg.out_limit;
        let neg_limit = limit.neg();
        let i_hi = self.cfg.i_limit;
        let i_lo = i_hi.neg();

        // Bumpless setpoint: reload the integrator to hold the last output
        // and skip this tick's integration, so the output does not jump.
        let preloaded = self.primed && setpoint != self.prev_setpoint;
        let i_next = if preloaded {
            last_hold(self.last_output, p, d).clamped(i_lo, i_hi)
        } else {
            self.integral
                .add(self.cfg.ki_dt.mul(error))
                .clamped(i_lo, i_hi)
        };

        let unsat = p.add(i_next).add(d);
        let output = unsat.clamped(neg_limit, limit);
        self.saturated = unsat > limit || unsat < neg_limit;

        // Clamping anti-windup: saturated and pushing further out -> freeze.
        let freeze = self.saturated
            && ((unsat > limit && i_next > self.integral)
                || (unsat < neg_limit && i_next < self.integral));
        if preloaded || !freeze {
            self.integral = i_next;
        }

        self.last_output = output;
        self.prev_setpoint = setpoint;
        self.primed = true;
        output
    }

    pub const fn integral(&self) -> T {
        self.integral
    }
    pub const fn last_output(&self) -> T {
        self.last_output
    }
    pub const fn saturated(&self) -> bool {
        self.saturated
    }
}

fn last_hold<T: PidNum>(last_output: T, p: T, d: T) -> T {
    last_output.sub(p).sub(d)
}

/// Combine two axis outputs into the firmware's actuator command. Final word
/// stays with the existing code veto in `Command::clamped`.
pub fn command_from_raw(pitch: i32, yaw: i32) -> Command {
    let to_i16 = |v: i32| -> i16 {
        if v > i16::MAX as i32 {
            i16::MAX
        } else if v < i16::MIN as i32 {
            i16::MIN
        } else {
            v as i16
        }
    };
    Command {
        pitch: to_i16(pitch),
        yaw: to_i16(yaw),
    }
    .clamped()
}
