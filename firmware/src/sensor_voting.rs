// SPDX-License-Identifier: GPL-3.0-or-later
//! Redundant-sensor voting for IMU and barometer readings.
//!
//! Pure logic: no pins, buses, clocks, allocation or I/O. Each tick the caller
//! supplies up to three readings in fixed logical slots and gets back one
//! selected value plus a confidence and the reasons it was degraded. A slot
//! must always mean the same physical sensor; slot 2 is simply absent in a
//! dual-sensor installation.
//!
//! Selection is the median for three readings (the medoid under
//! [`Sample::distance`], which is the ordinary median on a scalar) and the
//! midpoint for two. A sensor quarantined as stuck is excluded before
//! selection. A sensor is only flagged stuck when its reading has been
//! bit-identical for [`Config::stuck_samples`] ticks *and* another sensor has
//! moved beyond tolerance, so a stationary vehicle cannot quarantine its
//! sensors. Disagreement is reported from the remaining sensors; a dual vote
//! cannot arbitrate which one is wrong and never names an outlier.

/// Maximum number of readings considered per tick.
pub const SENSOR_COUNT: usize = 3;

/// Per-slot confidence before any disagreement penalty.
const CONF_SINGLE: f32 = 0.35;
const CONF_DUAL: f32 = 0.75;
const CONF_TRIPLE: f32 = 1.0;
/// Confidence lost per unit of disagreement-threshold overshoot.
const DISAGREEMENT_PENALTY: f32 = 0.8;

/// A reading that can be voted on: scalars (barometer) or vectors (IMU axes).
pub trait Sample: Copy {
    fn is_finite(self) -> bool;
    /// Non-negative scalar separation, zero only for identical readings.
    fn distance(self, other: Self) -> f32;
    /// Deterministic value halfway between two readings.
    fn midpoint(self, other: Self) -> Self;
    /// Scalar magnitude used by the relative part of [`Tolerance`].
    fn scale(self) -> f32;
}

fn finite(x: f32) -> bool {
    x.is_finite()
}

fn abs_diff(a: f32, b: f32) -> f32 {
    if a > b {
        a - b
    } else {
        b - a
    }
}

fn abs(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

fn clamp01(x: f32) -> f32 {
    x.clamp(0.0, 1.0)
}

impl Sample for f32 {
    fn is_finite(self) -> bool {
        finite(self)
    }

    fn distance(self, other: Self) -> f32 {
        abs_diff(self, other)
    }

    fn midpoint(self, other: Self) -> Self {
        (self + other) * 0.5
    }

    fn scale(self) -> f32 {
        abs(self)
    }
}

impl<const N: usize> Sample for [f32; N] {
    fn is_finite(self) -> bool {
        let mut ok = true;
        for &value in self.iter() {
            ok &= finite(value);
        }
        ok
    }

    fn distance(self, other: Self) -> f32 {
        let mut worst = 0.0;
        for i in 0..N {
            let d = abs_diff(self[i], other[i]);
            if d > worst {
                worst = d;
            }
        }
        worst
    }

    fn midpoint(self, other: Self) -> Self {
        let mut out = [0.0; N];
        for i in 0..N {
            out[i] = (self[i] + other[i]) * 0.5;
        }
        out
    }

    fn scale(self) -> f32 {
        let mut worst = 0.0;
        for &value in self.iter() {
            let a = abs(value);
            if a > worst {
                worst = a;
            }
        }
        worst
    }
}

/// One IMU frame. A single vote covers all six axes: the worst axis decides
/// disagreement, so a gyro or accelerometer fault cannot be hidden by the
/// other axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
}

impl Sample for ImuSample {
    fn is_finite(self) -> bool {
        self.gyro.is_finite() && self.accel.is_finite()
    }

    fn distance(self, other: Self) -> f32 {
        let g = self.gyro.distance(other.gyro);
        let a = self.accel.distance(other.accel);
        if g > a {
            g
        } else {
            a
        }
    }

    fn midpoint(self, other: Self) -> Self {
        Self {
            gyro: self.gyro.midpoint(other.gyro),
            accel: self.accel.midpoint(other.accel),
        }
    }

    fn scale(self) -> f32 {
        let g = self.gyro.scale();
        let a = self.accel.scale();
        if g > a {
            g
        } else {
            a
        }
    }
}

/// Maximum acceptable separation: `absolute + relative * larger_magnitude`.
/// `absolute` must be positive at the caller's noise floor; `relative` must be
/// non-negative.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerance {
    pub absolute: f32,
    pub relative: f32,
}

impl Tolerance {
    pub const fn new(absolute: f32, relative: f32) -> Self {
        Self { absolute, relative }
    }

    fn limit(self, a: f32, b: f32) -> f32 {
        let scale = if a > b { a } else { b };
        self.absolute + self.relative * scale
    }

    fn exceeds<S: Sample>(self, a: S, b: S) -> bool {
        a.distance(b) > self.limit(a.scale(), b.scale())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Identical readings in a row before a sensor may be quarantined.
    pub stuck_samples: u8,
}

impl Config {
    pub const DEFAULT: Self = Self { stuck_samples: 8 };

    pub const fn new(stuck_samples: u8) -> Self {
        Self { stuck_samples }
    }
}

/// Result of one vote. `confidence` is in 0.0..=1.0; [`Vote::confidence_u8`]
/// maps it onto the protocol's byte-scale confidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vote<S> {
    pub value: S,
    pub confidence: f32,
    pub disagreement: bool,
    /// Bit `i` marks slot `i` quarantined as stuck.
    pub stuck_mask: u8,
    /// Bit `i` marks slot `i` as a contributor to `value`.
    pub used_mask: u8,
    /// Slot furthest from a three-way consensus, if one is identifiable.
    pub outlier: Option<u8>,
}

impl<S> Vote<S> {
    pub fn confidence_u8(&self) -> u8 {
        (clamp01(self.confidence) * 255.0 + 0.5) as u8
    }

    pub fn used_count(&self) -> u32 {
        self.used_mask.count_ones()
    }
}

/// Stateful voter. One instance per measured quantity (barometric height, each
/// gyro/accel vector), reused every tick so stuck detection sees history.
pub struct VotingChannel<S: Sample> {
    cfg: Config,
    tol: Tolerance,
    last: [Option<S>; SENSOR_COUNT],
    frozen: [u8; SENSOR_COUNT],
    stuck: [bool; SENSOR_COUNT],
}

impl<S: Sample> VotingChannel<S> {
    pub const fn new(cfg: Config, tol: Tolerance) -> Self {
        Self {
            cfg,
            tol,
            last: [None; SENSOR_COUNT],
            frozen: [0; SENSOR_COUNT],
            stuck: [false; SENSOR_COUNT],
        }
    }

    /// Forget all history, e.g. after reconfiguration or a sensor swap.
    pub fn reset(&mut self) {
        self.last = [None; SENSOR_COUNT];
        self.frozen = [0; SENSOR_COUNT];
        self.stuck = [false; SENSOR_COUNT];
    }

    /// Stuck slots before this tick's readings are applied.
    pub const fn stuck_mask(&self) -> u8 {
        pack_bools(&self.stuck)
    }

    pub fn vote_dual(&mut self, a: Option<S>, b: Option<S>) -> Option<Vote<S>> {
        self.vote([a, b, None])
    }

    pub fn vote_triple(&mut self, a: Option<S>, b: Option<S>, c: Option<S>) -> Option<Vote<S>> {
        self.vote([a, b, c])
    }

    /// `None` means no usable reading at all: there is no safe value to select.
    pub fn vote(&mut self, readings: [Option<S>; SENSOR_COUNT]) -> Option<Vote<S>> {
        let mut valid = [false; SENSOR_COUNT];
        for i in 0..SENSOR_COUNT {
            valid[i] = match readings[i] {
                Some(s) => s.is_finite(),
                None => false,
            };
        }

        // Peers are taken from the pre-tick state so sensors cannot quarantine
        // each other within the same tick.
        let was_stuck = self.stuck;
        for i in 0..SENSOR_COUNT {
            if !valid[i] {
                self.last[i] = None;
                self.frozen[i] = 0;
                self.stuck[i] = false;
                continue;
            }
            let sample = match readings[i] {
                Some(s) => s,
                None => unreachable!(),
            };
            let repeated = match self.last[i] {
                Some(prev) => sample.distance(prev) == 0.0,
                None => false,
            };
            if repeated {
                self.frozen[i] = self.frozen[i].saturating_add(1);
            } else {
                self.frozen[i] = 0;
                self.stuck[i] = false;
            }
            self.last[i] = Some(sample);

            if !self.stuck[i] && self.frozen[i] >= self.cfg.stuck_samples {
                for j in 0..SENSOR_COUNT {
                    if j != i && valid[j] && !was_stuck[j] {
                        let peer = match readings[j] {
                            Some(s) => s,
                            None => unreachable!(),
                        };
                        if self.tol.exceeds(sample, peer) {
                            self.stuck[i] = true;
                        }
                        break;
                    }
                }
            }
        }

        let mut slots = [0usize; SENSOR_COUNT];
        let mut samples = [None; SENSOR_COUNT];
        let mut n = 0;
        for i in 0..SENSOR_COUNT {
            if valid[i] && !self.stuck[i] {
                slots[n] = i;
                samples[n] = readings[i];
                n += 1;
            }
        }
        if n == 0 {
            return None;
        }

        let mut worst_ratio = 0.0f32;
        let mut disagreement = false;
        let mut closest = (0usize, 1usize);
        let mut closest_dist = f32::INFINITY;
        for a in 0..n {
            for b in (a + 1)..n {
                let sa = samples[a].unwrap();
                let sb = samples[b].unwrap();
                let d = sa.distance(sb);
                let limit = self.tol.limit(sa.scale(), sb.scale());
                let ratio = if limit > 0.0 { d / limit } else { 0.0 };
                if ratio > worst_ratio {
                    worst_ratio = ratio;
                }
                if d > limit {
                    disagreement = true;
                }
                if d < closest_dist {
                    closest_dist = d;
                    closest = (a, b);
                }
            }
        }

        let value = match n {
            1 => samples[0].unwrap(),
            2 => samples[0].unwrap().midpoint(samples[1].unwrap()),
            _ => {
                let mut best = 0usize;
                let mut best_sum = f32::INFINITY;
                for a in 0..n {
                    let mut sum = 0.0;
                    for b in 0..n {
                        if a != b {
                            sum += samples[a].unwrap().distance(samples[b].unwrap());
                        }
                    }
                    if sum < best_sum {
                        best_sum = sum;
                        best = a;
                    }
                }
                samples[best].unwrap()
            }
        };

        let mut outlier = None;
        if n == SENSOR_COUNT && disagreement {
            let third = SENSOR_COUNT - slots[closest.0] - slots[closest.1];
            if self.tol.exceeds(readings[third].unwrap(), value) {
                outlier = Some(third as u8);
            }
        }

        let base = match n {
            1 => CONF_SINGLE,
            2 => CONF_DUAL,
            _ => CONF_TRIPLE,
        };
        // Agreement within tolerance earns the full base confidence; severity
        // is measured as overshoot beyond the threshold, not as ordinary noise.
        let overshoot = if worst_ratio > 1.0 {
            clamp01(worst_ratio - 1.0)
        } else {
            0.0
        };
        Some(Vote {
            value,
            confidence: base * (1.0 - DISAGREEMENT_PENALTY * overshoot),
            disagreement,
            stuck_mask: pack_bools(&self.stuck),
            used_mask: pack_slots(&slots, n),
            outlier,
        })
    }
}

const fn pack_bools(flags: &[bool; SENSOR_COUNT]) -> u8 {
    let mut mask = 0u8;
    let mut i = 0;
    while i < SENSOR_COUNT {
        if flags[i] {
            mask |= 1 << i;
        }
        i += 1;
    }
    mask
}

fn pack_slots(slots: &[usize; SENSOR_COUNT], n: usize) -> u8 {
    let mut mask = 0u8;
    for &slot in slots.iter().take(n) {
        mask |= 1 << slot;
    }
    mask
}
