// SPDX-License-Identifier: GPL-3.0-or-later
//! Redundant-sensor voting for IMU and barometer readings.
//!
//! Pure logic: no pins, buses, clocks, allocation or I/O. Each tick the caller
//! supplies up to three readings in fixed logical slots and gets back one
//! selected value plus a confidence and the reasons it was degraded. A slot
//! must always mean the same physical sensor; slot 2 is simply absent in a
//! dual-sensor installation.
//!
//! Selection is the median for three readings (the medoid under the
//! dimensionless separation defined by [`Sample::separation`]) and the
//! midpoint for two. A sensor quarantined as stuck is excluded before
//! selection. A sensor is only flagged stuck when its reading has been
//! bit-identical for [`Config::stuck_samples`] ticks *and* another sensor has
//! moved beyond tolerance, so a stationary vehicle cannot quarantine its
//! sensors. Disagreement is reported from the remaining sensors; a dual vote
//! cannot arbitrate which one is wrong and never names an outlier.
//!
//! **Fault-detection method.** This is analytic (hardware) redundancy: with
//! two or three sensors measuring the same quantity, the residual between
//! readings is compared with a threshold to generate a fault indicator; the
//! third channel, when present, isolates the faulty one by consensus. This is
//! the standard residual/threshold/isolation scheme of R. Isermann,
//! *Fault-Diagnosis Systems*, Springer (2006), DOI 10.1007/3-540-30368-5.
//! No flight or hardware validation is claimed here; see `docs/MATH_VOTING.md`
//! for the exact scope and for which numbers remain HEURISTIC.
//!
//! **Units.** A residual is `|Δ| / (absolute + relative·scale)`, dimensionless,
//! and a fault is indicated when it exceeds 1. Gyro differences are divided
//! by a limit in rad/s, accelerometer differences by a limit in m/s², each
//! with its own relative term. Unlike physical channels are never compared by
//! raw magnitude.

/// Maximum number of readings considered per tick.
pub const SENSOR_COUNT: usize = 3;

/// HEURISTIC (F3): per-slot confidence before any disagreement penalty.
/// No paper, calibration or flight datum supports these three numbers; they
/// are kept only for protocol compatibility and must be re-derived before a
/// confidence value is used as a safety argument. See `docs/MATH_VOTING.md`.
const HEURISTIC_CONF_SINGLE: f32 = 0.35;
const HEURISTIC_CONF_DUAL: f32 = 0.75;
const HEURISTIC_CONF_TRIPLE: f32 = 1.0;
/// HEURISTIC (F3): confidence lost per unit of threshold overshoot. Same
/// caveat as the three base values: this is a shape choice, not a measurement.
const HEURISTIC_DISAGREEMENT_PENALTY: f32 = 0.8;

/// A reading that can be voted on: scalars (barometer) or vectors (IMU axes).
pub trait Sample: Copy {
    fn is_finite(self) -> bool;
    /// Dimensionless residual against `other` under `tol`; `> 1.0` means the
    /// two readings disagree by more than the configured fault threshold.
    /// Zero only for identical readings.
    fn separation(self, other: Self, tol: Tolerance) -> f32;
    /// Deterministic value halfway between two readings.
    fn midpoint(self, other: Self) -> Self;
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

/// `distance / limit`, with the legacy convention that a non-positive limit
/// yields zero. Callers must configure positive absolute thresholds.
fn residual(distance: f32, limit: f32) -> f32 {
    if limit > 0.0 {
        distance / limit
    } else {
        0.0
    }
}

impl Sample for f32 {
    fn is_finite(self) -> bool {
        finite(self)
    }

    fn separation(self, other: Self, tol: Tolerance) -> f32 {
        let scale = if self.abs() > other.abs() {
            self.abs()
        } else {
            other.abs()
        };
        residual(abs_diff(self, other), tol.absolute + tol.relative * scale)
    }

    fn midpoint(self, other: Self) -> Self {
        (self + other) * 0.5
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

    fn separation(self, other: Self, tol: Tolerance) -> f32 {
        let mut worst = 0.0;
        for i in 0..N {
            let scale = if abs(self[i]) > abs(other[i]) {
                abs(self[i])
            } else {
                abs(other[i])
            };
            let r = residual(
                abs_diff(self[i], other[i]),
                tol.absolute + tol.relative * scale,
            );
            if r > worst {
                worst = r;
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
}

/// One IMU frame. A single vote covers all six axes: the worst normalized
/// residual decides disagreement, so a gyro or accelerometer fault cannot be
/// hidden by the other axes. Gyro axes are in rad/s, accel axes in m/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
}

impl Sample for ImuSample {
    fn is_finite(self) -> bool {
        self.gyro.is_finite() && self.accel.is_finite()
    }

    fn separation(self, other: Self, tol: Tolerance) -> f32 {
        let mut worst = 0.0;
        for i in 0..3 {
            let g_scale = if abs(self.gyro[i]) > abs(other.gyro[i]) {
                abs(self.gyro[i])
            } else {
                abs(other.gyro[i])
            };
            let g = residual(
                abs_diff(self.gyro[i], other.gyro[i]),
                tol.gyro_absolute_rad_s + tol.gyro_relative * g_scale,
            );
            if g > worst {
                worst = g;
            }

            let a_scale = if abs(self.accel[i]) > abs(other.accel[i]) {
                abs(self.accel[i])
            } else {
                abs(other.accel[i])
            };
            let a = residual(
                abs_diff(self.accel[i], other.accel[i]),
                tol.accel_absolute_m_s2 + tol.accel_relative * a_scale,
            );
            if a > worst {
                worst = a;
            }
        }
        worst
    }

    fn midpoint(self, other: Self) -> Self {
        Self {
            gyro: self.gyro.midpoint(other.gyro),
            accel: self.accel.midpoint(other.accel),
        }
    }
}

/// Fault thresholds for residual `|Δ| / (absolute + relative·scale) > 1`.
/// The absolute part is in the measured quantity's unit; the relative part is
/// dimensionless. Field names carry units where a physical channel is
/// involved: rad/s for gyro, m/s² for accelerometer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerance {
    /// Scalar/vector absolute threshold, in the sample's own unit (baro: m).
    pub absolute: f32,
    /// Scalar/vector relative threshold, dimensionless.
    pub relative: f32,
    /// Gyroscope absolute threshold in rad/s.
    pub gyro_absolute_rad_s: f32,
    /// Gyroscope relative threshold, dimensionless.
    pub gyro_relative: f32,
    /// Accelerometer absolute threshold in m/s².
    pub accel_absolute_m_s2: f32,
    /// Accelerometer relative threshold, dimensionless.
    pub accel_relative: f32,
}

impl Tolerance {
    /// Generic constructor for scalar or vector channels. For an
    /// [`ImuSample`] it applies the same numeric threshold to both physical
    /// channels, each in its *own* unit; this keeps older call sites valid but
    /// the values are HEURISTIC. Prefer [`Tolerance::imu`] when two independent
    /// channel thresholds are available.
    pub const fn new(absolute: f32, relative: f32) -> Self {
        let mut tol = Self::imu(absolute, relative, absolute, relative);
        tol.absolute = absolute;
        tol.relative = relative;
        tol
    }

    /// Per-physical-channel constructor. Use this for [`ImuSample`] channels:
    /// the gyro pair is in rad/s, the accel pair in m/s², and neither channel's
    /// relative term is allowed to scale from the other channel's magnitude.
    pub const fn imu(
        gyro_absolute_rad_s: f32,
        gyro_relative: f32,
        accel_absolute_m_s2: f32,
        accel_relative: f32,
    ) -> Self {
        Self {
            absolute: 0.0,
            relative: 0.0,
            gyro_absolute_rad_s,
            gyro_relative,
            accel_absolute_m_s2,
            accel_relative,
        }
    }
}

fn exceeds<S: Sample>(tol: Tolerance, a: S, b: S) -> bool {
    a.separation(b, tol) > 1.0
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Identical readings in a row before a sensor may be quarantined.
    /// HEURISTIC (F4): no noise-floor or flight datum fixes this count.
    pub stuck_samples: u8,
}

impl Config {
    /// HEURISTIC deployment values; see `docs/MATH_VOTING.md`.
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

/// Stateful voter. One instance per measured quantity (barometric height, the
/// IMU frame), reused every tick so stuck detection sees history.
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
                Some(prev) => sample.separation(prev, self.tol) == 0.0,
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
                        if exceeds(self.tol, sample, peer) {
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
        let mut closest_ratio = f32::INFINITY;
        for a in 0..n {
            for b in (a + 1)..n {
                let sa = samples[a].unwrap();
                let sb = samples[b].unwrap();
                let r = sa.separation(sb, self.tol);
                if r > worst_ratio {
                    worst_ratio = r;
                }
                if r > 1.0 {
                    disagreement = true;
                }
                if r < closest_ratio {
                    closest_ratio = r;
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
                            sum += samples[a]
                                .unwrap()
                                .separation(samples[b].unwrap(), self.tol);
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
            if exceeds(self.tol, readings[third].unwrap(), value) {
                outlier = Some(third as u8);
            }
        }

        let base = match n {
            1 => HEURISTIC_CONF_SINGLE,
            2 => HEURISTIC_CONF_DUAL,
            _ => HEURISTIC_CONF_TRIPLE,
        };
        // Agreement within tolerance earns the full HEURISTIC base confidence;
        // severity is overshoot beyond the threshold, not ordinary noise.
        let overshoot = if worst_ratio > 1.0 {
            clamp01(worst_ratio - 1.0)
        } else {
            0.0
        };
        Some(Vote {
            value,
            confidence: base * (1.0 - HEURISTIC_DISAGREEMENT_PENALTY * overshoot),
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
