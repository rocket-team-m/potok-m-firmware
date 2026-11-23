// SPDX-License-Identifier: GPL-3.0-or-later
//! Independent safety layer between guidance and the actuators.
//!
//! This is a second, self-contained gate. It does not read `LinkState`, does
//! not trust the phase or flags carried in an AI reply and does not call the
//! board: the caller feeds it the STM32-owned phase code, the sensor flags and
//! the candidate command, and it independently decides whether that command
//! may move an actuator. Every refusal yields [`Interlocks::FAILSAFE`].
//!
//! Two kinds of refusal are kept apart. Unknown phase, an unhealthy AI, a
//! board FAULT and an implausible command are *latching*: once seen they keep
//! producing the failsafe until [`Interlocks::reset`] is called by the
//! firmware. BOOST (`Phase::BURN`) and `MOTOR_ARMED` are *transient*: they
//! zero the output only while they hold, so burnout re-enables guidance
//! without a reset. A plausible command that simply moves too fast is slewed,
//! never jumped to.
//!
//! [`Interlocks::evaluate_shield`] adds the runtime Mach gate on top of these
//! finite-state checks: it takes the measured pitot dynamic pressure and the
//! caller's Mach estimate and zeroes the command with a reason code when the
//! estimate is non-finite, the pitot is unusable, or Mach exceeds
//! [`Limits::mach_limit`]. It is a post-posed shield outside the learner — a
//! refusal is never traded for a smaller command.

use potok_protocol::{flags, Phase};

use crate::Command;

/// The phase code is decoded here, independently of the FSM: a code outside
/// the generated `Phase` enum is unknown and may never move an actuator.
pub type PhaseCode = u8;

/// Phase class packed into the low two bits of the evaluated gate. An unknown
/// code fails closed; BURN is the transient boost refusal.
const PHASE_UNKNOWN: u8 = 0;
const PHASE_BOOST: u8 = 1;
const PHASE_ALLOWED: u8 = 2;
const PHASE_MASK: u8 = 0b11;

const MOTOR_BIT: u8 = 1 << 2;
const UNHEALTHY_BIT: u8 = 1 << 3;
const FAULT_BIT: u8 = 1 << 4;

/// Direct lookup by wire phase byte: one load replaces a `Phase::try_from` match
/// plus the separate BURN comparison. Every undefined code stays `PHASE_UNKNOWN`.
const fn build_phase_gate() -> [u8; 256] {
    let mut table = [PHASE_UNKNOWN; 256];
    let mut code = Phase::BURN as usize + 1;
    while code <= Phase::LANDED as usize {
        table[code] = PHASE_ALLOWED;
        code += 1;
    }
    table[Phase::BURN as usize] = PHASE_BOOST;
    table
}

const PHASE_GATE: [u8; 256] = build_phase_gate();

/// Fold phase, motor sense, AI health and the FAULT bit into one status word.
/// `flags` is loaded once here instead of once per gate.
#[inline(always)]
fn evaluated_gate(code: u8, flags: u8, ai_healthy: bool) -> u8 {
    let mut gate = PHASE_GATE[code as usize];
    if flags & flags::MOTOR_ARMED != 0 {
        gate |= MOTOR_BIT;
    }
    if !ai_healthy {
        gate |= UNHEALTHY_BIT;
    }
    if flags & flags::FAULT != 0 {
        gate |= FAULT_BIT;
    }
    gate
}

/// Why a command was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// Phase code does not decode to a known `Phase`. Latching.
    UnknownPhase(u8),
    /// Boost: motor still burning (`Phase::BURN`). Transient.
    Boost,
    /// Independent sensor bit `MOTOR_ARMED`. Transient.
    MotorArmed,
    /// AI declared itself unhealthy. Latching.
    AiUnhealthy,
    /// Board/sensor FAULT flag. Latching.
    Fault,
    /// Command outside the plausible range. Latching.
    Implausible,
    /// Mach estimate is NaN or infinite. Latching: a fault, not a condition.
    NonFiniteMach,
    /// Pitot dynamic pressure is non-positive, so there is no airspeed
    /// evidence to gate on. Transient.
    PitotInvalid,
    /// Estimated Mach exceeds the configured limit. Transient.
    MachLimit,
}

impl Refusal {
    /// True for the faults that survive their cause and need an explicit reset.
    pub const fn latches(self) -> bool {
        matches!(
            self,
            Self::UnknownPhase(_)
                | Self::AiUnhealthy
                | Self::Fault
                | Self::Implausible
                | Self::NonFiniteMach
        )
    }
}

/// Tunable gates. Defaults are the flight recipe: raw units are 0.001 rad,
/// so 100 raw is the model's 0.1 rad bound.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// Absolute per-axis bound on a plausible command, in raw units.
    pub max_command: i16,
    /// Largest per-axis change between two accepted commands, in raw units.
    pub max_delta: i16,
    /// Shortest time between two command changes, in microseconds. Must be
    /// well below 2^31 so the wrapping subtraction in `evaluate` is sound.
    pub min_interval_us: u32,
    /// Hard Mach ceiling for the runtime shield. An estimate above this zeroes
    /// the command; there is no soft tradeoff band.
    pub mach_limit: f32,
}

impl Limits {
    pub const DEFAULT: Self = Self {
        max_command: 100,
        max_delta: 10,
        min_interval_us: 1_000,
        mach_limit: 0.6,
    };
}

impl Default for Limits {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Everything the gate needs for one control tick. `now_us` is the board's
/// monotonic microsecond clock; it is allowed to wrap.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Input {
    pub phase_code: PhaseCode,
    pub flags: u8,
    pub ai_healthy: bool,
    pub requested: Command,
    pub now_us: u32,
}

/// Result of one interlock evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Decision {
    /// What may reach the actuator: `FAILSAFE` whenever `permitted` is false.
    pub command: Command,
    /// Whether a (possibly rate-limited) command may be applied.
    pub permitted: bool,
    /// Reason for a refusal, or the still-active latch reason.
    pub refusal: Option<Refusal>,
    /// True when a rate limit changed or held the requested command.
    pub limited: bool,
}

/// Observability counters. Never cleared by `reset`, so a latch episode stays
/// visible to telemetry after the firmware recovers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub accepted: u32,
    pub slewed: u32,
    pub throttled: u32,
    pub blocked: u32,
    pub trips: u32,
    pub resets: u32,
    /// Ticks zeroed by the Mach limit (transient refusals, not trips).
    pub mach_blocked: u32,
}

/// Stateless-in-the-large interlock: only the latch, the last accepted command
/// and the rate-limit clock are retained between ticks.
#[derive(Clone, Copy, Debug)]
pub struct Interlocks {
    limits: Limits,
    latch: Option<Refusal>,
    last: Command,
    last_change_us: Option<u32>,
    pub counters: Counters,
}

impl Interlocks {
    /// The only command a blocked, latched or stale tick may produce.
    pub const FAILSAFE: Command = Command::ZERO;

    /// Loud rejection of an unusable configuration; no silent fallback.
    pub const fn new(limits: Limits) -> Self {
        assert!(limits.max_command > 0, "max_command must be positive");
        assert!(limits.max_delta > 0, "max_delta must be positive");
        assert!(
            limits.min_interval_us < 1 << 31,
            "min_interval_us must fit the wrapping clock comparison"
        );
        assert!(
            limits.mach_limit > 0.0 && limits.mach_limit < f32::INFINITY,
            "mach_limit must be finite and positive"
        );
        Self {
            limits,
            latch: None,
            last: Self::FAILSAFE,
            last_change_us: None,
            counters: Counters {
                accepted: 0,
                slewed: 0,
                throttled: 0,
                blocked: 0,
                trips: 0,
                resets: 0,
                mach_blocked: 0,
            },
        }
    }

    pub const fn limits(&self) -> Limits {
        self.limits
    }

    pub const fn latched(&self) -> Option<Refusal> {
        self.latch
    }

    pub const fn last_command(&self) -> Command {
        self.last
    }

    /// Decide one tick. Order is deliberate: an active latch outranks any
    /// new input, phase/health faults outrank the command, and rate limiting
    /// only ever applies to a command that passed all of the above.
    pub fn evaluate(&mut self, input: Input) -> Decision {
        if let Some(decision) = self.preflight(input) {
            return decision;
        }
        self.accept(input)
    }

    /// Runtime shield: the interlock checks plus the Mach/pitot gate. `pitot_dp`
    /// is the measured pitot dynamic pressure in Pa; `mach_est` is the Mach
    /// estimate the caller derives from it (and static pressure) before this
    /// call. The gate uses comparisons and finiteness checks only — no `libm`,
    /// no host sync, deterministic on `no_std`. Every refusal is a zero command
    /// with a reason code; no measurement is ever traded for a smaller command.
    ///
    /// This is a *command* gate: it stops AI output above the limit, it does
    /// not bound the vehicle's Mach. If thrust drives M past the limit, no
    /// controller with these actuators can bring it back (design finding in
    /// `docs/RUNTIME_SHIELD.md`).
    pub fn evaluate_shield(&mut self, input: Input, pitot_dp: i16, mach_est: f32) -> Decision {
        if let Some(decision) = self.preflight(input) {
            return decision;
        }
        if !mach_est.is_finite() {
            return self.trip(Refusal::NonFiniteMach);
        }
        if pitot_dp <= 0 {
            return self.blocked(Refusal::PitotInvalid);
        }
        if mach_est > self.limits.mach_limit {
            self.counters.mach_blocked += 1;
            return self.blocked(Refusal::MachLimit);
        }
        self.accept(input)
    }

    /// Finite-state gates shared by `evaluate` and `evaluate_shield`. None of
    /// them may be skipped by the shield. All four inputs are evaluated into a
    /// single status word; one match resolves the refusal, in the same priority
    /// order as before (phase, motor, health, fault).
    fn preflight(&mut self, input: Input) -> Option<Decision> {
        if let Some(reason) = self.latch {
            return Some(self.blocked(reason));
        }
        let gate = evaluated_gate(input.phase_code, input.flags, input.ai_healthy);
        match gate {
            g if g & PHASE_MASK == PHASE_UNKNOWN => {
                Some(self.trip(Refusal::UnknownPhase(input.phase_code)))
            }
            g if g & PHASE_MASK == PHASE_BOOST => Some(self.blocked(Refusal::Boost)),
            g if g & MOTOR_BIT != 0 => Some(self.blocked(Refusal::MotorArmed)),
            g if g & UNHEALTHY_BIT != 0 => Some(self.trip(Refusal::AiUnhealthy)),
            g if g & FAULT_BIT != 0 => Some(self.trip(Refusal::Fault)),
            _ => None,
        }
    }

    /// Command plausibility and rate limiting, after every safety gate passed.
    fn accept(&mut self, input: Input) -> Decision {
        if !self.plausible(input.requested) {
            return self.trip(Refusal::Implausible);
        }
        let next = self.rate_limit(input.requested, input.now_us);
        self.counters.accepted += 1;
        Decision {
            command: next.command,
            permitted: true,
            refusal: None,
            limited: next.limited,
        }
    }

    /// The only way out of a latch. Clears the rate-limit history too, so the
    /// first command after recovery is slewed from zero rather than from the
    /// last command before the fault.
    pub fn reset(&mut self) {
        self.latch = None;
        self.last = Self::FAILSAFE;
        self.last_change_us = None;
        self.counters.resets += 1;
    }

    fn plausible(&self, cmd: Command) -> bool {
        let bound = i32::from(self.limits.max_command);
        // Widen before abs: `i16::MIN` has no i16 negation.
        let pitch = i32::from(cmd.pitch).abs();
        let yaw = i32::from(cmd.yaw).abs();
        pitch <= bound && yaw <= bound
    }

    /// Slew per axis, plus a minimum interval between changes. Holding the
    /// previous command on a throttle is safe: it was vetted on an earlier
    /// tick and never exceeds the slew bound from its own predecessor.
    fn rate_limit(&mut self, requested: Command, now_us: u32) -> Decision {
        let mut limited = false;
        let mut out = Command::ZERO;
        let axes = [
            (self.last.pitch, requested.pitch),
            (self.last.yaw, requested.yaw),
        ];
        let mut slewed = [0i16; 2];
        for (axis, (from, to)) in axes.iter().enumerate() {
            let delta = i32::from(self.limits.max_delta);
            let change = i32::from(*to) - i32::from(*from);
            let value = if change > delta {
                limited = true;
                i32::from(*from) + delta
            } else if change < -delta {
                limited = true;
                i32::from(*from) - delta
            } else {
                i32::from(*to)
            };
            let bound = i32::from(self.limits.max_command);
            slewed[axis] = value.clamp(-bound, bound) as i16;
        }
        out.pitch = slewed[0];
        out.yaw = slewed[1];

        if out == self.last {
            return Decision {
                command: out,
                permitted: true,
                refusal: None,
                limited: false,
            };
        }
        if let Some(last) = self.last_change_us {
            if now_us.wrapping_sub(last) < self.limits.min_interval_us {
                self.counters.throttled += 1;
                return Decision {
                    command: self.last,
                    permitted: true,
                    refusal: None,
                    limited: true,
                };
            }
        }
        if limited {
            self.counters.slewed += 1;
        }
        self.last = out;
        self.last_change_us = Some(now_us);
        Decision {
            command: out,
            permitted: true,
            refusal: None,
            limited,
        }
    }

    fn blocked(&mut self, reason: Refusal) -> Decision {
        self.counters.blocked += 1;
        Decision {
            command: Self::FAILSAFE,
            permitted: false,
            refusal: Some(reason),
            limited: false,
        }
    }

    fn trip(&mut self, reason: Refusal) -> Decision {
        debug_assert!(reason.latches(), "only latching reasons may trip");
        if self.latch.is_none() {
            self.counters.trips += 1;
        }
        self.latch = Some(reason);
        self.blocked(reason)
    }
}

impl Default for Interlocks {
    fn default() -> Self {
        Self::new(Limits::DEFAULT)
    }
}
