// SPDX-License-Identifier: GPL-3.0-or-later

//! Strict, forward-only flight phase FSM owned by the STM32. The AI never
//! selects or changes a phase.
//!
//! Every accepted event must be one of the explicit edges in [`TRANSITIONS`].
//! Anything else is refused: phase, stamps and log stay put, only the rejection
//! counter moves. Every committed edge carries the caller's timestamp and may
//! not predate the current phase.

use potok_protocol::{flags, Phase};

/// Events that can drive the machine. Each one names exactly one legal edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseEvent {
    /// Motor ignited: an in-phase marker, still BURN (no AI allowed).
    LiftOff,
    /// Motor burned out: BURN -> COAST.
    Burnout,
    /// COAST -> APOGEE.
    Apogee,
    /// APOGEE -> DESCENT.
    Descent,
    /// DESCENT -> LANDED (terminal).
    Touchdown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Rule {
    from: Phase,
    event: PhaseEvent,
    to: Phase,
}

/// The complete set of legal edges. Forward-only is structural: each `to`
/// phase is at or after its `from` phase in flight order. There is no path
/// back into BURN and no edge out of LANDED.
const TRANSITIONS: [Rule; 5] = [
    Rule {
        from: Phase::BURN,
        event: PhaseEvent::LiftOff,
        to: Phase::BURN,
    },
    Rule {
        from: Phase::BURN,
        event: PhaseEvent::Burnout,
        to: Phase::COAST,
    },
    Rule {
        from: Phase::COAST,
        event: PhaseEvent::Apogee,
        to: Phase::APOGEE,
    },
    Rule {
        from: Phase::APOGEE,
        event: PhaseEvent::Descent,
        to: Phase::DESCENT,
    },
    Rule {
        from: Phase::DESCENT,
        event: PhaseEvent::Touchdown,
        to: Phase::LANDED,
    },
];

/// One entry per wire phase code (BURN=0 ..= LANDED=4), so a phase-entry stamp
/// can be stored directly at its own index without a map or allocation.
const SLOTS: usize = Phase::LANDED as usize + 1;

/// A committed phase change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    pub from: Phase,
    pub to: Phase,
    pub event: PhaseEvent,
    /// Timestamp in microseconds at which the edge was committed.
    pub at_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseError {
    /// There is no edge from this phase for this event.
    IllegalTransition { from: Phase, event: PhaseEvent },
    /// The stamp predates the entry into the current phase.
    TimestampWentBackwards { now_us: u64, entered_at_us: u64 },
}

impl core::fmt::Display for PhaseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Self::IllegalTransition { from, event } => {
                write!(f, "phase event {event:?} is illegal from {from:?}")
            }
            Self::TimestampWentBackwards {
                now_us,
                entered_at_us,
            } => write!(
                f,
                "timestamp {now_us} us precedes phase entry at {entered_at_us} us"
            ),
        }
    }
}

/// BURN is the safe initial state: until sensors report burnout, AI output is
/// gated off. No allocation, no host-device dependency, usable from `no_std`.
#[derive(Clone, Copy, Debug)]
pub struct PhaseFsm {
    phase: Phase,
    started_at_us: u64,
    entered_at_us: u64,
    /// Last committed edge into each reached phase, indexed by phase code.
    /// Only BURN can be entered twice (LiftOff); it overwrites its own slot.
    transitions: [Option<Transition>; SLOTS],
    /// Refused applications, including illegal edges and stale timestamps.
    rejected: u32,
}

impl PhaseFsm {
    pub const fn new() -> Self {
        Self::new_at(0)
    }

    pub const fn new_at(started_at_us: u64) -> Self {
        Self {
            phase: Phase::BURN,
            started_at_us,
            entered_at_us: started_at_us,
            transitions: [None; SLOTS],
            rejected: 0,
        }
    }

    pub const fn phase(&self) -> Phase {
        self.phase
    }

    pub const fn started_at_us(&self) -> u64 {
        self.started_at_us
    }

    pub const fn entered_at_us(&self) -> u64 {
        self.entered_at_us
    }

    pub const fn rejected_count(&self) -> u32 {
        self.rejected
    }

    /// The committed entry into `phase`, if that phase has been reached.
    pub fn transition_into(&self, phase: Phase) -> Option<Transition> {
        self.transitions[phase as usize]
    }

    /// Forward-only means phase order is time order: the highest reached
    /// phase holds the newest stamp.
    pub fn last_transition(&self) -> Option<Transition> {
        let mut i = SLOTS;
        while i > 0 {
            i -= 1;
            if let Some(transition) = self.transitions[i] {
                return Some(transition);
            }
        }
        None
    }

    pub fn transition_count(&self) -> usize {
        self.transitions.iter().filter(|t| t.is_some()).count()
    }

    /// Apply one event at `now_us`. A refusal leaves phase, stamps and log
    /// untouched; only the rejection counter moves.
    pub fn apply(&mut self, event: PhaseEvent, now_us: u64) -> Result<Transition, PhaseError> {
        if now_us < self.entered_at_us {
            self.rejected = self.rejected.wrapping_add(1);
            return Err(PhaseError::TimestampWentBackwards {
                now_us,
                entered_at_us: self.entered_at_us,
            });
        }
        let rule = match rule_for(self.phase, event) {
            Some(rule) => rule,
            None => {
                self.rejected = self.rejected.wrapping_add(1);
                return Err(PhaseError::IllegalTransition {
                    from: self.phase,
                    event,
                });
            }
        };
        let transition = Transition {
            from: rule.from,
            to: rule.to,
            event,
            at_us: now_us,
        };
        self.phase = rule.to;
        self.entered_at_us = now_us;
        self.transitions[rule.to as usize] = Some(transition);
        Ok(transition)
    }

    /// Active stabilisation is forbidden in BURN or while the motor is still
    /// armed. The phase and the sensor flag are independent gates.
    pub const fn ai_allowed(&self, sensor_flags: u8) -> bool {
        !matches!(self.phase, Phase::BURN) && (sensor_flags & flags::MOTOR_ARMED) == 0
    }
}

impl Default for PhaseFsm {
    fn default() -> Self {
        Self::new()
    }
}

const fn rule_for(from: Phase, event: PhaseEvent) -> Option<Rule> {
    let mut i = 0;
    while i < TRANSITIONS.len() {
        let rule = TRANSITIONS[i];
        // Compare as integers: enum PartialEq is not usable in const contexts.
        if rule.from as u8 == from as u8 && rule.event as u8 == event as u8 {
            return Some(rule);
        }
        i += 1;
    }
    None
}
