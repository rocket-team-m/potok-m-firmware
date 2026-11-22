//! Flight phase FSM owned by the STM32. The AI never selects or changes phase.

use potok_protocol::{flags, Phase};

pub use potok_protocol::Phase as FlightPhase;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseEvent {
    /// Motor ignited (still BURN: no active stabilisation allowed).
    LiftOff,
    /// Motor burned out: BURN -> COAST.
    Burnout,
    Apogee,
    Descent,
    Touchdown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhaseError;

/// BURN is the safe initial state: until sensors say the motor burned out, the
/// AI output is gated off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhaseFsm {
    phase: Phase,
}

impl PhaseFsm {
    pub const fn new() -> Self {
        Self { phase: Phase::BURN }
    }

    pub const fn phase(&self) -> Phase {
        self.phase
    }

    /// Forward-only transitions; an impossible event is refused, never a
    /// reason to move backwards into a phase where the motor may still run.
    pub fn apply(&mut self, event: PhaseEvent) -> Result<(), PhaseError> {
        let next = match (self.phase, event) {
            (Phase::BURN, PhaseEvent::LiftOff) => Phase::BURN,
            (Phase::BURN, PhaseEvent::Burnout) => Phase::COAST,
            (Phase::COAST, PhaseEvent::Apogee) => Phase::APOGEE,
            (Phase::APOGEE, PhaseEvent::Descent) => Phase::DESCENT,
            (Phase::DESCENT, PhaseEvent::Touchdown) => Phase::LANDED,
            _ => return Err(PhaseError),
        };
        self.phase = next;
        Ok(())
    }

    /// Active stabilisation is forbidden while the motor works: both the
    /// phase and the sensor-provided `MOTOR_ARMED` flag gate the output.
    pub const fn ai_allowed(&self, sensor_flags: u8) -> bool {
        !matches!(self.phase, Phase::BURN) && (sensor_flags & flags::MOTOR_ARMED) == 0
    }
}

impl Default for PhaseFsm {
    fn default() -> Self {
        Self::new()
    }
}
