//! Hardware boundary. Everything below the link logic is a `Board`; host tests
//! use `mock::MockBoard`, the binary uses the thin Embassy adapter in `main.rs`.

use crate::{
    phase::PhaseEvent,
    uart::{DoubleBufferRx, FrameTx},
    watchdog::Watchdog,
    Command,
};

pub trait Board {
    type Rx: DoubleBufferRx;
    type Tx: FrameTx;
    type Wd: Watchdog;

    fn rx(&mut self) -> &mut Self::Rx;
    fn tx(&mut self) -> &mut Self::Tx;
    fn wd(&mut self) -> &mut Self::Wd;

    /// Monotonic microseconds for the request timestamp.
    fn now_us(&self) -> u32;
    /// Sensor health bits from the board, never from the AI reply.
    fn sensor_flags(&self) -> u8;
    /// Motor-armed sense: a second, independent gate on AI output.
    fn motor_armed(&self) -> bool;
    /// Sensor-derived phase transition; `None` keeps the current phase.
    fn poll_phase_event(&mut self) -> Option<PhaseEvent>;
    /// Actuation path. Reaches this method only after the safety gate.
    fn apply_command(&mut self, cmd: Command);
}
