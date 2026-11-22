//! Potok-M STM32H743 firmware: host-testable safety and link logic.
//!
//! Hardware calls live behind the [`board::Board`] trait. The binary in
//! `main.rs` is a thin adapter; `cargo test` runs the same code with mocks.

#![no_std]

#[cfg(not(target_os = "none"))]
extern crate std;

pub mod attitude;
pub mod buzzer;
pub mod calibration_table;
pub mod control_pid;
pub mod deployment;
pub mod interlocks;
pub mod perf;
pub mod phase_fsm;
pub mod protocol_link;
pub mod scheduler;
pub mod search_beacon;
pub mod self_test;
pub mod sensor_voting;
pub mod separation;
pub mod telemetry_log;
pub mod board;
pub mod control;
pub mod deadman;
#[cfg(not(target_os = "none"))]
pub mod mock;
pub mod phase;
pub mod protocol;
pub mod uart;
pub mod watchdog;

pub use control::{step, Counters, LinkState, StepResult};
pub use phase::{PhaseEvent, PhaseFsm};
pub use protocol::{FrameError, ReplyFrame, RequestFrame, SeqGate};

/// Three consecutive rejected frames, timeouts or stale replies latch AI off.
pub const MAX_CONSECUTIVE_MISSES: u8 = 3;
/// Reply angles are scaled by 0.001 rad; the model's bound is 0.1 rad.
pub const MAX_CORRECTION_RAW: i16 = 100;

/// The only safe command while the motor works, the AI is latched off or the
/// link is stale.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    pub pitch: i16,
    pub yaw: i16,
}

impl Command {
    pub const ZERO: Self = Self { pitch: 0, yaw: 0 };

    pub const fn is_zero(self) -> bool {
        self.pitch == 0 && self.yaw == 0
    }

    /// Code veto: the controller never applies an unbounded AI correction.
    pub const fn clamped(self) -> Self {
        Self {
            pitch: clamp_i16(self.pitch, MAX_CORRECTION_RAW),
            yaw: clamp_i16(self.yaw, MAX_CORRECTION_RAW),
        }
    }
}

const fn clamp_i16(value: i16, bound: i16) -> i16 {
    if value > bound {
        bound
    } else if value < -bound {
        -bound
    } else {
        value
    }
}
