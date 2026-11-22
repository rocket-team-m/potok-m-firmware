// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic fault injection over SIL frames.
//!
//! A SIL (software-in-the-loop) run exchanges two fixed-size frames with the
//! simulated vehicle: a 56-byte request carrying sensor samples from the
//! firmware and a 14-byte reply carrying the AI's pitch/yaw corrections. This
//! module is a hardware-free test instrument that corrupts those frames in a
//! reproducible way so the firmware's safety response can be asserted.
//!
//! Nothing here reads a peripheral, owns a pin or allocates. The caller supplies
//! the nominal frame and the sequence number the firmware expects; [`Fault`]
//! turns that into a concrete [`InjectedReply`], and [`drive_step`] runs one real
//! control step and reports what the safety logic did. Determinism comes from a
//! seeded PRNG in [`FaultPlan`]: the same seed yields the same fault order and
//! the same corrupted bytes on every run.
//!
//! Wiring into the crate root is owned by the parent task; this file declares
//! no `mod` entries elsewhere. Host tests include it by path.

use crate::{
    board::Board,
    phase::{PhaseEvent, PhaseFsm},
    protocol::{encode_reply, Phase, ReplyFrame, RequestFrame, REPLY_SIZE},
    step, Command, LinkState, StepResult,
};

/// Number of distinct faults this instrument can inject.
pub const FAULT_COUNT: usize = 8;

/// Every fault, in the order used to build a plan before it is shuffled.
pub const FAULTS: [Fault; FAULT_COUNT] = [
    Fault::StuckSensor,
    Fault::OutOfRange,
    Fault::CrcFailure,
    Fault::SeqGap,
    Fault::SeqDuplicate,
    Fault::LateReply,
    Fault::PhaseMismatch,
    Fault::AiUnhealthy,
];

/// One failure mode of a SIL frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    /// A sensor channel stops updating, so the same sample is reported forever.
    StuckSensor,
    /// A sensor sample or AI correction exceeds the protocol/model range.
    OutOfRange,
    /// The reply's CRC bytes are corrupted in transit.
    CrcFailure,
    /// The reply echoes a sequence number ahead of the one requested.
    SeqGap,
    /// The reply echoes a previously used sequence number.
    SeqDuplicate,
    /// A valid-looking reply arrives one or more control ticks late.
    LateReply,
    /// The frame's phase does not match the firmware's current phase.
    PhaseMismatch,
    /// The AI declares itself unhealthy (`ai_healthy = 0`).
    AiUnhealthy,
}

impl Fault {
    /// Stable lowercase name, used by reports and failure messages.
    pub const fn name(self) -> &'static str {
        match self {
            Self::StuckSensor => "stuck_sensor",
            Self::OutOfRange => "out_of_range",
            Self::CrcFailure => "crc_failure",
            Self::SeqGap => "seq_gap",
            Self::SeqDuplicate => "seq_duplicate",
            Self::LateReply => "late_reply",
            Self::PhaseMismatch => "phase_mismatch",
            Self::AiUnhealthy => "ai_unhealthy",
        }
    }

    /// True when this fault corrupts a reply the firmware receives (AI -> FW).
    /// The remaining faults live on the sensor/request side.
    pub const fn is_reply_fault(self) -> bool {
        matches!(
            self,
            Self::CrcFailure
                | Self::SeqGap
                | Self::SeqDuplicate
                | Self::LateReply
                | Self::AiUnhealthy
        )
    }

    /// Unhealthy AI is fatal on the first frame; it does not need a three-strike
    /// run. Every other reply fault is counted and latches at three misses.
    pub const fn latches_immediately(self) -> bool {
        matches!(self, Self::AiUnhealthy)
    }

    /// Number of faulted frames a plan of this fault must deliver to reach the
    /// latch. Reply-integrity faults need three consecutive misses; an
    /// unhealthy reply needs one.
    pub const fn strikes_to_latch(self) -> u8 {
        if self.latches_immediately() {
            1
        } else {
            crate_max_misses()
        }
    }

    /// Build the corrupted reply this fault injects. `expected_seq` is the seq
    /// of the request issued in the step that will consume the reply, so gap,
    /// duplicate and late faults can be made deterministically stale.
    pub fn inject_reply(self, nominal: &ReplyFrame, expected_seq: u16) -> InjectedReply {
        match self {
            Self::CrcFailure => {
                let mut bytes = [0u8; REPLY_SIZE];
                encode_reply(nominal, &mut bytes).expect("a nominal reply always encodes");
                bytes[potok_protocol::REPLY_OFF_CRC16] ^= 0xA5;
                InjectedReply::Bytes(bytes)
            }
            Self::SeqGap => {
                let mut reply = *nominal;
                reply.seq = expected_seq.wrapping_add(2);
                InjectedReply::Reply(reply)
            }
            Self::SeqDuplicate | Self::LateReply => {
                // Both echo a request that has already been answered. A late
                // reply is stale by construction: the seq gate has issued a
                // newer request by the time it arrives.
                let mut reply = *nominal;
                reply.seq = expected_seq.wrapping_sub(1);
                InjectedReply::Reply(reply)
            }
            Self::AiUnhealthy => {
                let mut reply = *nominal;
                reply.ai_healthy = 0;
                InjectedReply::Reply(reply)
            }
            // Sensor-side faults do not damage the reply itself. They are
            // applied to the request/board state by [`mutate_request`].
            Self::StuckSensor | Self::OutOfRange | Self::PhaseMismatch => {
                InjectedReply::Reply(*nominal)
            }
        }
    }

    /// Build the corrupted request this fault produces, if it is a sensor-side
    /// fault. Returns `None` for faults that only attack the reply stream.
    ///
    /// `nominal` is the request the firmware would send with healthy sensors.
    /// A stuck sensor reports a frozen all-zero payload; an out-of-range sample
    /// saturates every channel; a phase mismatch announces the terminal phase
    /// while the firmware is still earlier in flight.
    pub fn mutate_request(self, nominal: &RequestFrame) -> Option<RequestFrame> {
        let mut frame = *nominal;
        match self {
            Self::StuckSensor => {
                frame.gyro = [0.0; 3];
                frame.accel = [0.0; 3];
                frame.mag = [0.0; 3];
                frame.height = 0.0;
                frame.pitot_dp = 0;
                Some(frame)
            }
            Self::OutOfRange => {
                frame.gyro = [f32::MAX; 3];
                frame.accel = [f32::MIN; 3];
                frame.mag = [f32::MAX; 3];
                frame.height = f32::MAX;
                frame.pitot_dp = i16::MAX;
                Some(frame)
            }
            Self::PhaseMismatch => {
                frame.phase = Phase::LANDED;
                Some(frame)
            }
            _ => None,
        }
    }

    /// Phase event that models the sensor-side phase mismatch fault, if any.
    /// An event illegal for the current phase is refused by the FSM, which is
    /// the safety behavior the tests assert.
    pub fn phase_event(self) -> Option<PhaseEvent> {
        match self {
            Self::PhaseMismatch => Some(PhaseEvent::Touchdown),
            _ => None,
        }
    }
}

const fn crate_max_misses() -> u8 {
    crate::MAX_CONSECUTIVE_MISSES
}

/// What the injector puts on the wire (or deliberately leaves off it) for one
/// control tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InjectedReply {
    /// Raw bytes for the RX queue; used when the CRC itself is damaged, because
    /// a decoded frame could not carry a bad CRC.
    Bytes([u8; REPLY_SIZE]),
    /// A well-formed reply with a corrupted seq or health field.
    Reply(ReplyFrame),
    /// No frame delivered this tick: the timeout/deadman path.
    Withheld,
}

/// Small seeded PRNG (xorshift64*). No `std`, no dependency, identical stream on
/// every target. A zero seed is replaced by a fixed constant so a plan is never
/// degenerate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeededRng(u64);

impl SeededRng {
    pub const fn new(seed: u64) -> Self {
        Self(if seed == 0 {
            0x9E37_79B9_7F4A_7C15
        } else {
            seed
        })
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

/// A reproducible campaign: all eight faults, shuffled by a seed. Two plans
/// built from the same seed deliver the same faults in the same order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FaultPlan {
    order: [Fault; FAULT_COUNT],
    cursor: usize,
}

impl FaultPlan {
    pub fn new(seed: u64) -> Self {
        let mut order = FAULTS;
        let mut rng = SeededRng::new(seed);
        let mut i = FAULT_COUNT;
        while i > 1 {
            let j = (rng.next_u64() % i as u64) as usize;
            order.swap(i - 1, j);
            i -= 1;
        }
        Self { order, cursor: 0 }
    }

    pub fn next_fault(&mut self) -> Option<Fault> {
        let fault = self.order.get(self.cursor).copied()?;
        self.cursor += 1;
        Some(fault)
    }

    pub const fn as_slice(&self) -> &[Fault] {
        &self.order
    }

    pub const fn delivered(&self) -> usize {
        self.cursor
    }
}

/// Observable safety state after one faulted control step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Observation {
    pub result: StepResult,
    /// The command applied this step: the accepted value, or ZERO when the
    /// frame was rejected or no frame arrived.
    pub command: Command,
    pub latched: bool,
    pub misses: u8,
    pub deadman: bool,
    pub accepted: u32,
    pub rejected: u32,
    pub disables: u32,
}

/// Run one real control step and read back the safety state. The command is
/// taken from the step result: on `FrameRejected` and `Idle` the firmware
/// applies ZERO by construction, so this is the actuator command for the tick.
pub fn drive_step<B: Board>(
    board: &mut B,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
) -> Observation {
    let result = step(board, fsm, link, true);
    let command = match result {
        StepResult::Accepted(command) => command,
        StepResult::FrameRejected | StepResult::Idle => Command::ZERO,
    };
    Observation {
        result,
        command,
        latched: link.is_disabled(),
        misses: link.misses(),
        deadman: link.deadman_tripped(),
        accepted: link.counters.accepted,
        rejected: link.counters.rejected,
        disables: link.counters.disables,
    }
}

/// The unconditional safety invariant: a faulted tick must never actuate.
pub fn assert_safe_zero(obs: &Observation) {
    assert!(
        obs.command.is_zero(),
        "faulted tick applied a non-zero command: {obs:?}"
    );
}

/// The stronger invariant for a frame-integrity fault: AI is latched off and
/// the command is ZERO.
pub fn assert_latched_and_zero(obs: &Observation) {
    assert!(obs.latched, "AI was not latched off: {obs:?}");
    assert_safe_zero(obs);
}
