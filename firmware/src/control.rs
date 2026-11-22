//! Link state and one control step: sequencing, miss latching, phase gating,
//! watchdog feeding. No I/O and no allocation.

use potok_protocol as pp;

use crate::{
    board::Board,
    deadman::Deadman,
    phase::PhaseFsm,
    protocol::{
        decode_reply, encode_request, flags, ReplyAssembler, ReplyFrame, RequestFrame, SeqGate,
        REPLY_SIZE, REQUEST_SIZE,
    },
    uart::{DoubleBufferRx, FrameTx},
    watchdog::{self, Watchdog},
    Command,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub accepted: u32,
    pub rejected: u32,
    pub disables: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReplyOutcome {
    pub command: Command,
    /// False for stale, unhealthy or post-latch replies: the frame may be
    /// well-formed but must not move the actuator.
    pub accepted: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepResult {
    /// No complete reply frame was delivered this tick.
    Idle,
    /// Magic/CRC/version/seq rejected; the miss counter moved.
    FrameRejected,
    Accepted(Command),
}

#[derive(Clone, Copy, Debug)]
pub struct LinkState {
    seq: SeqGate,
    assembler: ReplyAssembler,
    misses: u8,
    disabled: bool,
    deadman: Deadman,
    pub counters: Counters,
}

impl LinkState {
    pub const fn new() -> Self {
        Self {
            seq: SeqGate::new(),
            assembler: ReplyAssembler::new(),
            misses: 0,
            disabled: false,
            deadman: Deadman::new(),
            counters: Counters {
                accepted: 0,
                rejected: 0,
                disables: 0,
            },
        }
    }

    pub const fn is_disabled(&self) -> bool {
        self.disabled
    }

    pub const fn misses(&self) -> u8 {
        self.misses
    }

    pub const fn last_request_seq(&self) -> Option<u16> {
        self.seq.last()
    }

    pub fn next_request_seq(&mut self) -> u16 {
        self.seq.issue()
    }

    pub const fn deadman_tripped(&self) -> bool {
        self.deadman.tripped()
    }

    pub fn feed_rx(&mut self, bytes: &[u8]) -> Option<[u8; REPLY_SIZE]> {
        self.assembler.push(bytes)
    }

    pub fn discard_partial_frame(&mut self) {
        self.assembler.reset();
    }

    /// Bad magic, CRC, version, seq or timeout: one miss. Three in a row latch.
    pub fn on_invalid(&mut self) {
        self.counters.rejected += 1;
        self.misses = self.misses.saturating_add(1);
        self.deadman.note_missed();
        if self.misses >= crate::MAX_CONSECUTIVE_MISSES {
            self.disable_now();
        }
    }

    pub fn on_timeout(&mut self) {
        self.on_invalid();
    }

    /// `ai_healthy = 0` is fatal immediately; it is not a three-strike case.
    pub fn disable_now(&mut self) {
        if !self.disabled {
            self.disabled = true;
            self.counters.disables += 1;
        }
    }

    /// Only an explicit firmware reset clears the latch. A valid late reply
    /// must not.
    pub fn reset(&mut self) {
        self.disabled = false;
        self.misses = 0;
        self.deadman.reset();
        self.seq.reset();
        self.assembler.reset();
    }

    /// Check sequencing and health, then compute the command under the phase
    /// and code veto gates. A well-formed reply clears the miss run even while
    /// AI is latched off, so the deadman can recover without clearing the latch.
    pub fn on_reply(&mut self, reply: &ReplyFrame, fsm: &PhaseFsm, flags: u8) -> ReplyOutcome {
        if self.seq.check(reply.seq).is_err() {
            self.on_invalid();
            return ReplyOutcome {
                command: Command::ZERO,
                accepted: false,
            };
        }
        if reply.ai_healthy == 0 {
            self.disable_now();
            return ReplyOutcome {
                command: Command::ZERO,
                accepted: false,
            };
        }
        self.misses = 0;
        self.deadman.note_valid();
        if self.disabled {
            return ReplyOutcome {
                command: Command::ZERO,
                accepted: false,
            };
        }
        self.counters.accepted += 1;
        ReplyOutcome {
            command: self.command(reply, fsm, flags),
            accepted: true,
        }
    }

    pub fn command(&self, reply: &ReplyFrame, fsm: &PhaseFsm, flags: u8) -> Command {
        if self.disabled || self.deadman.tripped() || !fsm.ai_allowed(flags) {
            return Command::ZERO;
        }
        Command {
            pitch: reply.g_pitch,
            yaw: reply.g_yaw,
        }
        .clamped()
    }
}

impl Default for LinkState {
    fn default() -> Self {
        Self::new()
    }
}

/// One 500 Hz control tick. Sends a request with an STM32-owned phase, consumes
/// at most one reply frame, gates the command and decides on the watchdog feed.
pub fn step<B: Board>(
    board: &mut B,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
    cycle_complete: bool,
) -> StepResult {
    if let Some(event) = board.poll_phase_event() {
        let _ = fsm.apply(event);
    }
    let phase = fsm.phase();
    let mut flags = board.sensor_flags();
    if board.motor_armed() {
        flags::set(&mut flags, flags::MOTOR_ARMED);
    }
    if !link.is_disabled() {
        flags::set(&mut flags, flags::AI_READY);
    }

    let seq = link.next_request_seq();
    let request = RequestFrame {
        seq,
        time_us: board.now_us(),
        phase,
        version: pp::REQUEST_VERSION,
        flags,
        reserved: 0,
        gyro: [0.0; 3],
        accel: [0.0; 3],
        mag: [0.0; 3],
        height: 0.0,
        pitot_dp: 0,
    };
    let mut request_buf = [0u8; REQUEST_SIZE];
    if encode_request(&request, &mut request_buf).is_err() || board.tx().send(&request_buf).is_err()
    {
        link.on_invalid();
    }

    let mut rx_buf = [0u8; REPLY_SIZE * 2];
    let mut frame = None;
    if let Ok(Some(len)) = board.rx().take(&mut rx_buf) {
        if len > 0 {
            frame = link.feed_rx(&rx_buf[..len]);
        }
    }

    let outcome = match frame {
        Some(bytes) => match decode_reply(&bytes) {
            Ok(reply) => {
                let reply_outcome = link.on_reply(&reply, fsm, flags);
                board.apply_command(reply_outcome.command);
                if reply_outcome.accepted {
                    StepResult::Accepted(reply_outcome.command)
                } else {
                    StepResult::FrameRejected
                }
            }
            Err(_) => {
                link.on_invalid();
                link.discard_partial_frame();
                board.apply_command(Command::ZERO);
                StepResult::FrameRejected
            }
        },
        None => {
            link.on_timeout();
            board.apply_command(Command::ZERO);
            StepResult::Idle
        }
    };

    if watchdog::may_feed(cycle_complete, link.deadman_tripped()) {
        board.wd().pet();
    }
    outcome
}
