// SPDX-License-Identifier: GPL-3.0-or-later
//! Link state and one control step: sequencing, miss latching, phase gating,
//! watchdog feeding. No I/O and no allocation.

use potok_protocol as pp;

use crate::{
    board::Board,
    deadman::Deadman,
    interlocks::{Input as InterlockInput, Interlocks, Refusal},
    phase::PhaseFsm,
    protocol::{
        decode_reply, encode_request, flags, ReplyAssembler, ReplyFrame, RequestFrame, SeqGate,
        REPLY_SIZE, REQUEST_SIZE,
    },
    uart::{DoubleBufferRx, FrameTx},
    watchdog::{self, FeedDecision, Watchdog, WatchdogPolicy},
    Command,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub accepted: u32,
    pub rejected: u32,
    pub disables: u32,
}

/// The two measurements the runtime Mach shield gates on. They ride with the
/// link tick because the request and the shield must see the same tick's air
/// evidence; no new board pin or wire field is introduced.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShieldSample {
    /// Pitot dynamic pressure in Pa.
    pub pitot_dp: i16,
    /// Mach estimate the caller derives from pitot and static pressure.
    pub mach_est: f32,
}

impl ShieldSample {
    /// No pitot driver is wired (the schematic-bound binary still has the
    /// sensor TODO). The shield vetoes with `PitotInvalid`: zero command.
    pub const UNAVAILABLE: Self = Self {
        pitot_dp: 0,
        mach_est: 0.0,
    };

    /// Seam for host tests that construct `LinkState` without a sensor supply.
    /// It preserves the pre-shield no-pitot behavior while every other shield
    /// gate still runs; the shipping binary never uses it.
    pub const LEGACY_NO_SENSOR: Self = Self {
        pitot_dp: 1,
        mach_est: 0.0,
    };
}

/// Per-reason tally of the shield's decisions. The interlock's own counters
/// aggregate all blocks; this keeps one slot per refusal so a telemetry reader
/// can tell which gate fired without a tracer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReasonCounts {
    pub accepted: u32,
    pub unknown_phase: u32,
    pub boost: u32,
    pub motor_armed: u32,
    pub ai_unhealthy: u32,
    pub fault: u32,
    pub implausible: u32,
    pub non_finite_mach: u32,
    pub pitot_invalid: u32,
    pub mach_limit: u32,
}

impl ReasonCounts {
    pub const EMPTY: Self = Self {
        accepted: 0,
        unknown_phase: 0,
        boost: 0,
        motor_armed: 0,
        ai_unhealthy: 0,
        fault: 0,
        implausible: 0,
        non_finite_mach: 0,
        pitot_invalid: 0,
        mach_limit: 0,
    };

    pub fn note(&mut self, refusal: Option<Refusal>) {
        match refusal {
            None => self.accepted += 1,
            Some(Refusal::UnknownPhase(_)) => self.unknown_phase += 1,
            Some(Refusal::Boost) => self.boost += 1,
            Some(Refusal::MotorArmed) => self.motor_armed += 1,
            Some(Refusal::AiUnhealthy) => self.ai_unhealthy += 1,
            Some(Refusal::Fault) => self.fault += 1,
            Some(Refusal::Implausible) => self.implausible += 1,
            Some(Refusal::NonFiniteMach) => self.non_finite_mach += 1,
            Some(Refusal::PitotInvalid) => self.pitot_invalid += 1,
            Some(Refusal::MachLimit) => self.mach_limit += 1,
        }
    }
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

/// The link layer's decision before the independent safety gates run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TickDecision {
    result: StepResult,
    /// Command the link layer asks to apply; ZERO on every rejected outcome.
    command: Command,
    /// Health byte of a decoded reply. True when no reply decoded, so a frame
    /// fault cannot latch the interlock's AiUnhealthy refusal.
    ai_healthy: bool,
    /// Sensor/motor/AI-ready bits the link placed in this tick's request.
    flags: u8,
    /// Timestamp of this tick, shared by the interlock and the deadline monitor.
    now_us: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct LinkState {
    seq: SeqGate,
    assembler: ReplyAssembler,
    misses: u8,
    disabled: bool,
    deadman: Deadman,
    pub counters: Counters,
    shield_sample: ShieldSample,
    /// Per-reason tally of the shield that runs inside `step`.
    pub shield_reasons: ReasonCounts,
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
            shield_sample: ShieldSample::LEGACY_NO_SENSOR,
            shield_reasons: ReasonCounts::EMPTY,
        }
    }

    /// Supply this tick's air evidence to the Mach shield. The firmware calls
    /// this on every tick; a driver that cannot read the pitot supplies
    /// [`ShieldSample::UNAVAILABLE`], which zeroes the command.
    pub fn set_shield_sample(&mut self, sample: ShieldSample) {
        self.shield_sample = sample;
    }

    pub const fn shield_sample(&self) -> ShieldSample {
        self.shield_sample
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

/// One 500 Hz link tick: send a request with an STM32-owned phase, consume at
/// most one reply frame and compute the command the link layer asks to apply.
/// No hardware boundary is crossed here; both [`step_link`] and the gated
/// [`step`] build on this single implementation.
fn link_tick<B: Board>(board: &mut B, fsm: &mut PhaseFsm, link: &mut LinkState) -> TickDecision {
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
    let now_us = board.now_us();

    let seq = link.next_request_seq();
    let request = RequestFrame {
        seq,
        time_us: now_us,
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

    match frame {
        Some(bytes) => match decode_reply(&bytes) {
            Ok(reply) => {
                let ai_healthy = reply.ai_healthy != 0;
                let reply_outcome = link.on_reply(&reply, fsm, flags);
                let result = if reply_outcome.accepted {
                    StepResult::Accepted(reply_outcome.command)
                } else {
                    StepResult::FrameRejected
                };
                TickDecision {
                    result,
                    command: reply_outcome.command,
                    ai_healthy,
                    flags,
                    now_us,
                }
            }
            Err(_) => {
                link.on_invalid();
                link.discard_partial_frame();
                TickDecision {
                    result: StepResult::FrameRejected,
                    command: Command::ZERO,
                    ai_healthy: true,
                    flags,
                    now_us,
                }
            }
        },
        None => {
            link.on_timeout();
            TickDecision {
                result: StepResult::Idle,
                command: Command::ZERO,
                ai_healthy: true,
                flags,
                now_us,
            }
        }
    }
}

/// Link-level control tick: link, phase and wire-fault gates only.
///
/// Used by the link/host tests and the SIL harness, whose board decorator
/// runs the same [`Interlocks::evaluate_shield`] inside the actuation path.
/// The shipping binary uses [`step`] instead.
pub fn step_link<B: Board>(
    board: &mut B,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
    cycle_complete: bool,
) -> StepResult {
    let tick = link_tick(board, fsm, link);
    board.apply_command(tick.command);
    if watchdog::may_feed(cycle_complete, link.deadman_tripped()) {
        board.wd().pet();
    }
    tick.result
}

/// Production 500 Hz control tick.
///
/// Every command crosses the independent [`Interlocks::evaluate_shield`] gate
/// (phase/health/latch *and* Mach/pitot) and the [`WatchdogPolicy`] deadline
/// monitor before it reaches the hardware boundary. The sample is the one this
/// tick's link carries; a veto collapses the output to [`Command::ZERO`] and is
/// tallied in [`LinkState::shield_reasons`]. Feeding still requires
/// the legacy link-deadman gate as well as the watchguard feed decision, so AI
/// silence cannot keep IWDG alive and a missed control deadline stops feeding.
pub fn step<B: Board>(
    board: &mut B,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
    interlock: &mut Interlocks,
    policy: &mut WatchdogPolicy,
    cycle_complete: bool,
) -> StepResult {
    let tick = link_tick(board, fsm, link);

    // Keep the policy's AI latch in step with the link latch. Both force zero,
    // neither clears the other implicitly; the interlock keeps its own latch.
    if link.is_disabled() {
        policy.latch_ai();
    } else {
        policy.reset_ai_latch();
    }

    let sample = link.shield_sample;
    let decision = interlock.evaluate_shield(
        InterlockInput {
            phase_code: fsm.phase() as u8,
            flags: tick.flags,
            ai_healthy: tick.ai_healthy,
            requested: tick.command,
            now_us: tick.now_us,
        },
        sample.pitot_dp,
        sample.mach_est,
    );
    link.shield_reasons.note(decision.refusal);
    let feed = policy.on_control_cycle(tick.now_us, cycle_complete, true);

    let output = if decision.permitted && !policy.must_zero_output() {
        decision.command
    } else {
        Command::ZERO
    };
    board.apply_command(output);

    if feed == FeedDecision::Feed && watchdog::may_feed(cycle_complete, link.deadman_tripped()) {
        board.wd().pet();
    }

    // Report the command that actually reached the board.
    match tick.result {
        StepResult::Accepted(_) => StepResult::Accepted(output),
        other => other,
    }
}
