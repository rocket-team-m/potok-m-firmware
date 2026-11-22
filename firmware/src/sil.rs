// SPDX-License-Identifier: GPL-3.0-or-later
//! Software-in-the-loop harness for the 500 Hz control loop.
//!
//! The core below uses only `core`: scripted inputs, a fixed-capacity trace and
//! a generic [`HarnessBoard`]. It drives the real [`step`], [`PhaseFsm`],
//! [`LinkState`] and [`Interlocks`]; nothing is reimplemented. The interlock is
//! inserted *inside* the actuation path by [`Guarded`], so a command produced
//! by the controller still has to pass the independent gate before it reaches
//! the board.
//!
//! `lib.rs` is owned by the parent task, so this module is included by path in
//! the host tests and benchmark. Their crate roots re-export the modules named
//! below. The only host-specific code is the gated [`host`] module at the end,
//! which carries the `MockBoard` adapter; no pin, UART or clock call exists.
//!
//! Time is synthetic: tick *n* runs at `n * TICK_PERIOD_US`, and every frame is
//! supplied by the script. Running the same script twice gives a bit-identical
//! trace, which is the property the gates check.

use potok_m_firmware::{
    board::Board,
    control::{step, LinkState, StepResult},
    interlocks::{Decision, Input as InterlockInput, Interlocks, Refusal},
    phase::{PhaseEvent, PhaseFsm},
    protocol::{encode_reply, flags, Phase, ReplyFrame, REPLY_SIZE},
    Command,
};

/// Control-loop rate the harness emulates.
pub const TICK_HZ: u32 = 500;
/// Synthetic time between two ticks, in microseconds.
pub const TICK_PERIOD_US: u32 = 1_000_000 / TICK_HZ;

/// The frame delivered to the board on one tick.
#[derive(Clone, Copy, Debug)]
pub enum Frame {
    /// No DMA chunk: the link records a timeout.
    Silent,
    /// Well-formed reply. `seq` is overwritten with the value `step` will issue.
    Reply(ReplyFrame),
    /// Reply with `seq = expected + 1`: guaranteed stale.
    Stale(ReplyFrame),
    /// Well-formed reply encoded with the expected seq, then one byte XORed.
    Corrupt {
        reply: ReplyFrame,
        byte: usize,
        mask: u8,
    },
    /// Verbatim DMA bytes, for short frames and hand-built corruption.
    Raw { bytes: [u8; REPLY_SIZE], len: usize },
}

impl Frame {
    /// Reply fields shared by the scripted command frames.
    pub const fn template(pitch: i16, yaw: i16) -> ReplyFrame {
        ReplyFrame {
            seq: 0,
            g_pitch: pitch,
            g_yaw: yaw,
            confidence: 200,
            ai_healthy: 1,
            version: 1,
        }
    }

    /// A healthy, in-bounds command reply.
    pub const fn command(pitch: i16, yaw: i16) -> Self {
        Self::Reply(Self::template(pitch, yaw))
    }

    /// A reply that is stale whatever the current sequence number is.
    pub const fn stale(pitch: i16, yaw: i16) -> Self {
        Self::Stale(Self::template(pitch, yaw))
    }

    /// A reply whose encoded form is corrupted at `byte` with `mask`.
    pub const fn corrupt(reply: ReplyFrame, byte: usize, mask: u8) -> Self {
        Self::Corrupt { reply, byte, mask }
    }

    /// A healthy reply with the AI health byte cleared.
    pub const fn unhealthy(pitch: i16, yaw: i16) -> Self {
        let mut reply = Self::template(pitch, yaw);
        reply.ai_healthy = 0;
        Self::Reply(reply)
    }
}

/// One scripted tick: board inputs plus the frame to inject.
#[derive(Clone, Copy, Debug)]
pub struct Tick {
    pub frame: Frame,
    pub phase_event: Option<PhaseEvent>,
    pub sensor_flags: u8,
    pub motor_armed: bool,
    pub cycle_complete: bool,
    /// Fault injection: phase code the interlock decodes instead of the FSM's.
    pub phase_code_override: Option<u8>,
}

impl Tick {
    /// A healthy command tick, with no phase event and nominal sensors.
    pub const fn reply(pitch: i16, yaw: i16) -> Self {
        Self {
            frame: Frame::command(pitch, yaw),
            phase_event: None,
            sensor_flags: 0,
            motor_armed: false,
            cycle_complete: true,
            phase_code_override: None,
        }
    }

    /// A well-formed reply with one encoded byte corrupted.
    pub fn corrupt(pitch: i16, yaw: i16, byte: usize, mask: u8) -> Self {
        Self {
            frame: Frame::corrupt(Frame::template(pitch, yaw), byte, mask),
            ..Self::reply(0, 0)
        }
    }

    /// A reply carrying a sequence number the link will reject.
    pub fn stale(pitch: i16, yaw: i16) -> Self {
        Self {
            frame: Frame::stale(pitch, yaw),
            ..Self::reply(0, 0)
        }
    }

    /// A reply with the AI health byte cleared.
    pub fn unhealthy(pitch: i16, yaw: i16) -> Self {
        Self {
            frame: Frame::unhealthy(pitch, yaw),
            ..Self::reply(0, 0)
        }
    }

    /// A tick on which no frame arrives.
    pub const fn silent() -> Self {
        Self {
            frame: Frame::Silent,
            phase_event: None,
            sensor_flags: 0,
            motor_armed: false,
            cycle_complete: true,
            phase_code_override: None,
        }
    }

    pub const fn event(mut self, event: PhaseEvent) -> Self {
        self.phase_event = Some(event);
        self
    }

    pub const fn flags(mut self, flags: u8) -> Self {
        self.sensor_flags = flags;
        self
    }

    pub const fn armed(mut self, armed: bool) -> Self {
        self.motor_armed = armed;
        self
    }

    pub const fn incomplete(mut self) -> Self {
        self.cycle_complete = false;
        self
    }

    pub const fn phase_code(mut self, code: u8) -> Self {
        self.phase_code_override = Some(code);
        self
    }
}

/// Knobs the harness needs to drive a board deterministically.
///
/// `Board` itself only describes I/O. This adapter adds the clock, sensor and
/// phase-event inputs a script must control; it is implemented for `MockBoard`
/// in the host module below. The core never names a concrete board.
pub trait HarnessBoard: Board {
    fn set_now_us(&mut self, us: u32);
    fn set_inputs(
        &mut self,
        sensor_flags: u8,
        motor_armed: bool,
        phase_event: Option<PhaseEvent>,
        cycle_complete: bool,
    );
    fn inject(&mut self, bytes: &[u8]);
    fn last_applied(&self) -> Command;
    fn pet_count(&self) -> u32;
}

#[derive(Clone, Copy, Debug)]
struct Pending {
    phase_code: u8,
    flags: u8,
    ai_healthy: bool,
    now_us: u32,
}

/// Board decorator that puts the independent interlock in the actuation path.
///
/// Every command `step` produces is evaluated here; only [`Decision::command`]
/// reaches the inner board. The decision and the original request are retained
/// for the trace.
pub struct Guarded<B: Board> {
    inner: B,
    gate: Interlocks,
    pending: Pending,
    last_decision: Decision,
    last_requested: Command,
}

impl<B: Board> Guarded<B> {
    pub fn new(inner: B) -> Self {
        Self {
            inner,
            gate: Interlocks::default(),
            pending: Pending {
                phase_code: Phase::BURN as u8,
                flags: 0,
                ai_healthy: true,
                now_us: 0,
            },
            last_decision: Decision {
                command: Interlocks::FAILSAFE,
                permitted: false,
                refusal: None,
                limited: false,
            },
            last_requested: Command::ZERO,
        }
    }

    pub fn inner(&self) -> &B {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut B {
        &mut self.inner
    }

    pub fn gate(&self) -> &Interlocks {
        &self.gate
    }

    pub fn gate_mut(&mut self) -> &mut Interlocks {
        &mut self.gate
    }

    fn prepare(&mut self, pending: Pending) {
        self.pending = pending;
    }

    pub fn last_decision(&self) -> Decision {
        self.last_decision
    }

    pub fn last_requested(&self) -> Command {
        self.last_requested
    }
}

impl<B: Board> Board for Guarded<B> {
    type Rx = B::Rx;
    type Tx = B::Tx;
    type Wd = B::Wd;

    fn rx(&mut self) -> &mut Self::Rx {
        self.inner.rx()
    }
    fn tx(&mut self) -> &mut Self::Tx {
        self.inner.tx()
    }
    fn wd(&mut self) -> &mut Self::Wd {
        self.inner.wd()
    }
    fn now_us(&self) -> u32 {
        self.inner.now_us()
    }
    fn sensor_flags(&self) -> u8 {
        self.inner.sensor_flags()
    }
    fn motor_armed(&self) -> bool {
        self.inner.motor_armed()
    }
    fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
        self.inner.poll_phase_event()
    }
    fn apply_command(&mut self, cmd: Command) {
        self.last_requested = cmd;
        let decision = self.gate.evaluate(InterlockInput {
            phase_code: self.pending.phase_code,
            flags: self.pending.flags,
            ai_healthy: self.pending.ai_healthy,
            requested: cmd,
            now_us: self.pending.now_us,
        });
        self.last_decision = decision;
        self.inner.apply_command(decision.command);
    }
}

/// One recorded tick: every output and every state bit a reader needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickRecord {
    pub tick: u32,
    pub now_us: u32,
    pub result: StepResult,
    /// Command requested by the controller (after its own code veto).
    pub requested: Command,
    /// Command actually applied, after the independent interlock.
    pub applied: Command,
    pub phase: u8,
    pub interlock_permitted: bool,
    pub interlock_refusal: Option<Refusal>,
    pub interlock_limited: bool,
    pub misses: u8,
    pub disabled: bool,
    pub deadman_tripped: bool,
    pub pets: u32,
}

impl TickRecord {
    pub const EMPTY: Self = Self {
        tick: 0,
        now_us: 0,
        result: StepResult::Idle,
        requested: Command::ZERO,
        applied: Command::ZERO,
        phase: Phase::BURN as u8,
        interlock_permitted: false,
        interlock_refusal: None,
        interlock_limited: false,
        misses: 0,
        disabled: false,
        deadman_tripped: false,
        pets: 0,
    };
}

/// A state change observed between two ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TraceEvent {
    Phase { tick: u32, from: u8, to: u8 },
    LinkDisabled { tick: u32 },
    InterlockLatched { tick: u32, refusal: Refusal },
    InterlockReset { tick: u32 },
}

/// Fixed-capacity, heap-free trace. Overflow is a loud panic, never a drop.
#[derive(Clone, Debug, PartialEq)]
pub struct Trace<const CAP: usize> {
    records: [TickRecord; CAP],
    events: [TraceEvent; CAP],
    len: usize,
    event_len: usize,
}

impl<const CAP: usize> Trace<CAP> {
    pub fn new() -> Self {
        assert!(CAP > 0, "sil: trace capacity must be positive");
        Self {
            records: core::array::from_fn(|_| TickRecord::EMPTY),
            events: core::array::from_fn(|_| TraceEvent::InterlockReset { tick: 0 }),
            len: 0,
            event_len: 0,
        }
    }

    pub fn push(&mut self, record: TickRecord) {
        assert!(self.len < CAP, "sil: trace record capacity exceeded");
        self.records[self.len] = record;
        self.len += 1;
    }

    pub fn event(&mut self, event: TraceEvent) {
        assert!(self.event_len < CAP, "sil: trace event capacity exceeded");
        self.events[self.event_len] = event;
        self.event_len += 1;
    }

    pub fn records(&self) -> &[TickRecord] {
        &self.records[..self.len]
    }

    pub fn events(&self) -> &[TraceEvent] {
        &self.events[..self.event_len]
    }

    pub fn len(&self) -> usize {
        self.len
    }
}

impl<const CAP: usize> Default for Trace<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

/// Owns the board, the phase FSM and the link state for one scripted run.
pub struct Simulator<B: HarnessBoard> {
    pub board: Guarded<B>,
    pub fsm: PhaseFsm,
    pub link: LinkState,
    tick_index: u32,
}

impl<B: HarnessBoard> Simulator<B> {
    pub fn new(inner: B) -> Self {
        Self {
            board: Guarded::new(inner),
            fsm: PhaseFsm::new(),
            link: LinkState::new(),
            tick_index: 0,
        }
    }

    pub fn inner_board(&self) -> &B {
        self.board.inner()
    }

    pub fn gate(&self) -> &Interlocks {
        self.board.gate()
    }

    pub fn gate_mut(&mut self) -> &mut Interlocks {
        self.board.gate_mut()
    }

    /// The only way out of an interlock latch, recorded in the trace.
    pub fn reset_gate<const CAP: usize>(&mut self, trace: &mut Trace<CAP>) {
        self.gate_mut().reset();
        trace.event(TraceEvent::InterlockReset {
            tick: self.tick_index,
        });
    }

    /// Drive one scripted 500 Hz tick and record its outputs and transitions.
    pub fn tick<const CAP: usize>(&mut self, spec: &Tick, trace: &mut Trace<CAP>) {
        let tick = self.tick_index;
        let now_us = tick.wrapping_mul(TICK_PERIOD_US);
        let phase_before = self.fsm.phase();
        let disabled_before = self.link.is_disabled();
        let latch_before = self.gate().latched();
        let expected_seq = self
            .link
            .last_request_seq()
            .map_or(0, |last| last.wrapping_add(1));

        let ai_healthy = match spec.frame {
            Frame::Reply(reply) | Frame::Stale(reply) | Frame::Corrupt { reply, .. } => {
                reply.ai_healthy != 0
            }
            Frame::Silent | Frame::Raw { .. } => true,
        };

        {
            let board = self.board.inner_mut();
            board.set_now_us(now_us);
            board.set_inputs(
                spec.sensor_flags,
                spec.motor_armed,
                spec.phase_event,
                spec.cycle_complete,
            );
            let mut bytes = [0u8; REPLY_SIZE];
            match spec.frame {
                Frame::Silent => {}
                Frame::Reply(mut reply) => {
                    reply.seq = expected_seq;
                    encode_reply(&reply, &mut bytes).expect("sil: scripted reply must encode");
                    board.inject(&bytes);
                }
                Frame::Stale(mut reply) => {
                    reply.seq = expected_seq.wrapping_add(1);
                    encode_reply(&reply, &mut bytes).expect("sil: scripted reply must encode");
                    board.inject(&bytes);
                }
                Frame::Corrupt {
                    mut reply,
                    byte,
                    mask,
                } => {
                    assert!(byte < REPLY_SIZE, "sil: corrupt byte out of range");
                    reply.seq = expected_seq;
                    encode_reply(&reply, &mut bytes).expect("sil: scripted reply must encode");
                    bytes[byte] ^= mask;
                    board.inject(&bytes);
                }
                Frame::Raw { bytes: raw, len } => {
                    assert!(len <= REPLY_SIZE, "sil: raw frame longer than a reply");
                    board.inject(&raw[..len]);
                }
            }
        }

        // `step` applies a scheduled event at the start of the tick. Use a
        // copy of the real FSM to sample the post-event phase the same way,
        // so the interlock sees what the controller put in the request.
        let mut phase_probe = self.fsm;
        if let Some(event) = spec.phase_event {
            let _ = phase_probe.apply(event);
        }
        let phase_code = spec
            .phase_code_override
            .unwrap_or(phase_probe.phase() as u8);
        let mut interlock_flags = spec.sensor_flags;
        if spec.motor_armed {
            flags::set(&mut interlock_flags, flags::MOTOR_ARMED);
        }
        self.board.prepare(Pending {
            phase_code,
            flags: interlock_flags,
            ai_healthy,
            now_us,
        });

        let result = step(
            &mut self.board,
            &mut self.fsm,
            &mut self.link,
            spec.cycle_complete,
        );

        let decision = self.board.last_decision();
        let requested = self.board.last_requested();
        let applied = self.board.inner().last_applied();
        let pets = self.board.inner().pet_count();
        trace.push(TickRecord {
            tick,
            now_us,
            result,
            requested,
            applied,
            phase: self.fsm.phase() as u8,
            interlock_permitted: decision.permitted,
            interlock_refusal: decision.refusal,
            interlock_limited: decision.limited,
            misses: self.link.misses(),
            disabled: self.link.is_disabled(),
            deadman_tripped: self.link.deadman_tripped(),
            pets,
        });

        if self.fsm.phase() != phase_before {
            trace.event(TraceEvent::Phase {
                tick,
                from: phase_before as u8,
                to: self.fsm.phase() as u8,
            });
        }
        if self.link.is_disabled() && !disabled_before {
            trace.event(TraceEvent::LinkDisabled { tick });
        }
        let latch_now = self.gate().latched();
        if latch_now.is_some() && latch_now != latch_before {
            trace.event(TraceEvent::InterlockLatched {
                tick,
                refusal: latch_now.expect("checked above"),
            });
        }

        self.tick_index = self.tick_index.wrapping_add(1);
    }

    /// Run a whole script.
    pub fn run<const CAP: usize>(&mut self, ticks: &[Tick], trace: &mut Trace<CAP>) {
        for tick in ticks {
            self.tick(tick, trace);
        }
    }
}

/// Convenience entry point: build a simulator, run a script, return the state.
pub fn simulate<B: HarnessBoard, const CAP: usize>(
    inner: B,
    ticks: &[Tick],
    trace: &mut Trace<CAP>,
) -> Simulator<B> {
    let mut sim = Simulator::new(inner);
    sim.run(ticks, trace);
    sim
}

#[cfg(not(target_os = "none"))]
mod host {
    use super::{Command, HarnessBoard, PhaseEvent};
    use potok_m_firmware::mock::MockBoard;

    impl HarnessBoard for MockBoard {
        fn set_now_us(&mut self, us: u32) {
            self.now_us = us;
        }

        fn set_inputs(
            &mut self,
            sensor_flags: u8,
            motor_armed: bool,
            phase_event: Option<PhaseEvent>,
            cycle_complete: bool,
        ) {
            self.sensor_flags = sensor_flags;
            self.motor_armed = motor_armed;
            self.phase_event = phase_event;
            self.cycle_complete = cycle_complete;
        }

        fn inject(&mut self, bytes: &[u8]) {
            self.rx_mut().queue(bytes);
        }

        fn last_applied(&self) -> Command {
            *self
                .applied
                .last()
                .expect("sil: every step applies a command, even zero")
        }

        fn pet_count(&self) -> u32 {
            self.pets()
        }
    }
}
