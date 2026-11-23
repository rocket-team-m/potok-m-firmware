// SPDX-License-Identifier: GPL-3.0-or-later
//! Software-in-the-loop harness for the 500 Hz control loop.
//!
//! The core below uses only `core`: scripted inputs, a fixed-capacity trace and
//! a generic [`HarnessBoard`]. It drives the real [`step`], [`PhaseFsm`],
//! [`LinkState`] and [`Interlocks`]; nothing is reimplemented. The interlock is
//! inserted *inside* the actuation path by [`Guarded`], so a command produced
//! by the controller still has to pass the independent gate before it reaches
//! the board. Every command goes through [`Interlocks::evaluate_shield`], so
//! the Mach/pitot gate is in the harness actuation path too.
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
    control::{step_link, LinkState, ReasonCounts, ShieldSample, StepResult},
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
    /// Pitot dynamic pressure (Pa) this tick supplies to the Mach shield.
    pub pitot_dp: i16,
    /// Mach estimate this tick supplies to the Mach shield.
    pub mach_est: f32,
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
            pitot_dp: ShieldSample::LEGACY_NO_SENSOR.pitot_dp,
            mach_est: ShieldSample::LEGACY_NO_SENSOR.mach_est,
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
            pitot_dp: ShieldSample::LEGACY_NO_SENSOR.pitot_dp,
            mach_est: ShieldSample::LEGACY_NO_SENSOR.mach_est,
        }
    }

    /// Supply the pitot dynamic pressure and Mach estimate the shield gates on.
    /// Path-included into crates that do not script airdata, hence the allow.
    #[allow(dead_code)]
    pub const fn airdata(mut self, pitot_dp: i16, mach_est: f32) -> Self {
        self.pitot_dp = pitot_dp;
        self.mach_est = mach_est;
        self
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
    sample: ShieldSample,
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
    /// Per-reason tally of every shielded command in this run.
    pub shield_reasons: ReasonCounts,
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
                sample: ShieldSample::LEGACY_NO_SENSOR,
            },
            last_decision: Decision {
                command: Interlocks::FAILSAFE,
                permitted: false,
                refusal: None,
                limited: false,
            },
            last_requested: Command::ZERO,
            shield_reasons: ReasonCounts::EMPTY,
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
        let sample = self.pending.sample;
        let decision = self.gate.evaluate_shield(
            InterlockInput {
                phase_code: self.pending.phase_code,
                flags: self.pending.flags,
                ai_healthy: self.pending.ai_healthy,
                requested: cmd,
                now_us: self.pending.now_us,
            },
            sample.pitot_dp,
            sample.mach_est,
        );
        self.shield_reasons.note(decision.refusal);
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

    /// Per-reason tally of the harness shield; see `Guarded::shield_reasons`.
    #[allow(dead_code)] // path-included into crates that only read the trace
    pub fn shield_reasons(&self) -> ReasonCounts {
        self.board.shield_reasons
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
            sample: ShieldSample {
                pitot_dp: spec.pitot_dp,
                mach_est: spec.mach_est,
            },
        });

        // The harness owns the independent interlock in `Guarded`, so it drives
        // the ungated link tick; the binary uses `control::step` with its own
        // interlock and watchguard policy.
        let result = step_link(
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
        if let Some(latch_now) = self.gate().latched() {
            if Some(latch_now) != latch_before {
                trace.event(TraceEvent::InterlockLatched {
                    tick,
                    refusal: latch_now,
                });
            }
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

// ---------------------------------------------------------------------------
// Full shipped path: main.rs semantics minus pins.
//
// The harness above drives the *link* half through `step_link` with a
// decorator that places the interlock in the actuation path. That is a useful
// fault-injection microscope, but it is not the binary: `main.rs` calls
// `step()` (its own interlock, shield and watchdog policy) and, since the
// flight-path seam landed, `FlightPath::tick` before it.
//
// This module reproduces that loop body with scripted inputs and a MockBoard.
// Every stage is the real module: `ShieldSampler`, `FlightPath`, `step`, the
// `LogGate` cadences. No stage is reimplemented. A capability the mock cannot
// supply stays `None`; `FlightPath` and `ShieldSampler` count it as Unavailable
// with a reason, and the harness never substitutes a fabricated measurement.
//
// Host only. No pin, UART, clock, flash or flight claim is made here.
// ---------------------------------------------------------------------------

#[cfg(not(target_os = "none"))]
#[allow(dead_code)] // NoAirdata is the hardware stub; the harness scripts its own source.
#[path = "shield_sample.rs"]
pub mod airdata;

#[cfg(not(target_os = "none"))]
#[allow(dead_code)]
pub mod full {
    use super::{encode_reply, Command, Frame, HarnessBoard, PhaseEvent, TICK_HZ, TICK_PERIOD_US};
    use potok_m_firmware::{
        attitude::Quat,
        board::Board,
        control::{step, LinkState, ReasonCounts, ShieldSample, StepResult},
        deployment::SpringOutputs,
        interlocks::{Interlocks, Refusal},
        mock::MockBoard,
        phase::PhaseFsm,
        protocol::REPLY_SIZE,
        search_beacon::{RadioHint, SignalInputs},
        separation::Output as SeparationOutput,
        telemetry_log::{BoundedSink, Cadence, Emission, FlightLogger, LogGate},
        watchdog::{WatchdogPolicy, CONTROL_PERIOD_US},
        Capability, CapabilityLedger, DeploymentInput, FlightBoard, FlightPath, OutputError,
        SelfTestInput, SeparationInput, StabilizationSample, Unavailable as CapUnavailable,
        CAPABILITIES,
    };

    use super::airdata;
    use airdata::{Airdata, ShieldSampler, UnavailableCounts};

    /// Slot count of the capability ledger; pinned against the real table.
    const CAP_SLOTS: usize = 10;

    /// One tick's typed board inputs. `None` means the board does not own the
    /// capability this tick: the module records `Unavailable`, not success.
    #[derive(Clone, Copy, Debug)]
    pub struct FlightInputs {
        pub imu: Option<[Option<potok_m_firmware::sensor_voting::ImuSample>; 3]>,
        pub height: Option<[Option<f32>; 3]>,
        pub stabilization: Option<StabilizationSample>,
        pub deployment: Option<DeploymentInput>,
        pub separation: Option<SeparationInput>,
        pub self_test: Option<SelfTestInput>,
        pub signals: Option<SignalInputs>,
        /// `(pitot dynamic pressure Pa, static pressure Pa)`. `None` is a
        /// missing driver, counted by `ShieldSampler` as `NoDriver`.
        pub airdata: Option<(f32, f32)>,
    }

    impl FlightInputs {
        /// Every capability absent. Used to prove that a bare board is loud
        /// about what it cannot do instead of quietly reporting safe no-ops.
        pub const ABSENT: Self = Self {
            imu: None,
            height: None,
            stabilization: None,
            deployment: None,
            separation: None,
            self_test: None,
            signals: None,
            airdata: None,
        };
    }

    /// One scripted tick of the shipped loop: board link, phase event and the
    /// typed flight-path inputs consumed before `step()`.
    #[derive(Clone, Copy, Debug)]
    pub struct FlightTick {
        pub frame: Frame,
        pub phase_event: Option<PhaseEvent>,
        pub sensor_flags: u8,
        pub motor_armed: bool,
        pub cycle_complete: bool,
        pub inputs: FlightInputs,
    }

    impl FlightTick {
        pub const fn reply(pitch: i16, yaw: i16) -> Self {
            Self {
                frame: Frame::command(pitch, yaw),
                phase_event: None,
                sensor_flags: 0,
                motor_armed: false,
                cycle_complete: true,
                inputs: FlightInputs::ABSENT,
            }
        }

        pub const fn silent() -> Self {
            Self {
                frame: Frame::Silent,
                ..Self::reply(0, 0)
            }
        }

        pub fn corrupt(pitch: i16, yaw: i16, byte: usize, mask: u8) -> Self {
            Self {
                frame: Frame::corrupt(Frame::template(pitch, yaw), byte, mask),
                ..Self::reply(0, 0)
            }
        }

        pub fn stale(pitch: i16, yaw: i16) -> Self {
            Self {
                frame: Frame::stale(pitch, yaw),
                ..Self::reply(0, 0)
            }
        }

        pub fn unhealthy(pitch: i16, yaw: i16) -> Self {
            Self {
                frame: Frame::unhealthy(pitch, yaw),
                ..Self::reply(0, 0)
            }
        }

        pub const fn with(mut self, inputs: FlightInputs) -> Self {
            self.inputs = inputs;
            self
        }

        pub const fn event(mut self, event: PhaseEvent) -> Self {
            self.phase_event = Some(event);
            self
        }

        pub const fn flags(mut self, sensor_flags: u8) -> Self {
            self.sensor_flags = sensor_flags;
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
    }

    /// MockBoard plus the typed capability outputs the flight path writes.
    /// `Board` remains the only actuator gate; the four `apply_*` methods are
    /// separate, capability-scoped sinks that cannot reach `apply_command`.
    pub struct FlightMockBoard {
        inner: MockBoard,
        inputs: FlightInputs,
        /// Board-level capability debt, the same three gates the hardware
        /// adapter counts in `apply_command`, `motor_armed` and phase polling.
        pub board_caps: CapabilityLedger,
        /// Mirror the hardware adapter's unimplemented stubs: every board
        /// capability becomes a counted `Unavailable` instead of a silent no-op.
        stubbed: bool,
        last_applied: Option<Command>,
        pub stabilization_applied: Vec<Command>,
        pub springs_applied: Vec<SpringOutputs>,
        pub separation_applied: Vec<SeparationOutput>,
        pub radio_applied: Vec<RadioHint>,
    }

    impl FlightMockBoard {
        pub fn new() -> Self {
            Self {
                inner: MockBoard::new(),
                inputs: FlightInputs::ABSENT,
                board_caps: CapabilityLedger::default(),
                stubbed: false,
                last_applied: None,
                stabilization_applied: Vec::new(),
                springs_applied: Vec::new(),
                separation_applied: Vec::new(),
                radio_applied: Vec::new(),
            }
        }

        /// A board that owns none of the three `Board` capabilities, matching
        /// the shipping `HwBoard` stubs. Still records the command so the
        /// harness can observe what would have been discarded.
        pub fn stubbed() -> Self {
            Self {
                stubbed: true,
                ..Self::new()
            }
        }

        pub fn inner(&self) -> &MockBoard {
            &self.inner
        }

        pub fn inner_mut(&mut self) -> &mut MockBoard {
            &mut self.inner
        }

        /// Last command that crossed the board boundary, even for a stubbed
        /// board that would discard it on hardware.
        pub fn last_applied_command(&self) -> Command {
            self.last_applied
                .expect("sil: step always applies a command, even zero")
        }

        fn set_flight_inputs(&mut self, inputs: FlightInputs) {
            self.inputs = inputs;
        }
    }

    impl Default for FlightMockBoard {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Board for FlightMockBoard {
        type Rx = <MockBoard as Board>::Rx;
        type Tx = <MockBoard as Board>::Tx;
        type Wd = <MockBoard as Board>::Wd;

        fn rx(&mut self) -> &mut Self::Rx {
            <MockBoard as Board>::rx(&mut self.inner)
        }
        fn tx(&mut self) -> &mut Self::Tx {
            <MockBoard as Board>::tx(&mut self.inner)
        }
        fn wd(&mut self) -> &mut Self::Wd {
            <MockBoard as Board>::wd(&mut self.inner)
        }
        fn now_us(&self) -> u32 {
            self.inner.now_us()
        }
        fn sensor_flags(&self) -> u8 {
            self.inner.sensor_flags()
        }
        fn motor_armed(&self) -> bool {
            if self.stubbed {
                self.board_caps
                    .note_unavailable(Capability::MotorArmedSense, CapUnavailable::NoDriver);
                false
            } else {
                self.inner.motor_armed()
            }
        }
        fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
            if self.stubbed {
                self.board_caps
                    .note_unavailable(Capability::PhaseSense, CapUnavailable::NoDriver);
                None
            } else {
                self.inner.poll_phase_event()
            }
        }
        fn apply_command(&mut self, cmd: Command) {
            self.last_applied = Some(cmd);
            if self.stubbed {
                self.board_caps
                    .note_unavailable(Capability::Actuation, CapUnavailable::NoDriver);
            } else {
                self.inner.apply_command(cmd);
            }
        }
    }

    impl HarnessBoard for FlightMockBoard {
        fn set_now_us(&mut self, us: u32) {
            self.inner.now_us = us;
        }
        fn set_inputs(
            &mut self,
            sensor_flags: u8,
            motor_armed: bool,
            phase_event: Option<PhaseEvent>,
            cycle_complete: bool,
        ) {
            self.inner.sensor_flags = sensor_flags;
            self.inner.motor_armed = motor_armed;
            self.inner.phase_event = phase_event;
            self.inner.cycle_complete = cycle_complete;
        }
        fn inject(&mut self, bytes: &[u8]) {
            self.inner.rx_mut().queue(bytes);
        }
        fn last_applied(&self) -> Command {
            self.last_applied_command()
        }
        fn pet_count(&self) -> u32 {
            self.inner.pets()
        }
    }

    impl FlightBoard for FlightMockBoard {
        fn imu_readings(&self) -> Option<[Option<potok_m_firmware::sensor_voting::ImuSample>; 3]> {
            self.inputs.imu
        }
        fn height_readings(&self) -> Option<[Option<f32>; 3]> {
            self.inputs.height
        }
        fn stabilization(&self) -> Option<StabilizationSample> {
            self.inputs.stabilization
        }
        fn deployment(&self) -> Option<DeploymentInput> {
            self.inputs.deployment
        }
        fn separation(&self) -> Option<SeparationInput> {
            self.inputs.separation
        }
        fn self_test_input(&self) -> Option<SelfTestInput> {
            self.inputs.self_test
        }
        fn signal_inputs(&self) -> Option<SignalInputs> {
            self.inputs.signals
        }
        fn apply_stabilization(&mut self, cmd: Command) -> Result<(), OutputError> {
            self.stabilization_applied.push(cmd);
            Ok(())
        }
        fn apply_springs(&mut self, out: SpringOutputs) -> Result<(), OutputError> {
            self.springs_applied.push(out);
            Ok(())
        }
        fn apply_separation(&mut self, out: SeparationOutput) -> Result<(), OutputError> {
            self.separation_applied.push(out);
            Ok(())
        }
        fn apply_radio(&mut self, hint: RadioHint) -> Result<(), OutputError> {
            self.radio_applied.push(hint);
            Ok(())
        }
    }

    impl Airdata for FlightMockBoard {
        fn pitot_dp_pa(&self) -> Option<f32> {
            self.inputs.airdata.map(|(dp, _)| dp)
        }
        fn static_pa(&self) -> Option<f32> {
            self.inputs.airdata.map(|(_, p)| p)
        }
    }

    /// Per-capability totals copied out of the `Cell`-backed ledger so a
    /// record is `Copy` and two runs can be compared bit for bit.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct CapabilitySnapshot {
        pub used: [u32; CAP_SLOTS],
        pub unavailable: [u32; CAP_SLOTS],
    }

    impl CapabilitySnapshot {
        pub fn of(ledger: &CapabilityLedger) -> Self {
            let mut out = Self::default();
            for &cap in CAPABILITIES.iter() {
                let i = cap as usize;
                out.used[i] = ledger.used(cap);
                out.unavailable[i] = ledger.unavailable(cap);
            }
            out
        }
    }

    pub const EMIT_REJECT: u16 = 1 << 0;
    pub const EMIT_APPLIED: u16 = 1 << 1;
    pub const EMIT_AIRDATA: u16 = 1 << 2;
    pub const EMIT_AI_OFF: u16 = 1 << 3;
    pub const EMIT_INTERLOCK_OFF: u16 = 1 << 4;
    pub const EMIT_DEADMAN: u16 = 1 << 5;
    pub const EMIT_CAPABILITY: u16 = 1 << 6;

    /// Serialized telemetry frame size: 13-byte header + payload + 2-byte CRC.
    pub const LOG_SZ: usize = 24;
    /// Retained records in the harness ring.
    pub const LOG_RING: usize = 16;
    /// Sink bytes: enough for several full ring flushes before a refusal.
    pub const LOG_SINK_BYTES: usize = 64 * LOG_SZ;

    /// Record tags, in emission order. Stable telemetry data.
    pub const TAG_REJECT: u8 = 1;
    pub const TAG_APPLIED: u8 = 2;
    pub const TAG_AIRDATA: u8 = 3;
    pub const TAG_AI_OFF: u8 = 4;
    pub const TAG_INTERLOCK_OFF: u8 = 5;
    pub const TAG_DEADMAN: u8 = 6;
    pub const TAG_CAPABILITY: u8 = 7;

    /// One recorded tick of the full shipped path.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct FlightRecord {
        pub tick: u32,
        pub now_us: u32,
        pub result: StepResult,
        pub applied: Command,
        pub phase: u8,
        pub shield: ShieldSample,
        pub shield_reasons: ReasonCounts,
        pub interlock_latched: Option<Refusal>,
        pub ai_latched: bool,
        pub deadman_tripped: bool,
        pub pets: u32,
        pub emissions: u16,
        pub capabilities: CapabilitySnapshot,
        pub unavailable: UnavailableCounts,
        pub attitude: Option<Quat>,
        pub stabilization: Option<Command>,
        pub beacon_tick: u32,
    }

    /// Fixed-capacity trace; overflow panics rather than dropping a tick.
    #[derive(Clone, Debug, PartialEq)]
    pub struct FlightTrace<const CAP: usize> {
        records: [FlightRecord; CAP],
        len: usize,
    }

    impl<const CAP: usize> FlightTrace<CAP> {
        pub fn new() -> Self {
            assert!(CAP > 0, "sil: flight trace capacity must be positive");
            Self {
                records: core::array::from_fn(|_| FlightRecord {
                    tick: 0,
                    now_us: 0,
                    result: StepResult::Idle,
                    applied: Command::ZERO,
                    phase: 0,
                    shield: ShieldSample::UNAVAILABLE,
                    shield_reasons: ReasonCounts::EMPTY,
                    interlock_latched: None,
                    ai_latched: false,
                    deadman_tripped: false,
                    pets: 0,
                    emissions: 0,
                    capabilities: CapabilitySnapshot::default(),
                    unavailable: UnavailableCounts::default(),
                    attitude: None,
                    stabilization: None,
                    beacon_tick: 0,
                }),
                len: 0,
            }
        }

        pub fn push(&mut self, record: FlightRecord) {
            assert!(self.len < CAP, "sil: flight trace capacity exceeded");
            self.records[self.len] = record;
            self.len += 1;
        }

        pub fn records(&self) -> &[FlightRecord] {
            &self.records[..self.len]
        }

        pub fn len(&self) -> usize {
            self.len
        }
    }

    impl<const CAP: usize> Default for FlightTrace<CAP> {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Qualitative airdata key, identical to the one the binary logs: reason
    /// bits, never growing tallies.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct AirdataState {
        shield: u8,
        airdata: u8,
    }

    struct FlightGates {
        reject: LogGate<()>,
        applied: LogGate<()>,
        airdata: LogGate<AirdataState>,
        ai_off: LogGate<bool>,
        interlock_off: LogGate<bool>,
        deadman: LogGate<bool>,
        capability: LogGate<()>,
    }

    impl FlightGates {
        fn new() -> Self {
            let periodic = Cadence::periodic(TICK_HZ);
            Self {
                reject: LogGate::new(periodic),
                applied: LogGate::new(periodic),
                airdata: LogGate::new(periodic),
                ai_off: LogGate::new(Cadence::on_change()),
                interlock_off: LogGate::new(Cadence::on_change()),
                deadman: LogGate::new(Cadence::on_change()),
                capability: LogGate::new(periodic),
            }
        }
    }

    fn latch_entered(gate: &mut LogGate<bool>, latched: bool) -> bool {
        latched && matches!(gate.observe_error(latched), Emission::Safety)
    }

    /// Stage-reachability bits. Set only from an observation made that tick:
    /// a module whose output never moved cannot mark itself reached.
    pub const REACH_LINK: u32 = 1 << 0;
    pub const REACH_PHASE: u32 = 1 << 1;
    pub const REACH_VOTING: u32 = 1 << 2;
    pub const REACH_MAHONY: u32 = 1 << 3;
    pub const REACH_PID: u32 = 1 << 4;
    pub const REACH_DEPLOYMENT: u32 = 1 << 5;
    pub const REACH_SEPARATION: u32 = 1 << 6;
    pub const REACH_SELF_TEST: u32 = 1 << 7;
    pub const REACH_RADIO: u32 = 1 << 8;
    pub const REACH_INTERLOCK: u32 = 1 << 9;
    pub const REACH_SHIELD: u32 = 1 << 10;
    pub const REACH_APPLY: u32 = 1 << 11;
    pub const REACH_WATCHDOG: u32 = 1 << 12;
    pub const REACH_TELEMETRY: u32 = 1 << 13;
    pub const REACH_AIRDATA: u32 = 1 << 14;
    pub const REACH_ALL: u32 = (1 << 15) - 1;

    /// Runs the `main.rs` loop body with scripted board inputs.
    pub struct FlightSimulator {
        pub board: FlightMockBoard,
        pub fsm: PhaseFsm,
        pub link: LinkState,
        pub interlock: Interlocks,
        pub policy: WatchdogPolicy,
        pub flight: FlightPath,
        pub sampler: ShieldSampler,
        gates: FlightGates,
        logger: FlightLogger<LOG_RING, LOG_SZ>,
        sink: BoundedSink<LOG_SINK_BYTES>,
        flushed: u64,
        flush_refusals: u64,
        tick_index: u32,
        reached: u32,
    }

    impl FlightSimulator {
        pub fn new() -> Self {
            assert_eq!(
                CAPABILITIES.len(),
                CAP_SLOTS,
                "sil: capability ledger layout changed"
            );
            Self {
                board: FlightMockBoard::new(),
                fsm: PhaseFsm::new(),
                link: LinkState::new(),
                interlock: Interlocks::default(),
                policy: WatchdogPolicy::new(),
                flight: FlightPath::new(),
                sampler: ShieldSampler::new(),
                gates: FlightGates::new(),
                logger: FlightLogger::new(),
                sink: BoundedSink::new(),
                flushed: 0,
                flush_refusals: 0,
                tick_index: 0,
                reached: 0,
            }
        }

        /// A simulator whose `Board` capabilities match the unimplemented
        /// hardware stubs: every absence is counted, never skipped.
        pub fn stubbed() -> Self {
            Self {
                board: FlightMockBoard::stubbed(),
                ..Self::new()
            }
        }

        pub fn reached(&self) -> u32 {
            self.reached
        }

        pub fn logger(&self) -> &FlightLogger<LOG_RING, LOG_SZ> {
            &self.logger
        }

        pub fn sink(&self) -> &BoundedSink<LOG_SINK_BYTES> {
            &self.sink
        }

        pub fn flushed(&self) -> u64 {
            self.flushed
        }

        /// Flush batches the bounded sink refused to take. Counted, never silent.
        pub fn flush_refusals(&self) -> u64 {
            self.flush_refusals
        }

        fn note(&mut self, bit: u32, observed: bool) {
            if observed {
                self.reached |= bit;
            }
        }

        /// One framed record per emitted cadence, then a bounded flush.
        /// A full sink is counted, not dropped silently: the frames stay in
        /// the logger's ring for a later retry.
        fn log(&mut self, now_us: u32, phase: u8, emissions: u16) {
            for (bit, tag) in [
                (EMIT_REJECT, TAG_REJECT),
                (EMIT_APPLIED, TAG_APPLIED),
                (EMIT_AIRDATA, TAG_AIRDATA),
                (EMIT_AI_OFF, TAG_AI_OFF),
                (EMIT_INTERLOCK_OFF, TAG_INTERLOCK_OFF),
                (EMIT_DEADMAN, TAG_DEADMAN),
                (EMIT_CAPABILITY, TAG_CAPABILITY),
            ] {
                if emissions & bit != 0 {
                    self.logger
                        .push(now_us, tag, 0, &[phase, tag])
                        .expect("sil: telemetry frame must encode");
                }
            }
            match self.logger.flush::<4, _>(&mut self.sink) {
                Ok(n) => self.flushed += n as u64,
                Err(_) => self.flush_refusals += 1,
            }
        }

        fn script_frame(board: &mut FlightMockBoard, frame: Frame, expected_seq: u16) {
            let mut bytes = [0u8; REPLY_SIZE];
            let mut len = REPLY_SIZE;
            match frame {
                Frame::Silent => return,
                Frame::Reply(mut reply) => {
                    reply.seq = expected_seq;
                    encode_reply(&reply, &mut bytes).expect("sil: scripted reply must encode");
                }
                Frame::Stale(mut reply) => {
                    reply.seq = expected_seq.wrapping_add(1);
                    encode_reply(&reply, &mut bytes).expect("sil: scripted reply must encode");
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
                }
                Frame::Raw { bytes: raw, len: raw_len } => {
                    assert!(raw_len <= REPLY_SIZE, "sil: raw frame longer than a reply");
                    bytes[..raw_len].copy_from_slice(&raw[..raw_len]);
                    len = raw_len;
                }
            }
            board.inject(&bytes[..len]);
        }

        /// Drive one 500 Hz tick in the binary's order: sample airdata, run
        /// the flight-path modules, then the shipped `step()` gate, then the
        /// telemetry cadences.
        pub fn tick<const CAP: usize>(&mut self, spec: &FlightTick, trace: &mut FlightTrace<CAP>) {
            let tick = self.tick_index;
            let now_us = tick.wrapping_mul(TICK_PERIOD_US);
            let expected_seq = self
                .link
                .last_request_seq()
                .map_or(0, |last| last.wrapping_add(1));

            self.board.set_now_us(now_us);
            self.board.set_inputs(
                spec.sensor_flags,
                spec.motor_armed,
                spec.phase_event,
                spec.cycle_complete,
            );
            self.board.set_flight_inputs(spec.inputs);
            Self::script_frame(&mut self.board, spec.frame, expected_seq);

            // main.rs: the shield sample and the flight-path pass both happen
            // before `step()`, and the FSM phase is the one `step()` will see.
            let shield = self.sampler.sample(&self.board);
            self.link.set_shield_sample(shield);
            self.flight
                .tick(&mut self.board, self.fsm.phase(), now_us, CONTROL_PERIOD_US);
            let result = step(
                &mut self.board,
                &mut self.fsm,
                &mut self.link,
                &mut self.interlock,
                &mut self.policy,
                spec.cycle_complete,
            );

            let mut emissions = 0u16;
            match result {
                StepResult::FrameRejected => {
                    if self.gates.reject.observe(()).should_emit() {
                        emissions |= EMIT_REJECT;
                    }
                }
                StepResult::Accepted(cmd) if cmd.is_zero() => {
                    let u = self.sampler.unavailable_counts();
                    let bit = |count: u32| (count > 0) as u8;
                    let r = self.link.shield_reasons;
                    let key = AirdataState {
                        shield: bit(r.mach_limit)
                            | (bit(r.pitot_invalid) << 1)
                            | (bit(r.non_finite_mach) << 2),
                        airdata: bit(u.no_driver)
                            | (bit(u.non_finite) << 1)
                            | (bit(u.non_positive) << 2),
                    };
                    if self.gates.airdata.observe(key).should_emit() {
                        emissions |= EMIT_AIRDATA;
                    }
                }
                StepResult::Accepted(_) => {
                    if self.gates.applied.observe(()).should_emit() {
                        emissions |= EMIT_APPLIED;
                    }
                }
                StepResult::Idle => {}
            }
            if latch_entered(
                &mut self.gates.ai_off,
                self.link.is_disabled() || self.policy.ai_latched(),
            ) {
                emissions |= EMIT_AI_OFF;
            }
            if latch_entered(&mut self.gates.interlock_off, self.interlock.latched().is_some()) {
                emissions |= EMIT_INTERLOCK_OFF;
            }
            if latch_entered(&mut self.gates.deadman, self.policy.deadman().tripped()) {
                emissions |= EMIT_DEADMAN;
            }
            if self.gates.capability.observe(()).should_emit() {
                emissions |= EMIT_CAPABILITY;
            }

            let capabilities = CapabilitySnapshot::of(self.flight.ledger());
            let record = FlightRecord {
                tick,
                now_us,
                result,
                applied: self.board.last_applied(),
                phase: self.fsm.phase() as u8,
                shield,
                shield_reasons: self.link.shield_reasons,
                interlock_latched: self.interlock.latched(),
                ai_latched: self.policy.ai_latched(),
                deadman_tripped: self.policy.deadman().tripped(),
                pets: self.board.pet_count(),
                emissions,
                capabilities,
                unavailable: self.sampler.unavailable_counts(),
                attitude: self.flight.attitude(),
                stabilization: self.flight.stabilization(),
                beacon_tick: self.flight.beacon_tick(),
            };

            self.log(now_us, record.phase, emissions);

            let u = record.unavailable;
            let r = record.shield_reasons;
            let shield_seen = r.accepted
                + r.unknown_phase
                + r.boost
                + r.motor_armed
                + r.ai_unhealthy
                + r.fault
                + r.implausible
                + r.non_finite_mach
                + r.pitot_invalid
                + r.mach_limit;
            self.note(
                REACH_LINK,
                self.board.inner().sent_frames().len() == tick as usize + 1,
            );
            self.note(REACH_PHASE, spec.phase_event.is_some());
            self.note(
                REACH_VOTING,
                capabilities.used[Capability::SensorVoting as usize] > 0,
            );
            self.note(
                REACH_MAHONY,
                capabilities.used[Capability::Mahony as usize] > 0,
            );
            self.note(REACH_PID, capabilities.used[Capability::Pid as usize] > 0);
            self.note(
                REACH_DEPLOYMENT,
                capabilities.used[Capability::Deployment as usize] > 0,
            );
            self.note(
                REACH_SEPARATION,
                capabilities.used[Capability::Separation as usize] > 0,
            );
            self.note(
                REACH_SELF_TEST,
                capabilities.used[Capability::SelfTest as usize] > 0,
            );
            self.note(REACH_RADIO, capabilities.used[Capability::Radio as usize] > 0);
            self.note(
                REACH_INTERLOCK,
                self.interlock.counters.accepted
                    + self.interlock.counters.blocked
                    + self.interlock.counters.trips
                    > 0,
            );
            self.note(REACH_SHIELD, shield_seen > 0);
            self.note(REACH_APPLY, self.board.last_applied.is_some());
            self.note(REACH_WATCHDOG, record.pets > 0);
            self.note(REACH_TELEMETRY, emissions != 0);
            self.note(
                REACH_AIRDATA,
                shield.pitot_dp > 0 || u.no_driver + u.non_finite + u.non_positive > 0,
            );

            trace.push(record);
            self.tick_index = self.tick_index.wrapping_add(1);
        }

        pub fn run<const CAP: usize>(&mut self, ticks: &[FlightTick], trace: &mut FlightTrace<CAP>) {
            for tick in ticks {
                self.tick(tick, trace);
            }
        }
    }

    impl Default for FlightSimulator {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Build a simulator, run a script and return it with the trace filled.
    pub fn simulate_full<const CAP: usize>(
        ticks: &[FlightTick],
        trace: &mut FlightTrace<CAP>,
    ) -> FlightSimulator {
        let mut sim = FlightSimulator::new();
        sim.run(ticks, trace);
        sim
    }
}
