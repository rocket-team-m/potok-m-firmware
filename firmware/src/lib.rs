// SPDX-License-Identifier: GPL-3.0-or-later
//! Potok-M STM32H743 firmware: host-testable safety and link logic.
//!
//! Hardware calls live behind the [`board::Board`] trait. The binary in
//! `main.rs` is a thin adapter; `cargo test` runs the same code with mocks.

#![no_std]

#[cfg(not(target_os = "none"))]
extern crate std;

pub mod attitude;
pub mod board;
pub mod buzzer;
pub mod calibration_table;
pub mod control;
pub mod control_pid;
pub mod deadman;
pub mod deployment;
pub mod interlocks;
#[cfg(not(target_os = "none"))]
pub mod mock;
pub mod perf;
pub mod phase;
pub mod phase_fsm;
pub mod protocol;
pub mod protocol_link;
pub mod scheduler;
pub mod search_beacon;
pub mod self_test;
pub mod sensor_voting;
pub mod separation;
pub mod telemetry_log;
pub mod uart;
pub mod watchdog;

pub use control::{step, step_link, Counters, LinkState, StepResult};
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

// ---------------------------------------------------------------------------
// Flight path: typed capability gates over the pure decision modules.
//
// Every module in this crate is testable but only `step()` had a production
// call site. This seam gives each module exactly one caller. A board supplies
// the typed input it actually owns; a missing input is a counted `Unavailable`
// with a reason, never a silent discard, a fabricated zero or a faked pass.
// The existing `step()` remains the only gate in front of `Board::apply_command`.
// ---------------------------------------------------------------------------

use core::cell::Cell;

use potok_protocol::Phase;

use crate::{
    attitude::{Mahony, Quat, Vec3, RATE, UNIT},
    control_pid::{command_from_raw, Fx, Pid, PidConfig},
    deployment::{
        ArmChannels as DeployArm, Decision as DeployDecision, DeploymentController,
        DeploymentSample, SpringOutputs,
    },
    search_beacon::{RadioHint, SearchBeacon, SignalInputs},
    self_test::{
        Inputs as SelfTestRawInputs, Progress as SelfTestProgress, SelfTest as SelfTestRunner,
    },
    sensor_voting::{Config as VoteConfig, ImuSample, Tolerance, VotingChannel, SENSOR_COUNT},
    separation::{
        ArmChannels as SeparationArm, Output as SeparationOutput, PhaseReading,
        Plan as SeparationPlan, Sample as SeparationSample, SensorPair, Sequencer,
        State as SeparationState,
    },
};

/// One flight-path capability. Presence is declared by the typed methods on
/// [`FlightBoard`]; absence is a counted [`Unavailable`] with a reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Capability {
    SensorVoting,
    Mahony,
    Pid,
    Deployment,
    Separation,
    SelfTest,
    Radio,
    Actuation,
    PhaseSense,
    MotorArmedSense,
}

/// All capabilities in ledger order. The order is stable telemetry data.
pub const CAPABILITIES: [Capability; 10] = [
    Capability::SensorVoting,
    Capability::Mahony,
    Capability::Pid,
    Capability::Deployment,
    Capability::Separation,
    Capability::SelfTest,
    Capability::Radio,
    Capability::Actuation,
    Capability::PhaseSense,
    Capability::MotorArmedSense,
];

const CAP_COUNT: usize = CAPABILITIES.len();

/// Why a capability did not run or did not reach an output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Unavailable {
    /// No driver or output path is wired on this board.
    NoDriver,
    /// The module's input was absent this tick.
    NoSample,
    /// The module refused the sample: degenerate data, wrong phase, incomplete
    /// arm, or a latched fault.
    Refused,
    /// The board has the output path but rejected the write.
    OutputFailed,
}

impl From<OutputError> for Unavailable {
    fn from(err: OutputError) -> Self {
        match err {
            OutputError::NoDriver => Self::NoDriver,
            OutputError::Rejected => Self::OutputFailed,
        }
    }
}

/// Per-capability tally. `Cell` keeps the `&self` stubs on the hardware board
/// (`motor_armed`, `apply_command`) countable without a mutex or a global.
#[derive(Debug, Default)]
pub struct CapabilityLedger {
    used: [Cell<u32>; CAP_COUNT],
    unavailable: [Cell<u32>; CAP_COUNT],
    last_reason: [Cell<Option<Unavailable>>; CAP_COUNT],
}

impl CapabilityLedger {
    pub fn note_used(&self, cap: Capability) {
        let i = cap as usize;
        self.used[i].set(self.used[i].get().saturating_add(1));
    }

    pub fn note_unavailable(&self, cap: Capability, reason: Unavailable) {
        let i = cap as usize;
        self.unavailable[i].set(self.unavailable[i].get().saturating_add(1));
        self.last_reason[i].set(Some(reason));
    }

    pub fn used(&self, cap: Capability) -> u32 {
        self.used[cap as usize].get()
    }

    pub fn unavailable(&self, cap: Capability) -> u32 {
        self.unavailable[cap as usize].get()
    }

    pub fn last_reason(&self, cap: Capability) -> Option<Unavailable> {
        self.last_reason[cap as usize].get()
    }

    pub fn total_unavailable(&self) -> u32 {
        CAPABILITIES.iter().map(|&cap| self.unavailable(cap)).sum()
    }
}

/// One axis of a stabilization sample. Both fields are raw 0.001 rad counts,
/// the same unit as [`Command`]. The board owns the conversion from its
/// attitude estimate to `measurement`; this module does not invent a scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisSample {
    pub setpoint: i32,
    pub measurement: i32,
}

/// PID inputs plus the gains to use. Gains live with the board so no flight
/// tuning constant is fabricated in the library.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StabilizationSample {
    pub pitch: AxisSample,
    pub yaw: AxisSample,
    pub pitch_config: PidConfig<Fx>,
    pub yaw_config: PidConfig<Fx>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeploymentInput {
    pub arm: DeployArm,
    pub sample: DeploymentSample,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeparationInput {
    pub plan: SeparationPlan,
    pub arm: SeparationArm,
    pub phase: [PhaseReading; 2],
    pub sensors: SensorPair,
}

/// Owned preflight observations. The runner borrows no tables, so its state
/// can live for the whole flight without a lifetime parameter on `FlightPath`.
#[derive(Clone, Copy, Debug)]
pub struct SelfTestInput {
    pub sensor: crate::self_test::SensorSample,
    pub limits: crate::self_test::SensorLimits,
    pub config: crate::self_test::ConfigImage,
    pub config_crc: u16,
    pub continuity: crate::self_test::ContinuityMock,
    pub latch_state: crate::self_test::LatchState,
    pub ticker: crate::self_test::Ticker,
}

/// An output the board claimed but could not perform.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutputError {
    NoDriver,
    Rejected,
}

/// The typed capability seam. Every method defaults to *absent*: a board opts
/// in only for what it actually owns, and each absent method is counted by
/// [`FlightPath::tick`] instead of being treated as a successful no-op.
/// No pin, bus, peripheral or radio channel is named here.
pub trait FlightBoard {
    fn imu_readings(&self) -> Option<[Option<ImuSample>; SENSOR_COUNT]> {
        None
    }
    fn height_readings(&self) -> Option<[Option<f32>; SENSOR_COUNT]> {
        None
    }
    fn stabilization(&self) -> Option<StabilizationSample> {
        None
    }
    fn deployment(&self) -> Option<DeploymentInput> {
        None
    }
    fn separation(&self) -> Option<SeparationInput> {
        None
    }
    fn self_test_input(&self) -> Option<SelfTestInput> {
        None
    }
    fn signal_inputs(&self) -> Option<SignalInputs> {
        None
    }

    fn apply_stabilization(&mut self, _cmd: Command) -> Result<(), OutputError> {
        Err(OutputError::NoDriver)
    }
    fn apply_springs(&mut self, _out: SpringOutputs) -> Result<(), OutputError> {
        Err(OutputError::NoDriver)
    }
    fn apply_separation(&mut self, _out: SeparationOutput) -> Result<(), OutputError> {
        Err(OutputError::NoDriver)
    }
    fn apply_radio(&mut self, _hint: RadioHint) -> Result<(), OutputError> {
        Err(OutputError::NoDriver)
    }
}

/// The two PIDs and the configs they were built from. A config change is a
/// ground action: it rebuilds the controllers instead of carrying integral
/// state across a different control law.
struct PidPair {
    pitch_cfg: PidConfig<Fx>,
    yaw_cfg: PidConfig<Fx>,
    pitch: Pid<Fx>,
    yaw: Pid<Fx>,
}

fn rate_vec(v: [f32; 3]) -> Vec3 {
    let scale = RATE as f32;
    Vec3::new(
        (v[0] * scale) as i32,
        (v[1] * scale) as i32,
        (v[2] * scale) as i32,
    )
}

fn unit_vec(v: [f32; 3]) -> Vec3 {
    let scale = UNIT as f32;
    Vec3::new(
        (v[0] * scale) as i32,
        (v[1] * scale) as i32,
        (v[2] * scale) as i32,
    )
}

/// State of all flight-path modules. One instance lives in the shipping loop.
/// It never calls a peripheral directly and never changes `step()`'s gates.
pub struct FlightPath {
    mahony: Mahony,
    imu_vote: VotingChannel<ImuSample>,
    height_vote: VotingChannel<f32>,
    deployment: DeploymentController,
    separation: Option<Sequencer>,
    self_test: Option<SelfTestRunner<'static>>,
    self_test_report: Option<crate::self_test::SelfTestReport>,
    last_pid: Option<PidPair>,
    beacon: SearchBeacon,
    ledger: CapabilityLedger,
    attitude: Option<Quat>,
    height_m: Option<f32>,
    stabilization: Option<Command>,
}

impl FlightPath {
    pub fn new() -> Self {
        Self {
            mahony: Mahony::default(),
            imu_vote: VotingChannel::new(VoteConfig::DEFAULT, Tolerance::new(0.05, 0.02)),
            height_vote: VotingChannel::new(VoteConfig::DEFAULT, Tolerance::new(0.5, 0.02)),
            deployment: DeploymentController::default(),
            separation: None,
            self_test: None,
            self_test_report: None,
            last_pid: None,
            beacon: SearchBeacon::new(),
            ledger: CapabilityLedger::default(),
            attitude: None,
            height_m: None,
            stabilization: None,
        }
    }

    /// One bounded pass over every module. Inputs and outputs are gated
    /// independently; a missing capability counts once per tick and cannot
    /// masquerade as a module that ran and produced a safe value.
    pub fn tick<B: FlightBoard>(&mut self, board: &mut B, phase: Phase, now_us: u32, dt_us: u32) {
        self.tick_self_test(board, now_us);
        self.tick_sensors(board, dt_us);
        self.tick_stabilization(board);
        self.tick_deployment(board, phase);
        self.tick_separation(board, now_us);
        self.tick_radio(board);
    }

    fn tick_self_test<B: FlightBoard>(&mut self, board: &B, now_us: u32) {
        let Some(input) = board.self_test_input() else {
            self.ledger
                .note_unavailable(Capability::SelfTest, Unavailable::NoDriver);
            return;
        };
        let runner = self.self_test.get_or_insert_with(|| {
            SelfTestRunner::new(SelfTestRawInputs {
                sensor: input.sensor,
                limits: input.limits,
                tables: &[],
                config: input.config,
                config_crc: input.config_crc,
                continuity: input.continuity,
                latch_state: input.latch_state,
                ticker: input.ticker,
            })
        });
        if runner.step(now_us) == SelfTestProgress::Done && self.self_test_report.is_none() {
            self.self_test_report = Some(runner.report());
            self.ledger.note_used(Capability::SelfTest);
        }
    }

    fn tick_sensors<B: FlightBoard>(&mut self, board: &B, dt_us: u32) {
        let imu_readings = board.imu_readings();
        let height_readings = board.height_readings();
        if imu_readings.is_none() && height_readings.is_none() {
            self.ledger
                .note_unavailable(Capability::SensorVoting, Unavailable::NoDriver);
        }
        let mut imu = None;
        let mut voted = false;
        let mut refused = false;
        if let Some(readings) = imu_readings {
            match self.imu_vote.vote(readings) {
                Some(vote) => {
                    voted = true;
                    imu = Some(vote.value);
                }
                None => refused = true,
            }
        }
        if let Some(readings) = height_readings {
            match self.height_vote.vote(readings) {
                Some(vote) => {
                    voted = true;
                    self.height_m = Some(vote.value);
                }
                None => refused = true,
            }
        }
        if voted {
            self.ledger.note_used(Capability::SensorVoting);
        } else if refused || imu_readings.is_some() || height_readings.is_some() {
            self.ledger
                .note_unavailable(Capability::SensorVoting, Unavailable::NoSample);
        }

        let mahony_reason = if imu_readings.is_none() {
            Some(Unavailable::NoDriver)
        } else if imu.is_none() || dt_us == 0 {
            Some(Unavailable::NoSample)
        } else {
            None
        };
        if let Some(reason) = mahony_reason {
            self.ledger.note_unavailable(Capability::Mahony, reason);
        } else if let Some(sample) = imu {
            match self.mahony.update(
                rate_vec(sample.gyro),
                Some(unit_vec(sample.accel)),
                None,
                dt_us,
            ) {
                Ok(()) => {
                    self.attitude = Some(self.mahony.quaternion());
                    self.ledger.note_used(Capability::Mahony);
                }
                Err(_) => self
                    .ledger
                    .note_unavailable(Capability::Mahony, Unavailable::Refused),
            }
        }
    }

    fn tick_stabilization<B: FlightBoard>(&mut self, board: &mut B) {
        let Some(input) = board.stabilization() else {
            self.ledger
                .note_unavailable(Capability::Pid, Unavailable::NoDriver);
            return;
        };
        let changed = self.last_pid.as_ref().is_none_or(|pair| {
            pair.pitch_cfg != input.pitch_config || pair.yaw_cfg != input.yaw_config
        });
        if changed {
            self.last_pid = Some(PidPair {
                pitch_cfg: input.pitch_config,
                yaw_cfg: input.yaw_config,
                pitch: Pid::new(input.pitch_config),
                yaw: Pid::new(input.yaw_config),
            });
        }
        let pair = match self.last_pid.as_mut() {
            Some(pair) => pair,
            None => return,
        };
        let pitch = pair.pitch.update(
            Fx::from_raw(input.pitch.setpoint),
            Fx::from_raw(input.pitch.measurement),
        );
        let yaw = pair.yaw.update(
            Fx::from_raw(input.yaw.setpoint),
            Fx::from_raw(input.yaw.measurement),
        );
        let cmd = command_from_raw(pitch.to_raw(), yaw.to_raw());
        match board.apply_stabilization(cmd) {
            Ok(()) => {
                self.stabilization = Some(cmd);
                self.ledger.note_used(Capability::Pid);
            }
            Err(err) => self.ledger.note_unavailable(Capability::Pid, err.into()),
        }
    }

    fn tick_deployment<B: FlightBoard>(&mut self, board: &mut B, phase: Phase) {
        let Some(input) = board.deployment() else {
            self.ledger
                .note_unavailable(Capability::Deployment, Unavailable::NoDriver);
            return;
        };
        match self.deployment.update(phase, input.arm, input.sample) {
            DeployDecision::Hold => self.ledger.note_used(Capability::Deployment),
            DeployDecision::Deploy(_) => match board.apply_springs(self.deployment.outputs()) {
                Ok(()) => self.ledger.note_used(Capability::Deployment),
                Err(err) => self
                    .ledger
                    .note_unavailable(Capability::Deployment, err.into()),
            },
            DeployDecision::Refused(_) => self
                .ledger
                .note_unavailable(Capability::Deployment, Unavailable::Refused),
        }
    }

    fn tick_separation<B: FlightBoard>(&mut self, board: &mut B, now_us: u32) {
        let Some(input) = board.separation() else {
            self.ledger
                .note_unavailable(Capability::Separation, Unavailable::NoDriver);
            return;
        };
        let seq = self
            .separation
            .get_or_insert_with(|| Sequencer::new(input.plan));
        if seq.state() == SeparationState::Idle {
            let _ = seq.arm(input.arm);
        }
        let sample = SeparationSample {
            now_us: u64::from(now_us),
            phase: input.phase,
            sensors: input.sensors,
        };
        match seq.tick(sample) {
            Ok(SeparationOutput::None) => self.ledger.note_used(Capability::Separation),
            Ok(out) => match board.apply_separation(out) {
                Ok(()) => self.ledger.note_used(Capability::Separation),
                Err(err) => self
                    .ledger
                    .note_unavailable(Capability::Separation, err.into()),
            },
            Err(_) => self
                .ledger
                .note_unavailable(Capability::Separation, Unavailable::Refused),
        }
    }

    fn tick_radio<B: FlightBoard>(&mut self, board: &mut B) {
        let Some(inputs) = board.signal_inputs() else {
            self.ledger
                .note_unavailable(Capability::Radio, Unavailable::NoDriver);
            return;
        };
        let hint = self.beacon.tick(inputs).radio;
        match board.apply_radio(hint) {
            Ok(()) => self.ledger.note_used(Capability::Radio),
            Err(err) => self.ledger.note_unavailable(Capability::Radio, err.into()),
        }
    }

    pub const fn ledger(&self) -> &CapabilityLedger {
        &self.ledger
    }
    pub const fn attitude(&self) -> Option<Quat> {
        self.attitude
    }
    pub const fn height_m(&self) -> Option<f32> {
        self.height_m
    }
    pub const fn stabilization(&self) -> Option<Command> {
        self.stabilization
    }
    pub fn separation(&self) -> Option<&Sequencer> {
        self.separation.as_ref()
    }
    pub const fn self_test_report(&self) -> Option<crate::self_test::SelfTestReport> {
        self.self_test_report
    }
    pub const fn beacon_tick(&self) -> u32 {
        self.beacon.tick_count()
    }
}

impl Default for FlightPath {
    fn default() -> Self {
        Self::new()
    }
}
