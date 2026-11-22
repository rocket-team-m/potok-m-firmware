// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic, hardware-free preflight self-test.
//!
//! The suite checks six things a flight controller can verify before it arms:
//! sensor readings against configured ranges, CRCs of calibration tables and
//! of the configuration image, actuator-path continuity through a mock, the
//! phase latch's safety invariants, and the non-blocking tick helper used to
//! drive the suite itself. Every result carries a human-readable reason.
//!
//! Nothing here owns a peripheral, a pin or a clock. The caller supplies all
//! observations in [`Inputs`], and drives the suite by calling [`SelfTest::step`]
//! with a monotonic `now_us`. A step does a bounded amount of work and returns;
//! the suite never waits, spins or allocates, so it can run inside the same
//! cooperative executor as the control loop.
//!
//! Determinism: results depend only on [`Inputs`] and the number of steps
//! taken. `now_us` is recorded in the report but never decides a verdict. The
//! same inputs always yield the same verdicts in the same order, which is what
//! makes a preflight log comparable across boots.
//!
//! Wiring into the crate root is owned by the parent task; this file declares
//! no `mod` entries elsewhere.

use core::fmt::{Display, Formatter, Result as FmtResult};

use potok_protocol::crc16;

/// Number of independent checks in one preflight run.
pub const CHECK_COUNT: usize = 6;
/// Maximum number of calibration tables a single run inspects.
pub const MAX_TABLES: usize = 4;
/// Maximum bytes in one calibration table. Bounds the work of one step.
pub const MAX_TABLE_LEN: usize = 256;
/// Consecutive good continuity samples required per actuator channel.
pub const CONTINUITY_SAMPLES: u8 = 4;
/// Misses that trip the phase latch.
pub const LATCH_MISSES: u8 = 3;
/// Byte length of the serialized configuration image.
pub const CONFIG_LEN: usize = 8;

/// Golden configuration values shipped with this firmware revision.
pub const GOLDEN_CONFIG: ConfigImage = ConfigImage {
    version: 1,
    model_id: 0x4D50,
    pitch_gain: 100,
    yaw_gain: 80,
};
/// CRC-16/CCITT-FALSE of [`GOLDEN_CONFIG`] serialized little-endian by
/// [`ConfigImage::to_bytes`]. Computed offline from the golden bytes; a boot
/// with a different value must be treated as a corrupted configuration.
pub const GOLDEN_CONFIG_CRC: u16 = 0xABF7;

/// All checks in the order the state machine runs them.
pub const CHECK_IDS: [CheckId; CHECK_COUNT] = [
    CheckId::SensorRange,
    CheckId::TableCrc,
    CheckId::ConfigCrc,
    CheckId::ActuatorContinuity,
    CheckId::PhaseLatch,
    CheckId::StateMachine,
];

/// Stable name of one preflight check. Names are roles, not pin numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckId {
    SensorRange,
    TableCrc,
    ConfigCrc,
    ActuatorContinuity,
    PhaseLatch,
    StateMachine,
}

impl Display for CheckId {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        let name = match self {
            Self::SensorRange => "sensor_range",
            Self::TableCrc => "table_crc",
            Self::ConfigCrc => "config_crc",
            Self::ActuatorContinuity => "actuator_continuity",
            Self::PhaseLatch => "phase_latch",
            Self::StateMachine => "state_machine",
        };
        f.write_str(name)
    }
}

/// A sensor channel, identified by what it measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorChannel {
    Accel,
    Baro,
    Battery,
    Temperature,
}

/// All channels checked by [`CheckId::SensorRange`], in report order.
pub const SENSOR_CHANNELS: [SensorChannel; 4] = [
    SensorChannel::Accel,
    SensorChannel::Baro,
    SensorChannel::Battery,
    SensorChannel::Temperature,
];

impl Display for SensorChannel {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        let name = match self {
            Self::Accel => "accel",
            Self::Baro => "baro",
            Self::Battery => "battery",
            Self::Temperature => "temperature",
        };
        f.write_str(name)
    }
}

/// Inclusive raw bounds. Raw units belong to the driver; this module only
/// compares integers and never interprets a scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Range {
    pub min: i32,
    pub max: i32,
}

/// One observation per sensor channel, in raw units.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSample {
    pub accel: i32,
    pub baro: i32,
    pub battery_mv: i32,
    pub temp_c: i32,
}

impl SensorSample {
    pub const fn get(self, channel: SensorChannel) -> i32 {
        match channel {
            SensorChannel::Accel => self.accel,
            SensorChannel::Baro => self.baro,
            SensorChannel::Battery => self.battery_mv,
            SensorChannel::Temperature => self.temp_c,
        }
    }
}

/// Expected bounds for each channel of a [`SensorSample`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorLimits {
    pub accel: Range,
    pub baro: Range,
    pub battery: Range,
    pub temperature: Range,
}

impl SensorLimits {
    pub const fn range(self, channel: SensorChannel) -> Range {
        match channel {
            SensorChannel::Accel => self.accel,
            SensorChannel::Baro => self.baro,
            SensorChannel::Battery => self.battery,
            SensorChannel::Temperature => self.temperature,
        }
    }
}

/// One calibration table and the CRC it must hash to at boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TableSpec<'a> {
    pub name: &'static str,
    pub bytes: &'a [u8],
    pub expected_crc: u16,
}

/// Configuration image whose serialized form is CRC-checked at boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConfigImage {
    pub version: u16,
    pub model_id: u16,
    pub pitch_gain: i16,
    pub yaw_gain: i16,
}

impl ConfigImage {
    /// Fixed little-endian layout: version, model, pitch gain, yaw gain.
    /// Zero padding makes two equal images byte-identical.
    pub fn to_bytes(self) -> [u8; CONFIG_LEN] {
        let mut out = [0u8; CONFIG_LEN];
        out[0..2].copy_from_slice(&self.version.to_le_bytes());
        out[2..4].copy_from_slice(&self.model_id.to_le_bytes());
        out[4..6].copy_from_slice(&self.pitch_gain.to_le_bytes());
        out[6..8].copy_from_slice(&self.yaw_gain.to_le_bytes());
        out
    }
}

/// An actuator channel, identified by the axis it drives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorChannel {
    Pitch,
    Yaw,
}

impl Display for ActuatorChannel {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str(match self {
            Self::Pitch => "pitch",
            Self::Yaw => "yaw",
        })
    }
}

/// Mock of the actuator continuity sense line. `true` means the path is
/// present; no voltage, pin or connector is modelled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContinuityMock {
    pitch: bool,
    yaw: bool,
    glitch: Option<(ActuatorChannel, u8)>,
}

impl ContinuityMock {
    pub const fn healthy() -> Self {
        Self {
            pitch: true,
            yaw: true,
            glitch: None,
        }
    }

    /// A channel that never reports continuity (an open path).
    pub const fn open(channel: ActuatorChannel) -> Self {
        Self {
            pitch: !matches!(channel, ActuatorChannel::Pitch),
            yaw: !matches!(channel, ActuatorChannel::Yaw),
            glitch: None,
        }
    }

    /// A single missed sample at `at_tick`, to exercise the debounce counter.
    pub const fn glitch(channel: ActuatorChannel, at_tick: u8) -> Self {
        Self {
            pitch: true,
            yaw: true,
            glitch: Some((channel, at_tick)),
        }
    }

    /// Deterministic read of one channel at one local sample tick.
    pub const fn sample(self, channel: ActuatorChannel, tick: u8) -> bool {
        let present = match channel {
            ActuatorChannel::Pitch => self.pitch,
            ActuatorChannel::Yaw => self.yaw,
        };
        let glitched = match self.glitch {
            Some((c, t)) => c as u8 == channel as u8 && t == tick,
            None => false,
        };
        present && !glitched
    }
}

/// Phase of the latch as observed at preflight.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatchPhase {
    Burn,
    Flight,
}

/// Readback of the phase latch safety gates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LatchState {
    pub phase: LatchPhase,
    pub motor_armed: bool,
    pub latched: bool,
    pub ai_allowed: bool,
}

/// The latch itself: AI is allowed only in flight, with the motor disarmed and
/// the latch explicitly released. A valid reply only clears the miss counter;
/// only `release` clears the latch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhaseLatch {
    phase: LatchPhase,
    motor_armed: bool,
    misses: u8,
    latched: bool,
}

impl PhaseLatch {
    /// Safe boot state: BURN and latched, so no AI output can reach the
    /// actuators before an explicit, guarded release.
    pub const fn new() -> Self {
        Self {
            phase: LatchPhase::Burn,
            motor_armed: false,
            misses: 0,
            latched: true,
        }
    }

    pub fn burnout(&mut self) {
        self.phase = LatchPhase::Flight;
    }

    pub fn set_motor_armed(&mut self, armed: bool) {
        self.motor_armed = armed;
        if armed {
            self.latched = true;
        }
    }

    pub fn note_missed(&mut self) {
        if self.misses < LATCH_MISSES {
            self.misses += 1;
        }
        if self.misses >= LATCH_MISSES {
            self.latched = true;
        }
    }

    /// A late valid reply does not clear the latch.
    pub fn note_valid(&mut self) {
        self.misses = 0;
    }

    /// Explicit firmware release, refused while still burning or armed.
    pub fn release(&mut self) -> bool {
        if self.phase == LatchPhase::Burn || self.motor_armed {
            return false;
        }
        self.latched = false;
        true
    }

    pub fn readback(self) -> LatchState {
        LatchState {
            phase: self.phase,
            motor_armed: self.motor_armed,
            latched: self.latched,
            ai_allowed: self.phase == LatchPhase::Flight && !self.motor_armed && !self.latched,
        }
    }
}

impl Default for PhaseLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Pure, monotonic-by-subtraction deadline probe. Polling mutates nothing, so a
/// caller can never spin waiting for it to "advance"; only `now_us` changes the
/// answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ticker {
    start_us: u32,
    interval_us: u32,
}

impl Ticker {
    pub const fn new(start_us: u32, interval_us: u32) -> Self {
        Self {
            start_us,
            interval_us,
        }
    }

    /// `true` once the deadline is reached. A zero interval never fires, so a
    /// misconfigured ticker cannot turn a caller into a busy loop.
    pub const fn poll(self, now_us: u32) -> bool {
        self.interval_us != 0 && now_us.wrapping_sub(self.start_us) >= self.interval_us
    }
}

/// Everything the suite observes. No field reaches hardware or a pin.
#[derive(Clone, Copy, Debug)]
pub struct Inputs<'a> {
    pub sensor: SensorSample,
    pub limits: SensorLimits,
    pub tables: &'a [TableSpec<'a>],
    pub config: ConfigImage,
    pub config_crc: u16,
    pub continuity: ContinuityMock,
    pub latch_state: LatchState,
    pub ticker: Ticker,
}

/// Why a check passed or failed, in words a person can act on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    NotRun,
    Passed(&'static str),
    Failed(Failure),
}

/// A concrete preflight failure. Values are carried so a log names the channel
/// and the number, not just "check failed".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    SensorOutOfRange {
        channel: SensorChannel,
        got: i32,
        min: i32,
        max: i32,
    },
    CrcMismatch {
        what: &'static str,
        expected: u16,
        got: u16,
    },
    ContinuityOpen {
        channel: ActuatorChannel,
    },
    ContinuityIntermittent {
        channel: ActuatorChannel,
        good: u8,
        need: u8,
    },
    LatchAllowsAiInBurn,
    LatchIgnoresMotorArm,
    LatchIgnoresStickyLatch,
    LatchScenarioBroken(&'static str),
    StateMachineZeroInterval,
    StateMachinePollNotMonotonic,
}

impl Display for Failure {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        match *self {
            Self::SensorOutOfRange {
                channel,
                got,
                min,
                max,
            } => write!(
                f,
                "{channel} reads {got}, outside configured range {min}..={max}"
            ),
            Self::CrcMismatch {
                what,
                expected,
                got,
            } => write!(
                f,
                "{what} CRC mismatch: expected 0x{expected:04X}, got 0x{got:04X}"
            ),
            Self::ContinuityOpen { channel } => {
                write!(f, "{channel} actuator continuity never closed (open path)")
            }
            Self::ContinuityIntermittent {
                channel,
                good,
                need,
            } => write!(
                f,
                "{channel} continuity unstable: {good} of {need} samples closed"
            ),
            Self::LatchAllowsAiInBurn => f.write_str("phase latch allows AI during BURN"),
            Self::LatchIgnoresMotorArm => {
                f.write_str("phase latch allows AI while the motor is armed")
            }
            Self::LatchIgnoresStickyLatch => {
                f.write_str("phase latch allows AI while still latched")
            }
            Self::LatchScenarioBroken(step) => {
                write!(f, "phase latch scenario broke at '{step}'")
            }
            Self::StateMachineZeroInterval => {
                f.write_str("state machine tick interval is zero; a caller would spin")
            }
            Self::StateMachinePollNotMonotonic => {
                f.write_str("state machine ticker poll is not monotonic at its deadline")
            }
        }
    }
}

impl Display for Verdict {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        match self {
            Self::NotRun => f.write_str("not run"),
            Self::Passed(reason) => f.write_str(reason),
            Self::Failed(failure) => failure.fmt(f),
        }
    }
}

/// Verdict of one check. `NotRun` is the initial state and is never a pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CheckResult {
    id: CheckId,
    verdict: Verdict,
}

impl CheckResult {
    const fn new(id: CheckId) -> Self {
        Self {
            id,
            verdict: Verdict::NotRun,
        }
    }

    pub const fn id(self) -> CheckId {
        self.id
    }

    pub const fn verdict(self) -> Verdict {
        self.verdict
    }

    pub const fn passed(self) -> bool {
        matches!(self.verdict, Verdict::Passed(_))
    }
}

/// What the next `step` call will do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Progress {
    /// `check` is the check that will run next; `step` is its local tick.
    Running { check: CheckId, step: u8 },
    /// Every check has a verdict; call [`SelfTest::report`].
    Done,
}

/// Immutable snapshot of a completed (or in-progress) run.
#[derive(Clone, Copy, Debug)]
pub struct SelfTestReport {
    results: [CheckResult; CHECK_COUNT],
    ticks: u32,
    started_us: u32,
    finished_us: u32,
    done: bool,
}

impl SelfTestReport {
    pub fn results(&self) -> &[CheckResult; CHECK_COUNT] {
        &self.results
    }

    pub const fn ticks(self) -> u32 {
        self.ticks
    }

    pub const fn started_us(self) -> u32 {
        self.started_us
    }

    pub const fn finished_us(self) -> u32 {
        self.finished_us
    }

    pub const fn done(self) -> bool {
        self.done
    }

    pub fn all_passed(self) -> bool {
        self.results.iter().all(|result| result.passed())
    }

    pub fn first_failure(self) -> Option<CheckResult> {
        self.results.iter().copied().find(|result| !result.passed())
    }

    pub fn passed_count(self) -> usize {
        self.results.iter().filter(|result| result.passed()).count()
    }
}

impl Display for SelfTestReport {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        for result in self.results.iter() {
            let mark = if result.passed() { "PASS" } else { "FAIL" };
            writeln!(f, "[{mark}] {}: {}", result.id, result.verdict)?;
        }
        write!(
            f,
            "{} of {} checks passed in {} ticks",
            self.passed_count(),
            CHECK_COUNT,
            self.ticks
        )
    }
}

/// Bounded index over the fixed check list. It visits each check once and then
/// stops; it never wraps and never hides a repeated run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StepCursor {
    next: u8,
}

impl StepCursor {
    pub const fn new() -> Self {
        Self { next: 0 }
    }

    pub const fn done(self) -> bool {
        self.next as usize >= CHECK_COUNT
    }

    pub const fn index(self) -> usize {
        self.next as usize
    }

    pub fn advance(&mut self) {
        if !self.done() {
            self.next += 1;
        }
    }
}

impl Default for StepCursor {
    fn default() -> Self {
        Self::new()
    }
}

/// The preflight suite. Drive it with [`SelfTest::step`] until it returns
/// [`Progress::Done`], then read [`SelfTest::report`].
#[derive(Clone, Copy, Debug)]
pub struct SelfTest<'a> {
    inputs: Inputs<'a>,
    cursor: StepCursor,
    continuity_tick: u8,
    continuity_good: [u8; 2],
    continuity_bad: [u8; 2],
    results: [CheckResult; CHECK_COUNT],
    started_us: u32,
    finished_us: u32,
    ticks: u32,
}

impl<'a> SelfTest<'a> {
    /// Loudly refuses malformed inputs: this is a boundary, not a case for a
    /// silent fallback that would make a check cheaper than it claims.
    pub fn new(inputs: Inputs<'a>) -> Self {
        assert!(
            inputs.tables.len() <= MAX_TABLES,
            "too many self-test tables"
        );
        let mut i = 0;
        while i < inputs.tables.len() {
            assert!(
                inputs.tables[i].bytes.len() <= MAX_TABLE_LEN,
                "self-test table exceeds MAX_TABLE_LEN"
            );
            i += 1;
        }
        for (channel, range) in [
            (SensorChannel::Accel, inputs.limits.accel),
            (SensorChannel::Baro, inputs.limits.baro),
            (SensorChannel::Battery, inputs.limits.battery),
            (SensorChannel::Temperature, inputs.limits.temperature),
        ] {
            assert!(
                range.min <= range.max,
                "self-test range for {channel} is inverted"
            );
        }

        let mut results = [CheckResult::new(CheckId::SensorRange); CHECK_COUNT];
        let mut idx = 0;
        while idx < CHECK_COUNT {
            results[idx] = CheckResult::new(CHECK_IDS[idx]);
            idx += 1;
        }
        Self {
            inputs,
            cursor: StepCursor::new(),
            continuity_tick: 0,
            continuity_good: [0; 2],
            continuity_bad: [0; 2],
            results,
            started_us: 0,
            finished_us: 0,
            ticks: 0,
        }
    }

    /// Advance exactly one bounded slice of one check. Never blocks.
    pub fn step(&mut self, now_us: u32) -> Progress {
        if self.cursor.done() {
            return Progress::Done;
        }
        if self.ticks == 0 {
            self.started_us = now_us;
        }
        self.ticks += 1;

        let id = CHECK_IDS[self.cursor.index()];
        let verdict = match id {
            CheckId::SensorRange => self.check_sensor(),
            CheckId::TableCrc => self.check_tables(),
            CheckId::ConfigCrc => self.check_config(),
            CheckId::ActuatorContinuity => {
                self.sample_continuity();
                self.continuity_tick += 1;
                if self.continuity_tick < CONTINUITY_SAMPLES {
                    return Progress::Running {
                        check: id,
                        step: self.continuity_tick,
                    };
                }
                self.evaluate_continuity()
            }
            CheckId::PhaseLatch => self.check_phase_latch(),
            CheckId::StateMachine => self.check_state_machine(),
        };
        self.results[self.cursor.index()] = CheckResult { id, verdict };
        self.cursor.advance();

        if self.cursor.done() {
            self.finished_us = now_us;
            Progress::Done
        } else {
            Progress::Running {
                check: CHECK_IDS[self.cursor.index()],
                step: 0,
            }
        }
    }

    /// Run to completion. Only for callers that can afford a bounded burst;
    /// the executor path should call [`SelfTest::step`] instead.
    pub fn run_to_completion(&mut self, start_us: u32) -> SelfTestReport {
        while self.step(start_us) != Progress::Done {}
        self.report()
    }

    pub fn report(&self) -> SelfTestReport {
        SelfTestReport {
            results: self.results,
            ticks: self.ticks,
            started_us: self.started_us,
            finished_us: self.finished_us,
            done: self.cursor.done(),
        }
    }

    fn check_sensor(&self) -> Verdict {
        for channel in SENSOR_CHANNELS {
            let got = self.inputs.sensor.get(channel);
            let range = self.inputs.limits.range(channel);
            if got < range.min || got > range.max {
                return failure(Failure::SensorOutOfRange {
                    channel,
                    got,
                    min: range.min,
                    max: range.max,
                });
            }
        }
        Verdict::Passed("all sensor channels inside their configured ranges")
    }

    fn check_tables(&self) -> Verdict {
        for table in self.inputs.tables {
            let got = crc16(table.bytes);
            if got != table.expected_crc {
                return failure(Failure::CrcMismatch {
                    what: table.name,
                    expected: table.expected_crc,
                    got,
                });
            }
        }
        Verdict::Passed("all calibration table CRCs match their golden values")
    }

    fn check_config(&self) -> Verdict {
        let got = crc16(&self.inputs.config.to_bytes());
        if got != self.inputs.config_crc {
            return failure(Failure::CrcMismatch {
                what: "config",
                expected: self.inputs.config_crc,
                got,
            });
        }
        Verdict::Passed("configuration image CRC matches its golden value")
    }

    fn sample_continuity(&mut self) {
        let tick = self.continuity_tick;
        for (slot, channel) in [ActuatorChannel::Pitch, ActuatorChannel::Yaw]
            .into_iter()
            .enumerate()
        {
            if self.inputs.continuity.sample(channel, tick) {
                self.continuity_good[slot] += 1;
            } else {
                self.continuity_bad[slot] += 1;
            }
        }
    }

    fn evaluate_continuity(&self) -> Verdict {
        let channels = [ActuatorChannel::Pitch, ActuatorChannel::Yaw];
        for (slot, channel) in channels.into_iter().enumerate() {
            let good = self.continuity_good[slot];
            let bad = self.continuity_bad[slot];
            if bad == CONTINUITY_SAMPLES {
                return failure(Failure::ContinuityOpen { channel });
            }
            if good < CONTINUITY_SAMPLES {
                return failure(Failure::ContinuityIntermittent {
                    channel,
                    good,
                    need: CONTINUITY_SAMPLES,
                });
            }
        }
        Verdict::Passed("both actuator paths held continuity for every sample")
    }

    fn check_phase_latch(&self) -> Verdict {
        let state = self.inputs.latch_state;
        if state.phase == LatchPhase::Burn && state.ai_allowed {
            return failure(Failure::LatchAllowsAiInBurn);
        }
        if state.motor_armed && state.ai_allowed {
            return failure(Failure::LatchIgnoresMotorArm);
        }
        if state.latched && state.ai_allowed {
            return failure(Failure::LatchIgnoresStickyLatch);
        }

        let mut latch = PhaseLatch::new();
        let expect = |step: &'static str, want: bool, got: bool| -> Option<Verdict> {
            if got != want {
                Some(failure(Failure::LatchScenarioBroken(step)))
            } else {
                None
            }
        };

        let mut wrong = expect("boot is gated", false, latch.readback().ai_allowed);
        latch.burnout();
        if wrong.is_none() {
            wrong = expect("burnout stays latched", false, latch.readback().ai_allowed);
        }
        latch.set_motor_armed(true);
        if wrong.is_none() {
            wrong = expect("armed motor gates AI", false, latch.readback().ai_allowed);
        }
        latch.set_motor_armed(false);
        if wrong.is_none() {
            wrong = expect(
                "disarm alone does not release",
                false,
                latch.readback().ai_allowed,
            );
        }
        if wrong.is_none() && !latch.release() {
            wrong = Some(failure(Failure::LatchScenarioBroken("release")));
        }
        if wrong.is_none() {
            wrong = expect(
                "explicit release allows AI",
                true,
                latch.readback().ai_allowed,
            );
        }
        for _ in 0..LATCH_MISSES {
            latch.note_missed();
        }
        latch.note_valid();
        if wrong.is_none() {
            wrong = expect(
                "three misses latch again",
                false,
                latch.readback().ai_allowed,
            );
        }
        if wrong.is_none() && !latch.release() {
            wrong = Some(failure(Failure::LatchScenarioBroken(
                "release after misses",
            )));
        }
        if wrong.is_none() {
            wrong = expect("released again", true, latch.readback().ai_allowed);
        }
        wrong.unwrap_or(Verdict::Passed(
            "phase latch gates AI in BURN, while armed and while latched",
        ))
    }

    fn check_state_machine(&self) -> Verdict {
        let ticker = self.inputs.ticker;
        let start = ticker.start_us;
        let interval = ticker.interval_us;
        if interval == 0 {
            return failure(Failure::StateMachineZeroInterval);
        }
        let before = start.wrapping_add(interval).wrapping_sub(1);
        let at = start.wrapping_add(interval);
        let after = at.wrapping_add(1);
        let monotonic = !ticker.poll(start)
            && !ticker.poll(before)
            && ticker.poll(at)
            && ticker.poll(at)
            && ticker.poll(after);
        if !monotonic {
            return failure(Failure::StateMachinePollNotMonotonic);
        }
        Verdict::Passed("state machine ticker polls purely and fires only at its deadline")
    }
}

const fn failure(failure: Failure) -> Verdict {
    Verdict::Failed(failure)
}
