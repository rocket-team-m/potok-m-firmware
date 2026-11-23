// SPDX-License-Identifier: GPL-3.0-or-later
//! Gates for the production flight path: every pure module has one caller,
//! and a missing or refused capability is counted with a reason rather than
//! silently discarded or reported as a successful no-op.
//!
//! The test board supplies typed inputs only for the capabilities under test.
//! Everything it does not supply takes the trait's `None` default, which is the
//! same path the schematic-bound `HwBoard` takes on hardware.

use potok_m_firmware::{
    control_pid::{Fx, PidConfig},
    deployment::{ArmChannels as DeployArm, DeploymentSample, SpringOutputs},
    interlocks::Interlocks,
    mock::MockBoard,
    search_beacon::{RadioHint, SignalInputs, RADIO_BAND_MHZ},
    self_test::{
        ContinuityMock, LatchPhase, LatchState, Range, SensorLimits, SensorSample, Ticker,
        GOLDEN_CONFIG, GOLDEN_CONFIG_CRC,
    },
    sensor_voting::ImuSample,
    separation::{
        ArmChannels as SepArm, Fault, Output as SeparationOutput, PhaseReading,
        Plan as SeparationPlan, SensorPair, State as SeparationState,
    },
    step,
    watchdog::WatchdogPolicy,
    AxisSample, Capability, Command, DeploymentInput, FlightBoard, FlightPath, LinkState,
    OutputError, PhaseFsm, SelfTestInput, SeparationInput, StabilizationSample, StepResult,
    Unavailable,
};
use potok_protocol::{Phase, ReplyFrame};

/// Board with explicit capability inputs. Fields left at `None` prove the
/// absence path; `reject_outputs` proves an output failure is counted too.
#[derive(Default)]
struct TestBoard {
    imu: Option<[Option<ImuSample>; 3]>,
    height: Option<[Option<f32>; 3]>,
    stabilization: Option<StabilizationSample>,
    deployment: Option<DeploymentInput>,
    separation: Option<SeparationInput>,
    self_test: Option<SelfTestInput>,
    signals: Option<SignalInputs>,
    reject_outputs: bool,
    fin: Vec<Command>,
    springs: Vec<SpringOutputs>,
    separation_outputs: Vec<SeparationOutput>,
    radio: Vec<RadioHint>,
}

impl FlightBoard for TestBoard {
    fn imu_readings(&self) -> Option<[Option<ImuSample>; 3]> {
        self.imu
    }
    fn height_readings(&self) -> Option<[Option<f32>; 3]> {
        self.height
    }
    fn stabilization(&self) -> Option<StabilizationSample> {
        self.stabilization
    }
    fn deployment(&self) -> Option<DeploymentInput> {
        self.deployment
    }
    fn separation(&self) -> Option<SeparationInput> {
        self.separation
    }
    fn self_test_input(&self) -> Option<SelfTestInput> {
        self.self_test
    }
    fn signal_inputs(&self) -> Option<SignalInputs> {
        self.signals
    }
    fn apply_stabilization(&mut self, cmd: Command) -> Result<(), OutputError> {
        if self.reject_outputs {
            return Err(OutputError::Rejected);
        }
        self.fin.push(cmd);
        Ok(())
    }
    fn apply_springs(&mut self, out: SpringOutputs) -> Result<(), OutputError> {
        if self.reject_outputs {
            return Err(OutputError::Rejected);
        }
        self.springs.push(out);
        Ok(())
    }
    fn apply_separation(&mut self, out: SeparationOutput) -> Result<(), OutputError> {
        if self.reject_outputs {
            return Err(OutputError::Rejected);
        }
        self.separation_outputs.push(out);
        Ok(())
    }
    fn apply_radio(&mut self, hint: RadioHint) -> Result<(), OutputError> {
        if self.reject_outputs {
            return Err(OutputError::Rejected);
        }
        self.radio.push(hint);
        Ok(())
    }
}

fn pid_config() -> PidConfig<Fx> {
    PidConfig {
        kp: Fx::from_raw(1),
        ki_dt: Fx::from_raw(0),
        kd_dt: Fx::from_raw(0),
        out_limit: Fx::from_raw(100),
        i_limit: Fx::from_raw(0),
        deriv_alpha: Fx::from_raw(1),
    }
}

fn sample_imu() -> ImuSample {
    ImuSample {
        gyro: [0.0, 0.0, 0.01],
        accel: [0.0, 0.0, 9.81],
    }
}

fn stabilization(setpoint: i32) -> StabilizationSample {
    StabilizationSample {
        pitch: AxisSample {
            setpoint,
            measurement: 0,
        },
        yaw: AxisSample {
            setpoint: 0,
            measurement: 0,
        },
        pitch_config: pid_config(),
        yaw_config: pid_config(),
    }
}

fn self_test_input() -> SelfTestInput {
    SelfTestInput {
        sensor: SensorSample {
            accel: 1,
            baro: 100_000,
            battery_mv: 3_900,
            temp_c: 20,
        },
        limits: SensorLimits {
            accel: Range { min: 0, max: 1_000 },
            baro: Range {
                min: 0,
                max: 200_000,
            },
            battery: Range {
                min: 3_000,
                max: 4_200,
            },
            temperature: Range { min: -40, max: 80 },
        },
        config: GOLDEN_CONFIG,
        config_crc: GOLDEN_CONFIG_CRC,
        continuity: ContinuityMock::healthy(),
        latch_state: LatchState {
            phase: LatchPhase::Burn,
            motor_armed: false,
            latched: true,
            ai_allowed: false,
        },
        ticker: Ticker::new(0, 1_000),
    }
}

#[test]
fn absent_capabilities_are_counted_with_a_reason() {
    let mut path = FlightPath::new();
    let mut board = TestBoard::default();
    path.tick(&mut board, Phase::COAST, 2_000, 2_000);

    for cap in [
        Capability::SensorVoting,
        Capability::Mahony,
        Capability::Pid,
        Capability::Deployment,
        Capability::Separation,
        Capability::SelfTest,
        Capability::Radio,
    ] {
        assert_eq!(path.ledger().used(cap), 0, "absent {cap:?} was used");
        assert_eq!(path.ledger().unavailable(cap), 1, "{cap:?} not counted");
        assert_eq!(
            path.ledger().last_reason(cap),
            Some(Unavailable::NoDriver),
            "{cap:?} has no reason"
        );
    }
    assert!(board.fin.is_empty());
    assert!(board.springs.is_empty());
    assert!(board.separation_outputs.is_empty());
    assert!(board.radio.is_empty());
    assert!(path.attitude().is_none());
    assert!(path.stabilization().is_none());
}

#[test]
fn sensors_feed_attitude_and_pid_when_the_board_supplies_them() {
    let mut path = FlightPath::new();
    let mut board = TestBoard {
        imu: Some([Some(sample_imu()), None, None]),
        height: Some([Some(12.0), None, None]),
        stabilization: Some(stabilization(5)),
        ..Default::default()
    };

    path.tick(&mut board, Phase::COAST, 2_000, 2_000);

    assert_eq!(path.ledger().used(Capability::SensorVoting), 1);
    assert_eq!(path.ledger().used(Capability::Mahony), 1);
    assert!(path.attitude().is_some());
    assert_eq!(path.height_m(), Some(12.0));
    assert_eq!(path.ledger().used(Capability::Pid), 1);
    assert_eq!(board.fin.len(), 1);
    assert!(!board.fin[0].is_zero());
    assert_eq!(path.stabilization(), Some(board.fin[0]));
}

#[test]
fn zero_pid_input_keeps_the_zero_command() {
    let mut path = FlightPath::new();
    let mut board = TestBoard {
        stabilization: Some(stabilization(0)),
        ..Default::default()
    };

    path.tick(&mut board, Phase::COAST, 2_000, 2_000);

    assert_eq!(board.fin, vec![Command::ZERO]);
    assert_eq!(path.stabilization(), Some(Command::ZERO));
}

#[test]
fn output_failure_is_counted_not_pretended() {
    let mut path = FlightPath::new();
    let mut board = TestBoard {
        stabilization: Some(stabilization(5)),
        reject_outputs: true,
        ..Default::default()
    };

    path.tick(&mut board, Phase::COAST, 2_000, 2_000);

    assert!(board.fin.is_empty());
    assert_eq!(path.ledger().unavailable(Capability::Pid), 1);
    assert_eq!(
        path.ledger().last_reason(Capability::Pid),
        Some(Unavailable::OutputFailed)
    );
}

#[test]
fn deployment_refuses_burn_then_releases_after_confirmed_descent() {
    let mut path = FlightPath::new();
    let mut board = TestBoard::default();
    let arm = DeployArm::BOTH;
    fn set(board: &mut TestBoard, arm: DeployArm, altitude: i32) {
        board.deployment = Some(DeploymentInput {
            arm,
            sample: DeploymentSample {
                altitude_m: altitude,
                vertical_speed_cm_s: -200,
            },
        });
    }

    set(&mut board, arm, 40);
    path.tick(&mut board, Phase::BURN, 2_000, 2_000);
    assert_eq!(path.ledger().unavailable(Capability::Deployment), 1);
    assert_eq!(
        path.ledger().last_reason(Capability::Deployment),
        Some(Unavailable::Refused)
    );
    assert!(board.springs.is_empty());

    for (tick, altitude) in [40, 38, 36].into_iter().enumerate() {
        set(&mut board, arm, altitude);
        path.tick(&mut board, Phase::COAST, 2_000 * (tick as u32 + 2), 2_000);
    }
    assert_eq!(board.springs, vec![SpringOutputs::RELEASED]);
    assert_eq!(path.ledger().used(Capability::Deployment), 3);
}

#[test]
fn separation_fault_is_counted_and_latched() {
    let mut path = FlightPath::new();
    let mut board = TestBoard {
        separation: Some(SeparationInput {
            plan: SeparationPlan {
                trigger: Phase::COAST,
                delay_us: 0,
                phase_skew_tolerance_us: 100,
                sensor_tolerance: 5,
                separation_level: 100,
            },
            arm: SepArm {
                primary: true,
                secondary: true,
            },
            phase: [
                PhaseReading {
                    phase: Phase::COAST,
                    at_us: 100,
                },
                PhaseReading {
                    phase: Phase::COAST,
                    at_us: 100,
                },
            ],
            sensors: SensorPair { a: 0, b: 100 },
        }),
        ..Default::default()
    };

    path.tick(&mut board, Phase::COAST, 1_000, 2_000);

    assert_eq!(path.ledger().unavailable(Capability::Separation), 1);
    assert_eq!(
        path.ledger().last_reason(Capability::Separation),
        Some(Unavailable::Refused)
    );
    assert!(matches!(
        path.separation().unwrap().state(),
        SeparationState::Aborted(Fault::SensorMismatch { .. })
    ));
    assert!(board.separation_outputs.is_empty());
}

#[test]
fn self_test_runs_to_a_report_when_supplied() {
    let mut path = FlightPath::new();
    let mut board = TestBoard {
        self_test: Some(self_test_input()),
        ..Default::default()
    };

    for tick in 0..12 {
        path.tick(&mut board, Phase::BURN, 2_000 * (tick + 1), 2_000);
    }

    let report = path.self_test_report().expect("no self-test report");
    assert!(report.done());
    assert!(report.all_passed(), "{report}");
    assert_eq!(path.ledger().used(Capability::SelfTest), 1);
    assert_eq!(path.ledger().unavailable(Capability::SelfTest), 0);
}

#[test]
fn radio_hint_reaches_the_board_only_with_the_capability() {
    let mut path = FlightPath::new();
    let mut board = TestBoard {
        signals: Some(SignalInputs {
            post_landing: true,
            melody_requested: false,
            boost_active: false,
            battery_mv: 3_900,
        }),
        ..Default::default()
    };

    path.tick(&mut board, Phase::LANDED, 2_000, 2_000);

    assert_eq!(board.radio.len(), 1);
    assert_eq!(board.radio[0].band_mhz, RADIO_BAND_MHZ);
    assert!(board.radio[0].transmit);
    assert_eq!(path.ledger().used(Capability::Radio), 1);
    assert_eq!(path.beacon_tick(), 1);
}

#[test]
fn production_step_still_zeroes_a_command_in_burn() {
    let mut board = MockBoard::new();
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();
    board.queue_reply(&ReplyFrame {
        seq: 0,
        g_pitch: 50,
        g_yaw: -50,
        confidence: 240,
        ai_healthy: 1,
        version: 1,
    });

    let result = step(
        &mut board,
        &mut fsm,
        &mut link,
        &mut interlock,
        &mut policy,
        true,
    );

    assert_eq!(result, StepResult::Accepted(Command::ZERO));
    assert_eq!(board.applied, vec![Command::ZERO]);
}
