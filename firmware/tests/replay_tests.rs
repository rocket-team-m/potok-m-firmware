// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic host gates for the potok-sim -> MockBoard replay adapter.

#[path = "../src/replay.rs"]
mod replay;

use potok_m_firmware::{
    control::{self, LinkState},
    mock::MockBoard,
    phase::PhaseEvent,
    protocol::{decode_reply, encode_reply, REPLY_SIZE},
    Command, PhaseFsm,
};
use potok_protocol::{flags, Phase};

use replay::{
    map_phase, phase_code, phase_event, pitot_i16, synthetic_tick, synthetic_validation_bytes,
    PitotDp, Replay, ReplayError, ReplayEvent, SimPhaseCode, TrajectoryReader, ValidationReader,
    FIRMWARE_TICK_HZ, SIM_FRAME_RATE_HZ, TICKS_PER_SIM_FRAME, TICK_US, TRAJECTORY_HEADER_SIZE,
    TRAJECTORY_MAGIC, VALIDATION_HEADER_SIZE,
};

fn fixture(frames: u32) -> Vec<u8> {
    let mut bytes = vec![0u8; VALIDATION_HEADER_SIZE + 8 + 17 * 4 + frames as usize * 14 * 4];
    let used = synthetic_validation_bytes(frames, &mut bytes).unwrap();
    bytes.truncate(used);
    bytes
}

fn to_board_event(event: ReplayEvent) -> PhaseEvent {
    match event {
        ReplayEvent::LiftOff => PhaseEvent::LiftOff,
        ReplayEvent::Burnout => PhaseEvent::Burnout,
        ReplayEvent::Apogee => PhaseEvent::Apogee,
        ReplayEvent::Descent => PhaseEvent::Descent,
        ReplayEvent::Touchdown => PhaseEvent::Touchdown,
    }
}

#[test]
fn fixture_header_is_valid_v3_and_corruption_is_loud() {
    let bytes = fixture(3);
    let reader = ValidationReader::new(&bytes).unwrap();
    assert_eq!(reader.version, 3);
    assert_eq!(reader.episodes, 1);
    assert_eq!(reader.frames_per_episode, 3);
    assert_eq!(reader.feature_count, 8);
    assert_eq!(reader.target_count, 6);
    assert_eq!(reader.frame_rate_hz, SIM_FRAME_RATE_HZ);
    assert_eq!(reader.terminal_truth_floats(), 0);
    assert_eq!(
        ValidationReader::new(&[]).unwrap_err(),
        ReplayError::ShortHeader
    );
    let mut bad = bytes.clone();
    bad[0] = b'X';
    assert_eq!(
        ValidationReader::new(&bad).unwrap_err(),
        ReplayError::BadMagic
    );
    let mut bad = bytes.clone();
    bad[8..12].copy_from_slice(&9u32.to_le_bytes());
    assert_eq!(
        ValidationReader::new(&bad).unwrap_err(),
        ReplayError::UnsupportedVersion(9)
    );
    let mut bad = bytes.clone();
    bad[32..36].copy_from_slice(&4u32.to_le_bytes());
    assert_eq!(
        ValidationReader::new(&bad).unwrap_err(),
        ReplayError::BadTargetCount(4)
    );
    let short = synthetic_validation_bytes(3, &mut [0u8; 16]);
    assert_eq!(short, Err(ReplayError::ShortBuffer));
}

#[test]
fn one_sim_frame_expands_into_ten_500hz_ticks() {
    assert_eq!(
        SIM_FRAME_RATE_HZ as u32 * TICKS_PER_SIM_FRAME as u32,
        FIRMWARE_TICK_HZ
    );
    assert_eq!(TICK_US, 2000);
    let bytes = fixture(3);
    let mut replay = Replay::new(&bytes, None).unwrap();
    let ticks: Vec<_> = (0..30)
        .map(|_| replay.next_tick().unwrap().unwrap())
        .collect();
    assert_eq!(ticks.len(), 30);
    assert_eq!(replay.next_tick().unwrap(), None);
    for (i, tick) in ticks.iter().enumerate() {
        assert_eq!(tick.time_us, i as u32 * TICK_US);
        assert_eq!(tick.seq, i as u16);
        assert_eq!(tick.reply.seq, i as u16);
        assert_eq!(tick.reply.version, potok_protocol::REPLY_VERSION);
        assert_eq!(tick.reply.ai_healthy, 1);
        // Same source frame for ten consecutive firmware ticks.
        let source = synthetic_tick((i / 10) as u32);
        assert_eq!(tick.gyro, source.gyro);
        assert_eq!(tick.accel, source.accel);
        assert_eq!(tick.height, source.height);
        assert_eq!(tick.pitot_dp, source.pitot_dp);
    }
}

#[test]
fn reply_angles_confidence_and_health_follow_the_target_columns() {
    let bytes = fixture(2);
    let mut replay = Replay::new(&bytes, None).unwrap();
    let tick = replay.next_tick().unwrap().unwrap();
    let source = synthetic_tick(0);
    assert_eq!(tick.reply.g_pitch, -20);
    assert_eq!(tick.reply.g_yaw, -20);
    assert_eq!(
        tick.reply.g_pitch,
        potok_protocol::reply_g_pitch_from_rad(f64::from(source.g_theta[1]))
    );
    assert_eq!(
        tick.reply.g_yaw,
        potok_protocol::reply_g_yaw_from_rad(f64::from(source.g_theta[0]))
    );
    assert_eq!(tick.reply.confidence, 128);
    assert_eq!(tick.reply.ai_healthy, 1);

    let mut buf = [0u8; REPLY_SIZE];
    encode_reply(&tick.reply, &mut buf).unwrap();
    assert_eq!(decode_reply(&buf).unwrap(), tick.reply);
}

#[test]
fn phase_mapping_is_forward_only_and_drives_motor_armed() {
    assert_eq!(map_phase(SimPhaseCode::Pad), Phase::BURN);
    assert_eq!(map_phase(SimPhaseCode::Boost), Phase::BURN);
    assert_eq!(map_phase(SimPhaseCode::Coast), Phase::COAST);
    assert_eq!(map_phase(SimPhaseCode::Apogee), Phase::APOGEE);
    assert_eq!(map_phase(SimPhaseCode::Descent), Phase::DESCENT);
    assert_eq!(map_phase(SimPhaseCode::Landed), Phase::LANDED);
    assert_eq!(
        phase_event(SimPhaseCode::Boost, SimPhaseCode::Coast),
        Some(ReplayEvent::Burnout)
    );
    assert_eq!(
        phase_event(SimPhaseCode::Coast, SimPhaseCode::Boost),
        None,
        "a backwards transition never produces a board event"
    );
    assert_eq!(phase_code(0.0, 0.0, 0.0, 9.95), SimPhaseCode::Pad);
    assert_eq!(phase_code(1.0, 100.0, 2.0, 9.95), SimPhaseCode::Boost);
    assert_eq!(phase_code(11.0, 2000.0, -40.0, 9.95), SimPhaseCode::Descent);
    assert_eq!(phase_code(11.0, 1.0, -1.0, 9.95), SimPhaseCode::Landed);

    // Frame 0 is Pad (t=0), frame 1 is Boost (t=0.02 <= 0.03),
    // frame 2 is Coast (t=0.04 > 0.03): a real Boost -> Coast event.
    let bytes = fixture(3);
    let mut replay = Replay::new(&bytes, None).unwrap().with_burn_end_s(0.03);
    let first = replay.next_tick().unwrap().unwrap();
    assert_eq!(first.phase, Phase::BURN);
    assert!(first.motor_armed);
    assert_ne!(first.sensor_flags & flags::MOTOR_ARMED, 0);
    assert_ne!(first.sensor_flags & flags::IMU_OK, 0);
    assert_eq!(
        first.sensor_flags & flags::AI_READY,
        0,
        "firmware sets AI_READY"
    );
    assert_eq!(first.event, None);
    let after_burnout = (0..20)
        .map(|_| replay.next_tick().unwrap().unwrap())
        .last()
        .unwrap();
    assert_eq!(after_burnout.phase, Phase::COAST);
    assert_eq!(after_burnout.event, Some(ReplayEvent::Burnout));
    assert!(!after_burnout.motor_armed);
    assert_eq!(after_burnout.sensor_flags & flags::MOTOR_ARMED, 0);
}

#[test]
fn replay_is_byte_identical_across_runs_and_accepts_seq_offset() {
    let bytes = fixture(4);
    let collect = || {
        let mut replay = Replay::new(&bytes, None).unwrap();
        (0..40)
            .map(|_| replay.next_tick().unwrap().unwrap())
            .collect::<Vec<_>>()
    };
    let a = collect();
    let b = collect();
    assert_eq!(a, b);
    let mut encoded_a = Vec::new();
    let mut encoded_b = Vec::new();
    let mut buf = [0u8; REPLY_SIZE];
    for (x, y) in a.iter().zip(b.iter()) {
        encode_reply(&x.reply, &mut buf).unwrap();
        encoded_a.extend_from_slice(&buf);
        encode_reply(&y.reply, &mut buf).unwrap();
        encoded_b.extend_from_slice(&buf);
    }
    assert_eq!(encoded_a, encoded_b);

    let mut offset = Replay::new(&bytes, None).unwrap().with_seq_offset(u16::MAX);
    let t0 = offset.next_tick().unwrap().unwrap();
    let t1 = offset.next_tick().unwrap().unwrap();
    assert_eq!(t0.seq, u16::MAX);
    assert_eq!(t1.seq, 0);
}

#[test]
fn synthetic_source_needs_no_artifact_and_matches_the_binary_fixture() {
    let bytes = fixture(3);
    let mut parsed = Replay::new(&bytes, None).unwrap();
    let mut synthetic = Replay::synthetic(3);
    for _ in 0..30 {
        assert_eq!(
            parsed.next_tick().unwrap().unwrap(),
            synthetic.next_tick().unwrap().unwrap()
        );
    }
    assert_eq!(parsed.next_tick().unwrap(), None);
    assert_eq!(synthetic.next_tick().unwrap(), None);
}

#[test]
fn trajectory_sidecar_reader_parses_v3_and_feeds_height() {
    let frames = 40u32;
    let mut bytes = vec![0u8; TRAJECTORY_HEADER_SIZE + 8 + frames as usize * 26 * 4];
    bytes[0..8].copy_from_slice(TRAJECTORY_MAGIC);
    bytes[8..12].copy_from_slice(&3u32.to_le_bytes());
    bytes[12..16].copy_from_slice(&1u32.to_le_bytes());
    bytes[16..20].copy_from_slice(&frames.to_le_bytes());
    bytes[20..24].copy_from_slice(&26u32.to_le_bytes());
    let mut pos = TRAJECTORY_HEADER_SIZE + 8;
    for _ in 0..frames {
        // Truth says the vehicle is on the ground; the validation features
        // say it is climbing. The sidecar must win.
        let height = -1.0f32;
        let climb = 0.0f32;
        bytes[pos + 2 * 4..pos + 3 * 4].copy_from_slice(&height.to_le_bytes());
        bytes[pos + 5 * 4..pos + 6 * 4].copy_from_slice(&climb.to_le_bytes());
        pos += 26 * 4;
    }
    let mut reader = TrajectoryReader::new(&bytes).unwrap();
    assert_eq!(reader.version, 3);
    assert_eq!(reader.stride, 26);
    assert_eq!(reader.next_tick().unwrap(), Some((0, -1.0, 0.0)));
    assert_eq!(reader.next_tick().unwrap(), Some((0, -1.0, 0.0)));
    let mut remaining = 2;
    while reader.next_tick().unwrap().is_some() {
        remaining += 1;
    }
    assert_eq!(remaining, frames);
    let mut bad = bytes.clone();
    bad[20..24].copy_from_slice(&24u32.to_le_bytes());
    assert_eq!(
        TrajectoryReader::new(&bad).unwrap_err(),
        ReplayError::BadStride(24)
    );

    let validation = fixture(frames);
    let mut truthful = Replay::new(&validation, Some(&bytes))
        .unwrap()
        .with_burn_end_s(0.03);
    let mut features_only = Replay::new(&validation, None)
        .unwrap()
        .with_burn_end_s(0.03);
    for _ in 0..260 {
        let a = truthful.next_tick().unwrap().unwrap();
        let b = features_only.next_tick().unwrap().unwrap();
        if a.time_us == 20 * TICK_US {
            assert_eq!(a.event, Some(ReplayEvent::Burnout));
        }
        if a.time_us == 260 * TICK_US {
            assert_eq!(a.phase, Phase::LANDED, "truth height wins");
            assert_eq!(b.phase, Phase::COAST, "feature height otherwise wins");
        }
    }
}

#[test]
fn replayed_ticks_drive_the_real_mockboard_control_loop() {
    let bytes = fixture(30);
    let mut replay = Replay::new(&bytes, None).unwrap().with_burn_end_s(0.03);
    let mut board = MockBoard::new();
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();
    let mut non_zero = 0usize;
    let mut accepted = 0usize;
    for _ in 0..300 {
        let tick = replay.next_tick().unwrap().unwrap();
        board.now_us = tick.time_us;
        board.phase_event = tick.event.map(to_board_event);
        board.sensor_flags = tick.sensor_flags;
        board.motor_armed = tick.motor_armed;
        let mut buf = [0u8; REPLY_SIZE];
        encode_reply(&tick.reply, &mut buf).unwrap();
        board.queue_bytes(&buf);
        let outcome = control::step_link(&mut board, &mut fsm, &mut link, true);
        if matches!(outcome, control::StepResult::Accepted(_)) {
            accepted += 1;
        }
        if board
            .applied
            .last()
            .is_some_and(|cmd| *cmd != Command::ZERO)
        {
            non_zero += 1;
        }
    }
    assert_eq!(fsm.phase(), Phase::COAST);
    assert!(
        accepted > 250,
        "replies should match the mirror seq, got {accepted}"
    );
    assert!(
        non_zero > 200,
        "AI corrections should reach the board after burnout"
    );
    assert_eq!(board.applied.len(), 300);
    assert!(board.pets() > 250);
}

#[test]
fn non_finite_pitot_is_unavailable_not_a_valid_zero_reading() {
    assert_eq!(pitot_i16(f32::NAN), PitotDp::Unavailable);
    assert_eq!(pitot_i16(f32::INFINITY), PitotDp::Unavailable);
    assert_eq!(pitot_i16(f32::NEG_INFINITY), PitotDp::Unavailable);
    assert_eq!(pitot_i16(0.0), PitotDp::Available(0));
    assert_eq!(pitot_i16(12.7), PitotDp::Available(12));
    assert_eq!(pitot_i16(40_000.0), PitotDp::Available(i16::MAX));
    assert_eq!(pitot_i16(-40_000.0), PitotDp::Available(i16::MIN));
}

#[test]
fn replay_distinguishes_a_corrupt_pitot_from_a_zero_reading() {
    // features[7] is pitot_pa; one source frame is 8 features * 4 bytes.
    let pitot_offset = |frame: usize| VALIDATION_HEADER_SIZE + 8 + 17 * 4 + frame * 8 * 4 + 7 * 4;

    let mut corrupt = fixture(2);
    let off = pitot_offset(0);
    corrupt[off..off + 4].copy_from_slice(&f32::NAN.to_le_bytes());
    let bad = Replay::new(&corrupt, None)
        .unwrap()
        .next_tick()
        .unwrap()
        .unwrap();
    assert_eq!(bad.pitot_dp, PitotDp::Unavailable);
    assert_ne!(bad.sensor_flags & flags::FAULT, 0);

    let good = Replay::new(&fixture(2), None)
        .unwrap()
        .next_tick()
        .unwrap()
        .unwrap();
    assert_eq!(good.pitot_dp, PitotDp::Available(0));
    assert_eq!(good.sensor_flags & flags::FAULT, 0);
}

#[test]
fn phase_code_orders_boost_ahead_of_measured_ground_contact() {
    // (time_s, height, climb_rate, burn_end_s, expected)
    let cases = [
        (0.0, -5.0, 0.0, 9.95, SimPhaseCode::Pad),
        (0.6, 0.0, 0.0, 9.95, SimPhaseCode::Boost),
        (0.6, -1.0, 0.0, 9.95, SimPhaseCode::Boost),
        (9.95, 0.0, 0.0, 9.95, SimPhaseCode::Boost),
        (9.951, 0.0, 0.0, 9.95, SimPhaseCode::Landed),
        (10.0, 1.0, -1.0, 9.95, SimPhaseCode::Landed),
        (10.0, 5.0, -1.0, 9.95, SimPhaseCode::Descent),
        (10.0, 6.0, -2.0, 9.95, SimPhaseCode::Descent),
        (10.0, 10.0, 0.0, 9.95, SimPhaseCode::Coast),
        (10.0, 11.0, 0.0, 9.95, SimPhaseCode::Apogee),
        (10.0, 50.0, 1.999, 9.95, SimPhaseCode::Apogee),
        (10.0, 50.0, 2.0, 9.95, SimPhaseCode::Coast),
    ];
    for (t, h, v, burn_end, expected) in cases {
        assert_eq!(
            phase_code(t, h, v, burn_end),
            expected,
            "t={t} h={h} v={v} burn_end={burn_end}"
        );
    }
}

#[test]
fn phase_gate_uses_only_available_columns_and_keeps_the_ground_gate() {
    // validation.bin v3 has 8 features and 6 targets, no phase column. The
    // trajectory sidecar has true height and climb rate, no phase either.
    // Drive every baro height to zero so the old ground-first ordering would
    // call a stationary pad Landed at t > 0.5 s, before boost.
    let frames = 510u32;
    let mut validation = fixture(frames);
    let frame_bytes = VALIDATION_HEADER_SIZE + 8 + 17 * 4;
    for frame in 0..frames as usize {
        let off = frame_bytes + frame * 8 * 4 + 6 * 4;
        validation[off..off + 4].copy_from_slice(&0.0f32.to_le_bytes());
    }
    let reader = ValidationReader::new(&validation).unwrap();
    assert_eq!(reader.feature_count, 8);
    assert_eq!(
        reader.target_count, 6,
        "no phase column in validation.bin v3"
    );

    // Truth: a real climb at 100 m, +5 m/s for every sidecar tick.
    let mut trajectory = vec![0u8; TRAJECTORY_HEADER_SIZE + 8 + frames as usize * 26 * 4];
    trajectory[0..8].copy_from_slice(TRAJECTORY_MAGIC);
    trajectory[8..12].copy_from_slice(&3u32.to_le_bytes());
    trajectory[12..16].copy_from_slice(&1u32.to_le_bytes());
    trajectory[16..20].copy_from_slice(&frames.to_le_bytes());
    trajectory[20..24].copy_from_slice(&26u32.to_le_bytes());
    let mut pos = TRAJECTORY_HEADER_SIZE + 8;
    for _ in 0..frames {
        trajectory[pos + 2 * 4..pos + 3 * 4].copy_from_slice(&100.0f32.to_le_bytes());
        trajectory[pos + 5 * 4..pos + 6 * 4].copy_from_slice(&5.0f32.to_le_bytes());
        pos += 26 * 4;
    }

    let mut truthful = Replay::new(&validation, Some(&trajectory)).unwrap();
    let mut features_only = Replay::new(&validation, None).unwrap();
    let mut events = Vec::new();
    for i in 0..frames as usize * TICKS_PER_SIM_FRAME as usize {
        let a = truthful.next_tick().unwrap().unwrap();
        let b = features_only.next_tick().unwrap().unwrap();
        events.extend(a.event);
        // phase_code sees the 50 Hz source time, not the 500 Hz tick clock.
        let t = (i / TICKS_PER_SIM_FRAME as usize) as f32 / SIM_FRAME_RATE_HZ;
        if t > 0.0 && t <= 0.5 {
            assert_eq!(a.phase_code, SimPhaseCode::Boost, "t={t} with sidecar");
            assert_eq!(b.phase_code, SimPhaseCode::Boost, "t={t} without sidecar");
        }
        if i == 500 * TICKS_PER_SIM_FRAME as usize {
            assert_eq!(a.phase_code, SimPhaseCode::Coast, "truth height wins");
            assert_eq!(
                b.phase_code,
                SimPhaseCode::Landed,
                "baro ground still gates"
            );
        }
    }
    assert_eq!(events, vec![ReplayEvent::LiftOff, ReplayEvent::Burnout]);
}
