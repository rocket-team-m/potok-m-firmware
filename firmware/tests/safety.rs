//! Host-side gates for framing, sequencing and the safety state machine.

use potok_m_firmware::{
    deadman::Deadman,
    mock::MockBoard,
    phase::{PhaseEvent, PhaseFsm},
    protocol::{
        decode_reply, encode_reply, encode_request, flags, Phase, ReplyAssembler, ReplyFrame,
        RequestFrame, SeqGate, REPLY_SIZE, REQUEST_SIZE,
    },
    step_link,
    uart::DoubleBufferRx,
    watchdog::may_feed,
    Command, LinkState, StepResult,
};

fn reply(seq: u16, pitch: i16, yaw: i16) -> ReplyFrame {
    ReplyFrame {
        seq,
        g_pitch: pitch,
        g_yaw: yaw,
        confidence: 200,
        ai_healthy: 1,
        version: 1,
    }
}

fn next_seq(link: &LinkState) -> u16 {
    link.last_request_seq()
        .map_or(0, |last| last.wrapping_add(1))
}

fn reply_step(
    board: &mut MockBoard,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
    pitch: i16,
    yaw: i16,
) -> Command {
    board.queue_reply(&reply(next_seq(link), pitch, yaw));
    step_link(board, fsm, link, true);
    *board
        .applied
        .last()
        .expect("every step applies a command, even zero")
}

fn coast_fsm() -> PhaseFsm {
    let mut fsm = PhaseFsm::new();
    assert_eq!(fsm.phase(), Phase::BURN);
    fsm.apply(PhaseEvent::Burnout).unwrap();
    fsm
}

#[test]
fn frame_encode_decode_roundtrip() {
    let request = RequestFrame {
        seq: 42,
        time_us: 1_700_000_123,
        phase: Phase::APOGEE,
        version: 1,
        flags: flags::IMU_OK | flags::AI_READY,
        reserved: 0,
        gyro: [0.5, -0.25, 0.125],
        accel: [0.0, 9.81, 0.0],
        mag: [1.0, 2.0, 3.0],
        height: 123.5,
        pitot_dp: -17,
    };
    let mut buf = [0u8; REQUEST_SIZE];
    encode_request(&request, &mut buf).unwrap();
    assert_eq!(potok_protocol::decode_request(&buf).unwrap(), request);

    let expected = reply(42, 1000, -1000);
    let mut buf = [0u8; REPLY_SIZE];
    encode_reply(&expected, &mut buf).unwrap();
    assert_eq!(decode_reply(&buf).unwrap(), expected);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 0x5250);
}

#[test]
fn crc_rejection_is_a_miss_and_produces_zero_command() {
    let mut board = MockBoard::new();
    let mut fsm = coast_fsm();
    let mut link = LinkState::new();

    assert_eq!(
        reply_step(&mut board, &mut fsm, &mut link, 50, -40),
        Command {
            pitch: 50,
            yaw: -40,
        }
    );

    let mut corrupt = [0u8; REPLY_SIZE];
    encode_reply(&reply(next_seq(&link), 500, 500), &mut corrupt).unwrap();
    corrupt[4] ^= 0x01;
    board.queue_bytes(&corrupt);
    assert_eq!(
        step_link(&mut board, &mut fsm, &mut link, true),
        StepResult::FrameRejected
    );
    assert_eq!(link.misses(), 1);
    assert_eq!(board.applied.last(), Some(&Command::ZERO));
    assert_eq!(
        decode_reply(&corrupt).unwrap_err(),
        potok_m_firmware::protocol::FrameError::BadCrc
    );
}

#[test]
fn seq_wrap_accepts_zero_after_max_and_rejects_stale() {
    let mut gate = SeqGate::new();
    for _ in 0..=u16::MAX {
        gate.issue();
    }
    assert_eq!(gate.last(), Some(u16::MAX));
    assert_eq!(gate.issue(), 0);
    assert!(gate.check(0).is_ok());
    assert_eq!(
        gate.check(1).unwrap_err(),
        potok_m_firmware::protocol::FrameError::BadSeq {
            expected: 0,
            got: 1
        }
    );

    let mut board = MockBoard::new();
    let mut fsm = coast_fsm();
    let mut link = LinkState::new();
    board.queue_reply(&reply(999, 100, 100));
    assert_eq!(
        step_link(&mut board, &mut fsm, &mut link, true),
        StepResult::FrameRejected
    );
    assert_eq!(board.applied.last(), Some(&Command::ZERO));
    assert_eq!(link.misses(), 1);
}

#[test]
fn ai_ignored_on_burn_and_while_motor_armed() {
    let mut board = MockBoard::new();
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();

    // BURN: a perfectly valid, CRC-checked reply still produces zero output.
    assert_eq!(
        reply_step(&mut board, &mut fsm, &mut link, 500, 500),
        Command::ZERO
    );
    assert_eq!(board.applied.last(), Some(&Command::ZERO));

    fsm.apply(PhaseEvent::Burnout).unwrap();
    assert_eq!(
        reply_step(&mut board, &mut fsm, &mut link, 30, -20),
        Command {
            pitch: 30,
            yaw: -20,
        }
    );

    // Motor armed is an independent gate, even outside BURN.
    board.motor_armed = true;
    assert_eq!(
        reply_step(&mut board, &mut fsm, &mut link, 500, 500),
        Command::ZERO
    );
}

#[test]
fn three_misses_latch_ai_off_until_explicit_reset() {
    let mut board = MockBoard::new();
    let mut fsm = coast_fsm();
    let mut link = LinkState::new();

    for _ in 0..3 {
        let mut corrupt = [0u8; REPLY_SIZE];
        encode_reply(&reply(next_seq(&link), 100, 100), &mut corrupt).unwrap();
        corrupt[12] ^= 0x02; // CRC bytes
        board.queue_bytes(&corrupt);
        assert_eq!(
            step_link(&mut board, &mut fsm, &mut link, true),
            StepResult::FrameRejected
        );
    }
    assert!(link.is_disabled());
    assert_eq!(link.counters.disables, 1);
    assert_eq!(link.counters.rejected, 3);

    // A valid reply does not clear the latch.
    assert_eq!(
        reply_step(&mut board, &mut fsm, &mut link, 100, 100),
        Command::ZERO
    );
    assert!(link.is_disabled());

    // Only the firmware's explicit reset does.
    link.reset();
    assert!(!link.is_disabled());
    assert_eq!(
        reply_step(&mut board, &mut fsm, &mut link, 60, -60),
        Command {
            pitch: 60,
            yaw: -60,
        }
    );
}

#[test]
fn unhealthy_ai_disables_immediately() {
    let mut board = MockBoard::new();
    let mut fsm = coast_fsm();
    let mut link = LinkState::new();
    let mut unhealthy = reply(next_seq(&link), 100, 100);
    unhealthy.ai_healthy = 0;
    board.queue_reply(&unhealthy);
    step_link(&mut board, &mut fsm, &mut link, true);
    assert_eq!(board.applied.last(), Some(&Command::ZERO));
    assert!(link.is_disabled());
    assert_eq!(link.counters.disables, 1);
}

#[test]
fn watchdog_and_deadman_state_machine() {
    assert!(may_feed(true, false));
    assert!(!may_feed(false, false));
    assert!(!may_feed(true, true));

    let mut deadman = Deadman::new();
    deadman.note_missed();
    deadman.note_missed();
    assert!(!deadman.tripped());
    deadman.note_missed();
    assert!(deadman.tripped());
    deadman.note_valid();
    assert!(!deadman.tripped());

    let mut board = MockBoard::new();
    let mut fsm = coast_fsm();
    let mut link = LinkState::new();

    board.cycle_complete = false;
    step_link(&mut board, &mut fsm, &mut link, false);
    assert_eq!(board.pets(), 0);

    board.cycle_complete = true;
    step_link(&mut board, &mut fsm, &mut link, true);
    assert_eq!(board.pets(), 1);

    // Two more silent ticks trip the deadman; feeding stops thereafter.
    step_link(&mut board, &mut fsm, &mut link, true);
    step_link(&mut board, &mut fsm, &mut link, true);
    assert!(link.deadman_tripped());
    let pets = board.pets();
    step_link(&mut board, &mut fsm, &mut link, true);
    assert_eq!(board.pets(), pets);

    // A fresh valid reply clears the deadman and feeding resumes.
    reply_step(&mut board, &mut fsm, &mut link, 10, 10);
    assert!(!link.deadman_tripped());
    let after_reply = board.pets();
    assert!(
        after_reply > pets,
        "a fresh reply restores watchdog feeding"
    );
    step_link(&mut board, &mut fsm, &mut link, true);
    assert_eq!(board.pets(), after_reply + 1);
}

#[test]
fn assembler_drops_noise_and_split_magic() {
    let mut frame = [0u8; REPLY_SIZE];
    encode_reply(&reply(3, 12, -12), &mut frame).unwrap();

    let mut assembler = ReplyAssembler::new();
    assert!(assembler.push(&[0x00, 0x01, 0xff]).is_none());
    assert!(assembler.push(&frame[..1]).is_none());
    assert_eq!(assembler.push(&frame[1..]), Some(frame));
}

#[test]
fn double_buffer_receive_swaps_slots() {
    let mut board = MockBoard::new();
    board.queue_bytes(&[1, 2, 3]);
    board.queue_bytes(&[4, 5]);
    let mut out = [0u8; REPLY_SIZE];
    assert_eq!(board.rx_mut().take(&mut out).unwrap(), Some(3));
    assert_eq!(&out[..3], &[1, 2, 3]);
    assert_eq!(board.rx_mut().take(&mut out).unwrap(), Some(2));
    assert_eq!(&out[..2], &[4, 5]);
    assert_eq!(board.rx().swaps, 2);
}

#[test]
fn phase_fsm_is_forward_only() {
    let mut fsm = PhaseFsm::new();
    assert!(fsm.apply(PhaseEvent::Apogee).is_err());
    assert!(fsm.apply(PhaseEvent::LiftOff).is_ok());
    assert_eq!(fsm.phase(), Phase::BURN);
    fsm.apply(PhaseEvent::Burnout).unwrap();
    assert_eq!(fsm.phase(), Phase::COAST);
    fsm.apply(PhaseEvent::Apogee).unwrap();
    fsm.apply(PhaseEvent::Descent).unwrap();
    fsm.apply(PhaseEvent::Touchdown).unwrap();
    assert_eq!(fsm.phase(), Phase::LANDED);
    assert!(fsm.apply(PhaseEvent::Burnout).is_err());
}
