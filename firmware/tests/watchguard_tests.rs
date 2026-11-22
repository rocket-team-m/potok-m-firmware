// SPDX-License-Identifier: GPL-3.0-or-later
//! Firmware-side watchguard gates, exercised entirely with simulated frames.
//!
//! This is the host counterpart of the AI-side watchguard: a reply is injected
//! into `MockBoard`, driven through the real `step()`, and the resulting
//! command, latch and watchdog-feed state are asserted. No pin numbers, no
//! UART, no hardware and no `unsafe` are involved.

use potok_m_firmware::{
    mock::MockBoard,
    phase::PhaseFsm,
    protocol::{encode_reply, flags, Phase, ReplyFrame, REPLY_SIZE},
    step,
    watchdog::{
        FeedDecision, Severity, WatchdogPolicy, CONTROL_PERIOD_US, DEADLINE_LIMIT,
        DEADLINE_SLACK_US,
    },
    Command, LinkState, PhaseEvent, StepResult, MAX_CONSECUTIVE_MISSES, MAX_CORRECTION_RAW,
};

fn reply(seq: u16, pitch: i16, yaw: i16) -> ReplyFrame {
    ReplyFrame {
        seq,
        g_pitch: pitch,
        g_yaw: yaw,
        confidence: 240,
        ai_healthy: 1,
        version: 1,
    }
}

/// The seq the next `step()` will issue; a reply must echo it to be accepted.
fn next_seq(link: &LinkState) -> u16 {
    link.last_request_seq()
        .map_or(0, |last| last.wrapping_add(1))
}

fn coast() -> PhaseFsm {
    let mut fsm = PhaseFsm::new();
    assert_eq!(fsm.phase(), Phase::BURN);
    fsm.apply(PhaseEvent::Burnout).unwrap();
    fsm
}

fn drive(
    board: &mut MockBoard,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
    frame: Option<ReplyFrame>,
) -> StepResult {
    if let Some(frame) = frame {
        board.queue_reply(&frame);
    }
    step(board, fsm, link, true)
}

/// Queue a well-formed reply carrying the seq the next step will issue.
fn drive_reply(
    board: &mut MockBoard,
    fsm: &mut PhaseFsm,
    link: &mut LinkState,
    pitch: i16,
    yaw: i16,
) -> StepResult {
    let seq = next_seq(link);
    drive(board, fsm, link, Some(reply(seq, pitch, yaw)))
}

fn applied(board: &MockBoard) -> Command {
    *board
        .applied
        .last()
        .expect("every step applies a command, even zero")
}

fn corrupt_bytes(link: &LinkState, pitch: i16, yaw: i16) -> [u8; REPLY_SIZE] {
    let mut bytes = [0u8; REPLY_SIZE];
    encode_reply(&reply(next_seq(link), pitch, yaw), &mut bytes).unwrap();
    bytes[12] ^= 0x02; // CRC bytes: decode must reject the frame
    assert!(potok_m_firmware::protocol::decode_reply(&bytes).is_err());
    bytes
}

#[test]
fn flight_constants_pin_the_watchguard_budget() {
    assert_eq!(MAX_CONSECUTIVE_MISSES, 3);
    assert_eq!(MAX_CORRECTION_RAW, 100); // 0.001 rad scale, 0.1 rad model bound
    assert_eq!(CONTROL_PERIOD_US, 2_000);
    assert_eq!(DEADLINE_SLACK_US, 250);
    assert_eq!(DEADLINE_LIMIT, 3);
    assert_ne!(flags::MOTOR_ARMED, flags::FAULT);
    assert_ne!(flags::AI_READY, flags::FAULT);
}

#[test]
fn bounds_veto_clamps_every_simulated_command_to_the_model_limit() {
    let mut board = MockBoard::new();
    let mut fsm = coast();
    let mut link = LinkState::new();

    let extreme = drive_reply(&mut board, &mut fsm, &mut link, i16::MAX, i16::MIN);
    assert_eq!(
        extreme,
        StepResult::Accepted(Command {
            pitch: MAX_CORRECTION_RAW,
            yaw: -MAX_CORRECTION_RAW,
        })
    );
    assert_eq!(
        applied(&board),
        Command {
            pitch: MAX_CORRECTION_RAW,
            yaw: -MAX_CORRECTION_RAW,
        }
    );

    // A command inside the bound is not modified.
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, -37, 42),
        StepResult::Accepted(Command {
            pitch: -37,
            yaw: 42
        })
    );
}

#[test]
fn mixed_bad_frames_and_a_timeout_share_one_three_strike_latch() {
    let mut board = MockBoard::new();
    let mut fsm = coast();
    let mut link = LinkState::new();

    // Bad CRC.
    board.queue_bytes(&corrupt_bytes(&link, 50, 50));
    assert_eq!(
        step(&mut board, &mut fsm, &mut link, true),
        StepResult::FrameRejected
    );
    assert_eq!(link.misses(), 1);

    // Stale seq: the reply uses a seq that was never issued.
    let stale = next_seq(&link).wrapping_add(1);
    board.queue_reply(&reply(stale, 50, 50));
    assert_eq!(
        step(&mut board, &mut fsm, &mut link, true),
        StepResult::FrameRejected
    );
    assert_eq!(link.misses(), 2);
    assert!(!link.is_disabled());

    // Silence counts as the third miss and latches.
    assert_eq!(
        step(&mut board, &mut fsm, &mut link, true),
        StepResult::Idle
    );
    assert!(link.is_disabled());
    assert_eq!(link.counters.disables, 1);
    assert_eq!(link.counters.rejected, 3);
    assert!(applied(&board).is_zero());

    // A good frame never clears the latch, but does clear the deadman.
    drive_reply(&mut board, &mut fsm, &mut link, 60, -60);
    assert!(link.is_disabled());
    assert!(!link.deadman_tripped());
    assert!(applied(&board).is_zero());

    // Only the firmware's explicit reset restores guidance.
    link.reset();
    assert!(!link.is_disabled());
    assert_eq!(link.misses(), 0);
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 60, -60),
        StepResult::Accepted(Command {
            pitch: 60,
            yaw: -60
        })
    );
}

#[test]
fn a_valid_frame_resets_the_miss_run_before_the_latch() {
    let mut board = MockBoard::new();
    let mut fsm = coast();
    let mut link = LinkState::new();

    for expected in 1..=2 {
        board.queue_bytes(&corrupt_bytes(&link, 10, 10));
        assert_eq!(
            step(&mut board, &mut fsm, &mut link, true),
            StepResult::FrameRejected
        );
        assert_eq!(link.misses(), expected);
    }
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 10, 10),
        StepResult::Accepted(Command { pitch: 10, yaw: 10 })
    );
    assert_eq!(link.misses(), 0);
    assert!(!link.is_disabled());

    board.queue_bytes(&corrupt_bytes(&link, 10, 10));
    step(&mut board, &mut fsm, &mut link, true);
    assert_eq!(
        link.misses(),
        1,
        "the run starts over after an accepted frame"
    );

    board.queue_bytes(&corrupt_bytes(&link, 10, 10));
    step(&mut board, &mut fsm, &mut link, true);
    assert_eq!(link.misses(), 2);
    assert!(!link.is_disabled(), "two misses are still below the latch");
}

#[test]
fn unhealthy_reply_disables_immediately_without_a_three_strike_run() {
    let mut board = MockBoard::new();
    let mut fsm = coast();
    let mut link = LinkState::new();

    let mut sick = reply(next_seq(&link), 70, 70);
    sick.ai_healthy = 0;
    assert_eq!(
        drive(&mut board, &mut fsm, &mut link, Some(sick)),
        StepResult::FrameRejected
    );
    assert!(link.is_disabled());
    assert_eq!(link.misses(), 0, "unhealthy is not a counted miss");
    assert_eq!(link.counters.disables, 1);
    assert!(applied(&board).is_zero());

    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 70, 70),
        StepResult::FrameRejected
    );
    assert!(applied(&board).is_zero());
    assert!(link.is_disabled());

    link.reset();
    assert!(!link.is_disabled());
}

#[test]
fn silent_link_trips_the_deadman_and_stops_feeding_until_a_frame_arrives() {
    let mut board = MockBoard::new();
    let mut fsm = coast();
    let mut link = LinkState::new();

    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 10, 10),
        StepResult::Accepted(Command { pitch: 10, yaw: 10 })
    );
    assert_eq!(board.pets(), 1);
    assert!(!link.deadman_tripped());

    for expected in 1..=MAX_CONSECUTIVE_MISSES {
        assert_eq!(
            step(&mut board, &mut fsm, &mut link, true),
            StepResult::Idle
        );
        assert_eq!(link.misses(), expected);
        assert_eq!(link.deadman_tripped(), expected == MAX_CONSECUTIVE_MISSES);
    }
    assert!(applied(&board).is_zero());
    let pets_at_trip = board.pets();

    // Still silent and tripped: the watchdog is not fed.
    assert_eq!(
        step(&mut board, &mut fsm, &mut link, true),
        StepResult::Idle
    );
    assert_eq!(board.pets(), pets_at_trip);

    // A fresh frame clears the deadman and feeding resumes, while the AI latch
    // still forces zero output.
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 20, 20),
        StepResult::FrameRejected
    );
    assert!(!link.deadman_tripped());
    assert!(link.is_disabled());
    assert!(applied(&board).is_zero());
    assert_eq!(board.pets(), pets_at_trip + 1);

    // The explicit reset finally restores guidance.
    link.reset();
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 30, -30),
        StepResult::Accepted(Command {
            pitch: 30,
            yaw: -30
        })
    );
}

#[test]
fn ai_latch_forces_zero_without_stopping_the_control_loop_feed() {
    let mut board = MockBoard::new();
    let mut fsm = coast();
    let mut link = LinkState::new();

    link.disable_now();
    assert!(link.is_disabled());

    for _ in 0..2 {
        assert_eq!(
            drive_reply(&mut board, &mut fsm, &mut link, 40, 40),
            StepResult::FrameRejected
        );
        assert!(applied(&board).is_zero());
    }
    assert_eq!(board.pets(), 2, "a latched AI must not starve the watchdog");

    // An incomplete cycle never feeds, latched or not.
    let seq = next_seq(&link);
    board.queue_reply(&reply(seq, 40, 40));
    step(&mut board, &mut fsm, &mut link, false);
    assert_eq!(board.pets(), 2);
}

#[test]
fn burn_and_motor_armed_veto_zero_every_valid_simulated_frame() {
    let mut board = MockBoard::new();
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();

    // BURN: a healthy, in-bounds, correctly sequenced reply still moves nothing.
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 90, -90),
        StepResult::Accepted(Command::ZERO)
    );
    assert!(applied(&board).is_zero());
    assert_eq!(link.counters.accepted, 1);

    fsm.apply(PhaseEvent::Burnout).unwrap();
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 90, -90),
        StepResult::Accepted(Command {
            pitch: 90,
            yaw: -90
        })
    );

    board.motor_armed = true;
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 90, -90),
        StepResult::Accepted(Command::ZERO)
    );
    assert!(applied(&board).is_zero());

    board.motor_armed = false;
    assert_eq!(
        drive_reply(&mut board, &mut fsm, &mut link, 90, -90),
        StepResult::Accepted(Command {
            pitch: 90,
            yaw: -90
        })
    );
}

#[test]
fn watchdog_ai_latch_and_deadline_monitor_recover_independently() {
    let mut policy = WatchdogPolicy::new();
    policy.latch_ai();
    assert!(policy.must_zero_output());

    // The AI latch does not gate feeding.
    assert_eq!(
        policy.on_control_cycle(CONTROL_PERIOD_US, true, true),
        FeedDecision::Feed
    );

    // Three late cycles trip the cadence monitor independently of the latch.
    let mut now = CONTROL_PERIOD_US;
    for _ in 0..DEADLINE_LIMIT {
        now += CONTROL_PERIOD_US + DEADLINE_SLACK_US + 1;
        policy.on_control_cycle(now, true, true);
    }
    assert_eq!(policy.severity(), Severity::Tripped);
    assert_eq!(policy.feed_decision(), FeedDecision::Withhold);

    policy.reset_ai_latch();
    assert!(!policy.ai_latched());
    assert_eq!(
        policy.severity(),
        Severity::Tripped,
        "AI reset must not disarm the deadline monitor"
    );
    assert!(policy.must_zero_output());

    policy.reset_deadman();
    assert_eq!(policy.severity(), Severity::Healthy);
    assert!(
        !policy.ai_latched(),
        "deadman reset must not re-latch the AI"
    );
    assert_eq!(
        policy.feed_decision(),
        FeedDecision::Withhold,
        "re-arm needs a full cycle"
    );
    assert_eq!(
        policy.on_control_cycle(now + CONTROL_PERIOD_US, true, true),
        FeedDecision::Feed
    );
    assert!(!policy.must_zero_output());
}
