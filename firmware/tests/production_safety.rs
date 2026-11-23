// SPDX-License-Identifier: GPL-3.0-or-later
//! Gates for the shipping control path: the real `step()` with the independent
//! interlock and the watchguard deadline monitor wired in.
//!
//! The link-level suite drives `step_link`, and SIL drives its own `Guarded`
//! board. Neither proves that the binary enforces anything. These gates drive
//! `step` exactly as `main.rs` does and assert that a vetoed command never
//! reaches `Board::apply_command`, and that a missed deadline stops feeding.

use potok_m_firmware::{
    interlocks::{Interlocks, Refusal},
    mock::MockBoard,
    phase::{PhaseEvent, PhaseFsm},
    protocol::{flags, Phase, ReplyFrame},
    step,
    watchdog::{
        FeedDecision, WatchdogPolicy, CONTROL_PERIOD_US, DEADLINE_LIMIT, DEADLINE_SLACK_US,
    },
    Command, LinkState, StepResult,
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

struct Rig {
    board: MockBoard,
    fsm: PhaseFsm,
    link: LinkState,
    interlock: Interlocks,
    policy: WatchdogPolicy,
}

impl Rig {
    fn coasting() -> Self {
        Self {
            board: MockBoard::new(),
            fsm: coast(),
            link: LinkState::new(),
            interlock: Interlocks::default(),
            policy: WatchdogPolicy::new(),
        }
    }

    fn drive(&mut self, frame: Option<ReplyFrame>) -> StepResult {
        if let Some(frame) = frame {
            self.board.queue_reply(&frame);
        }
        step(
            &mut self.board,
            &mut self.fsm,
            &mut self.link,
            &mut self.interlock,
            &mut self.policy,
            true,
        )
    }

    fn drive_reply(&mut self, pitch: i16, yaw: i16) -> StepResult {
        let seq = next_seq(&self.link);
        self.drive(Some(reply(seq, pitch, yaw)))
    }

    fn applied(&self) -> Command {
        *self
            .board
            .applied
            .last()
            .expect("every step applies a command, even zero")
    }
}

#[test]
fn healthy_reply_crosses_both_gates_before_the_board() {
    let mut rig = Rig::coasting();
    assert_eq!(
        rig.drive_reply(40, -40),
        StepResult::Accepted(Command {
            pitch: 10,
            yaw: -10
        })
    );
    assert_eq!(
        rig.applied(),
        Command {
            pitch: 10,
            yaw: -10
        }
    );
    assert_eq!(rig.link.counters.accepted, 1);
    assert_eq!(rig.interlock.counters.accepted, 1);
}

#[test]
fn fault_flag_a_link_ignores_is_vetoed_before_actuation() {
    let mut rig = Rig::coasting();
    assert_eq!(
        rig.drive_reply(40, -40),
        StepResult::Accepted(Command {
            pitch: 10,
            yaw: -10
        })
    );

    // FAULT is not a LinkState veto: the link accepts the frame unconditionally.
    // Only the independent interlock can stop it from reaching the board.
    rig.board.sensor_flags = flags::FAULT;
    assert_eq!(
        rig.drive_reply(80, -80),
        StepResult::Accepted(Command::ZERO)
    );
    assert_eq!(rig.applied(), Command::ZERO);
    assert_eq!(rig.link.counters.accepted, 2, "the link still accepted it");
    assert_eq!(rig.interlock.latched(), Some(Refusal::Fault));

    // The latch survives the cause disappearing, until the firmware resets it.
    rig.board.sensor_flags = 0;
    assert_eq!(
        rig.drive_reply(80, -80),
        StepResult::Accepted(Command::ZERO)
    );
    assert_eq!(rig.applied(), Command::ZERO);
    rig.interlock.reset();
    assert_eq!(
        rig.drive_reply(80, -80),
        StepResult::Accepted(Command {
            pitch: 10,
            yaw: -10
        })
    );
}

#[test]
fn unhealthy_reply_latches_both_layers_and_zeroes_output() {
    let mut rig = Rig::coasting();
    let mut sick = reply(next_seq(&rig.link), 70, 70);
    sick.ai_healthy = 0;
    assert_eq!(rig.drive(Some(sick)), StepResult::FrameRejected);
    assert_eq!(rig.applied(), Command::ZERO);
    assert!(rig.link.is_disabled());
    assert_eq!(rig.interlock.latched(), Some(Refusal::AiUnhealthy));
    assert!(rig.policy.ai_latched());

    assert_eq!(rig.drive_reply(70, 70), StepResult::FrameRejected);
    assert_eq!(rig.applied(), Command::ZERO);
}

#[test]
fn burn_phase_is_vetoed_by_the_production_gate() {
    let mut rig = Rig {
        board: MockBoard::new(),
        fsm: PhaseFsm::new(),
        link: LinkState::new(),
        interlock: Interlocks::default(),
        policy: WatchdogPolicy::new(),
    };
    assert_eq!(
        rig.drive_reply(90, -90),
        StepResult::Accepted(Command::ZERO)
    );
    assert_eq!(rig.applied(), Command::ZERO);
    assert_eq!(
        rig.interlock.latched(),
        None,
        "BURN is transient, not a latch"
    );
}

#[test]
fn three_missed_control_deadlines_trip_watchguard_and_hold_zero() {
    let mut rig = Rig::coasting();
    rig.drive_reply(50, -50);
    assert_eq!(rig.board.pets(), 1);
    assert_eq!(
        rig.applied(),
        Command {
            pitch: 10,
            yaw: -10
        }
    );

    let mut now = 0u32;
    for missed in 1..=DEADLINE_LIMIT {
        now += CONTROL_PERIOD_US + DEADLINE_SLACK_US + 1;
        rig.board.now_us = now;
        rig.drive_reply(50, -50);
        assert_eq!(rig.policy.deadman().missed(), missed);
        if missed == DEADLINE_LIMIT {
            assert!(rig.policy.deadman().tripped());
            assert_eq!(
                rig.applied(),
                Command::ZERO,
                "a missed deadline is a zero command"
            );
        }
    }
    let pets_at_trip = rig.board.pets();
    assert_eq!(
        rig.interlock.latched(),
        None,
        "the deadline trip is not an interlock latch"
    );

    // Late-but-tripped ticks keep the output at zero and withhold the feed.
    now += CONTROL_PERIOD_US + DEADLINE_SLACK_US + 1;
    rig.board.now_us = now;
    rig.drive_reply(50, -50);
    assert_eq!(rig.applied(), Command::ZERO);
    assert_eq!(rig.board.pets(), pets_at_trip);

    // On-time cycles de-escalate and re-arm feeding without any explicit reset.
    for _ in 0..DEADLINE_LIMIT {
        now += CONTROL_PERIOD_US;
        rig.board.now_us = now;
        rig.drive_reply(50, -50);
    }
    assert!(!rig.policy.deadman().tripped());
    assert!(rig.board.pets() > pets_at_trip);
    assert!(!rig.applied().is_zero());
}

#[test]
fn an_incomplete_cycle_never_feeds_the_watchdog() {
    let mut rig = Rig::coasting();
    assert_eq!(
        rig.drive_reply(40, -40),
        StepResult::Accepted(Command {
            pitch: 10,
            yaw: -10
        })
    );
    assert_eq!(rig.board.pets(), 1);

    let seq = next_seq(&rig.link);
    rig.board.queue_reply(&reply(seq, 40, -40));
    let result = step(
        &mut rig.board,
        &mut rig.fsm,
        &mut rig.link,
        &mut rig.interlock,
        &mut rig.policy,
        false,
    );
    assert!(matches!(result, StepResult::Accepted(_)));
    assert_eq!(
        rig.board.pets(),
        1,
        "an incomplete cycle must not feed the watchdog"
    );
    assert_eq!(rig.policy.feed_decision(), FeedDecision::Withhold);
}

#[test]
fn link_latch_forces_zero_without_starving_the_watchdog() {
    let mut rig = Rig::coasting();
    rig.link.disable_now();
    for _ in 0..2 {
        assert_eq!(rig.drive_reply(40, 40), StepResult::FrameRejected);
        assert_eq!(rig.applied(), Command::ZERO);
    }
    assert_eq!(
        rig.board.pets(),
        2,
        "the loop keeps feeding while AI is latched"
    );
    assert!(rig.policy.ai_latched());
}
