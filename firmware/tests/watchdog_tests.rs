// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the software watchdog policy: control-loop-only feed,
//! missed-deadline escalation, and an AI latch reset independent of the
//! watchdog deadline monitor. No hardware is involved.

use potok_m_firmware::{
    mock::MockBoard,
    phase::PhaseFsm,
    step_link,
    watchdog::{
        may_feed, FeedDecision, Severity, SoftwareDeadman, WatchdogPolicy, CONTROL_PERIOD_US,
        DEADLINE_LIMIT, DEADLINE_SLACK_US, WATCHDOG_TIMEOUT_US,
    },
    LinkState,
};

#[test]
fn constants_pin_the_control_cadence_and_legacy_feed_gate() {
    assert_eq!(CONTROL_PERIOD_US, 2_000); // 500 Hz
    assert_eq!(DEADLINE_SLACK_US, 250);
    assert_eq!(DEADLINE_LIMIT, 3);
    assert_eq!(WATCHDOG_TIMEOUT_US, 100_000);
    assert!(may_feed(true, false));
    assert!(!may_feed(false, false));
    assert!(!may_feed(true, true));
    assert!(!may_feed(false, true));
}

#[test]
fn feed_requires_a_complete_cycle_with_a_valid_phase() {
    let mut policy = WatchdogPolicy::new();
    assert_eq!(policy.feed_decision(), FeedDecision::Withhold);

    assert_eq!(
        policy.on_control_cycle(0, false, true),
        FeedDecision::Withhold
    );
    assert_eq!(
        policy.on_control_cycle(CONTROL_PERIOD_US, true, false),
        FeedDecision::Withhold
    );
    assert_eq!(
        policy.on_control_cycle(2 * CONTROL_PERIOD_US, true, true),
        FeedDecision::Feed
    );
    assert!(policy.may_feed());
}

#[test]
fn control_loop_is_the_only_feed_path_in_the_real_step() {
    let mut board = MockBoard::new();
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();

    step_link(&mut board, &mut fsm, &mut link, false);
    assert_eq!(board.pets(), 0, "an incomplete cycle must not pet the WDT");
    step_link(&mut board, &mut fsm, &mut link, true);
    assert_eq!(
        board.pets(),
        1,
        "a complete cycle with a valid phase pets once"
    );
}

#[test]
fn missed_deadlines_escalate_to_a_trip_then_deescalate_on_time() {
    let mut policy = WatchdogPolicy::new();
    let mut now = 0;
    for _ in 0..3 {
        assert_eq!(policy.on_control_cycle(now, true, true), FeedDecision::Feed);
        now += CONTROL_PERIOD_US;
    }
    assert_eq!(policy.severity(), Severity::Healthy);
    assert_eq!(policy.deadman().missed(), 0);

    // Each late cycle escalates. The trip withholds feed once the limit is hit.
    let late = CONTROL_PERIOD_US + DEADLINE_SLACK_US + 1;
    for expected in 1..=DEADLINE_LIMIT {
        now += late;
        let decision = policy.on_control_cycle(now, true, true);
        assert_eq!(policy.deadman().missed(), expected);
        if expected < DEADLINE_LIMIT {
            assert_eq!(policy.severity(), Severity::Degraded);
            assert_eq!(decision, FeedDecision::Feed);
        } else {
            assert_eq!(policy.severity(), Severity::Tripped);
            assert_eq!(decision, FeedDecision::Withhold);
            assert!(!policy.may_feed());
        }
    }

    // One on-time cycle clears the escalation and restores feeding at once,
    // matching the existing link deadman's note_valid recovery.
    now += CONTROL_PERIOD_US;
    assert_eq!(policy.on_control_cycle(now, true, true), FeedDecision::Feed);
    assert_eq!(policy.severity(), Severity::Healthy);
    assert_eq!(policy.deadman().missed(), 0);
}

#[test]
fn wrapped_microsecond_clock_does_not_look_like_a_missed_deadline() {
    let mut deadman = SoftwareDeadman::new();
    assert!(deadman.note_cycle(u32::MAX - 100));
    let now = 50u32;
    assert_eq!(now.wrapping_sub(u32::MAX - 100), 151);
    assert!(deadman.note_cycle(now));
    assert_eq!(deadman.severity(), Severity::Healthy);
    assert_eq!(deadman.missed(), 0);
}

#[test]
fn explicit_deadman_reset_clears_the_monitor_but_keeps_the_ai_latch() {
    let mut policy = WatchdogPolicy::new();
    policy.latch_ai();
    let mut now = 0;
    policy.on_control_cycle(now, true, true); // prime the cadence
    for _ in 0..DEADLINE_LIMIT {
        now += CONTROL_PERIOD_US + DEADLINE_SLACK_US + 1;
        policy.on_control_cycle(now, true, true);
    }
    assert_eq!(policy.severity(), Severity::Tripped);

    policy.reset_deadman();
    assert_eq!(policy.severity(), Severity::Healthy);
    assert!(
        policy.ai_latched(),
        "watchdog reset must not clear the AI latch"
    );
    assert_eq!(policy.feed_decision(), FeedDecision::Withhold);
    assert!(policy.must_zero_output());

    assert_eq!(
        policy.on_control_cycle(now + CONTROL_PERIOD_US, true, true),
        FeedDecision::Feed
    );
    assert!(policy.must_zero_output(), "AI is still latched");
}

#[test]
fn ai_latch_reset_is_independent_of_the_watchdog() {
    let mut policy = WatchdogPolicy::new();
    policy.latch_ai();
    assert!(policy.must_zero_output());
    assert_eq!(
        policy.on_control_cycle(CONTROL_PERIOD_US, true, true),
        FeedDecision::Feed,
        "a latched AI must not stop the control loop feeding"
    );

    // Trip the deadline monitor while AI is latched, then clear only the latch.
    let mut now = CONTROL_PERIOD_US;
    for _ in 0..DEADLINE_LIMIT {
        now += CONTROL_PERIOD_US + DEADLINE_SLACK_US + 1;
        policy.on_control_cycle(now, true, true);
    }
    assert_eq!(policy.severity(), Severity::Tripped);
    policy.reset_ai_latch();
    assert!(!policy.ai_latched());
    assert_eq!(
        policy.severity(),
        Severity::Tripped,
        "AI reset must not disarm the watchdog"
    );
    assert_eq!(policy.feed_decision(), FeedDecision::Withhold);
    assert!(policy.must_zero_output(), "the deadman still forces zero");
}
