// SPDX-License-Identifier: GPL-3.0-or-later
//! Contract of the independent interlock gate: it refuses on its own, without
//! relying on `LinkState`, and the only way out of a latch is `reset()`.

// `interlocks.rs` uses `crate::Command` so the same file compiles both in the
// firmware library and when included here via `#[path]`. Re-export the type at
// this test crate's root to keep that path valid in both contexts.
pub use potok_m_firmware::Command;

#[path = "../src/interlocks.rs"]
mod interlocks;

use interlocks::{Input, Interlocks, Limits, Refusal};
use potok_protocol::{flags, Phase};

fn input(phase: Phase, pitch: i16, yaw: i16, now_us: u32) -> Input {
    Input {
        phase_code: phase as u8,
        flags: 0,
        ai_healthy: true,
        requested: Command { pitch, yaw },
        now_us,
    }
}

fn gate() -> Interlocks {
    let g = Interlocks::default();
    assert_eq!(g.limits(), Limits::DEFAULT);
    g
}

#[test]
fn failsafe_is_zero_and_bound_matches_the_protocol_scale() {
    assert!(Interlocks::FAILSAFE.is_zero());
    assert_eq!(
        Limits::DEFAULT.max_command,
        potok_m_firmware::MAX_CORRECTION_RAW,
        "raw units: 0.001 rad, model bound 0.1 rad"
    );
    let mut g = gate();
    let d = g.evaluate(input(Phase::COAST, 101, 0, 0));
    assert_eq!(d.command, Interlocks::FAILSAFE);
    assert_eq!(d.refusal, Some(Refusal::Implausible));
    assert_eq!(g.latched(), Some(Refusal::Implausible));
}

#[test]
fn unknown_phase_is_zero_and_latches_until_reset() {
    let mut g = gate();
    let mut raw = input(Phase::COAST, 50, 50, 0);
    raw.phase_code = 0xff;
    let d = g.evaluate(raw);
    assert_eq!(d.command, Command::ZERO);
    assert!(!d.permitted);
    assert_eq!(d.refusal, Some(Refusal::UnknownPhase(0xff)));

    // A well-formed, plausible request on a known phase cannot clear it.
    for t in [0, 1_000, 5_000] {
        let d = g.evaluate(input(Phase::COAST, 50, -50, t));
        assert_eq!(d.command, Command::ZERO);
        assert_eq!(g.latched(), Some(Refusal::UnknownPhase(0xff)));
    }
    assert_eq!(g.counters.trips, 1);

    g.reset();
    assert_eq!(g.latched(), None);
    let d = g.evaluate(input(Phase::COAST, 50, -50, 6_000));
    assert!(d.permitted);
    assert_eq!(d.command, Command { pitch: 10, yaw: -10 });
}

#[test]
fn boost_and_motor_armed_ban_ai_without_latching() {
    let mut g = gate();

    let d = g.evaluate(input(Phase::BURN, 100, 100, 0));
    assert_eq!(d.command, Command::ZERO);
    assert_eq!(d.refusal, Some(Refusal::Boost));
    assert_eq!(g.latched(), None);

    let d = g.evaluate(input(Phase::COAST, 50, 0, 0));
    assert!(d.permitted);
    assert_eq!(d.command, Command { pitch: 10, yaw: 0 });

    let mut armed = input(Phase::COAST, 50, 0, 1_000);
    armed.flags = flags::MOTOR_ARMED;
    let d = g.evaluate(armed);
    assert_eq!(d.command, Command::ZERO);
    assert_eq!(d.refusal, Some(Refusal::MotorArmed));
    assert_eq!(g.latched(), None);

    let d = g.evaluate(input(Phase::COAST, 50, 0, 2_000));
    assert!(d.permitted);
    assert_eq!(d.command, Command { pitch: 20, yaw: 0 });
}

#[test]
fn rate_limit_slews_and_throttles_without_a_jump() {
    let mut g = gate();

    let d = g.evaluate(input(Phase::COAST, 100, 0, 0));
    assert_eq!(d.command, Command { pitch: 10, yaw: 0 });
    assert!(d.limited);
    assert_eq!(g.last_command(), d.command);

    // Same tick: hold the already-vetted command, never issue a second one.
    let d = g.evaluate(input(Phase::COAST, 100, 0, 500));
    assert_eq!(d.command, Command { pitch: 10, yaw: 0 });
    assert!(d.limited);

    let mut t = 500;
    for expected in (20..=100).step_by(10) {
        t += 1_000;
        let d = g.evaluate(input(Phase::COAST, 100, 0, t));
        assert!(d.permitted);
        assert_eq!(d.command.pitch, expected, "slew never exceeds 10 raw/tick");
    }

    // At the bound, an unchanged request is not a rate-limited change.
    let d = g.evaluate(input(Phase::COAST, 100, 0, t + 1_000));
    assert_eq!(d.command, Command { pitch: 100, yaw: 0 });
    assert!(!d.limited);

    assert_eq!(g.counters.throttled, 1);
    assert_eq!(g.counters.slewed, 9);
    assert_eq!(g.counters.accepted, 12);
}

#[test]
fn implausible_command_latches_even_at_the_i16_edge() {
    let mut g = gate();
    let d = g.evaluate(input(Phase::COAST, i16::MIN, 0, 0));
    assert_eq!(d.command, Command::ZERO);
    assert_eq!(d.refusal, Some(Refusal::Implausible));
    assert_eq!(g.latched(), Some(Refusal::Implausible));

    g.reset();
    let d = g.evaluate(input(Phase::COAST, 100, -100, 0));
    assert!(d.permitted);
    // A diagonal request is plausible, but still rate-limited per axis.
    assert_eq!(d.command, Command { pitch: 10, yaw: -10 });
}

#[test]
fn unhealthy_ai_and_fault_flag_latch_independently() {
    let mut g = gate();
    let mut sick = input(Phase::COAST, 50, 50, 0);
    sick.ai_healthy = false;
    let d = g.evaluate(sick);
    assert_eq!(d.command, Command::ZERO);
    assert_eq!(d.refusal, Some(Refusal::AiUnhealthy));

    g.reset();
    let mut faulty = input(Phase::COAST, 50, 50, 0);
    faulty.flags = flags::FAULT;
    let d = g.evaluate(faulty);
    assert_eq!(d.command, Command::ZERO);
    assert_eq!(d.refusal, Some(Refusal::Fault));

    assert!(Refusal::AiUnhealthy.latches());
    assert!(Refusal::Fault.latches());
    assert!(!Refusal::Boost.latches());
    assert!(!Refusal::MotorArmed.latches());
}

#[test]
fn latch_reason_persists_and_only_reset_clears_it() {
    let mut g = gate();
    let mut bad = input(Phase::COAST, 50, 50, 0);
    bad.ai_healthy = false;
    g.evaluate(bad);
    assert_eq!(g.counters.trips, 1);

    for t in [100, 10_000, 100_000] {
        let mut healthy = input(Phase::COAST, 50, 50, t);
        healthy.flags = 0;
        let d = g.evaluate(healthy);
        assert_eq!(d.command, Command::ZERO);
        assert_eq!(d.refusal, Some(Refusal::AiUnhealthy), "original cause stays");
        assert!(!d.permitted);
    }
    assert_eq!(g.counters.trips, 1, "a latch trips once, not once per tick");
    assert_eq!(g.counters.blocked, 4);

    g.reset();
    assert_eq!(g.counters.resets, 1);
    let d = g.evaluate(input(Phase::COAST, 50, 50, 100_100));
    assert!(d.permitted);
    assert_eq!(d.command, Command { pitch: 10, yaw: 10 });
}

#[test]
fn zero_request_is_accepted_without_tripping() {
    let mut g = gate();
    let d = g.evaluate(input(Phase::COAST, 0, 0, 0));
    assert_eq!(d.command, Command::ZERO);
    assert!(d.permitted);
    assert!(!d.limited);
    assert_eq!(d.refusal, None);
    assert_eq!(g.counters.accepted, 1);
    assert_eq!(g.counters.trips, 0);
    assert_eq!(g.latched(), None);
}

#[test]
fn rate_limit_clock_is_wrap_safe() {
    let mut g = gate();
    let d = g.evaluate(input(Phase::COAST, 50, 0, u32::MAX - 10));
    assert_eq!(d.command, Command { pitch: 10, yaw: 0 });

    let d = g.evaluate(input(Phase::COAST, 50, 0, 100));
    assert_eq!(d.command, Command { pitch: 10, yaw: 0 }, "wrapped 111 us: throttled");
    assert!(d.limited);

    let d = g.evaluate(input(Phase::COAST, 50, 0, 1_200));
    assert_eq!(d.command, Command { pitch: 20, yaw: 0 }, "wrapped 1211 us: accepted");
}

#[test]
fn reset_restores_the_zero_rate_history() {
    let mut g = gate();
    let d = g.evaluate(input(Phase::COAST, 100, 0, 0));
    assert_eq!(d.command.pitch, 10);
    g.reset();
    assert!(g.last_command().is_zero());

    // Without the reset this would slew from 10 to 20 at t = 1000.
    let d = g.evaluate(input(Phase::COAST, 100, 0, 1));
    assert_eq!(d.command.pitch, 10);
    assert_eq!(g.counters.resets, 1);
}

#[test]
#[should_panic(expected = "max_command must be positive")]
fn unusable_configuration_is_a_loud_error() {
    let _ = Interlocks::new(Limits {
        max_command: 0,
        ..Limits::DEFAULT
    });
}
