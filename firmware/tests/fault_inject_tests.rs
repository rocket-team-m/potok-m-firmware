// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for deterministic SIL frame fault injection.
//!
//! The module is included by path because `lib.rs` is owned by the parent task.
//! Every fault is driven through the real `step()` with `MockBoard`, so the
//! assertions cover the actual latch and zero-command path, not a copy of it.

#[path = "../src/fault_inject.rs"]
mod fault_inject;

use fault_inject::*;
use potok_m_firmware::{
    mock::MockBoard,
    phase::{PhaseEvent, PhaseFsm},
    protocol::{Phase, ReplyFrame, RequestFrame},
    Command, LinkState, MAX_CORRECTION_RAW,
};

fn reply(seq: u16, pitch: i16, yaw: i16) -> ReplyFrame {
    ReplyFrame {
        seq,
        g_pitch: pitch,
        ai_healthy: 1,
        g_yaw: yaw,
        confidence: 240,
        version: 1,
    }
}

fn nominal_request() -> RequestFrame {
    RequestFrame {
        seq: 1,
        time_us: 2_000,
        phase: Phase::COAST,
        version: 1,
        flags: 0,
        reserved: 0,
        gyro: [0.5, -0.25, 0.125],
        accel: [0.0, 9.81, 0.0],
        mag: [1.0, 2.0, 3.0],
        height: 123.5,
        pitot_dp: -17,
    }
}

fn next_seq(link: &LinkState) -> u16 {
    link.last_request_seq()
        .map_or(0, |last| last.wrapping_add(1))
}

struct Rig {
    board: MockBoard,
    fsm: PhaseFsm,
    link: LinkState,
}

impl Rig {
    /// Target in COAST, where a healthy reply may actuate: this is the regime
    /// in which a fault must be caught.
    fn coasting() -> Self {
        let mut fsm = PhaseFsm::new();
        assert_eq!(fsm.phase(), Phase::BURN);
        fsm.apply(PhaseEvent::Burnout).unwrap();
        Self {
            board: MockBoard::new(),
            fsm,
            link: LinkState::new(),
        }
    }

    /// BURN: the phase gate alone forces zero, used for sensor-side faults.
    fn burning() -> Self {
        Self {
            board: MockBoard::new(),
            fsm: PhaseFsm::new(),
            link: LinkState::new(),
        }
    }

    fn queue(&mut self, injected: &InjectedReply) {
        match injected {
            InjectedReply::Bytes(bytes) => self.board.queue_bytes(bytes),
            InjectedReply::Reply(frame) => self.board.queue_reply(frame),
            InjectedReply::Withheld => {}
        }
    }

    fn step(&mut self) -> Observation {
        drive_step(&mut self.board, &mut self.fsm, &mut self.link)
    }

    fn inject(&mut self, fault: Fault) -> Observation {
        let seq = next_seq(&self.link);
        let nominal = reply(seq, 40, -40);
        let injected = fault.inject_reply(&nominal, seq);
        self.queue(&injected);
        self.step()
    }
}

#[test]
fn taxonomy_covers_every_required_fault() {
    assert_eq!(FAULTS.len(), FAULT_COUNT);
    assert_eq!(FAULT_COUNT, 8);
    for (i, fault) in FAULTS.iter().enumerate() {
        assert_eq!(fault.name(), FAULTS[i].name());
        let names: Vec<_> = FAULTS.iter().map(|f| f.name()).collect();
        assert_eq!(names.iter().filter(|n| **n == fault.name()).count(), 1);
    }
    for fault in FAULTS {
        assert_eq!(
            fault.strikes_to_latch(),
            if fault.latches_immediately() { 1 } else { 3 }
        );
        assert_eq!(
            fault.is_reply_fault(),
            matches!(
                fault,
                Fault::CrcFailure
                    | Fault::SeqGap
                    | Fault::SeqDuplicate
                    | Fault::LateReply
                    | Fault::AiUnhealthy
            )
        );
    }
    assert!(Fault::AiUnhealthy.latches_immediately());
    assert!(!Fault::CrcFailure.latches_immediately());
}

#[test]
fn fault_plan_is_reproducible_and_covers_the_whole_campaign() {
    let a = FaultPlan::new(0xC0FFEE);
    let b = FaultPlan::new(0xC0FFEE);
    assert_eq!(a.as_slice(), b.as_slice());
    assert_eq!(a.as_slice().len(), FAULT_COUNT);

    let mut a = a;
    let mut seen = [false; FAULT_COUNT];
    let mut delivered = 0;
    while let Some(fault) = a.next_fault() {
        let index = FAULTS
            .iter()
            .position(|f| *f == fault)
            .expect("known fault");
        assert!(!seen[index], "a plan never repeats a fault");
        seen[index] = true;
        delivered += 1;
        assert_eq!(a.delivered(), delivered);
    }
    assert!(seen.iter().all(|s| *s));
    assert_eq!(a.next_fault(), None);
    assert_eq!(a.delivered(), FAULT_COUNT);

    let other = FaultPlan::new(0x1234_5678);
    assert_ne!(
        a.as_slice(),
        other.as_slice(),
        "a different seed reorders faults"
    );

    // A zero seed must still produce a complete, reproducible campaign.
    assert_eq!(FaultPlan::new(0).as_slice(), FaultPlan::new(0).as_slice());
    assert_eq!(FaultPlan::new(0).as_slice().len(), FAULT_COUNT);
}

#[test]
fn seeded_rng_never_collapses_to_zero() {
    let mut rng = SeededRng::new(0);
    let first = rng.next_u64();
    assert_ne!(first, 0);
    let mut again = SeededRng::new(0);
    assert_eq!(first, again.next_u64());
}

#[test]
fn crc_failure_is_counted_and_latches_at_three() {
    let mut rig = Rig::coasting();
    let mut last = rig.inject(Fault::CrcFailure);
    assert_safe_zero(&last);
    assert_eq!(last.misses, 1);
    assert!(!last.latched);
    for expected in 2..=3 {
        last = rig.inject(Fault::CrcFailure);
        assert_safe_zero(&last);
        assert_eq!(last.misses, expected);
        if expected < 3 {
            assert!(!last.latched);
        }
    }
    assert_latched_and_zero(&last);
    assert_eq!(last.rejected, 3);
    assert_eq!(last.disables, 1);
}

#[test]
fn seq_gap_and_duplicate_are_rejected_and_latch_at_three() {
    for fault in [Fault::SeqGap, Fault::SeqDuplicate] {
        let mut rig = Rig::coasting();
        let mut last = rig.inject(fault);
        for _ in 0..2 {
            last = rig.inject(fault);
        }
        assert_eq!(last.misses, 3, "{fault:?} must count three misses");
        assert_eq!(last.rejected, 3);
        assert_latched_and_zero(&last);
    }
}

#[test]
fn ai_unhealthy_latches_on_the_first_frame_without_a_miss_run() {
    let mut rig = Rig::coasting();
    let obs = rig.inject(Fault::AiUnhealthy);
    assert_latched_and_zero(&obs);
    assert_eq!(obs.misses, 0, "unhealthy is fatal, not a counted miss");
    assert_eq!(obs.disables, 1);
    assert_eq!(
        obs.rejected, 0,
        "unhealthy is a direct latch, not a rejected frame"
    );

    // A healthy reply cannot clear the latch, and the applied command stays zero.
    let healthy = rig.inject(Fault::StuckSensor);
    assert_latched_and_zero(&healthy);
    rig.link.reset();
    rig.board.queue_reply(&reply(next_seq(&rig.link), 25, -25));
    assert_eq!(
        rig.step().command,
        Command {
            pitch: 25,
            yaw: -25
        }
    );
}

#[test]
fn late_reply_stales_the_deadman_and_latches() {
    let mut rig = Rig::coasting();

    // Tick 1: silence is a timeout miss.
    rig.queue(&InjectedReply::Withheld);
    let first = rig.step();
    assert_safe_zero(&first);
    assert_eq!(first.misses, 1);
    assert!(!first.deadman);

    // Tick 2: the reply arrives but echoes the previous request seq, so it is
    // stale and rejected.
    let seq = next_seq(&rig.link);
    rig.queue(&Fault::LateReply.inject_reply(&reply(seq, 40, -40), seq));
    let second = rig.step();
    assert_safe_zero(&second);
    assert_eq!(second.misses, 2);

    // Tick 3: another silence reaches the three-strike latch; the deadman has
    // also been starved for three ticks.
    let third = rig.step();
    assert_latched_and_zero(&third);
    assert!(
        third.deadman,
        "a late reply must not keep the deadman healthy"
    );

    // A fresh well-formed reply clears the deadman but never the AI latch.
    rig.board.queue_reply(&reply(next_seq(&rig.link), 40, -40));
    let recovered = rig.step();
    assert!(!recovered.deadman);
    assert_latched_and_zero(&recovered);
}

#[test]
fn stuck_sensor_freezes_the_request_payload() {
    let nominal = nominal_request();
    for _ in 0..3 {
        let frozen = Fault::StuckSensor.mutate_request(&nominal).unwrap();
        assert_eq!(frozen.gyro, [0.0; 3]);
        assert_eq!(frozen.accel, [0.0; 3]);
        assert_eq!(frozen.height, 0.0);
        assert_eq!(frozen.pitot_dp, 0);
    }

    // A frozen sensor report must not open the actuator: in BURN the phase gate
    // forces zero even for a well-formed reply.
    let mut rig = Rig::burning();
    rig.board.queue_reply(&reply(next_seq(&rig.link), 90, -90));
    assert_safe_zero(&rig.step());
}

#[test]
fn out_of_range_sample_saturates_and_correction_is_clamped() {
    let nominal = nominal_request();
    let saturated = Fault::OutOfRange.mutate_request(&nominal).unwrap();
    assert_eq!(saturated.height, f32::MAX);
    assert_eq!(saturated.pitot_dp, i16::MAX);
    assert_eq!(saturated.gyro, [f32::MAX; 3]);

    // An out-of-range AI correction may be accepted but is never applied
    // unbounded: the code veto clamps it to the model's 0.1 rad limit.
    let mut rig = Rig::coasting();
    rig.board
        .queue_reply(&reply(next_seq(&rig.link), i16::MAX, i16::MIN));
    let obs = rig.step();
    assert_eq!(
        obs.command,
        Command {
            pitch: MAX_CORRECTION_RAW,
            yaw: -MAX_CORRECTION_RAW
        }
    );
}

#[test]
fn phase_mismatch_is_refused_and_the_zero_command_holds() {
    let nominal = nominal_request();
    assert_eq!(
        Fault::PhaseMismatch.mutate_request(&nominal).unwrap().phase,
        Phase::LANDED
    );
    assert_eq!(
        Fault::PhaseMismatch.phase_event(),
        Some(PhaseEvent::Touchdown)
    );
    assert_eq!(Fault::CrcFailure.phase_event(), None);

    let mut rig = Rig::burning();
    rig.board.phase_event = Fault::PhaseMismatch.phase_event();
    rig.board.queue_reply(&reply(next_seq(&rig.link), 90, 90));
    let obs = rig.step();
    assert_eq!(
        rig.fsm.phase(),
        Phase::BURN,
        "a mismatched phase is refused"
    );
    assert_safe_zero(&obs);
}

#[test]
fn a_full_planned_campaign_never_actuates_on_a_faulted_tick() {
    let plan = FaultPlan::new(0xFEED_FACE);
    for fault in plan.as_slice() {
        match fault {
            Fault::StuckSensor | Fault::OutOfRange | Fault::PhaseMismatch => {
                let mut rig = Rig::burning();
                if let Some(event) = fault.phase_event() {
                    rig.board.phase_event = Some(event);
                }
                rig.board.queue_reply(&reply(next_seq(&rig.link), 80, -80));
                assert_safe_zero(&rig.step());
            }
            Fault::AiUnhealthy => {
                let mut rig = Rig::coasting();
                assert_latched_and_zero(&rig.inject(*fault));
            }
            Fault::LateReply => {
                let mut rig = Rig::coasting();
                rig.queue(&InjectedReply::Withheld);
                assert_safe_zero(&rig.step());
                let seq = next_seq(&rig.link);
                rig.queue(&fault.inject_reply(&reply(seq, 40, -40), seq));
                assert_safe_zero(&rig.step());
                assert_latched_and_zero(&rig.step());
            }
            Fault::CrcFailure | Fault::SeqGap | Fault::SeqDuplicate => {
                let mut rig = Rig::coasting();
                let mut last = rig.inject(*fault);
                for _ in 1..fault.strikes_to_latch() {
                    last = rig.inject(*fault);
                }
                assert_latched_and_zero(&last);
            }
        }
    }
}
