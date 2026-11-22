// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the deterministic 500 Hz tick scheduler.
//!
//! `lib.rs` is owned by another lane, so the module is pulled in by path
//! instead of being re-exported. The scheduler depends only on `core`, so it
//! compiles unchanged inside this std test crate and inside the no_std
//! firmware library.

#[path = "../src/scheduler.rs"]
mod scheduler;

use scheduler::{Clock, JitterStats, Scheduler, TickReport, TICK_HZ, TICK_PERIOD_US};

#[test]
fn tick_period_is_exactly_500hz() {
    assert_eq!(TICK_HZ, 500);
    assert_eq!(TICK_PERIOD_US, 2_000);
    assert_eq!(TICK_HZ * TICK_PERIOD_US, 1_000_000);

    let s = Scheduler::new(10_000);
    assert_eq!(s.origin_us(), 10_000);
    assert_eq!(s.period_us(), TICK_PERIOD_US);
    assert_eq!(s.next_deadline_us(), 10_000);
    assert_eq!(s.next_index(), 0);
    assert_eq!(s.deadline_for(0), 10_000);
    assert_eq!(s.deadline_for(3), 16_000);
    assert_eq!(s.ticks(), 0);
    assert_eq!(s.missed_ticks(), 0);
    assert_eq!(s.jitter(), JitterStats::default());
}

#[test]
fn on_time_ticks_stay_on_grid_with_zero_jitter() {
    let mut s = Scheduler::new(1_000);
    for i in 0..10u32 {
        let now = 1_000 + i * 2_000;
        assert_eq!(s.us_until_next(now - 1), 1);
        assert_eq!(s.us_until_next(now), 0);
        let r = s.tick(now).expect("deadline reached");
        assert_eq!(
            r,
            TickReport {
                index: i,
                deadline_us: now,
                arrival_us: now,
                error_us: 0,
                missed: 0,
            }
        );
        assert_eq!(s.next_index(), i + 1);
        assert_eq!(s.next_deadline_us(), 1_000 + (i + 1) * 2_000);
    }
    assert_eq!(s.ticks(), 10);
    assert_eq!(s.missed_ticks(), 0);

    let j = s.jitter();
    assert_eq!(j.samples, 10);
    assert_eq!(j.min_us, 0);
    assert_eq!(j.max_us, 0);
    assert_eq!(j.last_us, 0);
    assert_eq!(j.sum_abs_us, 0);
    assert_eq!(j.mean_abs_us(), 0);
}

#[test]
fn early_or_duplicate_call_consumes_nothing() {
    let mut s = Scheduler::new(5_000);
    assert_eq!(s.tick(4_999), None);
    assert_eq!(s.tick(0), None);
    assert_eq!(s.ticks(), 0);
    assert_eq!(s.next_index(), 0);
    assert_eq!(s.next_deadline_us(), 5_000);
    assert_eq!(s.jitter().samples, 0);

    assert!(s.tick(5_000).is_some());
    assert_eq!(s.ticks(), 1);
    // A second release at the same instant must not fire twice in one period.
    assert_eq!(s.tick(5_000), None);
    assert_eq!(s.ticks(), 1);
    assert_eq!(s.next_deadline_us(), 7_000);
}

#[test]
fn late_arrivals_are_recorded_as_signed_jitter() {
    let mut s = Scheduler::new(0);
    assert_eq!(s.tick(0).unwrap().error_us, 0);
    let r1 = s.tick(2_050).unwrap(); // deadline 2000, 50 us late
    assert_eq!(r1.index, 1);
    assert_eq!(r1.deadline_us, 2_000);
    assert_eq!(r1.error_us, 50);
    assert_eq!(r1.missed, 0);
    let r2 = s.tick(4_017).unwrap(); // deadline 4000, 17 us late
    assert_eq!(r2.error_us, 17);

    let j = s.jitter();
    assert_eq!(j.samples, 3);
    assert_eq!(j.min_us, 0);
    assert_eq!(j.max_us, 50);
    assert_eq!(j.last_us, 17);
    assert_eq!(j.sum_abs_us, 67);
    assert_eq!(j.mean_abs_us(), 22); // 67 / 3 truncates
}

#[test]
fn long_stall_counts_missed_ticks_and_rejoins_the_grid() {
    let mut s = Scheduler::new(0);
    s.tick(0).unwrap(); // deadline 0 -> next 2000, index 1

    // Arrive at 7000 us: deadlines 4000 and 6000 were crossed after the
    // current one (2000), so two whole ticks were missed.
    let r = s.tick(7_000).unwrap();
    assert_eq!(r.index, 1);
    assert_eq!(r.deadline_us, 2_000);
    assert_eq!(r.arrival_us, 7_000);
    assert_eq!(r.error_us, 5_000);
    assert_eq!(r.missed, 2);
    assert_eq!(s.ticks(), 2);
    assert_eq!(s.missed_ticks(), 2);

    // Rejoined the fixed grid: index 4 -> deadline 8000, not "now + period".
    assert_eq!(s.next_index(), 4);
    assert_eq!(s.next_deadline_us(), 8_000);
    assert_eq!(s.deadline_for(4), 8_000);
    assert_eq!(s.jitter().max_us, 5_000);

    // A following on-time tick does not add to the missed counter.
    let r2 = s.tick(8_000).unwrap();
    assert_eq!(r2.index, 4);
    assert_eq!(r2.error_us, 0);
    assert_eq!(r2.missed, 0);
    assert_eq!(s.missed_ticks(), 2);
    assert_eq!(s.next_deadline_us(), 10_000);
}

#[test]
fn exactly_one_period_late_counts_one_missed_tick() {
    let mut s = Scheduler::new(0);
    s.tick(0).unwrap(); // deadline 0 -> next deadline 2000
    let r = s.tick(4_000).unwrap(); // deadline 2000, arrived one period late
    assert_eq!(r.error_us, 2_000);
    assert_eq!(r.missed, 1);
    assert_eq!(s.next_deadline_us(), 6_000);
    assert_eq!(s.next_index(), 3);
    assert_eq!(s.deadline_for(3), 6_000);
}

#[test]
fn custom_period_is_honored() {
    let mut s = Scheduler::with_period(100, 250);
    assert_eq!(s.period_us(), 250);
    assert_eq!(s.tick(100).unwrap().deadline_us, 100);
    assert_eq!(s.next_deadline_us(), 350);
    assert_eq!(s.tick(350).unwrap().deadline_us, 350);
    assert_eq!(s.next_deadline_us(), 600);
}

#[test]
fn clock_trait_drives_the_scheduler() {
    struct ScriptedClock {
        now_us: u32,
    }
    impl Clock for ScriptedClock {
        fn now_us(&self) -> u32 {
            self.now_us
        }
    }

    let mut s = Scheduler::new(10_000);
    let clock = ScriptedClock { now_us: 10_050 };
    let r = s.tick_now(&clock).unwrap();
    assert_eq!(r.arrival_us, 10_050);
    assert_eq!(r.deadline_us, 10_000);
    assert_eq!(r.error_us, 50);
    assert_eq!(s.next_deadline_us(), 12_000);

    clock_implies_same_result_as_direct_tick();
}

fn clock_implies_same_result_as_direct_tick() {
    struct ScriptedClock(u32);
    impl Clock for ScriptedClock {
        fn now_us(&self) -> u32 {
            self.0
        }
    }

    let arrivals = [0u32, 2_000, 4_100, 9_000, 12_000, 30_000];
    let mut via_clock = Scheduler::new(0);
    let mut direct = Scheduler::new(0);
    for &now in &arrivals {
        let a = via_clock.tick_now(&ScriptedClock(now));
        let b = direct.tick(now);
        assert_eq!(a, b);
        assert_eq!(via_clock, direct);
    }
    assert_eq!(via_clock.ticks(), arrivals.len() as u32);
    assert!(via_clock.missed_ticks() > 0);
}

#[test]
fn identical_clock_sequences_are_bit_identical() {
    let arrivals = [0u32, 2_000, 4_100, 9_000, 12_000, 30_000];
    let mut a = Scheduler::new(0);
    let mut b = Scheduler::new(0);
    for &now in &arrivals {
        assert_eq!(a.tick(now), b.tick(now));
        assert_eq!(a, b);
    }
    assert_eq!(a.ticks(), 6);
    assert_eq!(a.jitter(), b.jitter());
    assert!(a.missed_ticks() > 0);
}

#[test]
fn deadlines_and_jitter_survive_u32_rollover() {
    let origin = u32::MAX - 1_000;
    let mut s = Scheduler::new(origin);

    let r0 = s.tick(origin).unwrap();
    assert_eq!(r0.deadline_us, origin);
    let next = s.next_deadline_us();
    assert_eq!(next, 999); // wrapped past u32::MAX
    assert_eq!(s.us_until_next(0), 999);

    let r1 = s.tick(next).unwrap();
    assert_eq!(r1.index, 1);
    assert_eq!(r1.error_us, 0);

    // Deadline 2999, arrive at 6000 -> 3001 us late, one deadline crossed.
    let r2 = s.tick(6_000).unwrap();
    assert_eq!(r2.deadline_us, 2_999);
    assert_eq!(r2.error_us, 3_001);
    assert_eq!(r2.missed, 1);
    assert_eq!(s.next_deadline_us(), 6_999);
    assert_eq!(s.deadline_for(4), origin.wrapping_add(4 * TICK_PERIOD_US));
}

#[test]
fn scheduler_is_fixed_size_and_allocation_free() {
    use core::mem::size_of;

    fn assert_copy<T: Copy>() {}
    assert_copy::<Scheduler>();
    assert_copy::<JitterStats>();
    assert!(size_of::<Scheduler>() <= 64);
    assert!(size_of::<JitterStats>() <= 32);

    let mut s = Scheduler::new(0);
    let before = size_of::<Scheduler>();
    for i in 0..1_000u32 {
        let _ = s.tick(i * TICK_PERIOD_US);
    }
    assert_eq!(size_of::<Scheduler>(), before);
    assert_eq!(s.ticks(), 1_000);
    assert_eq!(s.jitter().samples, 1_000);
}

#[test]
fn reset_restores_origin_and_clears_counters() {
    let mut s = Scheduler::new(0);
    s.tick(0).unwrap();
    s.tick(9_000).unwrap();
    assert!(s.ticks() > 0 && s.missed_ticks() > 0 && s.jitter().samples > 0);

    s.reset(50_000);
    assert_eq!(s.origin_us(), 50_000);
    assert_eq!(s.period_us(), TICK_PERIOD_US);
    assert_eq!(s.next_index(), 0);
    assert_eq!(s.next_deadline_us(), 50_000);
    assert_eq!(s.ticks(), 0);
    assert_eq!(s.missed_ticks(), 0);
    assert_eq!(s.jitter(), JitterStats::default());
    assert!(s.tick(50_000).is_some());
}
