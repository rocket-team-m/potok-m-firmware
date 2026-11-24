// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the 500 Hz SIL timing budget.
//!
//! `lib.rs` is owned by the parent task, so the three modules are pulled in by
//! path: the exact firmware sources are exercised on the host. The re-exports
//! make `crate::Command` / `crate::MAX_CORRECTION_RAW` inside `perf.rs` resolve
//! to the real types, as in `perf_tests.rs`.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

#[path = "../src/perf.rs"]
pub mod perf;
#[path = "../src/scheduler.rs"]
pub mod scheduler;
#[path = "../src/timing_budget.rs"]
mod timing_budget;

use scheduler::TICK_HZ;
use timing_budget::{
    assert_tick_budget, BudgetReport, Scenario, CYCLES_PER_US, SCENARIOS, SIL_ACCEPTED,
    SIL_FRAME_FLOOD, SIL_IDLE, SIL_MAX_AXES, SIL_MAX_CRC_BYTES, SIL_MAX_QUANT_FIELDS,
    SIL_MAX_RX_BYTES, SIL_REJECTED, TICK_CYCLES, TICK_US, WORST_CASE_CYCLES,
};

#[test]
fn tick_budget_constants_are_pinned() {
    assert_eq!(TICK_HZ, 500);
    assert_eq!(TICK_US, 2_000);
    assert_eq!(CYCLES_PER_US, 550);
    assert_eq!(TICK_CYCLES, 1_100_000);
    assert_eq!(perf::CYCLE_BUDGET_US, 50);
    assert_eq!(perf::CYCLE_BUDGET, 27_500);
    assert_eq!(CYCLES_PER_US * TICK_US, TICK_CYCLES);
    // 50 us out of 2000 us is 2.5% of a tick.
    assert_eq!(perf::CYCLE_BUDGET_US * TICK_HZ, 25_000);
}

#[test]
fn sil_worst_case_inputs_cover_the_double_buffer() {
    assert_eq!(SIL_MAX_QUANT_FIELDS, 11);
    assert_eq!(SIL_MAX_AXES, 2);
    assert_eq!(SIL_MAX_RX_BYTES, 2 * potok_protocol::REPLY_SIZE as u32);
    assert_eq!(
        SIL_MAX_CRC_BYTES,
        (potok_protocol::REQUEST_SIZE + 2 * potok_protocol::REPLY_SIZE) as u32
    );
}

#[test]
fn scenario_estimate_is_the_perf_model_with_the_same_inputs() {
    for &s in &SCENARIOS {
        let e = s.estimate();
        assert_eq!(e.rx_scan, s.rx_bytes * 6, "{}", s.name);
        assert_eq!(e.crc, s.crc_bytes * 4, "{}", s.name);
        assert_eq!(e.quantize, s.quant_fields * 12, "{}", s.name);
        assert_eq!(e.gate, if s.gate { 12 } else { 0 }, "{}", s.name);
        assert_eq!(e.fixed_math, s.axes * 4, "{}", s.name);
        assert_eq!(e.overhead, 200, "{}", s.name);
        assert_eq!(e.cycles(), s.cycles(), "{}", s.name);
    }
}

#[test]
fn sil_scenario_costs_are_pinned_and_ordered() {
    assert_eq!(SIL_IDLE.cycles(), 556);
    assert_eq!(SIL_REJECTED.cycles(), 708);
    assert_eq!(SIL_ACCEPTED.cycles(), 716);
    assert_eq!(WORST_CASE_CYCLES, 856);
    assert_eq!(SIL_FRAME_FLOOD.cycles(), WORST_CASE_CYCLES);

    for pair in SCENARIOS.windows(2) {
        assert!(
            pair[0].cycles() < pair[1].cycles(),
            "{} must cost less than {}",
            pair[0].name,
            pair[1].name
        );
    }
}

#[test]
fn worst_case_report_has_pinned_cycles_margin_and_utilization() {
    let report = BudgetReport::worst_case();
    assert_eq!(report.scenario, SIL_FRAME_FLOOD);
    assert_eq!(report.cycles, 856);
    assert_eq!(report.budget, 27_500);
    assert_eq!(report.margin_cycles, 26_644);
    assert_eq!(report.margin_us, 48);
    assert_eq!(report.tick_us, 2_000);
    assert_eq!(report.tick_cycles, 1_100_000);
    assert_eq!(report.tick_utilization_ppm, 778);

    assert_eq!(report.cycles + report.margin_cycles, report.budget);
    assert_eq!(report.margin_us, report.margin_cycles / 550);
    assert_eq!(report, BudgetReport::evaluate(SIL_FRAME_FLOOD));
    assert_eq!(report, BudgetReport::worst_case());
}

#[test]
fn every_scenario_fits_and_keeps_positive_margin() {
    for &scenario in &SCENARIOS {
        let report = BudgetReport::evaluate(scenario);
        assert!(report.cycles <= report.budget, "{}", scenario.name);
        assert!(report.margin_cycles > 0, "{}", scenario.name);
        assert_eq!(report.cycles + report.margin_cycles, report.budget);
        assert!(report.tick_utilization_ppm < 50_000, "{}", scenario.name);
    }
}

#[test]
#[should_panic(expected = "500 Hz timing budget exceeded: SIL_FLOOD_TEST")]
fn over_budget_scenario_is_refused_loudly() {
    let flood = Scenario::new(
        "SIL_FLOOD_TEST",
        SIL_MAX_RX_BYTES * 1_000,
        SIL_MAX_CRC_BYTES,
        SIL_MAX_QUANT_FIELDS,
        SIL_MAX_AXES,
        true,
    );
    let _ = BudgetReport::evaluate(flood);
}

#[test]
fn assert_tick_budget_returns_the_dominant_scenario_deterministically() {
    let first = assert_tick_budget();
    let second = assert_tick_budget();
    assert_eq!(first, second);
    assert_eq!(first, BudgetReport::worst_case());
    assert_eq!(first.scenario, SIL_FRAME_FLOOD);
    assert!(SCENARIOS.iter().all(|&s| s.cycles() <= first.cycles));
}
