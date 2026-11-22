// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the post-landing search signal scheduler.
//!
//! `lib.rs` is owned by another lane, so the module is included by path: the
//! exact source that would be wired into the crate is compiled and exercised
//! here without touching any other file.

#[path = "../src/search_beacon.rs"]
mod search_beacon;

use search_beacon::{
    budget_for, budget_for as band, effective_budget, pattern_for, BuzzerPattern, PowerBudget,
    SearchBeacon, SignalFrame, SignalInputs, Tone, DUTY_FULL_BUZZER, DUTY_FULL_RADIO,
    DUTY_LOW_BUZZER, DUTY_LOW_RADIO, DUTY_NOMINAL_RADIO, DUTY_OFF_RADIO, RADIO_BAND_MHZ,
    RADIO_PERIOD_TICKS, SEARCH_CALL_HZ, SEARCH_PEAK_HZ, SILENT, TICK_MS, VBAT_CRITICAL_MV,
    VBAT_LOW_MV, VBAT_NOMINAL_MV,
};

fn inputs(post_landing: bool, melody: bool, boost: bool, battery_mv: u16) -> SignalInputs {
    SignalInputs {
        post_landing,
        melody_requested: melody,
        boost_active: boost,
        battery_mv,
    }
}

fn search(battery_mv: u16) -> SignalInputs {
    inputs(true, false, false, battery_mv)
}

fn frames(n: usize, input: SignalInputs) -> Vec<SignalFrame> {
    let mut beacon = SearchBeacon::new();
    (0..n).map(|_| beacon.tick(input)).collect()
}

#[test]
fn module_is_spdx_integer_only_and_driver_free() {
    let src = include_str!("../src/search_beacon.rs");
    assert!(src.starts_with("// SPDX-License-Identifier: GPL-3.0-or-later"));
    assert!(
        !src.contains("f32"),
        "no single precision on the scheduler path"
    );
    assert!(
        !src.contains("f64"),
        "no double precision on the scheduler path"
    );
    assert!(!src.contains("std::"), "no host-only types in the module");
    assert!(!src.contains("alloc::"), "the scheduler must not allocate");
    assert!(!src.to_ascii_lowercase().contains("pin_"));
    assert!(!src.to_ascii_lowercase().contains("radio_driver"));
    assert_eq!(RADIO_BAND_MHZ, 144);
    assert_eq!(TICK_MS, 100);
}

#[test]
fn battery_bands_and_duty_limits_are_exact_at_boundaries() {
    assert_eq!(VBAT_CRITICAL_MV, 3_300);
    assert_eq!(VBAT_LOW_MV, 3_600);
    assert_eq!(VBAT_NOMINAL_MV, 3_900);

    assert_eq!(band(VBAT_CRITICAL_MV - 1), PowerBudget::Off);
    assert_eq!(band(VBAT_CRITICAL_MV), PowerBudget::Low);
    assert_eq!(band(VBAT_LOW_MV - 1), PowerBudget::Low);
    assert_eq!(band(VBAT_LOW_MV), PowerBudget::Nominal);
    assert_eq!(band(VBAT_NOMINAL_MV - 1), PowerBudget::Nominal);
    assert_eq!(band(VBAT_NOMINAL_MV), PowerBudget::Full);
    assert_eq!(band(u16::MAX), PowerBudget::Full);

    assert_eq!(PowerBudget::Off.radio_duty_percent(), DUTY_OFF_RADIO);
    assert_eq!(PowerBudget::Low.radio_duty_percent(), DUTY_LOW_RADIO);
    assert_eq!(
        PowerBudget::Nominal.radio_duty_percent(),
        DUTY_NOMINAL_RADIO
    );
    assert_eq!(PowerBudget::Full.radio_duty_percent(), DUTY_FULL_RADIO);
    assert_eq!(PowerBudget::Off.buzzer_duty_percent(), 0);
    assert_eq!(PowerBudget::Low.buzzer_duty_percent(), DUTY_LOW_BUZZER);
    assert_eq!(PowerBudget::Full.buzzer_duty_percent(), DUTY_FULL_BUZZER);
    assert!(!PowerBudget::Off.allows_signals());
    assert!(PowerBudget::Low.allows_signals());
}

#[test]
fn boost_silences_radio_and_buzzer_even_at_full_charge() {
    let mut beacon = SearchBeacon::new();
    let boosted = inputs(true, true, true, 4_200);

    assert_eq!(effective_budget(boosted), PowerBudget::Off);
    assert_eq!(pattern_for(boosted), BuzzerPattern::Silence);
    for _ in 0..40 {
        let frame = beacon.tick(boosted);
        assert_eq!(frame.pattern, BuzzerPattern::Silence);
        assert_eq!(frame.budget, PowerBudget::Off);
        assert_eq!(frame.buzzer, SILENT);
        assert_eq!(frame.radio.duty_percent, DUTY_OFF_RADIO);
        assert!(!frame.radio.transmit);
        assert!(!frame.search_active());
    }

    // Clearing boost re-arms the beacon immediately, from its first call.
    let restored = beacon.tick(search(4_200));
    assert_eq!(restored.pattern, BuzzerPattern::Search);
    assert_eq!(restored.buzzer.freq_hz, SEARCH_CALL_HZ);
}

#[test]
fn critical_battery_suppresses_search_without_boost() {
    let frame = SearchBeacon::new().tick(inputs(true, true, false, VBAT_CRITICAL_MV - 1));
    assert_eq!(frame.budget, PowerBudget::Off);
    assert_eq!(frame.pattern, BuzzerPattern::Silence);
    assert_eq!(frame.buzzer, SILENT);
    assert_eq!(frame.radio.duty_percent, 0);
    assert!(!frame.radio.transmit);
}

#[test]
fn search_preempts_melody_and_restarts_when_rearmed() {
    let mut beacon = SearchBeacon::new();
    let idle = inputs(false, true, false, 4_000);

    let melody = beacon.tick(idle);
    assert_eq!(melody.pattern, BuzzerPattern::Melody);
    assert_eq!(melody.buzzer, SILENT, "the beacon releases the buzzer");
    assert!(!melody.radio.transmit);
    assert!(!melody.search_active());

    // Both requested at once: search wins the buzzer and the air.
    let both = inputs(true, true, false, 4_000);
    let searching = beacon.tick(both);
    assert_eq!(searching.pattern, BuzzerPattern::Search);
    assert_eq!(
        searching.buzzer,
        Tone {
            freq_hz: SEARCH_CALL_HZ,
            duty: DUTY_FULL_BUZZER,
        }
    );
    assert_eq!(searching.radio.band_mhz, RADIO_BAND_MHZ);
    assert!(searching.search_active());

    // A melody interlude must not resume the pattern mid-cycle.
    let _ = beacon.tick(idle);
    let rearmed = beacon.tick(both);
    assert_eq!(rearmed.buzzer, searching.buzzer);
    assert_eq!(rearmed.pattern, BuzzerPattern::Search);
}

#[test]
fn radio_hint_uses_144mhz_and_slot_count_matches_duty_cap() {
    assert_eq!(RADIO_PERIOD_TICKS, 20);
    let cases = [
        (3_200, DUTY_OFF_RADIO, 0),
        (3_400, DUTY_LOW_RADIO, 1),
        (3_700, DUTY_NOMINAL_RADIO, 2),
        (4_000, DUTY_FULL_RADIO, 4),
    ];

    for (mv, duty, expected_slots) in cases {
        let out = frames(RADIO_PERIOD_TICKS as usize, search(mv));
        let slots = out.iter().filter(|f| f.radio.transmit).count();
        assert!(
            out.iter()
                .all(|f| f.radio.band_mhz == 144 && f.radio.duty_percent == duty),
            "band and duty cap must stay fixed within a battery band"
        );
        assert_eq!(
            slots, expected_slots,
            "TX slots must implement the duty cap"
        );
        assert!(out.iter().all(|f| !f.radio.transmit || f.search_active()));
    }
}

#[test]
fn search_pattern_is_periodic_and_two_instances_agree() {
    let a = frames(120, search(4_000));
    let b = frames(120, search(4_000));
    assert_eq!(a, b, "identical input streams must be bit-identical");

    let tones: Vec<Tone> = a.iter().map(|f| f.buzzer).collect();
    let cycle = 24;
    assert_eq!(&tones[..cycle], &tones[cycle..2 * cycle]);
    assert_eq!(&tones[cycle..2 * cycle], &tones[2 * cycle..3 * cycle]);
    assert!(tones.iter().any(|t| t.freq_hz == SEARCH_CALL_HZ));
    assert!(tones.iter().any(|t| t.freq_hz == SEARCH_PEAK_HZ));
    assert!(tones.iter().any(|t| *t == SILENT), "pattern has gaps");
    assert!(tones
        .iter()
        .all(|t| t.freq_hz == 0 || t.duty == DUTY_FULL_BUZZER));
}

#[test]
fn duty_never_exceeds_the_battery_cap_over_a_voltage_sweep() {
    let mut beacon = SearchBeacon::new();
    let mut previous = 0;
    for mv in (0..=5_000u16).step_by(13) {
        let budget = budget_for(mv);
        let frame = beacon.tick(search(mv));
        assert_eq!(frame.budget, budget);
        assert_eq!(frame.radio.duty_percent, budget.radio_duty_percent());
        assert!(frame.radio.duty_percent <= DUTY_FULL_RADIO);
        assert!(
            frame.radio.duty_percent >= previous,
            "duty rises with charge"
        );
        previous = frame.radio.duty_percent;
        if budget == PowerBudget::Off {
            assert_eq!(frame.pattern, BuzzerPattern::Silence);
            assert!(frame.buzzer.is_silent());
            assert!(!frame.radio.transmit);
        }
    }
}

#[test]
fn wrapping_tick_counter_keeps_outputs_deterministic() {
    let mut wrapped = SearchBeacon::at_tick(u32::MAX);
    let first = wrapped.tick(search(4_000));
    assert_eq!(first.tick, 0, "the tick timeline wraps like Embassy time");

    let fresh = SearchBeacon::at_tick(0).tick(search(4_000));
    assert_eq!(first.pattern, fresh.pattern);
    assert_eq!(first.buzzer, fresh.buzzer);
    assert_eq!(first.radio.duty_percent, fresh.radio.duty_percent);

    let mut x = SearchBeacon::at_tick(u32::MAX - 20);
    let mut y = SearchBeacon::at_tick(u32::MAX - 20);
    for _ in 0..40 {
        assert_eq!(x.tick(search(4_000)), y.tick(search(4_000)));
    }
}

#[test]
fn reset_restores_the_idle_state() {
    let mut beacon = SearchBeacon::new();
    let _ = beacon.tick(search(4_000));
    beacon.reset();
    assert_eq!(beacon.tick_count(), 0);
    assert_eq!(beacon.pattern(), BuzzerPattern::Silence);

    let idle = beacon.tick(inputs(false, false, false, 4_000));
    assert_eq!(idle.tick, 1);
    assert_eq!(idle.pattern, BuzzerPattern::Silence);
    assert_eq!(idle.buzzer, SILENT);
    assert!(!idle.radio.transmit);

    beacon.reset();
    assert_eq!(beacon, SearchBeacon::new());
}
