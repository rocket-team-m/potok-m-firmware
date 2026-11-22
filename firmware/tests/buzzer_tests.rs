// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the deterministic PWM buzzer scheduler.

#[path = "../src/buzzer.rs"]
mod buzzer;

use buzzer::{
    mode_for_phase, Buzzer, BuzzerMode, Note, Output, NOTE_A5, NOTE_C4, NOTE_C5, NOTE_C6, NOTE_E4,
    NOTE_G4, SILENT, TICK_MS,
};
use potok_protocol::Phase;

fn run(b: &mut Buzzer, phase: Phase, ticks: u16) -> Vec<Output> {
    (0..ticks).map(|_| b.tick(phase)).collect()
}

#[test]
fn burn_is_always_silent() {
    let mut b = Buzzer::new();
    assert_eq!(mode_for_phase(Phase::BURN), BuzzerMode::Silence);
    let out = run(&mut b, Phase::BURN, 64);
    assert!(out.iter().all(|o| *o == SILENT));
    assert_eq!(TICK_MS, 10);
}

#[test]
fn phase_priority_maps_to_patterns() {
    assert_eq!(mode_for_phase(Phase::BURN), BuzzerMode::Silence);
    assert_eq!(mode_for_phase(Phase::COAST), BuzzerMode::Melody);
    assert_eq!(mode_for_phase(Phase::APOGEE), BuzzerMode::Melody);
    assert_eq!(mode_for_phase(Phase::DESCENT), BuzzerMode::Beacon);
    assert_eq!(mode_for_phase(Phase::LANDED), BuzzerMode::Beacon);
}

#[test]
fn beacon_after_deployment_preempts_melody() {
    let mut b = Buzzer::new();
    let melody = run(&mut b, Phase::COAST, 8);
    let beacon = run(&mut b, Phase::DESCENT, 8);
    assert_ne!(melody, beacon, "beacon must preempt the idle melody");
    assert_eq!(
        beacon[0],
        Output {
            freq_hz: NOTE_C6,
            duty: 50
        }
    );
    assert_eq!(run(&mut b, Phase::LANDED, 1)[0], beacon[0]);
}

#[test]
fn melody_only_when_idle_and_cycles_deterministically() {
    let mut b = Buzzer::new();
    let first_cycle = run(&mut b, Phase::COAST, 60);
    let second_cycle = run(&mut b, Phase::APOGEE, 60);
    assert_eq!(
        first_cycle, second_cycle,
        "pattern loops must repeat exactly"
    );
    // Each note lasts its full duration in ticks before the next begins.
    assert_eq!(
        first_cycle[0],
        Output {
            freq_hz: NOTE_C4,
            duty: 50
        }
    );
    assert_eq!(
        first_cycle[1],
        Output {
            freq_hz: NOTE_C4,
            duty: 50
        }
    );
    assert_eq!(
        first_cycle[2],
        Output {
            freq_hz: NOTE_E4,
            duty: 50
        }
    );
    assert_eq!(
        first_cycle[4],
        Output {
            freq_hz: NOTE_G4,
            duty: 50
        }
    );
    assert_eq!(
        first_cycle[6],
        Output {
            freq_hz: NOTE_C5,
            duty: 50
        }
    );
    assert!(first_cycle.contains(&SILENT), "melody has rests");
}

#[test]
fn beacon_is_repeating_and_has_rests() {
    let mut b = Buzzer::new();
    let first = run(&mut b, Phase::DESCENT, 32);
    let second = run(&mut b, Phase::DESCENT, 32);
    assert_eq!(first, second);
    assert!(first.contains(&SILENT));
    assert!(first.iter().any(|o| o.freq_hz == NOTE_C6 && o.duty == 50));
}

#[test]
fn pattern_restarts_on_mode_change() {
    let mut b = Buzzer::new();
    let melody = run(&mut b, Phase::COAST, 10);
    let beacon = run(&mut b, Phase::DESCENT, 10);
    let back_to_melody = run(&mut b, Phase::COAST, 10);
    assert_eq!(
        beacon[0],
        Output {
            freq_hz: NOTE_C6,
            duty: 50
        }
    );
    assert_eq!(
        back_to_melody, melody,
        "melody restarts from its first note"
    );
}

#[test]
fn two_instances_are_bit_identical() {
    let phases = [
        Phase::BURN,
        Phase::COAST,
        Phase::COAST,
        Phase::APOGEE,
        Phase::DESCENT,
        Phase::LANDED,
        Phase::BURN,
    ];
    let mut a = Buzzer::new();
    let mut c = Buzzer::new();
    let mut out_a = Vec::new();
    let mut out_c = Vec::new();
    for p in phases {
        for _ in 0..16 {
            out_a.push(a.tick(p));
            out_c.push(c.tick(p));
        }
    }
    assert_eq!(out_a, out_c);
}

#[test]
fn note_table_is_integer_hertz() {
    assert_eq!(
        Note::rest(3),
        Note {
            freq_hz: 0,
            ticks: 3
        }
    );
    assert_eq!(Note::tone(NOTE_A5, 2).freq_hz, 880);
    assert_eq!(NOTE_C6, 1047);
    assert_eq!(SILENT.duty, 0);
}

#[test]
fn isr_path_uses_no_floating_point() {
    let src = include_str!("../src/buzzer.rs");
    assert!(
        !src.contains("f32"),
        "ISR path must not use single precision"
    );
    assert!(
        !src.contains("f64"),
        "ISR path must not use double precision"
    );
}
