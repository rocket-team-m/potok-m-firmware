<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# HIL — real-board bring-up checklist

Hardware-in-the-loop verification for the STM32H743 controller. This is a
procedure to run on a real board, not a claim that it has been run. Every result
cell is a blank to be filled from the bench record; nothing is a pass by
silence.

Green host gates are a precondition, not a substitute: see
[SIL — what SIL does not cover](SIL.md#what-sil-does-not-cover) and the
`Аппаратные тесты` section of
[`FIRMWARE_RUST_PLAN.md`](../FIRMWARE_RUST_PLAN.md).

**Current state:** the firmware is **not flight-qualified**. The target adapter
`hw::build_board()` is `todo!("assign UART pins/DMA streams from the board
schematic")` in `firmware/src/main.rs`, and `Rx::take`, `Tx::send`,
`sensor_flags`, `motor_armed`, `poll_phase_event` and `apply_command` are still
safe stubs. Sections 4–9 cannot be executed until the pin map, the board adapter
and the actuator/sensor drivers exist. Record the blocking item rather than
marking a box green.

## 0. Session record

| Field | Value |
|---|---|
| Date / operator | |
| Board serial / revision | |
| Schematic revision | |
| BOM revision | |
| Firmware commit and profile (compat / lowlat) | |
| Flashed image hash | |
| Equipment (supply, DMM, logic analyser, scope) | |
| Host gates green before session (`cargo test --workspace`, `cargo check --target thumbv7em-none-eabihf`) | |

Stop the session on the first failed safety item. A partial run leaves the
failed item open in the record; it is never downgraded to a note.

## 1. Power and BEC

No value in this section is filled in advance: rail names, voltages, current
limits and test points come from the schematic and BOM, not from this document.

- [ ] Confirm chip variant, `memory.x` FLASH/RAM layout and the time-driver
      timer against the BOM **before** first power-on (README §5; the `small`-
      scope starting points there are not approvals).
- [ ] With the board unpowered, ohmmeter-check rails for shorts and check that
      no UART/USB path can back-feed the controller.
- [ ] Power from a current-limited bench supply at the schematic input rating;
      record the current-limit setting before connecting.
- [ ] Fill and measure every rail at its schematic test point:

| Rail / BEC output | Source (schematic ref) | Expected (from schematic) | Measured | Limit |
|---|---|---|---|---|
| MCU supply | | | | |
| Sensor supply | | | | |
| Actuator / servo supply | | | | |
| Deployment / pyro supply | | | | |
| Radio / beacon supply | | | | |

- [ ] Measure under representative load (servos commanded, radio transmitting,
      beacon active) and check ripple and sag with the scope; record worst case.
- [ ] Check power sequencing and inrush against the schematic; no rail may
      exceed its absolute maximum during startup.
- [ ] Brownout: cycle the input supply below the BEC dropout and back; confirm
      a clean reset with no latched half-state in the deployment or separation
      logic (both latch only until an explicit ground reset).
- [ ] Cold flash: erase, program and boot from a fully unpowered board with
      `probe-rs`; record any failure to boot.

## 2. Pin map placeholder

No pin numbers, alternate functions or DMA streams are assigned in this repo:
`main.rs:55` is the `todo!()` boundary. Fill this table from the schematic and
BOM only. A cell that has no schematic reference stays empty — never guess an
AF or a stream to make a build pass.

| Logical role (code-side name) | Port/pin | AF | DMA stream/channel | Schematic ref | Status |
|---|---|---|---|---|---|
| USART TX (`hw::Tx::send`) | | | | | not wired (`todo!`) |
| USART RX (`hw::Rx::take`, double-buffer slot A) | | | | | not wired (`todo!`) |
| USART RX (double-buffer slot B) | | | | | not wired (`todo!`) |
| Time driver (README starting point: TIM2) | | | | | confirm from BOM |
| IWDG (`setup_iwdg`, `IWDG1`) | — | — | — | | peripheral, no pin |
| Servo PWM — pitch (`apply_command`) | | | | | not wired (`todo!`) |
| Servo PWM — yaw (`apply_command`) | | | | | not wired (`todo!`) |
| Motor-armed sense (`Board::motor_armed`) | | | | | not wired (`todo!`) |
| Phase-event sources (`poll_phase_event`) | | | | | not wired (`todo!`) |
| Sensor health bits (`sensor_flags`: IMU/baro/mag/pitot per `main.rs`) | | | | | not wired (`todo!`) |
| Sensor buses — accel/baro/battery/temperature (`self_test::SENSOR_CHANNELS`) | | | | | not wired (`todo!`) |
| Buzzer PWM (`buzzer::Buzzer::tick`) | | | | | scheduler only |
| Search radio control (`search_beacon::RadioHint`) | | | | | scheduler only, no driver in repo |
| Deployment spring output A (`SpringOutputs::a`) | | | | | scheduler only |
| Deployment spring output B (`SpringOutputs::b`) | | | | | scheduler only |
| Deployment arm channel A / B (`ArmChannels::a/b`) | | | | | scheduler only |

- [ ] Schematic and BOM authors sign the completed table before any target run.
- [ ] Pin assignment, AF, DMA streams and servo wiring are then mirrored into
      `main.rs`, `memory.x` and `firmware/Cargo.toml`, replacing the
      `TODO(schematic)` / `TODO(bom)` markers (README rule `[07]`).

## 3. Sensor continuity and health

- [ ] Unpowered: harness continuity from each sensor connector to its board
      connector; record each channel. An open path is a stop, not a missing
      health bit.
- [ ] Powered: run the hardware-free preflight suite with **real** inputs and
      require 6/6: `sensor_range`, `table_crc`, `config_crc`,
      `actuator_continuity`, `phase_latch`, `state_machine` (`self_test.rs`).
      The suite's `ContinuityMock` covers pitch/yaw actuator paths only — it
      does not prove a wire is connected.
- [ ] Confirm each channel of `sensor_flags()` reports the real presence/health
      of its sensor; a stub returning `0` must be replaced before this passes.
- [ ] Exercise redundant IMU/barometer voting (`sensor_voting.rs`): three-slot
      median, two-slot midpoint, a stuck sensor excluded after
      `stuck_samples` identical readings *plus* another sensor moving, and a
      stationary bench that quarantines nothing.
- [ ] Withdraw one sensor at a time on the powered bench; the vote degrades
      with a reported reason, never to a silent substitute value.
- [ ] Move a sensor while the vehicle is stationary and confirm no false
      stuck/quarantine verdict.
- [ ] Verify calibration-table CRCs and the golden config CRC on the flashed
      image (`self_test::GOLDEN_CONFIG_CRC`), not just on the host.

## 4. UART loopback and wire timing

Prerequisites: section 2 complete, `hw::Rx`/`hw::Tx` implemented, target image
flashed. Until then this section is blocked.

- [ ] Local loopback (board TX → board RX at the assigned connector): request
      frames return intact through the real DMA path; record byte count.
- [ ] Logic analyser at 2 Mbit/s, 8N1 (README §5): confirm frame lengths
      56 + 14 (compat) and 42 + 16 (lowlat) and measure wire time against the
      0.25 ms budget in the plan. The SIL cycle model is arithmetic, not this
      measurement.
- [ ] Confirm DMA double-buffer slot swaps on the real idle-line/stream
      wiring; noise before magic does not count as a miss, split magic is
      reassembled (`ReplyAssembler`).
- [ ] Physical fault injection: corrupt magic, CRC, version and seq on the
      wire; each case must update rejection counters and drive the three-miss
      AI latch (`MAX_CONSECUTIVE_MISSES = 3`) and `Command::ZERO`.
- [ ] Cross-language interop: Raspberry Pi client against firmware on both
      profiles with the generated golden frames; record direction and profile
      (STM32 sends requests, Pi sends replies — the protocol README arrows are
      documented as reversed).
- [ ] Back-to-back frames at full rate for at least one hour, including
      idle-line gaps and Pi scheduling jitter: **zero** byte loss.
- [ ] Unplug UART and kill the Pi: zero command, stale detection, and the
      watchdog behaviour in section 5.
- [ ] Cross-check the measured counter values with the `defmt`/telemetry
      counters exported by the firmware; a counter that cannot be read back is
      not evidence.

## 5. Watchdog

- [ ] Confirm IWDG1 is unleashed at boot and configured from
      `WATCHDOG_TIMEOUT_US = 100_000` (`watchdog.rs`); once unleashed only a
      reset stops it.
- [ ] Confirm pets come only from the control task after a complete cycle with
      a valid STM32-owned phase; no pet from a UART callback.
- [ ] Software deadman: 500 Hz control period (`CONTROL_PERIOD_US = 2_000`),
      `DEADLINE_SLACK_US = 250`, trip after `DEADLINE_LIMIT = 3` missed
      deadlines — verify the escalation on the target clock, not the model.
- [ ] Stall the control loop deliberately: IWDG must reset the board within its
      configured window; record the measured interval and the logged reset
      reason (a reset reason that is not persisted is an open item).
- [ ] Unplug UART / kill the Pi: AI latches off, output goes to
      `Command::ZERO`, and a stalled loop is not fed — leading to a logged IWDG
      reset. A latched AI alone must **not** stop feeding; the loop stays alive
      so the latch can be explicitly cleared.
- [ ] Recovery: after a reset, boot state is BURN with AI gated; explicit
      firmware reset clears the AI latch; no implicit recovery from a stale
      reply.

## 6. BURN gate — with servos connected

Keep the vehicle safe: motors/servos connected for measurement, propulsion and
pyro disconnected.

- [ ] With phase BURN, inject valid AI replies (including maximum
      `MAX_CORRECTION_RAW = 100` corrections): measure **no actuation** on
      pitch/yaw servo lines (`phase.rs::ai_allowed`, `interlocks::Refusal::Boost`).
- [ ] Assert the independent `MOTOR_ARMED` sense input outside BURN: output
      returns to `Command::ZERO` (`Refusal::MotorArmed`). These two gates are
      transient: removing BURN or the armed bit must not require a reset.
- [ ] After burnout, confirm a valid correction reaches the servo lines
      (slew/rate limits from `interlocks::Limits::DEFAULT` apply).
- [ ] Re-assert `MOTOR_ARMED`: zero output again, then verify the latch
      behaviour separately — latching refusals (`UnknownPhase`, `AiUnhealthy`,
      `Fault`, `Implausible`) stay zero until explicit `Interlocks::reset()`;
      a valid reply after an AI latch does **not** clear it.
- [ ] Capture scope traces for each case and attach them to the session record.

## 7. Parachute continuity and deployment

Bench rule: pyro/squib disconnected; use a continuity tester appropriate to the
circuit. Never measure continuity across a live pyro channel.

- [ ] Unpowered: continuity of both spring output paths A and B through the
      board and harness (`deployment::SpringOutputs::a/b`); record each path.
- [ ] Unpowered: independence of the two arm channels A and B
      (`deployment::ArmChannels`); no shared short between them.
- [ ] Powered, phase BURN or LANDED: any trigger sample is refused
      (`Refusal::WrongPhase`) and no output asserts.
- [ ] Powered, one arm channel only: refusal is `Refusal::ArmIncomplete { a, b }`
      and no sensor state advances.
- [ ] Powered, both arms, scripted `DeploymentSample` stream: each of the three
      triggers commits — apogee descent after verified climb, emergency
      descent rate, low-altitude backup. Defaults from
      `DeploymentConfig::DEFAULT` (apogee 30 m / 2 m drop, 5 m climb, 15 m
      backup, −1 500 cm/s, 3 descending samples, 3 m floor) are the logic
      constants, **not** flight thresholds; the flight values come from the
      owner's decision and must be recorded here.
- [ ] Confirm `descent_confirm` consecutive descending samples are required:
      a single glitch must not release.
- [ ] Once released, both outputs stay asserted; further samples cannot change
      or retract them. Re-arming is a ground-only `reset()`.
- [ ] Confirm the two independent release channels actually fire together on
      the bench and record measured continuity/response.

## 8. Buzzer and search beacon

- [ ] Phase → mode (`buzzer::mode_for_phase`): BURN silent; COAST/APOGEE idle
      melody; DESCENT/LANDED priority locator beacon (three C6 calls then a
      long gap). Measure frequency and duty on the PWM output; the module is a
      scheduler and owns no pin or timer.
- [ ] BURN silence verified on the pin, not only in the scheduler.
- [ ] Search beacon (`search_beacon.rs`): with `post_landing`, the 880 / 1 047 Hz
      pattern preempts the melody; cadence stays fixed as pack voltage drains.
- [ ] Battery bands from code are 3 300 / 3 600 / 3 900 mV
      (`VBAT_CRITICAL_MV` / `VBAT_LOW_MV` / `VBAT_NOMINAL_MV`): below critical
      and during boost all signals are silent; duty caps follow the band.
      Inject each band on the bench and record measured behaviour.
- [ ] Radio `RadioHint` (144 MHz hint, duty 0/5/10/20 % by band, TX only inside
      a slot while Search owns the air): verify the board adapter applies the
      hint. No radio driver exists in this repo — if the adapter is absent,
      mark this blocked, not passed.
- [ ] No sound from either scheduler while boost is active; no stale tone when
      the pattern changes.

## 9. Flight-ready gates

All must be checked and signed before anyone calls the firmware flight-ready.
Today none is checked (README §7: no board bring-up, pins unknown, no flight
qualification).

- [ ] Host gates green: `cargo test --workspace`, `cargo fmt --all -- --check`,
      `cargo clippy --workspace --all-targets -- -D warnings`,
      `cargo check --target thumbv7em-none-eabihf` (README §5, `sil.yml`, `ci.yml`).
- [ ] Pin map (section 2) complete and signed against schematic and BOM; no
      `todo!()` remains on the flashed target path.
- [ ] Sections 1–8 passed with dated bench records and attached captures.
- [ ] BURN gate proven with servos connected (section 6).
- [ ] Watchdog hardware reset and logged reset reason proven (section 5).
- [ ] Deployment arm/release continuity and one-shot latch proven (section 7).
- [ ] One-hour back-to-back UART run and logic-analyser captures archived
      (section 4).
- [ ] Cold flash, brownout and RAM/flash headroom on the target recorded.
- [ ] Cross-language interop on both profiles with golden frames recorded.
- [ ] Preflight self-test 6/6 with real board inputs at boot.
- [ ] Deployment thresholds and separation `Plan` values signed by the owner.
- [ ] Open items from this checklist explicitly listed with owners; none
      marked "will fix in flight".

## 10. What SIL cannot prove

Scope reference: [SIL.md](SIL.md), especially
[What SIL does not cover](SIL.md#what-sil-does-not-cover). SIL runs the real
decision sources against `MockBoard`; it proves determinism of logic, not the
board. It cannot prove:

- electrical behaviour: BEC rails under load, ripple, brownout, power
  sequencing, connector/pin contact, harness continuity;
- pin numbers, alternate functions and DMA stream selection — `build_board()`
  is `todo!()`, so SIL never constructs the real adapter;
- real-time wire behaviour: 2 Mbit/s signalling, measured frame length and wire
  time against the 0.25 ms budget, DMA double-buffer swapping on silicon,
  one-hour back-to-back byte integrity, Pi scheduling jitter;
- that physical fault injection on the wire drives the same rejection counters
  as the deterministic `fault_inject` plan;
- IWDG hardware timeout, the actual reset, or persistence of the reset reason;
- that a dead Pi or unplugged UART yields zero actuation with servos connected;
- pyro/deployment path continuity, independent arm channels, or the one-shot
  release reaching both physical outputs;
- buzzer acoustics, PWM pin assignment, radio duty on the air, battery-band
  behaviour with a real pack;
- chip variant and `memory.x` correctness, cold flash, EMC, interrupt latency,
  RAM/flash headroom on the target.

A green `sil` job is necessary before a board session and is **not sufficient**
to call the firmware flight-qualified. HIL on the real board is the final
verification.
