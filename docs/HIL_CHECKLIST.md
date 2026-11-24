<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# HIL — typed bring-up matrix

Hardware-in-the-loop verification for the STM32H723 controller (family
correction from STM32H743; exact variant/package and every physical cell
`NOT ASSIGNED` until `PIN_MATRIX.md` §5 is signed). **This is a procedure to run
on a real board, not a claim that it has been run.** Every result cell is
blank; nothing is a pass by silence. No pin, rail, part or measurement in this
document is asserted as present on a physical board.

**H723 correction (task FW723-PINS).** The MCU line is STM32H723, not
STM32H743. H743 identifiers elsewhere (`README.md`, `firmware/Cargo.toml`
`stm32h743zi`, `.cargo/config.toml` `STM32H743ZI`, `docs/RENODE.md`'s
`stm32h743.repl`, `firmware/src/perf.rs:32` `CORE_HZ = 480_000_000`,
`memory.x` FLASH 2048K, `docs/POWER_BUDGET.md` A6/L1) are H743-era and are
**owner-lane items, not resolved here**. This document changes only the typed
bring-up procedure; it asserts no H723 pin, rail or measurement.

The owner's wiring block diagram is the BOM/interface source for the typed
rows below
(`/home/sehaxe/Downloads/5qWfXcTL-6tWOrtA-TiyO4iRbKyXnHvwpJ6E2ErQzmhOeHHmkUE3JOkdnEZg6Sugkj_in42w-5MGQiTDIiXvGwgk.jpg`).
It shows interface tags (`PWM`, `UART`, `SPI`, `1W`, `АЦП`, `USB`) and no pin
number, AF, DMA stream, bus instance or chip select. No `I2C` tag and no `QSPI`
tag appear on the drawing: the KX134 pair, LIS2MDL and W25Q512 are all
`SPI`-tagged (re-read 2026-11-24). `PIN_MATRIX.md` §1C is the single
authoritative tag transcription; §1D lists every interface ambiguity as
`NOT ASSIGNED` with the exact schematic/BOM question. Every physical cell stays
`NOT ASSIGNED` until schematic-to-BOM reconciliation (BU12) and `PIN_MATRIX.md`
§5 sign-off.

Typed companions:

- [PIN_MATRIX.md](PIN_MATRIX.md) — per-role pin/interface matrix; all physical
  cells `NOT ASSIGNED`.
- [POWER_BUDGET.md](POWER_BUDGET.md) — rail/load budget, worked arithmetic and
  the exact measurement steps referenced as `PB §6` below. `PIN §n` below means
  `PIN_MATRIX.md` section `n`.

Green host gates are a precondition, not a substitute: see
[SIL — what SIL does not cover](SIL.md#what-sil-does-not-cover) and the
`Аппаратные тесты` section of
[`FIRMWARE_RUST_PLAN.md`](../FIRMWARE_RUST_PLAN.md).

**Current state:** the firmware is **not flight-qualified**. The target adapter
`hw::build_board()` is `todo!("assign UART pins/DMA streams from the board
schematic")` in `firmware/src/main.rs:79`, and `Rx::take`, `Tx::send`,
`sensor_flags`, `motor_armed`, `poll_phase_event` and `apply_command` are safe
stubs. Sections 1–9 cannot be executed until the pin map, the board adapter and
the actuator/sensor drivers exist. Record the blocking item rather than marking
a box green.

## 0. Typed result vocabulary

| Token | Meaning for a result cell |
|---|---|
| blank | not run — the default, never a pass |
| `PASS` | executed on the board, measured, recorded |
| `FAIL` | executed, criterion missed — stop the session |
| `BLOCKED` | prerequisite row is `NOT ASSIGNED` / no driver |
| `N/A` | reason named in the same row; never a silent omission |
| `NOT ASSIGNED` | physical value unknown; schematic + BOM only |

Stop the session on the first `FAIL`. A partial run leaves that item open; it is
never downgraded to a note.

## 1. Session record

| Field | Value |
|---|---|
| Date / operator | |
| Board serial / revision | |
| H723 variant / package (from BOM) | |
| Schematic revision | |
| BOM revision | |
| Firmware commit and profile (compat / lowlat) | |
| H743-only tool identifiers reconciled (`stm32h743zi`, probe-rs chip, Renode platform) | |
| Flashed image hash | |
| Equipment (supply, DMM, logic analyser, scope, current probe) | |
| Host gates green before session (`cargo test --workspace`, `cargo check --target thumbv7em-none-eabihf`) | |

## 2. Bring-up gate matrix

Rows are typed; `Ref` names the authoritative procedure. Rail values are
`NOT ASSIGNED` here — see [POWER_BUDGET.md](POWER_BUDGET.md) for the placeholder
budget and `PB §6` for the measurement steps.

| ID | Gate | Type | Ref | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|---|
| BU01 | Confirm **STM32H723** exact variant/package, `memory.x` FLASH/RAM and time-driver timer against BOM **before** first power-on | precondition | PIN §1A/§1B, README §5 | H723 variant + package + map signed (FLASH ≤ 1 MB; 2048K is an H743 value); `stm32h743zi` / `STM32H743ZI` / TIM2 starting points are not approvals | | |
| BU02 | Unpowered rail short and back-feed check | electrical | PB §6 step 1 | no near-zero path; no UART/USB back-feed | | |
| BU03 | Supply voltage and current limit recorded before connecting | electrical | PB §6 step 2 | both values recorded; stop if either `NOT ASSIGNED` | | |
| BU04 | Each rail measured at its schematic test point | electrical | PB §6 steps 3–4 | voltage and current recorded per rail | | |
| BU05 | Under representative load: ripple and sag on scope | electrical | PB §6 step 7 | `V_pp` / `V_min` recorded, worst case named; limit from schematic | | |
| BU06 | Servo stall current measured, A10 replaced | electrical | PB §6 step 6 | per-servo and all-six peak recorded | | `BLOCKED` — no PWM driver |
| BU07 | Inrush and power sequencing | electrical | PB §6 step 8 | peak/duration and rail order recorded; no rail past abs max | | |
| BU08 | Brownout cycle below BEC dropout and back | electrical | PB §6 step 9 | clean reset; no latched half-state in deployment/separation | | |
| BU09 | Cold flash with `probe-rs` from fully unpowered board | electrical | PB §6 step 11 | boot succeeds; image hash matches §1 | | |
| BU10 | Pin map complete and signed | precondition | PIN §5 | every physical cell `ASSIGNED` with schematic ref | | `BLOCKED` |
| BU11 | `TODO(schematic)` / `TODO(bom)` replaced in `main.rs`, `memory.x`, `firmware/Cargo.toml` | code | README `[07]` | no target-path `todo!()` remains | | `BLOCKED` |
| BU12 | **Schematic-to-BOM reconciliation from the owner wiring diagram before any pin assignment** | precondition | PIN §1 items 6–9, §1C, C13; owner diagram path in header | signed list resolves ICM-42688-P/MS5611 absence, two KX134-1211, H3LIS331DL, two MMC5983MA, two LPS22HH, thermometer, photoresistors, GPS, USB/power, video pair, recovery servos 1–2, and the SPI-vs-OCTOSPI, LoRa UART-vs-SPI and Pi UART-vs-USB implementation decisions (PIN §1D Q5–Q7); no pin assigned before this closes | | `BLOCKED` |

## 2A. H723 bring-up order (power → clock → SWD → UART → sensors → servo outputs)

Execute strictly in this order; a later stage is `BLOCKED` until its
prerequisite is signed or measured. This is a procedure, not a run record:
every Result cell is blank. `PIN §n` and `PB §n` name the companion documents.

| Order | Stage | Prerequisite | Ref | Criterion before advancing | Result | Evidence / blocker |
|---:|---|---|---|---|---|---|
| 0 | Variant/tooling precondition | BOM + schematic signed for the exact H723 variant/package | PIN §1A/§1B/§1C, BU01, BU10–BU12 | H723 part and package identified; H743-only identifiers (`stm32h743zi`, `STM32H743ZI`, `stm32h743.repl`, `CORE_HZ`) reconciled in their owner lanes; **BU12 diagram-to-BOM reconciliation closed**; no §3 cell filled by assumption | | `BLOCKED` |
| 1 | Power | Order 0 | BU02–BU05, BU07, BU08; PB §6 steps 1–4, 7–9 | unpowered shorts/back-feed clear; supply V/I limits and R1–R4 voltages recorded; ripple/sag, inrush and brownout recorded; stop on any `FAIL` | | |
| 2 | Clock | Order 1 | BU01; PB §6 step 9; `perf.rs:32` | oscillator/PLL source and target system clock from the schematic measured; time-driver timer (TIM2 starting point only) and the 500 Hz tick verified; H743 480 MHz cycle-budget assumption flagged, not reused | | `BLOCKED` — clock source `NOT ASSIGNED` |
| 3 | SWD | Orders 1–2 | BU09, BU10, BU11; PB §6 step 11 | debug header pinout from schematic; probe attached with an H723 chip target (never `STM32H743ZI`); cold flash boots; image hash matches §1 | | `BLOCKED` — SWD header `NOT ASSIGNED` |
| 4 | UART | Order 3 + Rx/Tx implemented | PIN §3 P01–P04, C01/C06; §5 U01–U08 | USART instance/pins/AF and two distinct DMA slots signed; 2 Mbit/s 8N1 loopback and wire time recorded; double-buffer swap and fault injection measured | | `BLOCKED` |
| 5 | Sensors | Order 4 + bus drivers | PIN §3 P11–P27; §4 S01–S08; §1D Q2–Q4, Q18 | SPI bus pins and CS signed; any I²C remap decided; continuity, health bits and redundant vote recorded; ADC front-end measured; no sensor part assumed from the H743 BOM | | `BLOCKED` |
| 6 | Servo outputs | Orders 1–5 + PWM driver + pin map signed | PIN §3 P07–P09, C03; §7 G01–G05; PB §6 step 6 | timer channels/AF per servo signed; 6 servos ↔ 2 axes mapping decided; stall current measured; BURN gate proven with servos connected and no AI actuation | | `BLOCKED` |

After order 6: run §6 (watchdog), §8 (deployment), §9 (buzzer/search), then
the §10 flight-ready gates. Do not start a later stage to "unblock" an earlier
one.

## 3. Pin map (summary — full matrix in PIN_MATRIX.md)

The table is intentionally blank. Only `PIN_MATRIX.md` §3 may carry a physical
value, and only after `PIN_MATRIX.md` §5 is signed.

| Code-side role | Type | BOM | Port/pin | AF | DMA | Schematic ref | Status |
|---|---|---|---|---|---|---|---|
| USART TX (`hw::Tx::send`) | `UART_TX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| USART RX slot A (`hw::Rx::take`) | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| USART RX slot B | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| Time driver (TIM2 — H743-era starting point) | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — H723 timer instance `NOT ASSIGNED` |
| IWDG (`setup_iwdg(p.IWDG1)` starting point) | `PERIPHERAL` | B01 | — | — | — | `NOT ASSIGNED` | `PERIPHERAL`, no pin; instance `NOT ASSIGNED` |
| Servo PWM pitch / yaw | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| Servo 3..6 (BOM only) | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no code role |
| `MOTOR_ARMED` sense | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `STUB` |
| Phase-event sources | `GPIO_IN` | B02–B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| `sensor_flags` IMU/baro/mag/pitot | `GPIO_IN` | B02–B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB`; pitot part unknown |
| Sensor buses — SPI set (H3LIS331DL, KX134-1211 ×2, LIS2MDL, LPS22HH ×2, LSM6DSR, MMC5983MA ×2) | `SPI_*` | B03–B07, B12, B15 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no drivers; CS unknown (PIN §1D Q2–Q4) |
| Thermometer / photoresistors | `ONE_WIRE` / `ADC_IN` | B14 / B13 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no drivers; no 1-Wire peripheral |
| SPI-tagged flash W25Q512 512 Mbit / 64 MB | `SPI_*` (OCTOSPI open) | B09 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — instance/CS unknown (PIN §1D Q5) |
| LoRa 1262F30-T UART + GPS UART | `UART_TX`/`UART_RX` | B08 / B16 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no drivers; was SPI in old BOM |
| USB port / battery + toggle | `USB` / `GPIO_IN` | B18 | `NOT ASSIGNED` | `NOT ASSIGNED` | — | `NOT ASSIGNED` | `BLOCKED` — rails/test points unknown |
| Video camera + flash | `OFF_MCU` | B17 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no MCU interface drawn |
| Recovery servos 1–2 | `TIM_PWM` | B10/B11 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — conflicts with pyro/spring B11 |
| Buzzer PWM | `SCHEDULER` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` |
| Search radio control | `SCHEDULER` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` — no driver |
| Deployment spring A / B | `GPIO_OUT` | B11 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` |
| Deployment arm A / B | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` |

B01 is STM32H723 (family) with exact variant/package `NOT ASSIGNED`. No
H743-only peripheral (PIN §1B: FMC/SDRAM, JPEG, H743 Quad-SPI, 3×16-bit ADC)
is assumed available here. Wiring-derived rows are a summary of PIN §1C (the
single authoritative tag table) and are gated on BU12; they carry no pin, bus
instance, CS or address. PIN §1D lists the exact questions that close them.

## 4. Sensor continuity and health

`self_test::SENSOR_CHANNELS` is `Accel`, `Baro`, `Battery`, `Temperature`; the
BOM devices behind `Accel` are not uniquely mapped (PIN_MATRIX.md §1 row B04).

| ID | Check | Type | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|
| S01 | Unpowered harness continuity, each sensor connector | electrical | every channel recorded; an open path is a `FAIL`, not a missing bit | | |
| S02 | Preflight suite 6/6 with **real** inputs at boot | self-test | `sensor_range`, `table_crc`, `config_crc`, `actuator_continuity`, `phase_latch`, `state_machine`; `CHECK_COUNT = 6` | | |
| S03 | Note the mock boundary | precondition | `ContinuityMock` covers pitch/yaw actuator paths only; it does not prove a wire is connected | | `N/A` — code fact, no bench action |
| S04 | `sensor_flags()` reports real presence/health | code→board | stub returning `0` replaced before this passes | | `BLOCKED` — `main.rs:99` |
| S05 | Redundant IMU/baro vote (`sensor_voting.rs`) | logic-on-board | 3-slot median, 2-slot midpoint; stuck sensor excluded after `stuck_samples` identical readings **plus** another sensor moving; stationary bench quarantines nothing | | `BLOCKED` — no drivers |
| S06 | Withdraw one sensor at a time | electrical | vote degrades with a reported reason, never a silent substitute | | `BLOCKED` |
| S07 | Move a sensor while stationary | electrical | no false stuck/quarantine verdict | | `BLOCKED` |
| S08 | Calibration-table CRCs and golden config CRC on flashed image | self-test | `self_test::GOLDEN_CONFIG_CRC = 0xABF7` verified on target, not only host | | |
| S09 | Diagram sensor-set continuity: 9 SPI links (incl. KX134-1211 ×2), thermometer `1W`, photoresistor `АЦП`; no I²C/QSPI tag drawn | electrical | each drawn link recorded at its connector; CS/instance/arbitration still `NOT ASSIGNED` until schematic; a common drawn line is not proof of one selected bus (PIN §1C/§1D Q2–Q4, C10/C11) | | `BLOCKED` — no drivers/bus map |

## 5. UART loopback and wire timing

Prerequisites: PIN §5 signed, `hw::Rx`/`hw::Tx` implemented, target image
flashed. Until then every row is `BLOCKED`.

| ID | Check | Type | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|
| U01 | Local loopback TX → RX at assigned connector | wire | request frames intact through real DMA path; byte count recorded | | `BLOCKED` |
| U02 | Logic analyser at 2 Mbit/s, 8N1 | wire | frame lengths `56 + 14` (compat) and `42 + 16` (lowlat); wire time against the 0.25 ms budget recorded — SIL's cycle model is arithmetic, not this measurement | | `BLOCKED` |
| U03 | DMA double-buffer slot swaps on real idle-line wiring | wire | noise before magic is not a miss; split magic reassembled (`ReplyAssembler`) | | `BLOCKED` |
| U04 | Physical fault injection: magic, CRC, version, seq | wire | each updates rejection counters and drives the three-miss latch (`MAX_CONSECUTIVE_MISSES = 3`) and `Command::ZERO` | | `BLOCKED` |
| U05 | Cross-language interop, both profiles | wire | Raspberry Pi client vs firmware; record direction and profile — STM32 sends requests, Pi sends replies | | `BLOCKED` |
| U06 | Back-to-back frames ≥ 1 hour at full rate | wire | including idle-line gaps and Pi jitter: **zero** byte loss | | `BLOCKED` |
| U07 | Unplug UART / kill the Pi | safety | zero command, stale detection, watchdog behaviour per §6 | | `BLOCKED` |
| U08 | Counter readback | telemetry | measured counter values cross-checked with `defmt`/telemetry; a counter that cannot be read back is not evidence | | `BLOCKED` |
| U09 | LoRa 1262F30-T (915 MHz SMA) and GPS module UART links from the diagram | wire | each USART instance/pins/AF and DMA (if any) signed before loopback; record that the working BOM called LoRa SPI and was reconciled (BU12/PIN item 7) | | `BLOCKED` — no drivers |

## 6. Watchdog

| ID | Check | Type | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|
| W01 | IWDG1 unleashed at boot | peripheral | configured from `WATCHDOG_TIMEOUT_US = 100_000`; once unleashed only a reset stops it | | |
| W02 | Pet source | logic | pets only from the control task after a complete cycle with a valid STM32-owned phase; no pet from UART callback (`watchdog::may_feed`) | | |
| W03 | Software deadman | logic | `CONTROL_PERIOD_US = 2_000`, `DEADLINE_SLACK_US = 250`, trip after `DEADLINE_LIMIT = 3` missed deadlines; verified on target clock, not the model | | |
| W04 | Stall the control loop | hardware reset | IWDG resets within its configured window; measured interval and reset reason recorded (a reset reason not persisted is an open item) | | |
| W05 | Unplug UART / kill Pi | safety | AI latch off, `Command::ZERO`, stalled loop not fed → logged IWDG reset; a latched AI alone must **not** stop feeding | | `BLOCKED` on U01–U03 |
| W06 | Recovery | safety | after reset: boot state BURN with AI gated; explicit firmware reset clears AI latch; no implicit recovery from a stale reply | | |

## 7. BURN gate — with servos connected

Vehicle safe: servos connected for measurement, propulsion and pyro disconnected.

| ID | Check | Type | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|
| G01 | Phase BURN + valid AI replies, max correction | safety | `MAX_CORRECTION_RAW = 100` → **no actuation** on pitch/yaw lines (`phase.rs::ai_allowed`, `interlocks::Refusal::Boost`) | | `BLOCKED` — no PWM driver |
| G02 | Assert `MOTOR_ARMED` outside BURN | safety | output returns to `Command::ZERO` (`Refusal::MotorArmed`); transient — removing BURN or the bit must not require a reset | | `BLOCKED` |
| G03 | After burnout, valid correction | safety | reaches servo lines; slew/rate limits from `interlocks::Limits::DEFAULT` apply | | `BLOCKED` |
| G04 | Re-assert `MOTOR_ARMED`; latch behaviour | safety | zero output; latching refusals (`UnknownPhase`, `AiUnhealthy`, `Fault`, `Implausible`) stay zero until `Interlocks::reset()`; a valid reply after AI latch does not clear it | | `BLOCKED` |
| G05 | Scope traces for each case | evidence | attached to §1 session record | | `BLOCKED` |

## 8. Parachute continuity and deployment

Bench rule: pyro/squib disconnected; use a tester appropriate to the circuit.
Never measure continuity across a live pyro channel. BOM has 2 parachutes;
firmware decides two spring outputs and two independent arm channels.

| ID | Check | Type | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|
| D01 | Unpowered continuity, spring A and B | electrical | both paths through board and harness (`deployment::SpringOutputs::a/b`) recorded | | |
| D02 | Unpowered independence, arm A and B | electrical | no shared short between `ArmChannels::a/b` | | |
| D03 | Powered, BURN or LANDED | logic | any trigger refused (`Refusal::WrongPhase`), no output asserts | | |
| D04 | Powered, one arm only | logic | `Refusal::ArmIncomplete { a, b }`, no sensor state advances | | |
| D05 | Powered, both arms, scripted `DeploymentSample` | logic | apogee descent after verified climb; emergency descent rate; low-altitude backup — all three commit | | |
| D06 | Threshold authority | precondition | `DeploymentConfig::DEFAULT` (30 m / 2 m drop, 5 m climb, 15 m backup, −1 500 cm/s, 3 samples, 3 m floor) are logic constants, **not** flight thresholds; owner-signed values recorded here | | |
| D07 | `descent_confirm` | logic | 3 consecutive descending samples required; a single glitch must not release | | |
| D08 | One-shot latch | logic | once released both outputs stay asserted; later samples cannot change or retract; re-arm is ground-only `reset()` | | |
| D09 | Two independent release channels | electrical | both fire together on the bench; measured continuity/response recorded | | `BLOCKED` — outputs not wired |

## 9. Buzzer and search beacon

| ID | Check | Type | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|
| Z01 | Phase → mode (`buzzer::mode_for_phase`) | logic | BURN silent; COAST/APOGEE idle melody; DESCENT/LANDED priority locator (three C6 calls then long gap); frequency and duty measured on PWM output — module owns no pin or timer | | `BLOCKED` |
| Z02 | BURN silence on the pin | electrical | verified on the pin, not only in the scheduler | | `BLOCKED` |
| Z03 | Search beacon (`search_beacon.rs`) | logic | `post_landing`: 880 / 1 047 Hz pattern preempts melody; cadence fixed as pack voltage drains (`SEARCH_CALL_HZ`, `SEARCH_PEAK_HZ`) | | `BLOCKED` |
| Z04 | Battery bands | logic | 3 300 / 3 600 / 3 900 mV (`VBAT_CRITICAL_MV`, `VBAT_LOW_MV`, `VBAT_NOMINAL_MV`); below critical and during boost all signals silent; duty caps follow the band — inject each band, record measured behaviour | | `BLOCKED` |
| Z05 | Radio hint | adapter | `RadioHint` 144 MHz, duty 0/5/10/20 % by band, TX only inside a slot; board adapter applies the hint — no radio driver in repo, so mark `BLOCKED` if adapter absent | | `BLOCKED` |
| Z06 | No stale tone | logic | no sound from either scheduler during boost; no stale tone when pattern changes | | `BLOCKED` |

## 10. Flight-ready gates

All must be checked and signed before anyone calls the firmware flight-ready.
Today none is checked (README §7: no board bring-up, pins unknown, no flight
qualification).

| ID | Gate | Type | Result | Evidence / blocker |
|---|---|---|---|---|
| F01 | Host gates green: `cargo test --workspace`, `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo check --target thumbv7em-none-eabihf` | host | | |
| F02 | Pin map (PIN §5) complete and signed; no `todo!()` on the flashed target path | precondition | | `BLOCKED` |
| F03 | §§2–9 passed with dated bench records and attached captures | evidence | | |
| F04 | BURN gate proven with servos connected (§7) | safety | | `BLOCKED` |
| F05 | Watchdog hardware reset and logged reset reason proven (§6) | safety | | |
| F06 | Deployment arm/release continuity and one-shot latch proven (§8) | safety | | |
| F07 | One-hour back-to-back UART run and analyser captures archived (§5) | evidence | | `BLOCKED` |
| F08 | Cold flash, brownout and RAM/flash headroom on target recorded | evidence | | |
| F09 | Cross-language interop on both profiles with golden frames recorded | evidence | | `BLOCKED` |
| F10 | Preflight self-test 6/6 with real board inputs at boot | self-test | | |
| F11 | Deployment thresholds and separation `Plan` values signed by the owner | precondition | | |
| F12 | Open items listed with owners; none marked "will fix in flight" | evidence | | |
| F13 | BOM reconciliation closed (PIN §1 items 1–5) | precondition | | |
| F14 | H723 variant/package and toolchain identifiers signed; no H743-only peripheral or identifier carried forward (PIN §1A/§1B, §2A order 0) | precondition | | `BLOCKED` |
| F15 | Schematic-to-BOM reconciliation closed from the owner wiring diagram **before** any pin assignment (BU12, PIN §1 items 6–9, C13) | precondition | | `BLOCKED` |

## 11. What SIL cannot prove

SIL runs the real decision sources against `MockBoard`; it proves determinism of
logic, not the board. Each row is a scope boundary, not a claim about hardware.

| ID | Not provable by SIL | Covered by | Status |
|---|---|---|---|
| X01 | Electrical behaviour: BEC rails under load, ripple, brownout, sequencing, connector/pin contact, harness continuity | §2, PB §6 | `NOT RUN` |
| X02 | Pin numbers, AFs and DMA stream selection — `build_board()` is `todo!()`, so SIL never constructs the real adapter | PIN §3/§5 | `BLOCKED` |
| X03 | Real-time wire behaviour: 2 Mbit/s signalling, measured frame length and wire time vs the 0.25 ms budget, DMA swapping on silicon, one-hour integrity, Pi jitter | §5 | `BLOCKED` |
| X04 | Physical fault injection driving the same rejection counters as deterministic `fault_inject` | §5 U04 | `BLOCKED` |
| X05 | IWDG hardware timeout, the actual reset, or reset-reason persistence | §6 W04–W06 | `NOT RUN` |
| X06 | Dead Pi / unplugged UART yielding zero actuation with servos connected | §5 U07, §6 W05, §7 | `BLOCKED` |
| X07 | Pyro/deployment continuity, independent arm channels, one-shot release reaching both outputs | §8 | `NOT RUN` |
| X08 | Buzzer acoustics, PWM pin assignment, radio duty on the air, battery-band behaviour with a real pack | §9 | `BLOCKED` |
| X09 | H723 variant and `memory.x` correctness (FLASH ≤ 1 MB), clock source, cold flash, EMC, interrupt latency, RAM/flash headroom on target | §2 BU01/BU09, §2A orders 1–3 | `NOT RUN` |
| X10 | Wiring-derived interfaces: SPI-tagged flash (OCTOSPI open), LoRa/GPS/Pi UART, SPI/1-Wire/ADC sensor links (no I²C tag), USB, recovery `PWM` vs untagged control servos, video subsystem | §2 BU12, §3, §4 S09, §5 U09; PIN §1C/§1D | `BLOCKED` |

A green `sil` job is necessary before a board session and is **not sufficient**
to call the firmware flight-qualified. HIL on the real board is the final
verification, and this document records none of it yet.

## 12. Peripheral resource-collision check (FW723-IO)

**Procedure, not a run record.** Every Result cell is blank; nothing is a pass
by silence. This section checks that the *chosen* timers, PWM channels, DMA
streams, bus chip-selects and UART instances do not collide. The demand and
family availability are in [RESOURCE_MAP.md](RESOURCE_MAP.md) §§2–6; every
physical value there is `NOT ASSIGNED` until `PIN_MATRIX.md` §5 is signed. Drawn
tags come from the single authoritative table in `PIN_MATRIX.md` §1C, and the
exact schematic/BOM questions are in `PIN_MATRIX.md` §1D. The result vocabulary is §0 above;
`BLOCKED` is the correct entry while the schematic is unresolved.

Prerequisite for the whole table: RESOURCE_MAP D01–D13 closed against the signed
netlist and procured H723 variant/package. Without that, R02 and R03 cannot be
answered by reading code or by asserting in a host test.

| ID | Check | Type | Ref | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|---|
| R01 | Exact H723 variant/package, bonded timer/DMA/SPI/USART inventory and GPIO count signed | precondition | RESOURCE_MAP D01, D13; PIN §1A/§1B, BU01, BU10 | variant and package in the §1 session record; no H743-only peripheral or identifier used to fill a cell | | `BLOCKED` — variant/package `NOT ASSIGNED` |
| R02 | **Timer/PWM collision:** every role owns a distinct `(timer instance, channel)` pair | static + on-target | RESOURCE_MAP §4, D10; PIN §3 P05, P07–P09, P28 | from the signed board-adapter resource table: time driver, 6 servos, 1W thermometer, buzzer/search and any camera trigger have no duplicated pair; the time-driver timer's base settings are compatible with any timer it shares; scope shows **6 independent** servo waveforms at the configured rate | | `BLOCKED` — timer/channel/AF `NOT ASSIGNED`; servos 3–6 untagged |
| R03 | **DMA collision:** every role owns a distinct `(DMA controller, stream/channel, DMAMUX request)` triple | static + on-target | RESOURCE_MAP §4, D12; PIN §4 C01/C06, §5 U01–U03 | no two roles share a stream; Pi RX slot A ≠ slot B; no sensor/flash bus stream aliases UART TX/RX; DMA1/DMA2/BDMA/MDMA domains named per role; logic analyser shows both RX slots swapping and no byte loss under full sensor + flash load | | `BLOCKED` — no signed DMA map |
| R04 | SPI chip-select and I²C address uniqueness | static + wire | RESOURCE_MAP D02, D08, D09; PIN §4 C02/C11, §1D Q3/Q4/Q18; §5 S01 | ≥10 independent CS nets (9 sensors + flash), one CS per device; the two KX134 and two MMC5983MA parts have distinct CS; any I²C remap has unique addresses/straps; no two devices assert MISO together; continuity measured per CS | | `BLOCKED` — bus topology `NOT ASSIGNED` |
| R05 | UART/USB instance separation | static + wire | RESOURCE_MAP D05–D07; PIN §3 P01–P04 | LoRa, GPS and Pi are three distinct instances with distinct pins/AF; Pi RX/TX DMA per R03; baud and direction recorded per module; if Pi is on USB, OTG role and `USB ПОРТ` sharing decided | | `BLOCKED` — instances and link choice `NOT ASSIGNED` |
| R06 | W25Q512 bus choice and contention | static + on-target | RESOURCE_MAP D03, D14; PIN §1B, P23, §1D Q5 | drawn tag is `SPI`; SPI vs OCTOSPI implementation signed; flash streams distinct from sensors and UART (R03); worst-case erase/write burst measured against the 500 Hz loop and the Pi link with zero missed control cycles | | `BLOCKED` — bus and stream `NOT ASSIGNED` |
| R07 | Video subsystem resource | precondition | RESOURCE_MAP D04 | camera + video flash endpoint identified: if off-MCU, record reason and mark `N/A`; if on MCU, its SPI instance, CS, streams and measured bandwidth are recorded and it does not share the sensor bus without a measured budget | | `BLOCKED` — no MCU path drawn in the diagram |
| R08 | 1W thermometer and photoresistor resource | static + wire | RESOURCE_MAP D11; PIN §3 P26/P27 | chosen 1W mechanism (timer capture / UART / GPIO) does not duplicate an R02 channel; photoresistor ADC channels distinct from battery/temperature; front-end measured | | `BLOCKED` — mechanism `NOT ASSIGNED` |
| R09 | GPIO budget after CS/strap/control assignment | static | RESOURCE_MAP §4; PIN §1A | total assigned GPIO ≤ bonded count for the procured package; every output has one owner and every input one source | | `BLOCKED` — package `NOT ASSIGNED` |
| R10 | Runtime assertion of the authored resource table | code→target | R02–R04, R06 | the board adapter exposes one resource table and rejects duplicate `(timer, channel)` and `(DMA controller, stream)` pairs at startup (compile-time or first-boot check); a deliberate duplicate injected in a test image is rejected; the flashed image hash in §1 matches the checked table | | `BLOCKED` — no board adapter / resource table exists |

R02 and R03 are the hard gates: a signed schematic that assigns two roles to
one channel or one DMA stream is a `FAIL`, not a note, and the session stops
before any power-on stage of §2A. Passing R01–R10 is a precondition for §2A
orders 4–6 (UART, sensors, servo outputs), not a substitute for them.
