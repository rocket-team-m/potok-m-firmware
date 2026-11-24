<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# HIL — typed bring-up matrix

Hardware-in-the-loop verification for the STM32H743 controller. **This is a
procedure to run on a real board, not a claim that it has been run.** Every
result cell is blank; nothing is a pass by silence. No pin, rail, part or
measurement in this document is asserted as present on a physical board.

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
| Schematic revision | |
| BOM revision | |
| Firmware commit and profile (compat / lowlat) | |
| Flashed image hash | |
| Equipment (supply, DMM, logic analyser, scope, current probe) | |
| Host gates green before session (`cargo test --workspace`, `cargo check --target thumbv7em-none-eabihf`) | |

## 2. Bring-up gate matrix

Rows are typed; `Ref` names the authoritative procedure. Rail values are
`NOT ASSIGNED` here — see [POWER_BUDGET.md](POWER_BUDGET.md) for the placeholder
budget and `PB §6` for the measurement steps.

| ID | Gate | Type | Ref | Criterion | Result | Evidence / blocker |
|---|---|---|---|---|---|---|
| BU01 | Confirm chip variant, `memory.x` FLASH/RAM and time-driver timer against BOM **before** first power-on | precondition | README §5 | variant + layout + timer signed; the `STM32H743ZI` / TIM2 starting points are not approvals | | |
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

## 3. Pin map (summary — full matrix in PIN_MATRIX.md)

The table is intentionally blank. Only `PIN_MATRIX.md` §3 may carry a physical
value, and only after `PIN_MATRIX.md` §5 is signed.

| Code-side role | Type | BOM | Port/pin | AF | DMA | Schematic ref | Status |
|---|---|---|---|---|---|---|---|
| USART TX (`hw::Tx::send`) | `UART_TX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| USART RX slot A (`hw::Rx::take`) | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| USART RX slot B | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| Time driver (TIM2 starting point) | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` |
| IWDG1 (`setup_iwdg`) | `PERIPHERAL` | B01 | — | — | — | `NOT ASSIGNED` | `PERIPHERAL`, no pin |
| Servo PWM pitch / yaw | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| Servo 3..6 (BOM only) | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no code role |
| `MOTOR_ARMED` sense | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `STUB` |
| Phase-event sources | `GPIO_IN` | B02–B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` |
| `sensor_flags` IMU/baro/mag/pitot | `GPIO_IN` | B02–B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB`; pitot part unknown |
| Sensor buses (7 parts) + flash + radio | `SPI_*`/`I2C_*` | B02–B09 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no drivers |
| Buzzer PWM | `SCHEDULER` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` |
| Search radio control | `SCHEDULER` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` — no driver |
| Deployment spring A / B | `GPIO_OUT` | B11 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` |
| Deployment arm A / B | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` |

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
| F13 | BOM reconciliation closed (PIN §1 items 1–4) | precondition | | |

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
| X09 | Chip variant and `memory.x` correctness, cold flash, EMC, interrupt latency, RAM/flash headroom on target | §2 BU01/BU09 | `NOT RUN` |

A green `sil` job is necessary before a board session and is **not sufficient**
to call the firmware flight-qualified. HIL on the real board is the final
verification, and this document records none of it yet.
