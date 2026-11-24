<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# POWER_BUDGET — typed rail and load budget with measurement procedure

**No hardware claim.** No rail in this document has been measured on the
flight board. Rail names, BEC part, voltage tolerances, test points and the
battery voltage are `NOT ASSIGNED` until the schematic and BOM are signed. Every
current below is a *planning value*: A6 is transcribed from the STM32H723
datasheet (see the correction note below), the rest are placeholders taken from
the named part families. None is a datasheet verification of the procured part
and none is a measurement of this board. Replace each assumption with the
procured-part datasheet value before using this budget to size a supply. The arithmetic is exact given the
assumptions; the assumptions are not facts.

**MCU correction (FW723-CLOCK).** The part is an **STM32H723**, not an
STM32H743. Clock, voltage-scale, flash-latency and run-current assumptions in
this document were re-derived from the STM32H723 datasheet (ST DS13313) and
reference manual (RM0468) as transcribed on 2026-11-24. They are *datasheet*
facts at most; none was measured on the flight board. Values taken from a
transcription rather than the live ST PDF are marked `NOT VERIFIED`, and no
supply is sized from them here. No hardware measurement is claimed anywhere in
this document.

Companion documents: [PIN_MATRIX.md](PIN_MATRIX.md) (physical assignment) and
[HIL_CHECKLIST.md](HIL_CHECKLIST.md) (bring-up result cells).

## 1. Assumption register

| ID | Assumption | Value | Source | Status |
|---|---|---|---|---|
| A1 | MCU / sensor / flash rail nominal | 3.3 V | planning placeholder | `NOT ASSIGNED` from schematic |
| A2 | Servo rail nominal | 5.0 V | planning placeholder | `NOT ASSIGNED` from schematic |
| A3 | Radio module rail nominal | 3.3 V | planning placeholder | `NOT ASSIGNED` from module variant |
| A4 | Deployment / pyro rail | unknown | — | `NOT ASSIGNED` — actuator technology undecided |
| A5 | Battery / input voltage | unknown | — | `NOT ASSIGNED` |
| A6 | STM32H723 run-mode current ceiling | 360 mA @ 3.3 V: 550 MHz, VOS0 + `CPU_FREQ_BOOST`, cache ON, all peripherals enabled, characterization max at Tj = 105 °C; 220 mA typ at Tj = 25 °C | ST DS13313 table 21, transcribed from a Rev 2 rendering (current revision Rev 5, May 2025 — re-check the PDF before sizing) | `NOT VERIFIED` — transcription, not the flight board |
| A7 | Sensor currents @ 3.3 V | ICM-42688-P 1.5 mA; LSM6DSR 1.0 mA; KX134 1.0 mA; LPS22HH 0.1 mA; LIS2MDL 0.2 mA; MMC5983MA 0.2 mA ×2 | planning placeholder | `NOT VERIFIED` |
| A8 | W25Q512 current | 30 mA active, 40 mA erase peak, 0.01 mA deep power-down | planning placeholder | `NOT VERIFIED` |
| A9 | LoRa1262F30 current | 150 mA TX peak, 12 mA RX, 2 mA sleep | planning placeholder | `NOT VERIFIED` |
| A10 | Servo current @ 5 V | 150 mA holding, 1 000 mA peak **each** | planning placeholder; stall current unknown | `NOT VERIFIED` |
| A11 | Deployment channel current | unknown | — | `NOT ASSIGNED` |
| A12 | Regulator efficiency | η₃V₃ = 0.90, η₅V = 0.95 | planning placeholder | `NOT VERIFIED` |
| A13 | Rail tolerance and ripple limit | unknown | — | `NOT ASSIGNED` from schematic |
| A14 | Core / AXI clock and voltage scale | 550 MHz CPU at VOS0 with `CPU_FREQ_BOOST`; 275 MHz AXI; VCORE 1.35 V | DS13313 table 12 / §3.7.1, RM0468, transcribed | `NOT VERIFIED` on the flight board |
| A15 | VDD minimum for VOS0 | 1.7 V with the internal LDO (1.62 V in bypass) — assumed A1 = 3.3 V clears it | DS13313 table 13, transcribed | datasheet condition; the rail itself is `NOT ASSIGNED` |
| A16 | Flash wait states | 3 WS at 275 MHz AXI in VOS0; 32 KB I-cache + 32 KB D-cache give ~0-wait-state execution when enabled | RM0468 table 16 and ST product page, transcribed | `NOT VERIFIED` on the flight board |
| A17 | `CPU_FREQ_BOOST` option-byte state on the flight board | unknown (boost disables ITCM/DTCM ECC) | — | `NOT ASSIGNED` |
| A18 | Junction temperature ceiling at VOS0 | 105 °C (125 °C at VOS1) — this is why A6 quotes the 105 °C column | DS13313 table 13, transcribed | datasheet condition; board thermal test `NOT RUN` |

### 1a. H723 clock / voltage / flash-latency basis

| Fact | Value | Source | Status |
|---|---|---|---|
| Core | Cortex-M7, up to 550 MHz | DS13313 features; AN4891 table 7 | datasheet fact |
| 550 MHz precondition | VOS0 **and** `CPU_FREQ_BOOST` option byte; 520 MHz is the VOS0 limit without boost | DS13313 table 12, transcribed | `NOT VERIFIED` on the flight board |
| AXI clock | 275 MHz at 550 MHz CPU (CPU/2); flash is clocked by AXI, not by the core | DS13313 block diagram; AN4891 table 5 | datasheet fact for the assumed divider; board `NOT VERIFIED` |
| Core supply | VCORE 1.35 V at VOS0 (selectable 1.0–1.35 V) | DS13313 §3.7.1, transcribed | `NOT VERIFIED` on the flight board |
| External supply | VDD 1.62–3.6 V; VOS0 with LDO needs ≥ 1.7 V | DS13313 table 13, transcribed | datasheet condition; A1 `NOT ASSIGNED` |
| Flash latency | 3 WS at 275 MHz AXI / VOS0; RM0468 warns the wait-state table is indexed by **AXI** frequency, not CPU frequency | RM0468 table 16, transcribed | `NOT VERIFIED` on the flight board; re-derive if the AXI divider changes |
| Temperature | VOS0 junction maximum 105 °C, VOS1 125 °C | DS13313 table 13, transcribed | datasheet condition |
| Boost side effect | `CPU_FREQ_BOOST` drops ITCM/DTCM ECC | RM0468, transcribed | option byte `NOT ASSIGNED` (A17) |

No row above is a board reading. The code derives its cycle allowance from A14:
`cycle_budget_for(550 MHz, 50 us) = 27 500 cycles` and one 500 Hz tick is
`1 100 000 cycles` (`firmware/src/perf.rs`, `firmware/src/timing_budget.rs`).
Those are arithmetic consequences of the configured clock. The modelled SIL
work (856 cycles worst case) is an instruction-count upper bound; flash
latency is absorbed by the per-tick frame overhead, which reserves more than
the 3 wait states, not added as a second charge.

A10 is **not a guaranteed ceiling**: a stalled hobby servo can exceed 1 A. The
1 A figure is a lower planning floor for the worst-case rail, not a safe upper
bound. Do not size the servo BEC from A10 alone; measure the stall current in
step 6 below.

## 2. Rail topology (typed)

All `Schematic ref`, `BEC output` and `Test point` cells are `NOT ASSIGNED`.

| Rail ID | Function | Nominal (assumed) | BEC output | Test point | Loads | Tolerance | Status |
|---|---|---:|---|---|---|---|---|
| R1 | MCU core / digital | 3.3 V (A1) | `NOT ASSIGNED` | `NOT ASSIGNED` | B01, B09 | `NOT ASSIGNED` | `NOT ASSIGNED` |
| R2 | Sensors | 3.3 V (A1) | `NOT ASSIGNED` | `NOT ASSIGNED` | B02–B07 | `NOT ASSIGNED` | `NOT ASSIGNED` |
| R3 | Radio | 3.3 V (A3) | `NOT ASSIGNED` | `NOT ASSIGNED` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` |
| R4 | Actuator / servo | 5.0 V (A2) | `NOT ASSIGNED` | `NOT ASSIGNED` | B10 ×6 | `NOT ASSIGNED` | `NOT ASSIGNED` |
| R5 | Deployment / pyro | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | B11 ×2 | `NOT ASSIGNED` | `NOT ASSIGNED` |

R1–R3 are shown as separate rails because the BEC topology is unknown. They may
turn out to be one regulated rail; that is a schematic decision, not one made
here.

## 3. Load register and worked arithmetic

`P = I × V` with I in amperes. Sensor subtotal from A7:
`1.5 + 1.0 + 1.0 + 0.1 + 0.2 + 0.2 + 0.2 = 4.2 mA`.

| Load ID | Device(s) | Qty | Rail | I typ / active | I peak | Basis | Notes |
|---|---|---:|---|---:|---:|---|---|
| L1 | STM32H723 | 1 | R1 | 360 mA | 360 mA | A6 | characterization ceiling at 105 °C used for all scenarios; typ 220 mA at 25 °C |
| L2 | sensors (B02–B07) | 7 | R2 | 4.2 mA | 4.2 mA | A7 | all sampling concurrently |
| L3 | W25Q512 | 1 | R1 | 30 mA | 40 mA | A8 | erase peak only during writes |
| L4 | LoRa1262F30 | 1 | R3 | 12 mA RX / 2 mA sleep | 150 mA | A9 | TX duty cycle `NOT ASSIGNED` |
| L5 | servos | 6 | R4 | 150 mA each | 1 000 mA each | A10 | 900 mA hold / 6 000 mA peak subtotal |
| L6 | deployment channels | 2 | R5 | `NOT ASSIGNED` | `NOT ASSIGNED` | A11 | excluded from all scenario totals |

## 4. Scenario budget

`I_rail` is the sum of the listed loads on that rail; `P_rail = I_rail × V_rail`.
A scenario is executable on the bench only when the pin map and drivers exist;
the last column is a code-state statement, not a hardware result.

| Scenario | Active loads | I_R1 (mA) | I_R2 (mA) | I_R3 (mA) | I_R4 (mA) | P₃V₃ (W) | P₅V (W) | Total (W) | Executable now? |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| S0 idle | L1, L2, L3 sleep, L4 sleep, L5 off | 360 + 0.01 = 360.01 | 4.2 | 2 | 0 | 1.208 | 0.000 | **1.208** | `NO` — no sensor/radio driver |
| S1 telemetry | L1, L2, L3 read 30 mA, L4 RX | 390 | 4.2 | 12 | 0 | 1.340 | 0.000 | **1.340** | `NO` |
| S2 radio TX + flash erase | L1, L2, L3 erase 40 mA, L4 TX | 400 | 4.2 | 150 | 0 | 1.829 | 0.000 | **1.829** | `NO` |
| S3 all servos holding | S1 + L5 hold | 390 | 4.2 | 12 | 900 | 1.340 | 4.500 | **5.840** | `NO` — no PWM driver |
| S4 worst concurrent | S2 + L5 peak | 400 | 4.2 | 150 | 6 000 | 1.829 | 30.000 | **31.829** | `NO` — no PWM driver |

`P₃V₃` aggregates R1 + R2 + R3 (all three are assumed 3.3 V). Worked cells:

- S0 P₃V₃ = `(360.01 + 4.2 + 2) mA × 3.3 V = 0.36621 A × 3.3 V = 1.208493 W → 1.208 W`
- S1 P₃V₃ = `(390 + 4.2 + 12) mA × 3.3 V = 0.4062 A × 3.3 V = 1.340460 W → 1.340 W`
- S2 P₃V₃ = `(400 + 4.2 + 150) mA × 3.3 V = 0.5542 A × 3.3 V = 1.828860 W → 1.829 W`
- S3 P₅V = `6 × 0.150 A × 5.0 V = 4.500 W`; total `1.340460 + 4.500 = 5.840460 W → 5.840 W`
- S4 P₅V = `6 × 1.000 A × 5.0 V = 30.000 W`; total `1.828860 + 30.000 = 31.828860 W → 31.829 W`

The H743-era 250 mA placeholder is gone: every scenario current above comes
from A6's H723 datasheet-derived ceiling. It is still a characterization value
transcribed from a datasheet rendering, **not** a flight-board measurement and
not a guaranteed production maximum.

S4 is a simultaneous-peak construct, not a measured duty cycle. Radio TX duty
cycle and servo motion duty cycle are both `NOT ASSIGNED`, so S4 is a ceiling
exercise only.

## 5. Input-side estimate (illustration only)

Formula: `P_in = P₃V₃ / η₃V₃ + P₅V / η₅V`, then `I_in = P_in / V_BAT` (A12, A5).
Because `V_BAT` is `NOT ASSIGNED`, no input current is asserted. For illustration
**only**, substituting `V_BAT = 7.4 V` (an unapproved placeholder):

| Scenario | P_in (W) | I_in @ 7.4 V (A) — illustration only |
|---|---:|---:|
| S0 | 1.208 / 0.90 = 1.343 | 0.181 |
| S1 | 1.340 / 0.90 = 1.489 | 0.201 |
| S2 | 1.829 / 0.90 = 2.032 | 0.275 |
| S3 | 1.489 + 4.500 / 0.95 = 6.226 | 0.841 |
| S4 | 2.032 + 30.000 / 0.95 = 33.611 | **4.542** |

Sizing rule (to be signed, not applied here):
`I_BEC_rating ≥ I_in_peak × margin`. Both `I_BEC_rating` and `margin` are
`NOT ASSIGNED`. Do not fill them from the table above; fill them from the BEC
datasheet and the owner's margin decision after the measurements in §6.

## 6. Exact measurement steps

Bench rule: pyro/squib and propulsion disconnected; servos on a fixture that
cannot strike anything; current-limited bench supply; record before connecting.
A step is `NOT RUN` until its result cell in §7 is filled. Steps that require
unassigned pins or missing drivers are `BLOCKED`, not skipped.

1. **Unpowered short check.** DMM in continuity mode, probes on each rail test
   point (R1–R4; R5 only with the deployment device disconnected). Record
   resistance to GND and to every other rail. Also check no UART/USB path can
   back-feed R1. Expected: no near-zero path. Stop on a short.
2. **Supply set-up.** Set bench supply voltage to the schematic input rating
   (`NOT ASSIGNED`) and current limit to the signed BEC rating (`NOT ASSIGNED`).
   Record both settings *before* connecting. If either is unknown, stop.
3. **Rail voltage, no load.** Power on with the current limit at its minimum
   safe setting. DMM at each test point, record R1–R4 (and R5 if powered).
   Compute deviation from the schematic nominal; compare to A13 tolerance,
   which is `NOT ASSIGNED` until signed. Stop if any rail exceeds its absolute
   maximum.
4. **Rail current by shunt.** Insert a known shunt `R_s` in series with the
   rail; measure the drop `V_s` with a DMM; compute `I = V_s / R_s`. Choose `R_s`
   so `V_s ≤ 0.01 × V_rail` to keep the measurement from changing the operating
   point. Record `R_s`, `V_s`, computed `I`, and the rail. A current probe may
   replace the shunt if its calibration is recorded.
5. **Scenario loading.** Command each scenario S0–S4 in turn (S3/S4 are
   `BLOCKED` until the servo PWM driver and pin map exist). For each, record
   `I` from step 4 on R1–R4 and recompute `P = I × V_rail` using the *measured*
   rail voltage from step 3, not A1/A2. This table is the replacement for §3/§4.
6. **Servo stall current.** With the servo output on a rigid fixture, command a
   hard-over step and hold it; record the peak and 1-second current with the
   current probe, per servo and with all six together. Replace A10 with the
   measured value. This is the figure the servo BEC must be sized from, not the
   1 A placeholder.
7. **Ripple and sag.** Scope at the load-side test point, AC coupling on,
   20 MHz bandwidth limit on, ground spring (no long ground lead). Capture at
   least 1 s in each scenario S0–S4. Record `V_pp` and minimum `V` for each
   rail. Compare against A13 (currently `NOT ASSIGNED`). Record worst case.
8. **Inrush and sequencing.** Current probe on the input feed, trigger on the
   rising edge, capture 100 ms from first application of power. Record peak
   current, duration, and the order in which R1–R4 come up. Check against the
   schematic sequence (`NOT ASSIGNED`); no rail may exceed its absolute maximum
   during startup.
9. **Brownout.** Lower the input voltage slowly until the BEC drops out or the
   board resets; record the input voltage at reset. Cycle below dropout and
   back. Confirm a clean reset with no latched half-state in deployment or
   separation logic — both latch only until an explicit ground reset.
10. **Thermal.** Hold S4 (or the highest executable scenario) for 10 minutes;
    record regulator case temperature and any current derating. If S4 is
    `BLOCKED`, record the highest scenario actually reached and leave S4 open.
11. **Cold flash.** From a fully unpowered board, erase, program and boot with
    `probe-rs`; record any failure to boot. Verify the flashed image hash
    against the session record in `HIL_CHECKLIST.md` §0.
12. **Clock preconditions (H723).** With the debugger attached but no flight
    actuation, read back the actual `RCC_CFGR` core/AXI dividers, `PWR_D3CR`
    voltage scale, `FLASH_ACR` wait-state field and the `CPU_FREQ_BOOST` option
    byte. Record each value; A14/A16/A17 stay `NOT VERIFIED` until this row is
    filled. Do not assume 550 MHz from the firmware's `CORE_HZ` constant: that
    constant is a configured budget input, not evidence of the silicon's clock.

Never mark a rail good by absence of a failure. A blank result cell is
`NOT RUN`, not a pass.

## 7. Measurement record (blank by design)

| Step | Rail | Scenario | Instrument + setting | Measured | Computed | Limit (from schematic) | Status | Operator / date |
|---|---|---|---|---|---|---|---|---|
| 1 | | | DMM continuity | | | | `NOT RUN` | |
| 2 | input | — | supply V/I limit | | | | `NOT RUN` | |
| 3 | R1 | no load | DMM | | | | `NOT RUN` | |
| 3 | R2 | no load | DMM | | | | `NOT RUN` | |
| 3 | R3 | no load | DMM | | | | `NOT RUN` | |
| 3 | R4 | no load | DMM | | | | `NOT RUN` | |
| 4 | R1 | S0 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R2 | S0 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R3 | S0 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R1 | S1 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R2 | S1 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R3 | S1 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R1 | S2 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R2 | S2 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R3 | S2 | shunt + DMM | | | | `NOT RUN` | |
| 4 | R4 | S3 | shunt + DMM | | | | `BLOCKED` | |
| 4 | R4 | S4 | shunt + DMM | | | | `BLOCKED` | |
| 5 | each | S0–S4 | DMM + scope | | P = I × V | | `NOT RUN` | |
| 6 | R4 | servo stall | current probe | | I_peak | | `BLOCKED` | |
| 7 | each | S0–S4 | scope, AC, 20 MHz | | V_pp, V_min | | `NOT RUN` | |
| 8 | input | startup | current probe, 100 ms | | I_peak, order | | `NOT RUN` | |
| 9 | input | brownout | variable supply | | V_reset | | `NOT RUN` | |
| 10 | each | highest reachable | temperature probe | | °C, derating | | `NOT RUN` | |
| 11 | — | cold flash | probe-rs | | image hash | | `NOT RUN` | |
| 12 | — | clock config | debugger registers | | core/AXI/VOS/WS/boost | A14/A16/A17 | `NOT RUN` | |
