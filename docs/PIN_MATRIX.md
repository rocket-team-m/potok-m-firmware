<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# PIN_MATRIX — typed pin and interface matrix

**No hardware claim.** Every physical cell in this document is `NOT ASSIGNED`.
This is a template to be filled from the board schematic and the procured BOM,
not a record of a board that exists, has been wired, or has been powered. The
interface families below are *datasheet-level expectations for the named part
family*, to be confirmed against the procured part revision; they are not a
selection made on this board. A cell is filled only by a schematic reference.

The code-side boundary is `firmware/src/board.rs` (`trait Board`) and the Embassy
adapter `firmware/src/main.rs`. `build_board()` is still
`todo!("assign UART pins/DMA streams from the board schematic")` (`main.rs:79`),
and `Rx::take`, `Tx::send`, `sensor_flags`, `motor_armed`, `poll_phase_event` and
`apply_command` are safe stubs. No physical row below is therefore wired in
firmware either. See
[HIL_CHECKLIST.md](HIL_CHECKLIST.md) for the bring-up procedure and
[POWER_BUDGET.md](POWER_BUDGET.md) for rails and load currents.

## 1. BOM authority and open reconciliation

The part list used here is the team's working BOM supplied with this task. It is
a list of *parts*, not an assignment of pins. It does **not** agree with every
other document in the workspace; those disagreements are open items, to be
settled by the schematic/BOM owner, not silently resolved here.

| ID | Part | Qty | Logical role | Expected interface family (confirm from procured datasheet) | Bus selection status | Notes |
|---|---|---:|---|---|---|---|
| B01 | STM32H743 | 1 | flight controller, 500 Hz loop | SWD/JTAG debug, UART to Pi, SPI/I²C to sensors | `NOT ASSIGNED` | Variant (`STM32H743ZI` in README §5) is a starting point, not BOM approval |
| B02 | ICM-42688-P | 1 | primary IMU (6-axis) | SPI or I²C | `NOT ASSIGNED` | `sensor_flags` primary-IMU bit; no driver in repo |
| B03 | LSM6DSR | 1 | secondary IMU (6-axis) | SPI or I²C | `NOT ASSIGNED` | redundant-IMU vote in `sensor_voting.rs`; no driver in repo |
| B04 | KX134 | 1 | high-g accelerometer (active phase) | SPI or I²C | `NOT ASSIGNED` | `self_test::SensorChannel::Accel` is one raw channel; mapping of B02/B03/B04 to that channel is `NOT ASSIGNED` |
| B05 | LPS22HH | 1 | barometer | SPI or I²C | `NOT ASSIGNED` | `SensorChannel::Baro`; phase altitude source not wired |
| B06 | LIS2MDL | 1 | magnetometer | SPI or I²C | `NOT ASSIGNED` | `sensor_flags` mag bit |
| B07 | MMC5983MA | 2 | magnetometer, redundant pair | SPI or I²C | `NOT ASSIGNED` | both instances need independent CS/address straps; neither assigned |
| B08 | LoRa1262F30 | 1 | 144 MHz search/telemetry radio module | SPI, plus NSS/RESET/BUSY/DIO GPIO | `NOT ASSIGNED` | module, not a bare SX1262; `search_beacon::RadioHint` has no driver |
| B09 | W25Q512 | 1 | configuration / log NOR flash | SPI (incl. quad variants) | `NOT ASSIGNED` | `calibration_table` / config CRC storage; no driver in repo |
| B10 | servo | 6 | airframe actuator | PWM, one timer channel per servo | `NOT ASSIGNED` | code has **two** axes (`pitch`, `yaw`); mapping of 6 servos to 2 axes is open (see §4) |
| B11 | parachute | 2 | two-stage recovery device | GPIO_OUT spring channels A/B + independent arm inputs A/B | `NOT ASSIGNED` | see §4: actuator technology (pyro vs spring) is undecided |

**Open reconciliation items (not hardware claims, not resolved here):**

1. `potok-m/docs/src/content/docs/hardware/bom.md` lists `H3LIS331DL` where this
   BOM lists `KX134`, does not list the two `MMC5983MA` parts, and calls the BOM
   a draft with power supply unselected. The two lists must be reconciled by the
   owner before a pin assignment is signed.
2. `B10` is six servos in the BOM but only `ActuatorChannel::{Pitch, Yaw}` in
   `self_test.rs` and `Command { pitch, yaw }` in `lib.rs`. Four servos have no
   code-side role. Do not invent one.
3. `sensor_flags()` is documented as IMU/baro/mag/pitot; no pitot part appears in
   this BOM. The pitot row in §2 stays `NOT ASSIGNED`.
4. Deployment device technology (pyro/squib vs spring solenoid) is undecided;
   its supply current is `NOT ASSIGNED` in `POWER_BUDGET.md`.

## 2. Status and type vocabulary

Status is a typed value, never prose. Only `ASSIGNED` may carry a physical pin
number, and only after schematic sign-off in §5.

| Status | Meaning |
|---|---|
| `NOT ASSIGNED` | Physical pin/AF/DMA unknown; schematic + BOM are the only authority |
| `PERIPHERAL` | Internal peripheral with no external pin |
| `SCHEDULER` | Module owns logic only; it owns no pin, timer or driver |
| `STUB` | Code path exists but is `todo!()` or a safe no-op stub |
| `BLOCKED` | Cannot be exercised until a prerequisite row is `ASSIGNED` |
| `ASSIGNED` | Filled from a signed schematic reference (never pre-filled here) |

| Type | Meaning |
|---|---|
| `UART_TX` / `UART_RX` | USART data pin; RX rows are the two DMA double-buffer slots |
| `TIM_PWM` | Timer compare channel driving a servo or buzzer |
| `GPIO_IN` / `GPIO_OUT` | Discrete sense or command line |
| `SPI_*` / `I2C_*` | Bus signal (SCK/MISO/MOSI/NSS or SCL/SDA/straps) |
| `ADC_IN` | Analogue sense input (battery, temperature) |
| `PERIPHERAL` | Internal block (IWDG, time driver) |
| `SCHEDULER` | No physical resource owned |

## 3. Code-side role → physical matrix

One row per logical role. `Port/pin`, `AF`, `DMA` and `Schematic ref` are all
`NOT ASSIGNED` unless a signed schematic is attached; `Status` is the typed
state of the code path, not of the hardware.

| ID | Logical role (code-side name) | Type | BOM | Port/pin | AF | DMA stream/ch | Schematic ref | Status |
|---|---|---|---|---|---|---|---|---|
| P01 | USART TX (`hw::Tx::send`) | `UART_TX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:52`) |
| P02 | USART RX slot A (`hw::Rx::take`) | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:41`) |
| P03 | USART RX slot B (double-buffer peer) | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (same stream pairing) |
| P04 | USART instance / idle-line detect wiring | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` on `build_board()` |
| P05 | Time driver (README starting point TIM2) | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — confirm from BOM, `main.rs:79` |
| P06 | IWDG1 (`setup_iwdg`) | `PERIPHERAL` | B01 | — | — | — | `NOT ASSIGNED` | `PERIPHERAL` (no pin) |
| P07 | Servo PWM — pitch (`apply_command`) | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:117`) |
| P08 | Servo PWM — yaw (`apply_command`) | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:117`) |
| P09 | Servo 3..6 (BOM only) | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no code role |
| P10 | Motor-armed sense (`Board::motor_armed`) | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `STUB` (`main.rs:103`) |
| P11 | Phase-event sources (`poll_phase_event`: Burnout/Apogee/Descent/Touchdown) | `GPIO_IN` | B02–B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:110`) |
| P12 | Sensor health bit — IMU | `GPIO_IN` | B02, B03 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:99`) |
| P13 | Sensor health bit — baro | `GPIO_IN` | B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:99`) |
| P14 | Sensor health bit — mag | `GPIO_IN` | B06, B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:99`) |
| P15 | Sensor health bit — pitot | `GPIO_IN` | none in BOM | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — part unknown |
| P16 | ICM-42688-P bus | `SPI_*`/`I2C_*` | B02 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P17 | LSM6DSR bus | `SPI_*`/`I2C_*` | B03 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P18 | KX134 bus | `SPI_*`/`I2C_*` | B04 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P19 | LPS22HH bus | `SPI_*`/`I2C_*` | B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P20 | LIS2MDL bus | `SPI_*`/`I2C_*` | B06 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P21 | MMC5983MA #1 bus + strap | `SPI_*`/`I2C_*` | B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P22 | MMC5983MA #2 bus + strap (distinct from P21) | `SPI_*`/`I2C_*` | B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P23 | W25Q512 bus + CS | `SPI_*` | B09 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P24 | LoRa1262F30 SPI bus | `SPI_*` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P25 | LoRa1262F30 NSS/RESET/BUSY/DIO | `GPIO_IN`/`GPIO_OUT` | B08 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P26 | Battery sense (`self_test::Battery`) | `ADC_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — divider/ADC unknown |
| P27 | Temperature sense (`self_test::Temperature`) | `ADC_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — sensor unknown |
| P28 | Buzzer PWM (`buzzer::Buzzer::tick`) | `SCHEDULER` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` — owns no pin/timer |
| P29 | Search radio control (`search_beacon::RadioHint`) | `SCHEDULER` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` — adapter absent |
| P30 | Deployment spring output A (`SpringOutputs::a`) | `GPIO_OUT` | B11 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — decision only, no output driver |
| P31 | Deployment spring output B (`SpringOutputs::b`) | `GPIO_OUT` | B11 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — decision only, no output driver |
| P32 | Deployment arm channel A (`ArmChannels::a`) | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — no sense input wired |
| P33 | Deployment arm channel B (`ArmChannels::b`) | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — no sense input wired |
| P34 | SWD debug (probe-rs flash/debug) | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | — | `NOT ASSIGNED` | `BLOCKED` — debug header unknown |

`NOT ASSIGNED` is a status, not a placeholder value. A build must never be made
to pass by inventing an AF or a DMA stream to fill one of these cells
(README rule `[07]`).

## 4. Mapping constraints to prove when the schematic arrives

These are typed constraints, not pin assignments. Each is `NOT VERIFIED` until
signed against the schematic.

| ID | Constraint | Why it matters | Status |
|---|---|---|---|
| C01 | P02 and P03 are distinct DMA slots on one USART RX stream | `double_buffer_receive_swaps_slots` host gate has no board equivalent until the real stream pairing exists | `NOT VERIFIED` |
| C02 | P21 and P22 are electrically independent (separate CS or address) | two MMC5983MA instances cannot collapse to one device | `NOT VERIFIED` |
| C03 | B10 servo count 6 vs code axes 2 is resolved explicitly | four unmapped servos must not be assumed to mirror pitch/yaw | `NOT VERIFIED` |
| C04 | P30 and P31 are independent outputs; P32 and P33 independent inputs | deployment requires both arms and fires both springs together | `NOT VERIFIED` |
| C05 | P10 (motor-armed) is independent of the phase/AI path | AI gate must not be defeatable by a single shared sense line | `NOT VERIFIED` |
| C06 | No sensor bus shares a stream with UART TX/RX P01–P03 | silent DMA arbitration would not be visible in logic | `NOT VERIFIED` |
| C07 | Rail/power test points from `POWER_BUDGET.md` have physical probe access | otherwise ripple and sag cannot be measured at the load | `NOT VERIFIED` |

## 5. Sign-off record

No row is signed. A signature attaches the schematic revision it was read from.

| Field | Value |
|---|---|
| Schematic revision | |
| BOM revision | |
| Board serial / revision | |
| Pin-matrix reviewer (schematic author) | |
| Pin-matrix reviewer (firmware owner) | |
| Date signed | |
| `*TODO(schematic)` / `*TODO(bom)` markers replaced in `main.rs`, `memory.x`, `firmware/Cargo.toml` | `NOT DONE` |

After sign-off, mirror the assigned values into `main.rs`, `memory.x` and
`firmware/Cargo.toml` and set each signed row to `ASSIGNED` with its schematic
reference. `cargo check --target thumbv7em-none-eabihf` is necessary but not
sufficient: it does not prove a pin exists on the board.
