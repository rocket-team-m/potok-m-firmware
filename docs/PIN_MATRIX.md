<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# PIN_MATRIX — typed pin and interface matrix

**No hardware claim.** Every physical cell in this document is `NOT ASSIGNED`.
This is a template to be filled from the board schematic and the procured BOM,
not a record of a board that exists, has been wired, or has been powered. The
interface families below are *datasheet-level expectations for the named part
family*, to be confirmed against the procured part revision; they are not a
selection made on this board. A cell is filled only by a schematic reference.

**Target family correction (task FW723-PINS): STM32H723, not STM32H743.** This
is a family correction only: the exact H723 variant and package
(`STM32H723VE/VG/ZE/ZG`, LQFP100 / LQFP144 / UFBGA144 / TFBGA100) are
`NOT ASSIGNED` until the schematic and BOM are signed (§5). No pin, AF, DMA
stream or instance below is filled from the family datasheet. §1A lists the
peripherals this firmware actually needs; §1B flags H743-only or unavailable
peripherals that must not be carried into the H723 pin map. H743 identifiers
in code, toolchain and other documents are owner-lane items, not resolved here.

The owner's wiring block diagram is the BOM/interface source for §1C
(`/home/sehaxe/Downloads/5qWfXcTL-6tWOrtA-TiyO4iRbKyXnHvwpJ6E2ErQzmhOeHHmkUE3JOkdnEZg6Sugkj_in42w-5MGQiTDIiXvGwgk.jpg`).
It shows functional groups and interface labels only: **no pin numbers, AFs,
DMA streams, bus instances or chip selects**. Every such value stays
`NOT ASSIGNED`.

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

| ID | Part | Qty | Logical role | Expected interface family (label on wiring diagram where shown) | Bus selection status | Notes |
|---|---|---:|---|---|---|---|
| B01 | STM32H723 (family; exact variant/package `NOT ASSIGNED`) | 1 | flight controller, 500 Hz loop | SWD debug, USART/UART to Pi, SPI/I²C sensors, QSPI flash, timers, DMA, ADC, GPIO, IWDG | `NOT ASSIGNED` | H723 family correction; `STM32H743ZI` in README §5 is an H743-era starting point, not BOM approval |
| B02 | ICM-42688-P | 1 | primary IMU (6-axis) | SPI or I²C | `NOT ASSIGNED` | working-BOM part, **absent from the wiring diagram** — reconcile (item 6) |
| B03 | LSM6DSR | 1 | 3-axis accelerometer + 3-axis gyroscope | SPI (diagram `SPI`) | `NOT ASSIGNED` | diagram sensor group; redundant-IMU vote in `sensor_voting.rs`; no driver |
| B04 | KX134-1211 | 2 | high-G accelerometers, ±8 g and ±16 g | I²C (diagram `I2C`) | `NOT ASSIGNED` | diagram shows two instances; working BOM listed one `KX134` — reconcile (item 6) |
| B05 | LPS22HH | 1 | barometer | SPI (diagram `SPI`) | `NOT ASSIGNED` | `SensorChannel::Baro`; phase altitude source not wired |
| B06 | LIS2MDL | 1 | 3-axis magnetometer | SPI (diagram `SPI`) | `NOT ASSIGNED` | `sensor_flags` mag bit |
| B07 | MMC5983MA | 2 | magnetometer pair | SPI (diagram `SPI`) | `NOT ASSIGNED` | diagram shows both on SPI; independent CS unknown (C02/C10) |
| B08 | LoRa 1262F30-T (SX1262-based, 915 MHz, SMA) | 1 | search/telemetry radio module | UART to MCU (diagram `UART`); SMA RF to antenna | `NOT ASSIGNED` | working BOM said SPI; diagram says UART — reconcile (item 7). `RadioHint` has no driver |
| B09 | W25Q512 (512 Mbit / 64 MB) | 1 | configuration / log NOR flash | QSPI (diagram `QSPI`) | `NOT ASSIGNED` | H723 has OCTOSPI, not H743 QSPI (§1B); no driver in repo |
| B10 | servo | 6 | 1–2 recovery, 3–6 control | PWM, one timer channel per servo (diagram `PWM`) | `NOT ASSIGNED` | code has **two** axes (`pitch`, `yaw`); mapping open (C03/C12) |
| B11 | parachute / recovery device (working BOM) | 2 | two-stage recovery | `NOT ASSIGNED` | `NOT ASSIGNED` | diagram shows **recovery servos 1–2**, not pyro/spring GPIO — reconcile (C12) |
| B12 | H3LIS331DL | 1 | accelerometer | SPI (diagram `SPI`) | `NOT ASSIGNED` | diagram sensor group; absent from the working BOM as a distinct part — reconcile (item 6) |
| B13 | photoresistors | `NOT ASSIGNED` | light sense | ADC (diagram `АЦП`) | `NOT ASSIGNED` | plural on diagram, no part number; ADC channel unknown |
| B14 | thermometer | 1 | temperature sense | 1-Wire (diagram `1W`) | `NOT ASSIGNED` | no part number; H723 has no dedicated 1-Wire peripheral (§1A) |
| B15 | LPS22HH (second, flight-path barometer) | 1 | trajectory barometer | SPI (diagram `SPI`) | `NOT ASSIGNED` | diagram says LPS22HH; owner prose mentioned MS5611 — reconcile (item 6) |
| B16 | GPS module (owner: SIM28-class) | 1 | GNSS position | UART (diagram `UART`) | `NOT ASSIGNED` | diagram gives no part number |
| B17 | camera + flash memory (video) | 1 set | video subsystem | no interface to STM32 drawn | `NOT ASSIGNED` | separate `ВИДЕО` group; not in the MCU pin matrix |
| B18 | battery + toggle + USB port | 1 set | power and USB input | power lines; USB (diagram `USB`) | `NOT ASSIGNED` | `Питание` group; rails/test points from schematic |
| B19 | Raspberry Pi 4B 2 GB | 1 | AI advisor link | UART to MCU (P01–P04) | `NOT ASSIGNED` | `ИИ` group on diagram; direction per protocol, not diagram arrows |

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
5. The MCU line here is corrected to STM32H723, but `docs/hardware/bom.md` in
   `potok-m` and `docs/POWER_BUDGET.md` A6/L1 still name STM32H743. H743-only
   identifiers in this repo (`stm32h743zi`, `STM32H743ZI`, `stm32h743.repl`,
   `firmware/src/perf.rs:32` `CORE_HZ = 480_000_000`) and the H743-sized
   `memory.x` FLASH 2048K are **not** approved for an H723 board and are owned
   by other lanes; this document does not change them or invent replacements.
6. The owner wiring diagram is the BOM/interface authority for this revision.
   It does not show ICM-42688-P (B02), pyro/spring deployment outputs (B11) or
   an MS5611; it does show two KX134-1211, H3LIS331DL, LIS2MDL, two MMC5983MA,
   two LPS22HH, thermometer, photoresistors, GPS, USB/battery and a separate
   camera+flash video pair. The owner must reconcile the two lists before any
   pin assignment (HIL BU12).
7. Diagram interface labels disagree with the working BOM: LoRa and GPS are
   `UART` (not SPI), W25Q512 is `QSPI`, the KX134 pair is `I2C`, thermometer is
   `1W`, photoresistors are `АЦП`/ADC. Diagram labels define the typed
   interfaces here; bus instances remain `NOT ASSIGNED`.
8. Recovery is drawn as servos 1–2 on `PWM`, not spring/pyro GPIO. `B11` and
   the `SpringOutputs` / `ArmChannels` code seam stay open until reconciled.
9. The diagram draws shared lines and generic connector squares; it gives no
   chip-select, address strap, bus arbitration or DMA assignment. Those are
   `NOT ASSIGNED` (C10/C11) and must come from the signed schematic.

## 1A. Peripheral families actually needed on H723

Datasheet-level expectations for the H723 family only: ST DS13313 Rev 5
(2025-05-22), `STM32H723xE/G` feature list and Table 1, plus the ST H7 family
product overview for peripheral counts; ST AN4891 (2025-07-07) for the
H72x/H74x comparison. A family-level availability statement is **not**
a statement that a peripheral is bonded out on the procured package. Instance,
pin, AF and DMA cells remain `NOT ASSIGNED`.

| Family | Needed for (code-side role / BOM) | H723 family-level availability (datasheet expectation) | Unassigned value | Decision that closes it | Status |
|---|---|---|---|---|---|
| SPI | Sensors B03, B05–B07, B12, B15 (diagram `SPI`); P17–P22, P38, P42 | present — 6×SPI incl. 4×I2S (DS13313) | instance, SCK/MISO/MOSI/NSS pins, AF, DMA; CS per device | schematic netlist + exact H723 variant/package + procured part pinout | `NOT ASSIGNED` |
| I²C | KX134-1211 ×2 (B04, diagram `I2C`); P18/P41 | present — 5×I²C (DS13313) | instance, SCL/SDA, AF, device addresses | schematic bus selection + BOM addresses | `NOT ASSIGNED` |
| USART / UART | Pi AI link (B19, P01–P04), LoRa (B08, P24), GPS (B16, P37) | present — 5×USART + 5×UART + 1×ULP UART (DS13313) | instance, TX/RX pins, AF, two RX DMA slots | schematic instance/pins/AF/DMA + BOM variant | `NOT ASSIGNED` |
| Timers | Embassy time driver (P05; TIM2 is an H743-era starting point only), servo PWM P07–P09/P44 | present — 32-bit GP, 16-bit GP and advanced-control families (DS13313: 4 / 10 / 2) | instance + channel + AF per output | schematic timer assignment + BOM confirmation of `time-driver-tim2` | `NOT ASSIGNED` |
| DMA | UART double-buffer RX/TX (P01–P03), later sensor streams | present — DMA1/DMA2 plus MDMA and BDMA on H723 (DS13313 bus matrix) | controller, stream/channel per role, priorities | schematic + Embassy channel allocation; P02/P03 distinct (C01) | `NOT ASSIGNED` |
| ADC | Battery (P26), temperature (P27), photoresistors (B13/P39, diagram `АЦП`) | present — 2×16-bit + 1×12-bit (DS13313); **not** H743's 3×16-bit | instance, channel, pin, divider/thermistor | schematic analogue front-end + BOM sensor parts | `NOT ASSIGNED` |
| GPIO | P10–P15, P25, P30–P34, P36 | present; count is package-dependent (DS13313: 80 / 112 / 114) | port/pin per role | schematic + exact package from BOM | `NOT ASSIGNED` |
| IWDG | Watchdog (P06, `setup_iwdg(p.IWDG1)`) | present — 2× watchdog (DS13313); no external pin | instance — IWDG1 is a code starting point | BOM/variant confirm; no schematic pin | `PERIPHERAL`, instance `NOT ASSIGNED` |
| SWD | Flash/debug (P34) | debug port present on family | header/pins | schematic debug connector + BOM probe | `NOT ASSIGNED` |
| 1-Wire | Thermometer (B14/P40, diagram `1W`) | **no dedicated 1-Wire peripheral** on H723; GPIO + driver decision, not a peripheral claim | GPIO pin, driver choice, timing source | schematic net + BOM part + driver lane | `NOT ASSIGNED` |
| QSPI / OCTOSPI-class | W25Q512 (B09/P23, diagram `QSPI`) | H723 has OCTOSPI1/2, **not** H743's Quad-SPI (§1B) | instance, IO pins, AF, CS/clock | schematic + exact variant/package | `NOT ASSIGNED` |
| USB | USB port (B18/P35) | USB OTG present on H723 family | instance, pins, AF | schematic + BOM variant | `NOT ASSIGNED` |

## 1B. H743-only or unavailable peripherals — explicit flags

Carried over from H743-era documents or code. None is a claim about the
procured H723 board; none may fill a pin cell in §3.

| H743-era item | H723 status (datasheet-level) | Needed here? | Close-out owner / decision | Status |
|---|---|---|---|---|
| FMC with 32-bit bus / SDRAM on any package | H723 FMC is 16-bit; the NOR/SDRAM controller exists only on Z packages (ZG/ZI), not V packages (DS13313 Table 1) | No — no external RAM/SDRAM in the BOM | schematic/BOM if an FMC device is ever added | `NOT NEEDED` — do not assume |
| JPEG codec | H743 has it; H723 does not (ST H7 family overview) | No | none | `UNAVAILABLE on H723` |
| Dual-mode Quad-SPI (QSPI) | H723 has OCTOSPI1/2 instead of H743's QSPI | **Yes** — diagram labels the W25Q512 link `QSPI`; instance/pins `NOT ASSIGNED` | schematic bus choice + BOM part variant; do not reuse H743 QSPI | `DO NOT SUBSTITUTE` |
| ADC3 as a third 16-bit SAR ADC | H723's third ADC is 12-bit, not 16-bit (2×16-bit + 1×12-bit) | ADC sense only (P26/P27); instance unassigned | schematic front-end + BOM | `DIFFERENT — NOT ASSIGNED` |
| `stm32h743zi`, `STM32H743ZI`, `stm32h743.repl`, `CORE_HZ = 480_000_000` | H743-only toolchain/clock identifiers in `firmware/Cargo.toml`, `.cargo/config.toml`, `docs/RENODE.md` + `emulator/`, `firmware/src/perf.rs:32` | Not a peripheral, but must not be carried into an H723 flash | owner lanes outside this document | `OUT OF SCOPE — H743 identifier` |
| `memory.x` FLASH 2048K / RAM 128K @ `0x20000000` | H723 family: up to 1 MB flash, 564 KB SRAM; 2048K is an H743 value. The 128K at `0x20000000` matches family DTCM but is not a signed map (AXI/SRAM banks differ) | memory map must match the procured variant | BOM + schematic → `memory.x` owner | `NOT ASSIGNED` |
| Clock budget at 480 MHz (H743 rating) | H723 rated up to 550 MHz (DS13313); actual clock comes from schematic oscillators/PLL | cycle budget depends on it | owner code lane (`perf.rs`) + schematic clock source | `NOT ASSIGNED` |

## 1C. Wiring-derived capability table

Transcribed from the owner's wiring block diagram (path in the header note).
The diagram shows groups and interface **labels**, never a pin number, AF,
DMA stream, bus instance, chip select or address strap. Every physical cell
therefore stays `NOT ASSIGNED`; `BLOCKED` is the code/driver state, not a
hardware verdict. The diagram is the BOM/interface source for this revision,
subject to the reconciliation items in §1.

| Capability | Diagram group / part | Typed interface from diagram | MCU involvement | Physical pin / instance | Status | Decision that closes it |
|---|---|---|---|---|---|---|
| Power and USB input | `Питание`: battery, toggle, USB port (B18) | power lines; `USB` | MCU power pins + USB device | `NOT ASSIGNED` | `BLOCKED` — rails/test points unknown | schematic rails + connector + BOM regulator (PB §6) |
| Recovery actuation | `Система спасения`: servos 1–2 (B10) | `PWM` | MCU timer channels | `NOT ASSIGNED` | `BLOCKED` — no driver; conflicts with B11 (§1 item 8) | schematic PWM nets + owner servo↔recovery mapping (C12) |
| Control actuation | `Система управления`: servos 3–6 (B10) | `PWM` (one drawn line per servo) | MCU timer channels | `NOT ASSIGNED` | `BLOCKED` — code has only pitch/yaw | schematic PWM nets + 6↔2 axis mapping (C03) |
| AI advisor link | `ИИ`: Raspberry Pi 4B 2 GB (B19) | `UART` (as used by P01–P04) | MCU USART TX/RX + DMA | `NOT ASSIGNED` | `STUB` (`main.rs:52`) | schematic USART/AF/DMA + protocol direction |
| Radio search/telemetry | `Связь`: LoRa 1262F30-T, SX1262-based, 915 MHz SMA (B08) | `UART` to MCU; SMA RF to antenna | MCU USART + control GPIO | `NOT ASSIGNED` | `BLOCKED` — no driver | schematic UART/AF/GPIO + module pinout; reconcile item 7 |
| GNSS | `Связь`: GPS module, owner says SIM28-class (B16) | `UART` | MCU USART RX | `NOT ASSIGNED` | `BLOCKED` — no driver | schematic UART/AF + BOM part |
| Configuration / log memory | `Память`: W25Q512 512 Mbit / 64 MB (B09) | `QSPI` (H723: OCTOSPI-class) | MCU OCTOSPI IO/CS/clock | `NOT ASSIGNED` | `BLOCKED` — no driver | schematic QSPI net + variant/package; §1B do-not-substitute |
| SPI sensor set | `Датчики`: H3LIS331DL (B12), LIS2MDL (B06), LPS22HH ×2 (B05/B15), LSM6DSR 3-axis accel+gyro (B03), MMC5983MA ×2 (B07) | `SPI` on each drawn link | shared MCU SPI bus (drawing shows common lines) | `NOT ASSIGNED` | `BLOCKED` — no drivers | schematic CS lines, bus instance, arbitration (C10); no CS shown on diagram |
| I²C high-G pair | `Датчики`: KX134-1211 ±8 g and ±16 g (B04) | `I2C` | MCU I²C bus | `NOT ASSIGNED` | `BLOCKED` — no drivers | schematic bus instance + device addresses (C11) |
| Thermometer | `Датчики`: thermometer (B14) | `1W` (1-Wire; no dedicated H723 peripheral) | MCU GPIO + driver | `NOT ASSIGNED` | `BLOCKED` — no driver | schematic net + BOM part + driver decision |
| Light sense | `Датчики`: photoresistors (B13) | `АЦП` / ADC | MCU ADC channel + pin | `NOT ASSIGNED` | `BLOCKED` — no driver | schematic front-end + ADC channel |
| Video subsystem | `Видео`: camera + flash memory (B17) | no interface to STM32 drawn | none shown | `NOT ASSIGNED` (`OFF_MCU`) | `BLOCKED` — off-MCU subsystem | owner decision whether it belongs to STM32 at all; absent from MCU matrix |
| Debug | not shown on diagram | SWD | MCU debug port | `NOT ASSIGNED` | `BLOCKED` — debug header unknown | schematic debug connector + BOM probe (P34) |

**Unknown bus arbitration / CS:** the diagram draws several sensors on what
look like common lines and marks each link only by a generic connector square.
It does not show chip selects, I²C addresses, bus ownership, DMA channels or
the SPI instance. Do not read a shared drawn line as one selected bus and do
not assume a chip-select is duplicated: C10/C11 stay `NOT VERIFIED` until the
signed schematic names them.

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
| `QSPI_*` | QSPI/OCTOSPI-class IO, clock or chip select (diagram `QSPI`) |
| `ONE_WIRE` | 1-Wire data line; on H723 a GPIO+driver decision, not a dedicated peripheral |
| `USB` | USB device port (diagram `USB`) |
| `OFF_MCU` | Subsystem with no interface to STM32 drawn; owns no MCU pin |

## 3. Code-side role → physical matrix

One row per logical role. `Port/pin`, `AF`, `DMA` and `Schematic ref` are all
`NOT ASSIGNED` unless a signed schematic is attached; `Status` is the typed
state of the code path, not of the hardware.

| ID | Logical role (code-side name) | Type | BOM | Port/pin | AF | DMA stream/ch | Schematic ref | Status |
|---|---|---|---|---|---|---|---|---|
| P01 | USART TX (`hw::Tx::send`) | `UART_TX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:52`) |
| P02 | USART RX slot A (`hw::Rx::take`) | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:41`) |
| P03 | USART RX slot B (double-buffer peer) | `UART_RX` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (same stream pairing) |
| P04 | USART instance / idle-line detect wiring (Pi B19) | `PERIPHERAL` | B01/B19 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` on `build_board()` |
| P05 | Time driver (README starting point TIM2) | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — confirm from BOM, `main.rs:79` |
| P06 | IWDG1 (`setup_iwdg`) | `PERIPHERAL` | B01 | — | — | — | `NOT ASSIGNED` | `PERIPHERAL` (no pin) |
| P07 | Servo PWM — pitch (`apply_command`); diagram servos 3–6 are control | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:117`) |
| P08 | Servo PWM — yaw (`apply_command`); diagram servos 3–6 are control | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:117`) |
| P09 | Servos 3–6 (control, diagram `PWM`) | `TIM_PWM` | B10 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no code role for four of six |
| P10 | Motor-armed sense (`Board::motor_armed`) | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `STUB` (`main.rs:103`) |
| P11 | Phase-event sources (`poll_phase_event`: Burnout/Apogee/Descent/Touchdown) | `GPIO_IN` | B02–B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:110`) |
| P12 | Sensor health bit — IMU | `GPIO_IN` | B02, B03 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:99`) |
| P13 | Sensor health bit — baro | `GPIO_IN` | B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:99`) |
| P14 | Sensor health bit — mag | `GPIO_IN` | B06, B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `STUB` (`main.rs:99`) |
| P15 | Sensor health bit — pitot | `GPIO_IN` | none in BOM | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — part unknown |
| P16 | ICM-42688-P bus (working BOM; absent from diagram) | `SPI_*`/`I2C_*` | B02 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — reconcile, no driver |
| P17 | LSM6DSR 3-axis accel + gyro (diagram `SPI`) | `SPI_*` | B03 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P18 | KX134-1211 ×2 ±8 g/±16 g (diagram `I2C`) | `I2C_*` | B04 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver; addresses unknown |
| P19 | LPS22HH barometer (diagram `SPI`) | `SPI_*` | B05 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P20 | LIS2MDL magnetometer (diagram `SPI`) | `SPI_*` | B06 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P21 | MMC5983MA #1 (diagram `SPI`) | `SPI_*` | B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — CS `NOT ASSIGNED` |
| P22 | MMC5983MA #2 (diagram `SPI`) | `SPI_*` | B07 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — CS `NOT ASSIGNED` |
| P23 | W25Q512 512 Mbit / 64 MB (diagram `QSPI`) | `QSPI_*` | B09 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver; OCTOSPI instance unknown |
| P24 | LoRa 1262F30-T UART (diagram `UART`) | `UART_TX`/`UART_RX` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver; was SPI in working BOM |
| P25 | LoRa control GPIO (not shown on diagram) | `GPIO_IN`/`GPIO_OUT` | B08 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no driver; lines unknown |
| P26 | Battery sense (`self_test::Battery`, diagram battery) | `ADC_IN` | B18 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — divider/ADC unknown |
| P27 | Temperature sense (`self_test::Temperature`) | `ADC_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — diagram thermometer is `1W` (P40), not ADC |
| P28 | Buzzer PWM (`buzzer::Buzzer::tick`) | `SCHEDULER` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` — owns no pin/timer |
| P29 | Search radio control (`search_beacon::RadioHint`) | `SCHEDULER` | B08 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `SCHEDULER` — adapter absent |
| P30 | Deployment spring output A (`SpringOutputs::a`) | `GPIO_OUT` | B11 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — decision only, no output driver |
| P31 | Deployment spring output B (`SpringOutputs::b`) | `GPIO_OUT` | B11 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — decision only, no output driver |
| P32 | Deployment arm channel A (`ArmChannels::a`) | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — no sense input wired |
| P33 | Deployment arm channel B (`ArmChannels::b`) | `GPIO_IN` | `NOT ASSIGNED` | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `SCHEDULER` — no sense input wired |
| P34 | SWD debug (probe-rs flash/debug) | `PERIPHERAL` | B01 | `NOT ASSIGNED` | `NOT ASSIGNED` | — | `NOT ASSIGNED` | `BLOCKED` — debug header unknown |
| P35 | USB port (diagram `USB`) | `USB` | B18 | `NOT ASSIGNED` | `NOT ASSIGNED` | — | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P36 | Battery / toggle sense (diagram `Питание`) | `GPIO_IN` | B18 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — wiring intent unknown |
| P37 | GPS module UART (diagram `UART`) | `UART_RX` | B16 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P38 | H3LIS331DL accelerometer (diagram `SPI`) | `SPI_*` | B12 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver |
| P39 | Photoresistors (diagram `АЦП`) | `ADC_IN` | B13 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no driver; part count unknown |
| P40 | Thermometer (diagram `1W`) | `ONE_WIRE` | B14 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no 1-Wire peripheral/driver |
| P41 | KX134-1211 ×2 I²C (diagram `I2C`) | `I2C_*` | B04 | `NOT ASSIGNED` | `NOT ASSIGNED` | — | `NOT ASSIGNED` | `BLOCKED` — addresses unknown |
| P42 | Second LPS22HH, flight-path baro (diagram `SPI`) | `SPI_*` | B15 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — no driver; MS5611 conflict open |
| P43 | Camera + flash video pair | `OFF_MCU` | B17 | `NOT ASSIGNED` | — | — | `NOT ASSIGNED` | `BLOCKED` — no interface to STM32 drawn |
| P44 | Recovery servos 1–2 (diagram `PWM`) | `TIM_PWM` | B10/B11 | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `NOT ASSIGNED` | `BLOCKED` — conflicts with pyro/spring B11 (C12) |

`NOT ASSIGNED` is a status, not a placeholder value. A build must never be made
to pass by inventing an AF or a DMA stream to fill one of these cells
(README rule `[07]`).

### 3.1. Unresolved pin → decision map

Every non-`ASSIGNED` row above is held open by a named decision. This map does
not fill a cell; it names who closes it. `Schematic` means the signed netlist
and package pinout; `BOM` means the procured part/variant decision; `owner`
means a lane outside this document. Gate IDs without a `PIN` prefix are
`HIL_CHECKLIST.md` IDs.

| PIN rows | Unresolved value | Decision that closes it | Gate |
|---|---|---|---|
| P01–P04 | USART instance, TX/RX pins, AF, two DMA slots, idle-line wiring | schematic: instance + pins + AF + RX stream pairing; BOM: exact H723 variant/package | PIN §5; C01 |
| P05 | Time-driver timer instance | schematic timer selection; BOM confirmation of `time-driver-tim2` (TIM2 is H743-era only as a starting point) | BU01 |
| P06 | IWDG instance | BOM/variant confirm; internal peripheral, no schematic pin | BU01 |
| P07–P09 | PWM timer/channel/AF per servo; 6 servos ↔ 2 axes | schematic servo outputs and timer channels; owner mapping decision for servos 3–6 | C03; G01–G05 |
| P10 | Sense pin and source part | schematic net + connector; BOM part | C05 |
| P11 | Phase-event source pins (Burnout/Apogee/Descent/Touchdown) | schematic nets; BOM decision — no such sensor part appears in the current BOM | C05; S05 |
| P12–P15 | GPIO health bits; pitot source | schematic pins; BOM decision for pitot (part absent from BOM) | S04 |
| P16–P22 | Bus instance, pins, AF, CS/address straps | schematic bus netlist; BOM bus protocol per part; driver lane for the missing drivers. Diagram shows no CS/address — do not infer them | C02; C06; C10; C11 |
| P23–P25 | QSPI flash IO/CS/clock, LoRa UART + control GPIO | schematic nets; BOM module pinout; reconcile LoRa SPI→UART | C06; item 7 |
| P35–P36 | USB port pins and power/toggle sense | schematic power/USB nets + BOM regulator | BU04 |
| P37–P42 | GPS UART, H3LIS331DL/second LPS22HH SPI, photoresistor ADC, thermometer 1-Wire, KX134-1211 ×2 I²C | schematic nets + BOM part numbers; no CS/addresses shown; driver lane | C10; C11; S09 |
| P43 | Video subsystem interface | owner decision whether camera+flash connect to STM32 at all | §1 item 6 |
| P44 | Recovery servo PWM nets and B11 conflict | schematic PWM nets + owner decision servo recovery vs pyro/spring | C12; BU12 |
| P26–P27 | ADC instance/channel/pin and analogue front-end | schematic divider/thermistor and pin; BOM sensor parts | BU04 |
| P28–P29 | none — scheduler roles own no pin | owner adapter/driver lane, not a schematic cell | F12 open items |
| P30–P33 | Spring output pins and arm input pins | schematic outputs/inputs; BOM decision on actuator technology (pyro vs spring) | C04; D01–D09 |
| P34 | SWD header/pins | schematic debug connector + BOM probe; never the H743 probe chip string | BU09 |

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
| C08 | Exact H723 variant/package and FLASH/RAM map signed before any §3 cell is filled | package decides bonded pins, FMC availability and GPIO count; FLASH 2048K is an H743 value | `NOT VERIFIED` |
| C09 | No H743-only peripheral or identifier carried into the H723 pin map (§1B) | FMC/SDRAM, JPEG, H743 Quad-SPI, 3×16-bit ADC and H743 tool identifiers are not H723 facts | `NOT VERIFIED` |
| C10 | SPI chip selects and bus arbitration for the diagram's shared sensor lines are named | seven SPI devices plus QSPI memory cannot collapse to one unselected bus; diagram shows no CS | `NOT VERIFIED` |
| C11 | KX134-1211 ×2 I²C addresses and bus instance are distinct | ±8 g and ±16 g parts must not alias on one address | `NOT VERIFIED` |
| C12 | Recovery servos 1–2 vs `B11` pyro/spring outputs reconciled | diagram shows PWM recovery, working BOM shows spring/pyro GPIO; do not merge the two | `NOT VERIFIED` |
| C13 | Schematic-to-BOM reconciliation from the owner wiring diagram signed before any pin assignment | diagram omits ICM-42688-P/MS5611 and adds several parts; interface labels differ from the working BOM | `NOT VERIFIED` |

## 5. Sign-off record

No row is signed. A signature attaches the schematic revision it was read from.

| Field | Value |
|---|---|
| Schematic revision | |
| BOM revision | |
| H723 exact variant / package (from BOM) | |
| H723 peripheral availability cross-check (§1A / §1B) | |
| Board serial / revision | |
| Pin-matrix reviewer (schematic author) | |
| Pin-matrix reviewer (firmware owner) | |
| Date signed | |
| `*TODO(schematic)` / `*TODO(bom)` markers replaced in `main.rs`, `memory.x`, `firmware/Cargo.toml` | `NOT DONE` |
| H743-only tool/clock identifiers reconciled in their owner lanes (`stm32h743zi`, `STM32H743ZI`, `stm32h743.repl`, `CORE_HZ`) | `NOT DONE` |
| Schematic-to-BOM reconciliation from owner wiring diagram (§1 items 6–9; HIL BU12) | `NOT DONE` |

After sign-off, mirror the assigned values into `main.rs`, `memory.x` and
`firmware/Cargo.toml` and set each signed row to `ASSIGNED` with its schematic
reference. `cargo check --target thumbv7em-none-eabihf` is necessary but not
sufficient: it does not prove a pin exists on the board.
