<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# RESOURCE_MAP — FW723-IO peripheral resource plan

**Scope.** Turn the block-level wiring diagram into a *demand* list of STM32H723
peripherals, compare it with family-level *availability*, and name the schematic
decisions that close each gap. **No hardware claim and no pin assignment.** Every
instance, pin, AF, stream/channel and package value is `NOT ASSIGNED` until the
schematic and BOM are signed (`PIN_MATRIX.md` §5). This is a plan, not a record of
a board that exists or has been powered.

## 1. Authority

| Source | What it gives | What it does **not** give |
|---|---|---|
| Wiring diagram `5qWfXcTL-6tWOrtA-TiyO4iRbKyXnHvwpJ6E2ErQzmhOeHHmkUE3JOkdnEZg6Sugkj_in42w-5MGQiTDIiXvGwgk.jpg` (`/home/sehaxe/Downloads/`, re-read 2026-11-24, 1751×1530) | blocks, group names and *interface tags*: 11 `SPI`, 3 `UART`, 1 `USB`, 1 `1W`, 1 `АЦП`, 1 `PWM`; no `I2C` or `QSPI` tag. Authoritative transcription: `PIN_MATRIX.md` §1C | no netlist, no MCU pin/AF/DMA, no net names, no camera/GPS/Pi part numbers, no bus partitioning or CS topology |
| `PIN_MATRIX.md` §1A/§1B, citing ST DS13313 Rev 5 (2025-05-22) and ST AN4891 (2025-07-07) | family-level peripheral counts for STM32H723; H743-only items that must not be carried over | bonded pins and exact stream inventory for the procured variant/package (`NOT ASSIGNED`) |
| `PIN_MATRIX.md` §1 B01–B11; `potok-m/docs/src/content/docs/hardware/bom.md` (draft) | working part list and code-side roles | an agreed BOM: the two lists disagree (§3 below) |

The interface tags in the JPEG are printed **rotated 90°**; reading them in
rotated, high-zoom crops of the original on 2026-11-24 gives `SPI` on 11 links:
W25Q512, video flash, H3LIS331DL, both KX134-1211, LIS2MDL, LPS22HH barometer,
LSM6DSR, both MMC5983MA and the pitot LPS22HH. No `I2C` tag and no `QSPI` tag
appears on the drawing. This corrects an earlier reading of LIS2MDL as `I2C`;
the authoritative table for all drawn tags is `PIN_MATRIX.md` §1C and this map
is a derived demand view. A tag is evidence about a *label*, not about a bus
instance, CS net or DMA request; those are `NOT ASSIGNED` (`PIN_MATRIX.md` §1D).

Status values: `NOT ASSIGNED` (physical value unknown), `OPEN` (decision owed by
schematic/BOM owner), `AVAILABLE-FAMILY` (datasheet-level, not bonded-out here),
`CONFLICT-RISK` (two roles may claim one resource), `NOT NEEDED`, `BLOCKED`
(cannot be exercised until a prerequisite is signed).

## 2. What the diagram actually draws

Tags below are a derived view of the authoritative table in `PIN_MATRIX.md`
§1C, not electrical measurements. "MCU path drawn?" says whether a line
visibly reaches the STM32H723 block; an absent line is **not** evidence of an
absent connection. Every physical cell stays `NOT ASSIGNED`.

| Diagram group | Block / label | Tag on its link | MCU path drawn? | Resource implied | Status |
|---|---|---|---|---|---|
| ВИДЕО | `КАМЕРА`, `FLASH-ПАМЯТЬ` | `SPI` on the link toward the antenna-like off-page symbol | **No** — link leaves the group, no MCU line shown | 1 high-bandwidth SPI consumer *if* it lands on the MCU; part/rate unknown | `OPEN` |
| ПИТАНИЕ | `БАТАРЕЯ`, `ТУМБЛЕР`, `USB ПОРТ` | `USB` on the port→MCU link | Yes | 1 USB OTG FS interface; role (device/host) unknown | `OPEN` |
| СВЯЗЬ | `РАДИОМОДУЛЬ LoRa 1262F30-T (готовый LoRa-модуль на базе SX1262)` | `UART` | Yes | 1 UART (RX+TX); module control lines unknown | `OPEN` — conflicts with B08's SPI expectation in `PIN_MATRIX.md` §1 |
| СВЯЗЬ | `GPS-МОДУЛЬ` | `UART` | Yes | 1 UART (at least RX); baud/power unknown | `OPEN` |
| ПАМЯТЬ | `FLASH-ПАМЯТЬ W25Q512 (512 Мбит, 64 МБ)` | `SPI` (several parallel lines drawn near the block) | Yes | 1 SPI bus + CS, **or** OCTOSPI if a schematic decision takes the task's `QSPI`-class alternative | `OPEN` — H723 has OCTOSPI1/2, not H743 QSPI (`PIN_MATRIX.md` §1B/§1D Q5) |
| ИИ | `RASPBERRY Pi 4 Computer Module B, 2 GB RAM` | `UART` | Yes | 1 UART with RX/TX DMA double-buffer (firmware link, 2 Mbit/s 8N1) | `OPEN` — task also names "UART/USB"; diagram shows UART only |
| СИСТЕМА СПАСЕНИЯ | `СЕРВОМОТОР 1`, `СЕРВОМОТОР 2` | `PWM` on their trunk | Yes | 2 PWM channels | `NOT ASSIGNED` |
| СИСТЕМА УПРАВЛЕНИЯ | `СЕРВОМОТОР 3…6` | **no interface tag on the drawn lines** | Yes | 4 more outputs; PWM is *not* established by the diagram | `OPEN` |
| ДАТЧИКИ | `ТЕРМОМЕТР` | `1W` | Yes | 1 timer-capture / UART-1W / bit-bang resource | `OPEN` |
| ДАТЧИКИ | `АКСЕЛЕРОМЕТР H3LIS331DL` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |
| ДАТЧИКИ | `ФОТОРЕЗИСТОРЫ` | `ADC` (one tag; channel count not drawn) | Yes | ≥1 ADC input | `OPEN` |
| ДАТЧИКИ | `ВЫСОКОДИАПАЗОННЫЙ АКСЕЛЕРОМЕТР KX134-1211 ±8g` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |
| ДАТЧИКИ | `ВЫСОКОДИАПАЗОННЫЙ АКСЕЛЕРОМЕТР KX134-1211 ±16g` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |
| ДАТЧИКИ | `3х-осевой МАГНИТОМЕТР LIS2MDL` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |
| ДАТЧИКИ | `БАРОМЕТР LPS22HH` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |
| ДАТЧИКИ | `3х-осевой АКСЕЛЕРОМЕТР + 3х-осевой ГИРОСКОП LSM6DSR` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |
| ДАТЧИКИ | `МАГНИТОМЕТР MMC5983MA` ×2 | `SPI` tags on separate taps | Yes | 2 SPI links + 2 CS (or 2 distinct I²C straps) | `CONFLICT-RISK` — `PIN_MATRIX.md` §4 C02 |
| ДАТЧИКИ | `БАРОМЕТР ДЛЯ ТРУБКИ ПИТО LPS22HH` | `SPI` | Yes | 1 SPI link + CS | `NOT ASSIGNED` |

SPI-tagged links: **11** — 9 sensor links reaching the MCU (H3LIS331DL, both
KX134-1211, LIS2MDL, LPS22HH barometer, LSM6DSR, both MMC5983MA, pitot LPS22HH),
plus the W25Q512 and one off-MCU video link. **No `I2C` tag and no `QSPI` tag**
appear on the drawing. Three UART-tagged links (LoRa, GPS, Pi); one USB link;
one `1W` link; one `АЦП` tag (channel count unresolved); one `PWM` trunk for
recovery servos 1–2 only. Servos 3–6 have no interface tag (see `PIN_MATRIX.md` §1D Q12).

## 3. BOM / diagram disagreement (close before assigning anything)

- The diagram has **two KX134-1211 parts (±8 g and ±16 g)** and an **H3LIS331DL**;
  `PIN_MATRIX.md` §1 lists one KX134 and no H3LIS331DL, while `potok-m`'s draft BOM
  lists H3LIS331DL and no KX134. `ICM-42688-P` (B02 in `PIN_MATRIX.md`) does not
  appear in the diagram. Reconciliation is owed to the schematic/BOM owner; this
  document does not choose between them.
- The diagram adds blocks absent from `PIN_MATRIX.md` §1: camera, video flash,
  USB port, GPS module, Raspberry Pi CM, thermometer, photoresistors, 1W link,
  pitot barometer, and the servo 1–2 pair. Four servos have no code-side role
  (`PIN_MATRIX.md` §1 item 2); the six-servo ↔ two-axis mapping is still `OPEN`.
- The working BOM called LoRa SPI (B08); the authoritative drawn tag is
  **UART**. The W25Q512 tag is **SPI**, not `QSPI`; OCTOSPI1/2 on the H723 is an
  open implementation alternative. Both stay `NOT ASSIGNED` decisions
  (`PIN_MATRIX.md` §1D Q5/Q6), not resolved facts.
- The earlier tag-reading conflict (KX134 pair read `I2C`, LIS2MDL read `I2C`
  here, flash read `QSPI` in the other lane) is **resolved**: re-reading the
  original JPEG on 2026-11-24 shows `SPI` on all three. `PIN_MATRIX.md` §1C is
  now the single authoritative tag table. What remains open is not a reading
  but an implementation decision — SPI vs OCTOSPI (Q5), LoRa UART vs SPI (Q6),
  Pi UART vs USB (Q7) — with the exact questions in `PIN_MATRIX.md` §1D.

## 4. Demand vs family availability

Availability column is the datasheet-level expectation recorded in
`PIN_MATRIX.md` §1A (DS13313 / AN4891), **not** a claim about the procured
variant/package. Demand counts are from §2 and the task statement.

| Resource | Demand | H723 family availability | Close-out / risk |
|---|---:|---|---|
| SPI instances | **10–12 logical consumers**: 9 sensors + W25Q512 + video-SPI (if on MCU) + LoRa **only if** the module is really SPI | 6 SPI (incl. 4 with I2S) | Instances are enough only with multi-drop CS sharing; 1:1 dedicated SPI per device is impossible. Which consumers share SCK/MISO/MOSI is `OPEN` (`PIN_MATRIX.md` §1D Q2). |
| SPI CS lines | **≥10** if all SPI consumers share one bus (9 sensors + flash); +1 video, +1 LoRa NSS if those move to SPI; +2 straps only if a part is remapped to I²C | GPIO count 80 / 112 / 114 by package | Per-device CS is mandatory on a shared MISO bus; no two devices may share a CS net. Exact GPIO budget `NOT ASSIGNED` (`PIN_MATRIX.md` §1D Q3). |
| I²C instances | **0** drawn tags; conditional only if a schematic/BOM decision remaps an SPI part (e.g. MMC5983MA ×2 or a second LPS22HH) | 5 I²C | Enough; instance and addresses/straps must be unique if such a remap happens (`PIN_MATRIX.md` §1D Q18, §4 C02/C11). |
| USART/UART instances | **3**: LoRa, GPS, Pi | 5 USART + 5 UART + 1 ULP UART | Enough instances; each needs distinct pins/AF, Pi needs RX/TX plus two RX DMA slots, and no sharing with USB pins (§5). |
| USB | **1** (`USB ПОРТ`); **+1 alternative** if Pi moves to USB | USB OTG FS (role/package check owed) | Device vs host, VBUS/ID and the Pi link choice are all `OPEN`; diagram only shows UART to Pi. |
| Timer/PWM channels | **≥6 servo** + 1 time driver + 1 thermometer (1W capture) + 1 buzzer + radio duty = **≥9 channels** across ≥2 timer instances; only servos 1–2 are tagged PWM in the diagram | 4×32-bit GP, 10×16-bit GP, 2 advanced-control (family counts in `PIN_MATRIX.md` §1A) | Family-wide capacity is ample, but the time-driver instance (TIM2 is an H743-era *starting point*, not an assignment), the PWM AFs and the 1W capture must not collide. |
| DMA stream/channel assignments | **floor ≈10**: Pi RX×2 + TX×1, LoRa RX/TX (2, if UART), GPS RX×1, W25Q512 RX/TX×2 (or OCTOSPI request), ADC battery/temp×2. **+2 per DMA-driven SPI bus**, +video if on MCU | DMA1 + DMA2 plus MDMA and BDMA named at family level; **stream inventory and DMAMUX request map `NOT ASSIGNED`** | Exact count depends on which sensor buses use DMA; the two Pi RX slots must be distinct (`PIN_MATRIX.md` §4 C01) and no sensor stream may alias UART TX/RX (C06). |
| ADC inputs | **≥2** (battery, temperature) + photoresistors (count unresolved, ≥1) | 2×16-bit + 1×12-bit | Enough channels; divider/thermistor front-end and photoresistor channel count `OPEN`. |
| OCTOSPI / QSPI | **0 or 1** (W25Q512); the drawn tag is `SPI` | OCTOSPI1/2 on H723; no H743-style QSPI | If OCTOSPI is chosen, the 6 IO/CLK/CS signals + DMA replace SPI+CS+2 DMA streams; this is an open schematic decision, not a second tag (`PIN_MATRIX.md` §1D Q5). |
| GPIO | ≥9 SPI CS, 2 MMC straps, LoRa control lines (if module is SPI), health/phase/arm/deploy sense and outputs, SWD | 80 / 112 / 114 by package | Package sign-off closes the budget; no port/pin may be filled from this table. |

## 5. Bus contention and CS requirements

1. **Sensor SPI bus.** One shared SCK/MISO/MOSI triple with 8 device CS lines is
   the only way to fit 8 sensors + flash into 6 SPI instances without extra
   muxing. Consequences to check in the netlist: every device's MISO must
   tri-state when its CS is high; CS lines must be independent; the 64 MB flash
   (erase/write bursts) must not starve the 500 Hz control loop or the Pi link.
   Partitioning flash and sensors onto separate SPI instances is the cheaper
   contention fix *if* the package has the pins — a schematic decision, not a
   pin assignment here.
2. **CS count.** Minimum 9 CS (8 sensors + flash), 10 if video is an MCU SPI
   consumer. The two MMC5983MA parts need two distinct chip selects on SPI or
   two distinct I²C addresses (`PIN_MATRIX.md` §4 C02). LoRa in the diagram is
   UART, so no NSS is counted; if the module's actual interface is SPI, add NSS,
   RESET, BUSY and DIO GPIO (`PIN_MATRIX.md` P24/P25) and one more SPI consumer.
3. **I²C.** No link is tagged I²C — LIS2MDL is `SPI` (§1). If a schematic/BOM
   decision moves any currently SPI-tagged part to I²C, re-check instance and
   address collisions (two LPS22HH parts and two MMC5983MA parts are the obvious
   candidates) — `PIN_MATRIX.md` §1D Q18.
4. **UART.** LoRa, GPS and Pi must be three independent instances; none may
   share a TX or RX net with another. The Pi link is the only one with a codified
   DMA double-buffer requirement (2 Mbit/s, `PIN_MATRIX.md` §4 C01/C06). If the
   Pi migrates to USB, its UART instance and RX streams are released but USB
   OTG FS and the `USB ПОРТ` block then need a shared-vs-separate decision.
5. **DMA arbitration.** All role→stream pairs must be unique; the two Pi RX
   slots are distinct by contract, and no sensor/flash stream may alias UART
   TX/RX. DMA1/DMA2, BDMA and MDMA are different domains — the signed map must
   name the controller for each role, not just a stream number.
6. **Timers.** Time driver, six servos, 1W thermometer and buzzer/search
   schedules must not claim one timer *channel*; sharing one timer instance
   between different channels is acceptable only if the time-base settings
   (prescaler/period) are compatible — a schematic + firmware decision.
7. **Video group.** Camera and video flash are drawn outside the MCU block on
   an SPI-tagged link to an off-page antenna symbol. If that link terminates at
   a separate video SoC, it consumes **no** MCU resource and the MCU needs only
   the control link (its interface is `NOT ASSIGNED`). If it terminates at the
   MCU, its bandwidth and buffer/DMA demand are unmeasured and it must not share
   the sensor SPI bus without a measured budget.
8. **1W thermometer.** Physically a one-wire bus: implement with a timer
   capture compare, a UART in single-wire mode, or bit-banged GPIO — each
   consumes a different resource and changes the timer/DMA collision picture.

## 6. What must be measured or decided in the schematic

Owners are the schematic/BOM owner unless a row names another lane. Every row
starts `OPEN`; HIL gate IDs refer to `HIL_CHECKLIST.md` §12.

| ID | Decision / measurement that closes it | Gate |
|---|---|---|
| D01 | Exact STM32H723 variant and package from the BOM; bonded SPI/I²C/USART/timer/DMA instances and GPIO count; FLASH ≤ 1 MB (`PIN_MATRIX.md` §1A/§1B) | R01 |
| D02 | Full netlist: which of the 8 SPI sensor taps share SCK/MISO/MOSI, and the CS net per device (≥9) | R04 |
| D03 | W25Q512 interface: plain SPI vs OCTOSPI (task's "QSPI"); instance, CS/IO lines and DMA request | R06 |
| D04 | Video group: camera + video flash endpoint (MCU SPI vs separate SoC), part numbers, data rate, buffer size | R07 |
| D05 | LoRa link: diagram `UART` vs module SPI; exact module pinout, NSS/RESET/BUSY/DIO if SPI, baud if UART | R05 |
| D06 | Raspberry Pi link: UART (as drawn) vs USB (as task states); if USB, OTG role and relation to `USB ПОРТ` | R05 |
| D07 | GPS module part, baud rate, RX/TX direction and supply | R05 |
| D08 | Sensor BOM reconciliation: KX134 ×2 vs H3LIS331DL vs `ICM-42688-P`; which parts are fitted, on which bus and CS. Drawn tags are authoritative in `PIN_MATRIX.md` §1C (KX134 ×2, LIS2MDL and flash are all `SPI`; tag-reading conflict resolved 2026-11-24) | R04 |
| D09 | MMC5983MA ×2: distinct CS (SPI) or address straps (I²C); no aliasing | R04 |
| D10 | Six servos: interface tag for servos 3–6, timer/channel/AF per output, 6↔2 axes mapping, and the chosen time-driver timer | R02 |
| D11 | 1W thermometer implementation and resource (timer capture / UART / GPIO), plus photoresistor ADC channel count and analogue front-end | R08 |
| D12 | Signed DMA map: controller + stream/channel + DMAMUX request per role; Pi RX slots distinct; no sensor/flash alias on UART streams | R03 |
| D13 | Clock source / PLL and 500 Hz tick timer; SWD header; no H743-only identifier carried into an H723 image (`PIN_MATRIX.md` §1B) | R01 |
| D14 | Signal-integrity measurement on the final shared SPI bus (SCK/MISO at worst-case fan-out) and flash-write contention against the control loop, on the real board | R06 |

## 7. Status

Nothing in this document is `ASSIGNED`. There is no schematic revision, no BOM
revision, no procured H723 variant/package, no measured rate and no pass result.
The companion procedure in `HIL_CHECKLIST.md` §12 records the on-board checks
that will reject a colliding timer/DMA assignment when the board exists.
