# Архитектура прошивки

Слои идут сверху вниз; каждый слой зависит только от нижележащего. Нижний
слой — `Board` (железо), верхний — `step` (логика одного цикла). Хостовые
тесты подменяют `Board` моком и не знают о Embassy.

```
main.rs (Embassy, только target_os = "none")
    │  реализует Board
    ▼
board.rs (trait Board) ─── mock.rs (MockBoard, только хост)
    │
    ▼
control.rs (step, LinkState) ─ phase.rs ─ deadman.rs ─ watchdog.rs
    │                              │
    │                              ▼
    │                         Command / veto gates
    ▼
protocol.rs (SeqGate, ReplyAssembler, potok-protocol) ─ uart.rs (trait'ы RX/TX)
```

## Модули

| Файл | Граница ответственности | Ключевые места |
|---|---|---|
| `firmware/src/lib.rs` | `no_std`-корень, общие константы и `Command` (вето). `extern crate std` подключается только на хосте. | модули `11:11`–`19:19`, `MAX_CONSECUTIVE_MISSES` `26:26`, `MAX_CORRECTION_RAW` `28:28`, `Command` `33:33`, `clamped` `46:46` |
| `firmware/src/protocol.rs` | Обёртка над `potok-protocol`: магия/CRC/версия проверяются пакетом, здесь — строгий seq и сборка кадра из потока байтов. | `FrameError` `12:12`, `encode_request` `40:40`, `encode_reply` `45:45`, `decode_reply` `51:51`, `SeqGate` `58:58`, `issue` `68:68`, `check` `78:78`, `ReplyAssembler` `102:102`, `push` `117:117` |
| `firmware/src/uart.rs` | Trait'ы `DoubleBufferRx`/`FrameTx` и статические два слота `DmaBuffers`. Никаких вызовов HAL. | `UartError` `5:5`, `DoubleBufferRx` `15:15`, `FrameTx` `22:22`, `DmaBuffers` `30:30`, `swap` `59:59` |
| `firmware/src/phase.rs` | FSM фазы полёта, владельцем которого является STM32. BURN — начальное и безопасное состояние; переходы только вперёд. | `PhaseEvent` `8:8`, `PhaseFsm` `24:24`, `apply` `39:39`, `ai_allowed` `54:54` |
| `firmware/src/deadman.rs` | Счётчик пропущенных циклов/кадров; по достижении трёх запрещает кормление watchdog. | `Deadman` `5:5`, `note_missed` `20:20`, `tripped` `26:26` |
| `firmware/src/watchdog.rs` | Trait `Watchdog`, политика кормления, реальная настройка IWDG только для цели. | `may_feed` `5:5`, `Watchdog` `9:9`, `setup_iwdg` `20:20` |
| `firmware/src/board.rs` | Единственная граница железа: RX, TX, watchdog, время, флаги датчиков, событие фазы, применение команды. | `Board` `11:11` |
| `firmware/src/control.rs` | Состояние канала и один такт: отправка запроса, приём ответа, seq, блокировка, вето, watchdog. | `Counters` `20:20`, `ReplyOutcome` `27:27`, `LinkState` `44:44`, `on_invalid` `98:98`, `disable_now` `112:112`, `reset` `121:121`, `on_reply` `132:132`, `command` `162:162`, `step` `182:182` |
| `firmware/src/mock.rs` | Хост-only мок HAL: скриптованные байты RX, запись отправленных кадров, счётчик pet, журнал применённых команд. | `MockRx` `18:18`, `MockBoard` `85:85`, `queue_reply` `112:112`, `queue_bytes` `119:119`, `impl Board` `146:146` |
| `firmware/src/main.rs` | Тонкий адаптер Embassy; запускает IWDG и цикл 500 Гц; все pins/DMA — `TODO(schematic)`. | `hw::Rx` `25:25`, `hw::Tx` `38:38`, `build_board` `55:55`, `panic` `102:102`, async `main` `109:109` |
| `firmware/tests/safety.rs` | Хостовые гейты: кадры, CRC, seq wrap, BURN-гейт, три пропуска, watchdog/deadman. | 10 тестов, весь файл |

## Один такт (`step`, `firmware/src/control.rs:182`)

1. Спросить у `Board` событие фазы и применить его к FSM (`phase.rs:39`).
2. Собрать `RequestFrame` с фазой STM32, флагами датчиков и `AI_READY`,
   только что выданным seq; закодировать через `potok-protocol` и отдать
   `FrameTx`.
3. Взять готовый DMA-слот через `DoubleBufferRx`, собрать кадр ответа
   (`ReplyAssembler::push`) и проверить CRC/версию в `decode_reply`.
4. `LinkState::on_reply` (`control.rs:132`) проверяет seq и `ai_healthy`,
   сбрасывает счётчик пропусков и deadman, но **не** снимает latch.
5. `LinkState::command` (`control.rs:162`) обнуляет команду при BURN,
   `MOTOR_ARMED`, latch или сработавшем deadman, иначе ограничивает её
   (`Command::clamped`, `lib.rs:46`).
6. `board.apply_command` получает команду в **любом** исходе такта: при
   отклонённом кадре и таймауте явно выдаётся `Command::ZERO` (`control.rs:242`,
   `control.rs:248`).
7. `watchdog::may_feed` (`watchdog.rs:5`) разрешает pet только после полного
   цикла и если deadman не сработал; из UART-колбэка pet не вызывается.

## Инварианты безопасности

- Фаза живёт только в `PhaseFsm` (`phase.rs:24`) и пишется в запрос; в протоколе
  ответа поля фазы нет.
- Нулевая команда — не «результат модели», а состояние по умолчанию и для
  BURN, и для latch, и для любого отклонённого кадра (`control.rs:162`,
  `control.rs:242`, `control.rs:248`).
- Три последовательных промаха latch'ают (`control.rs:98`), снимает только
  `LinkState::reset` (`control.rs:121`), который вызывается явным сбросом
  прошивки. Валидный кадр после latch обнуляет только deadman, не latch.
- CRC и packing не переписаны: `potok-protocol` — единственный источник
  (`protocol.rs:1`).

## Сборка и границы cfg

`firmware/Cargo.toml` объявляет Embassy-зависимости только для
`cfg(target_os = "none")`. Поэтому `cargo test` на хосте собирает библиотеку с
моками без HAL; `cargo check --target thumbv7em-none-eabihf` поднимает реальный
адаптер и IWDG. Двоичный таргет на хосте содержит пустой `main` (`main.rs:10`).

## Незавершённое (железо, без выдуманных пинов)

`hw::build_board` (`main.rs:55`) намеренно `todo!()`: не заданы USART, TX/RX
пины, AF, DMA-потоки и схема ping-pong прерываний. `Rx::take`/`Tx::send`
(`main.rs:33`, `main.rs:44`) — заглушки с TODO. `sensor_flags`, `motor_armed`,
`poll_phase_event`, `apply_command` в `HwBoard` (`main.rs:81`–`main.rs:96`) пока
возвращают безопасные значения: фаза остаётся BURN, значит активной
стабилизации нет. Верификация на плате (логический анализатор 2 Мбит/с, fault
injection, BURN gate) описана в `FIRMWARE_RUST_PLAN.md` репозитория
potok-m-protocol и здесь не имитируется.
