# F1: SPI bus, `prosdk::SPI` (gap G1)

Platform package for the Boardoza SPI drivers. Boards covered: none directly. It is the bus for
TMAG5170, W25Q40, W25N01GV, microSD, SX1262 (E22), RFM68LCW, FT800/FT810 and their bundles, the
clocked LEDs (Phoenix family, rotary encoder board) and BMI088 in SPI mode.

WPILib 2027 has no SPI (SystemCore has none), so prosdk had no SPI HAL, sim or class. F1 keeps
the WPILib 2026 `frc::SPI` API shape as a hand-written class, as `Servo` does.

## Files

| File | What |
|---|---|
| `src/prosdk/drivers/bus/SPI.hpp`, `.cpp` | `prosdk::SPI` (drivers glob, both builds) |
| `src/prosdk/platform/hal/SPI.hpp` | backend interface, `prosdk::SpiSegment` |
| `src/prosdk/platform/hal/esp32/SPI.cpp` | ESP32-S3 backend (ESP-IDF `spi_master`), target only |
| `src/prosdk/platform/hal/esp32/SpiInternal.hpp` | `detail::SpiHostOf`, `RetainSpiBus`, `ReleaseSpiBus` (for the SD card, G7) |
| `src/prosdk/platform/hal/SPISim.cpp` | host sim backend; `#if !PROSDK_TARGET_HW`, empty on the robot |
| `src/prosdk/simulation/SpiSim.hpp` | sim hooks for tests |
| `src/prosdk/platform/HalConfig.hpp`, `.cpp` | `SetSPIPins`, `GetSPIConfig`, `kNumSPIBuses` |
| `test/prosdk/drivers/FakeSPIDevice.hpp`, `SPITest.cpp` | fake device and tests |

## Wiring

- Two buses: `SPI::kPort0` is SPI2 (FSPI), `SPI::kPort1` is SPI3. SPI0/SPI1 (flash and PSRAM)
  are never used.
- Bus 0 defaults to SCK 17 / MOSI 15 / MISO 16, the J5 header of the Probot main board (Boardoza
  Pulse S32-S3, since 2026-10-23); bus 1 has no default. Other pins, before the first device on
  a bus: `prosdk::platform::SetSPIPins(bus, sck, mosi, miso)`. `miso` may be -1 on a bus that only
  transmits (clocked LEDs). It returns false for a bad bus or pin (0-48) or two equal pins. A bus
  that is already running keeps its pins until its last device closes.
- Any free GPIO works through the GPIO matrix. The SPI2 IO_MUX pins are SCK 12, MOSI 11, MISO 13,
  CS 10 (ESP-IDF `soc/esp32s3/include/soc/spi_pins.h`). The pins given to `SetSPIPins` decide:
  ESP-IDF is expected to use the IO_MUX when all bus pins are IO_MUX pins and the matrix
  otherwise (ESP-IDF source behaviour, unverified here).
- Chip select is any GPIO, driven by the peripheral's hardware CS signal. The bus allows 6 chip
  selects on SPI2 and 3 on SPI3 (`SOC_SPI_PERIPH_CS_NUM`, ESP-IDF 5.5 `soc_caps.h`).
- **A device without chip select (`csGpio` -1) must be alone on its bus.** Clocked LEDs latch
  every clock edge, so traffic for any other device would corrupt them. Opening a device without
  chip select on a bus that has any user fails, and so does the reverse. In practice the clocked
  LEDs take `kPort1` and every CS device shares `kPort0`, which leaves one bus for flash, SD,
  LoRa, EVE and TMAG5170.
- **5 V hazard:** ESP32-S3 GPIOs are 3.3 V only. A board that drives MISO (or any line) at 5 V
  must not be wired straight to the controller. The board-specific facts are in each driver
  package's document.
- Arduino's `SPI` library (`SPIClass`, whose global `SPI` object is FSPI = SPI2) must not run on
  a bus that prosdk uses. A pin conflict is caught by the pin check. A peripheral conflict on
  other pins is expected to make `spi_bus_initialize` fail, which reports the error with a hint;
  this is **unverified**. With `using namespace prosdk;` a sketch that also includes `<SPI.h>`
  has two `SPI` names; write `prosdk::SPI`.

## Class API (`prosdk::SPI`)

```cpp
prosdk::platform::SetSPIPins(0, /*sck=*/12, /*mosi=*/11, /*miso=*/13);
prosdk::SPI flash{prosdk::SPI::kPort0, /*csGpio=*/10};
flash.SetClockRate(20'000'000);
uint8_t cmd = 0x9F;
uint8_t id[3];
prosdk::SpiSegment frame[] = {{&cmd, nullptr, 1}, {nullptr, id, 3}};
flash.TransactionSegments(frame);  // CS held over both segments; returns 4
```

- `SPI(Port bus, int csGpio)` opens the device: mode 0, 1 MHz, CS active low. It throws
  `prosdk::RuntimeError` when the bus is invalid or has no pins, a pin is in use (the message
  names the owner), the chip select is already open, the bus has no free chip select, or the
  no-CS rule is broken. This matches `prosdk::I2C` and `SerialPort`. Driver constructors still
  do not throw for a missing chip.
- The class is move-only. The destructor closes the device, and the last device on a bus stops
  the bus and frees its pins.
- `enum Port { kPort0, kPort1 }` and `enum Mode { kMode0 .. kMode3 }` are unscoped, as in WPILib
  2026, so both `SPI::kMode3` and `SPI::Mode::kMode3` compile.
- `SetClockRate(hz)` accepts 78 125 Hz to 40 MHz and throws `ParameterOutOfRange` outside that
  range. 78 125 Hz is 80 MHz / (prescaler 16 x divider 64), from ESP-IDF `hal/spi_ll.h`. The
  actual rate is the nearest divider result. The default is 1 MHz (WPILib 2026: 500 kHz).
- `SetMode`, `SetChipSelectActiveHigh`, `SetChipSelectActiveLow`: on the robot each change
  removes the device and adds it again with the new settings (ESP-IDF fixes them per device); a
  call that changes nothing does nothing. A setting the backend refuses (for example a clock
  that ESP-IDF's timing check rejects) throws, and the device keeps its previous settings.
- `Write(const uint8_t*, int)`, `Read(bool initiate, uint8_t*, int)` and
  `Transaction(const uint8_t*, uint8_t*, int)` each run one transaction. They return the size, or
  -1 on failure. `Read` clocks out zeros. `initiate` is ignored because there is no receive FIFO
  left over from an earlier write.
- `TransactionSegments(std::span<const SpiSegment>)` runs several segments with the chip select
  held from the first byte to the last. A segment is `{const uint8_t* tx, uint8_t* rx, int size}`:
  tx nullptr sends zeros, rx nullptr discards what comes back, and size 0 is allowed. It returns
  the total size, or -1 (also for a negative size).
- `GetPort()`, `GetChipSelect()`.
- Not ported from WPILib 2026: the auto-transfer engine (`InitAuto`, `StartAutoRate`, ...) and
  the accumulator. The ESP32-S3 has no equivalent.
- Driver convention: every SPI chip driver takes `(SPI::Port bus, int csGpio, ...)` and owns its
  `prosdk::SPI` member. No driver takes an `SPI&`.

## ESP32-S3 backend

- The bus starts lazily on its first device. There is no `Initialize*` hook, so
  `Esp32Internal.hpp` and `HAL.cpp` are unchanged. Setup is `spi_bus_initialize` with
  `SPI_DMA_CH_AUTO` and `max_transfer_sz` 4092 (the DMA default in `spi_common.h`), then one
  `spi_bus_add_device` per device with `queue_size` 1. The bus is freed with `spi_bus_free` when
  its last user leaves; if that fails (a `RetainSpiBus` user left its own device on the bus), one
  console line says so.
- SCK, MOSI, MISO and every CS pin are claimed in the HAL pin table with `ClaimPin` (handle type
  `VENDOR`, since WPILib 2027 has no SPI handle type), as the serial ports do. DIO, PWM and the
  other pin APIs on those pins fail and name the SPI owner. On release the pins return to
  `gpio_reset_pin` state.
- Transfers are full duplex. Pieces of at most 64 bytes (the CPU FIFO,
  `SOC_SPI_MAXIMUM_BUFFER_SIZE`) are polled (`spi_device_polling_transmit`). Longer ones use the
  interrupt and DMA (`spi_device_transmit`). A segment over 4092 bytes is split. When a
  transaction has more than one piece, the backend acquires the bus (`spi_device_acquire_bus`)
  and sets `SPI_TRANS_CS_KEEP_ACTIVE` on every piece but the last, so CS stays active and no
  other device can cut in. ESP-IDF copies buffers that are not DMA-capable or aligned into
  temporary buffers itself.
- Segments without tx send from a 4092-byte zero buffer. It is allocated in DMA-capable memory
  on first use and kept.
- A transmit-only bus (MISO -1) adds its devices with `SPI_DEVICE_NO_DUMMY`. This skips ESP-IDF's
  full-duplex read-timing check, which does not apply when nothing is read.
- Transfers on a bus are serialized by one mutex per bus.
- **Not gated:** neither the output gate nor the emergency stop stops SPI, as with I2C and serial
  (design/hal-esp32.md "Not gated", integration edit). No Boardoza SPI board drives an actuator.
- `SpiInternal.hpp` (G7, for P9's SD card): `detail::SpiHostOf(bus)` gives `SPI2_HOST` or
  `SPI3_HOST`. `detail::RetainSpiBus(bus, &status)` starts the bus if needed and counts one user,
  like a CS device. `detail::ReleaseSpiBus(bus)` drops that user. The SD code claims its own CS
  pin (`ClaimPin`) and lets `esp_vfs_fat_sdspi_mount` add its device to `SpiHostOf(bus)`.
  RetainSpiBus is refused on a bus that has a no-CS device.

## Host simulation

- `SPISim.cpp` implements the same interface with the same rules: bus range, one owner per CS
  GPIO across both buses, at most 6 or 3 chip selects, the no-CS rule, mode 0-3. It needs no
  pins and does not check the bus or CS GPIOs against DIO or other pin users, which the robot
  does.
- `prosdk/simulation/SpiSim.hpp` provides:
  - `sim::RegisterSpiDevice(bus, cs, fn)` and `sim::CancelSpiDevice(id)`. `fn(tx, rx)` is called
    once per transaction with all segments joined, which is one CS assertion. rx is zero-filled
    and as long as tx. With no callback, a transaction succeeds and reads zeros. A transaction of
    0 bytes calls nothing (the robot does not touch the bus either).
  - `sim::GetSpiDeviceConfig(bus, cs, &config)` returns the mode, clock and CS polarity that the
    open device last received.
- `test/prosdk/drivers/FakeSPIDevice.hpp` registers at `(bus, cs)` and records every frame in
  `frames`. It answers from a 256-byte register file: the first byte is the address, a set
  `readFlag` (default 0x80) marks a read, `readDummyBytes` bytes are skipped, and access
  auto-increments. A `handler` callback can be set instead for command protocols (flash, NAND,
  SX126x, FT8xx, TMAG5170 frames).

## Tests (`test/prosdk/drivers/SPITest.cpp`, `[spi]`)

- SetSPIPins validates the pins.
- SPI stores mode, clock and chip-select polarity.
- SPI routes transfers by chip select.
- SPI segmented transactions hold the chip select.
- SPI device without chip select is alone on its bus.
- SPI chip selects are owned once (also covers the SPI3 limit and moves).

## Verified and unverified

Verified against the ESP-IDF 5.5 headers shipped with Arduino-ESP32 3.3.10:
- 6 chip selects on SPI2 and 3 on SPI3 (`SOC_SPI_PERIPH_CS_NUM`).
- SPI2 IO_MUX pins.
- The 64-byte CPU FIFO.
- The 4092-byte DMA default.
- The clock divider range (`spi_ll_master_cal_clock`: prescaler 1-16, divider 2-64).
- The `SPI_TRANS_CS_KEEP_ACTIVE` and acquire-bus requirement (`spi_master.h`).

The code builds for the host and the ESP32-S3. The host tests ran on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): `[spi]` passes 67
assertions in 6 test cases. Nothing has run on hardware.

Unverified (needs hardware):
- Clock limits on GPIO-matrix pins. ESP-IDF's note on GPIO-matrix delay applies to the ESP32;
  `spi_master.h` says there is "no notable delay on chips other than ESP32". If a device is added
  at a clock that the full-duplex timing check refuses, the open fails with the ESP-IDF error in
  the message.
- DMA split boundaries and CS behaviour between pieces of a long transfer.
- CS state after a piece fails partway through a held transaction.
- Sharing a bus with the SD card (G7).
- Coexistence with Arduino's `SPIClass`.
- The 3.3 V input limit of the ESP32-S3 is taken from the ESP32-S3 datasheet (DC
  characteristics) and was not re-checked here.

## Resource use

- Per running bus: one SPI peripheral, one GDMA channel pair (`SPI_DMA_CH_AUTO`), the ESP-IDF
  driver state and its interrupt, and 2-3 pins plus one pin per CS device.
- Once, on the first segment without tx: a 4092-byte DMA-capable zero buffer.
- No thread, no timer, no RMT, LEDC or PCNT unit.
- Budget: 2 SPI buses. A no-CS device (clocked LEDs) takes a whole bus.

## Shared edits (integration)

- design/hal-esp32.md:
  - An "SPI (SPI.cpp)" module section (summary of the backend above).
  - SPI in the "Not gated" list.
  - Two capacities rows: "SPI buses | 2 | SPI2, SPI3" and "SPI chip selects | 6 + 3 | hardware
    CS; a no-CS device takes the whole bus".
- design/API-DIFFERENCES.md section 7: a `prosdk::SPI` row: WPILib 2026 `frc::SPI` shape, bus +
  CS GPIO constructor, `TransactionSegments`, no auto/accumulator, `Read` ignores `initiate`,
  default 1 MHz.
- design/drivers.md: a link to this file.
- tools/build/header_check.py `HAND_WRITTEN_GLOBS` (and the matching `_hand_headers` glob in
  tools/build/CMakeLists.txt): add `src/prosdk/simulation/SpiSim.hpp`. It is hand-written in
  the otherwise vendored `simulation/` folder (as F3's `PulseIOSim.hpp` is), so header-check does
  not see it yet. It compiles on its own on both builds. `wpilib.py` removes only files it
  generated before, so it leaves the file alone.
