# P2: Light and thermal sensors

Package P2 of the Boardoza plan. It adds five classes for six boards:

| Board | Ozdisan id | Chip | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| OPT3001 ambient light | 1590925 | TI OPT3001 | `prosdk::OPT3001` | OPT3001_Ambient_Light_Sensor_Breakout_Board |
| OPT4048 color | 1598828 | TI OPT4048 | `prosdk::OPT4048` | OPT4048_High_Precision_Color_Sensor_Breakout_Board |
| TPiS 1S 1385 presence/motion | 1473945 | Excelitas TPiS 1S 1385 (CaliPile) | `prosdk::CaliPile` | Excelitas_TPiS_1S_1385_IR_Presence_Motion_Detection_Breakout |
| TPiS 1T 1386 non-contact temperature | 1202719 | Excelitas TPiS 1T 1386 L5.5H (CaliPile) | `prosdk::CaliPile` | TPiS_1T_1386_Non_Contact_Temperature_Sensor_Breakout_Board |
| MLX90640 thermal camera | 1590924 | Melexis MLX90640 (32x24) | `prosdk::MLX90640` | MLX90640_Thermal_Camera_Breakout_Board |
| MLX90641 thermal camera | none (repo only) | Melexis MLX90641 (16x12) | `prosdk::MLX90641` | MLX90641_Thermal_Camera_Breakout_Board |

Files:
- `src/prosdk/drivers/light/OPT3001.hpp` and `.cpp`, `OPT4048.hpp` and `.cpp`
- `src/prosdk/drivers/thermal/CaliPile.hpp` and `.cpp`, `MLX90640.hpp` and `.cpp`,
  `MLX90641.hpp` and `.cpp`
- `src/prosdk/drivers/thermal/melexis/`: the vendored Melexis API (`MLX90640_API.h/.cpp`,
  `MLX90641_API.h/.cpp`) and the hand-written I2C glue `MelexisI2C.hpp` and `.cpp`
- `licenses/melexis/LICENSE` (Apache License 2.0, from the upstream repos)
- Tests (host only): `test/prosdk/drivers/OPT3001Test.cpp`, `OPT4048Test.cpp`,
  `CaliPileTest.cpp`, `MLX90640Test.cpp`, `MLX90641Test.cpp`
- Example: `examples/BoardozaThermalCamera/BoardozaThermalCamera.ino`

Sources (the section numbers are cited in the driver files):
- TI OPT3001 data sheet SBOS681C (revised November 2017).
- TI OPT4048 data sheet SBOSA84 (December 2022).
- Excelitas TPiS 1S 1385 product specification (revised 04/01/2021) and TPiS 1T 1386 L5.5H
  product specification (revised 20/06/2024).
- Melexis MLX90640 data sheet rev 12 (3 December 2019) and MLX90641 data sheet rev 4
  (14 September 2023); the Melexis driver documents for the `TA_SHIFT` values (8 and 5).
- The six board READMEs and KiCad schematics.

## 1. Vendored code

The Melexis API comes from github.com/melexis/mlx90640-library (commit f6be7ca1) and
github.com/melexis/mlx90641-library (commit 3c94aa87), Apache License 2.0. Changes, each
recorded in a "Modified by Probot Studio" notice under the upstream license header (section 4(b)):

- `.c` files renamed to `.cpp`; all code inside `namespace prosdk::melexis`;
- include paths changed to `prosdk/drivers/thermal/melexis/...`; the headers include
  `<stdint.h>`;
- CRLF line endings converted to LF;
- the upstream I2C layer (`MLX9064x_I2C_Driver`) replaced by `MelexisI2C.hpp/.cpp`.

The math is not changed. Two upstream `-Wtype-limits` warnings in `MLX90641_API.cpp` (an int16
compared against a value it cannot hold) are left as they are.

`MelexisI2C` binds a `prosdk::I2C` for the duration of one API call (`ScopedI2C`, holding a
process-wide mutex, because the API functions take only an address). Reads are split into
transfers of at most 512 bytes (256 words), each a write of the 2-byte register address (MSB
first) followed by a read; words are big-endian. A write is one 4-byte transfer (address, data).
`MLX9064x_I2CGeneralReset` returns an error: the drivers do not use it.

## 2. Wiring and hazards

**Power every board from 3.3 V when it is wired to the ESP32-S3.** The ESP32-S3 pins are not
5 V tolerant. Every board has a TLV73333 3.3 V regulator and accepts 3.3 V to 5 V (5.5 V for
OPT3001 and the MLX boards) on VCC, but on several boards the host-side I2C lines are pulled up
to VCC:

- TPiS 1S and 1T: SDA/SCL go through 2N7002 level shifters with 10k pull-ups to VCC on the header
  side, so at VCC = 5 V the bus idles at 5 V. INT is not shifted (2k pull-up to the on-board
  3.3 V).
- MLX90640 and MLX90641: SDA/SCL go through a TXS0102 between VCC and the sensor's 3.3 V, so at
  VCC = 5 V the host side is 5 V logic.
- OPT3001 and OPT4048: the pull-up rail (3.3 V or VCC) was not traced in the schematics
  (**unverified**). At VCC = 3.3 V it does not matter.

With 3.3 V on VCC the regulator is in dropout and the chip gets slightly less than 3.3 V, which is
inside every chip's range (the MLX data sheets ask for 3.3 V +/- 0.05 V for best accuracy; at
dropout this may not hold, **unverified**). Tie the board GND to the controller GND.

| Board | Connector | Pins | I2C address |
|---|---|---|---|
| OPT3001 | J1 | 1 VCC, 2 SCL, 3 SDA, 4 GND | 0x44 (ADDR pulled to GND on the board); 0x45 ADDR=VDD, 0x46 ADDR=SDA, 0x47 ADDR=SCL (Table 1) |
| | J2 | 1 ADDR, 2 INT | |
| OPT4048 | J1 | 1 VIN, 2 INT, 3 SDA, 4 SCL, 5 GND | 0x44 to 0x47 as the OPT3001. The data sheet table prints 1000101b (0x45) for ADDR=SCL, a typo for 0x47 (the Boardoza library uses 0x47) |
| TPiS 1S 1385 | J1 | 1 VCC, 2 SCL, 3 SDA, 4 INT, 5 GND | 0x0C, fixed (A0 and A1 tied to GND on the PCB) |
| TPiS 1T 1386 | J1 | 1 VCC, 2 SCL, 3 SDA, 4 GND | 0x0C or 0x0E: A1 goes to two 0R footprints (to GND and to 3.3 V); which one is fitted is not known from the KiCad files, most likely 0x0C (**unverified**) |
| | J2 | 1 INT | |
| MLX90640, MLX90641 | J1 | 1 VCC, 2 SCL, 3 SDA, 4 GND | 0x33 (stored in EEPROM; no address pads) |

The INT pins are not used by the drivers.

**I2C speed.** OPT3001 and OPT4048 work at 100 or 400 kHz. The CaliPile allows at most 400 kHz
and may stretch the clock. The MLX data sheets limit EEPROM reads to 400 kHz (MLX90640 rev 12
and MLX90641 rev 4, section 8, note 5: "Max EEPROM I2C speed operations to be done at 400kHz").
The plan suggested 1 MHz for the cameras; the drivers and the example use 400 kHz instead,
because the EEPROM is read at start-up and after every reconnect on the same bus speed.

## 3. API

All classes are `TelemetryLoggable`, take a `prosdk::I2C::Port` and an address, and never
throw. `IsConnected()` is the result of the last transfer. A chip that is missing or answers
with the wrong ID prints one console warning and is retried (OPT on every `Update()`, CaliPile
and MLX once per second). CaliPile and the MLX cameras also set the chip up again after a failed
transfer, because after a power loss the CaliPile answers only the general call and an MLX camera
is back at its EEPROM refresh rate; the OPT drivers rewrite the configuration when they read it
back in its power-down reset state.

### `OPT3001` (no thread)

```cpp
explicit OPT3001(I2C::Port port, int address = 0x44);
void Update();                      // two 2-byte reads
units::lux_t GetLux() const;        // last good value; 0 before the first
bool IsConnected() const;
static double ResultToLux(uint16_t result);  // lux = 0.01 * 2^E * R (equation 3)
```

The constructor checks the manufacturer ID (0x7E = 0x5449) and device ID (0x7F = 0x3001) and
writes the configuration 0xCC10: automatic full-scale range, 800 ms conversions, continuous mode.
`Update()` writes the configuration again if the chip is back in shutdown (power loss), then
reads the result. A new value comes every 800 ms.

### `OPT4048` (no thread)

```cpp
explicit OPT4048(I2C::Port port, int address = 0x44);
void Update();                      // one 16-byte read and one 2-byte read
XYZ GetXYZ() const;                 // CIE 1931 X, Y, Z
units::lux_t GetLux() const;        // channel 1 (Y) * 2.15e-3 (equation 9)
Chromaticity GetCIExy() const;      // x, y; 0, 0 in darkness
uint32_t GetCrcErrors() const;
bool IsConnected() const;
static Channel DecodeChannel(uint16_t msb, uint16_t lsb);
static uint8_t ChannelCrc(const Channel& channel);
```

The configuration is 0x3238: automatic range, 100 ms per channel (400 ms for the four channels),
continuous mode. The device ID is accepted when `id & 0x0FFF` is 0x820 or 0x821 (see section 5).
Every channel's 4-bit CRC is checked; a sample with a wrong CRC is dropped and counted. The XYZ
matrix is the one of the data sheet's application example (section 9.2.4, equation 16).

### `CaliPile` (no thread)

```cpp
enum class Variant { k1S1385, k1T1386 };
CaliPile(I2C::Port port, Variant variant, int address = 0x0C);
void Update();                      // one 19-byte read
bool Reload();                      // general call 0x00, 0x04
units::celsius_t GetObjectTemperature() const;
units::celsius_t GetAmbientTemperature() const;
bool IsPresence() const;            // chip status bit 3
bool IsMotion() const;              // chip status bit 2
bool IsOverTemperature() const;     // chip status bit 4, only once a threshold is set
void SetEmissivity(double emissivity);                 // default 1.0
void SetOverTemperatureThreshold(units::celsius_t t);  // object temperature
bool IsConnected() const;
```

After power-up the chip answers only the general call (1S 5.3 / 1T 6.3), so the first `Update()`
sends the reload (0x00, then 0x04; 5.5 / 6.5), waits 300 us, reads the 32 EEPROM cells
(ECR 0x80, then back to 0x00), checks the checksum (16-bit sum of cells 0 and 3-31 against
cell 1 as high byte and cell 2 as low byte, the sample code of 1T 9.3), and writes all control registers 20 to 29
with the recommended start settings (presence, motion and shock thresholds; TPOT threshold
0xFFFF, that is never). The calibration gives:

- ambient: `Tamb = 298.15 + (TPamb - PTAT25) / M` (K);
- object: `Tobj = ((TPobj - U0) / (k * emissivity) + Tamb^n)^(1/n)` with n = 3.8 for LOOKUP# 1
  and 4.2 for LOOKUP# 2, and `k = (UOUT1 - U0) / (TOBJ1^n - 298.15^n)`.

`SetOverTemperatureThreshold()` converts the temperature to a TPobject count with the inverse of
the formula above, at the current ambient temperature, and writes it to registers 28-29. It is
rewritten when the ambient temperature moves it by 8 register counts or more.

The reload goes to address 0x00, so every chip on the bus that answers general calls sees the
byte 0x04. In the I2C specification (NXP UM10204) 0x04 is "write programmable part of slave
address by hardware", which other chips that implement it may act on; it is not the general call
reset (0x06).

### `MLX90640` and `MLX90641` (reader thread)

```cpp
enum class RefreshRate : uint8_t { k0_5Hz, k1Hz, k2Hz, k4Hz, k8Hz, k16Hz, k32Hz, k64Hz };
explicit MLX90640(I2C::Port port, int address = 0x33, RefreshRate rate = RefreshRate::k8Hz,
                  units::millisecond_t updatePeriod = 10_ms);
void Update();
bool GetFrame(std::span<float, kPixelCount> out) const;  // false before the first frame
units::celsius_t GetAmbientTemperature() const;
uint32_t GetFrameCount() const;
void SetEmissivity(double emissivity);   // default 0.95, clamped to 0.01..1
bool IsConnected() const;
```

`MLX90641` has the same API with `kWidth` 16, `kHeight` 12, `kPixelCount` 192. Frames are row by
row, degrees Celsius. The refresh rate codes 0 to 7 are control register bits 9:7
(MLX90640 section 10.7.1, Figure 12; MLX90641 section 10.6.1, Figure 11).

A reader thread calls `Update()` every `updatePeriod` (10 ms). With an `updatePeriod` of 0 no
thread starts and the user calls `Update()`. `Update()` holds its own mutex, so a user call next
to the thread is safe (it only waits). The first call reads the 832-word EEPROM, sets the
refresh rate and extracts the calibration parameters. Each later call reads the status register
(0x8000) and, when the new-data bit (bit 3) is set, reads the subpage, clears the bit and
converts it with the reflected temperature `Tr = Ta - TA_SHIFT` (8 for the MLX90640, 5 for the
MLX90641).

- **MLX90640:** each subpage holds half the pixels (chess pattern, the factory default of control
  register bit 12; interleaved mode works as well, the API reads the bit from every frame). A frame is
  published after both subpages, with the broken and outlier pixels from the EEPROM interpolated
  (`MLX90640_BadPixelsCorrection`). At k8Hz that is 4 frames per second.
- **MLX90641:** each subpage holds all 192 pixels (data sheet rev 4, section 10.4), so every
  subpage is published. At k8Hz that is 8 frames per second.

The MLX90641 EEPROM is Hamming coded: a corrected single-bit error (-9) is accepted, an
uncorrectable word (-10) fails the init.

## 4. Verification

On the host simulation HAL with `FakeI2CBus`. Run on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): the five test
files pass 140 assertions in 19 test cases (OPT3001 22/4, OPT4048 28/4, CaliPile 42/4,
MLX90640 28/4, MLX90641 20/3):

- **OPT3001:** the result decoding against all rows of Table 9; ID check and configuration;
  reconfiguration after a simulated power loss; NACK keeps the last value; a wrong device ID is
  rejected and retried; a missing chip.
- **OPT4048:** channel decoding and CRC (hand-computed vector; any single flipped bit changes the
  CRC); XYZ, lux and CIE xy against values computed independently; a CRC error drops the sample;
  power-loss reconfiguration; IDs 0x0820 and 0x0821 accepted, 0x3001 rejected; darkness.
- **CaliPile:** the general-call reload (the fake chip NACKs until it gets 0x00/0x04), the exact
  write sequence (ECR, EEPROM pointer, ECR idle, registers 20-29), ambient 30 C and object 40 C
  from typical Table 37 values, presence/motion/over-temperature flags, the threshold register
  write, emissivity 0.9, a bad checksum retried after 1 s, a missing chip, a new general-call
  setup after a simulated power loss (not before 1 s).
- **MLX90640:** the data sheet calculation example (section 11.2) through the vendored API:
  Ta = 39.184 C, pixel 367 To = 80.18 C; refresh-rate write 0x0901 to 0x0A01; status clear;
  a frame after two subpages; every read at most 512 bytes; a missing camera; a new setup
  (EEPROM and refresh rate) after a lost transfer, not before 1 s; thread start and join.
- **MLX90641:** the data sheet calculation example (section 11.2): Ta = 42.098 C, pixel 88
  To = 80.24 C; a frame per subpage; a corrected and an uncorrectable EEPROM word; reads at most
  512 bytes; a missing camera.

Test vector notes:
- Expected values were recomputed in Python from the data sheet formulas, not copied. The
  MLX90640 example prints To = 80.36 C; the library's formula on the same inputs gives 80.18 C.
  The MLX90641 example prints Ta = 42.02 C and To = 80.12 C, but its own intermediate values give
  42.098 C and 80.24 C.
- MLX90640: EEPROM word 0x240A is set to 0x0800 (calibrated in interleaved mode, the mode of the
  example), so the library applies no interleaved/chess correction, as the example assumes.
- MLX90641: the data sheet prints 0xF847 for the subpage 1 offset words; that word has a
  single-bit Hamming error (syndrome 20). The main test uses 0xD847 (same 11 data bits, valid
  check bits); a second test uses 0xF847 to show the corrected word is accepted.

The sources build for the host and the ESP32-S3. The `BoardozaThermalCamera` example builds with
arduino-cli: 1,905,060 B flash, 72,364 B static RAM (design/examples.md, Sizes).

## 5. Unverified

- **No board has been tested on hardware.**
- **CaliPile general call on the ESP32** (plan G9): whether ESP-IDF 5.5 accepts device address 0
  and sends the general call is unverified (design/boardoza/F2-sim-i2c.md). Without it neither
  TPiS board ever answers: this blocks both boards.
- **CaliPile TPOT flag after start-up:** the control registers are undefined until written, so
  the over-temperature flag may be set by a stale threshold before the first `Update()` writes
  0xFFFF. `IsOverTemperature()` therefore reports nothing until a threshold is set and written.
- **CaliPile PROTOCOL and variant:** a PROTOCOL other than 3, or an EEPROM that does not match the
  chosen variant, only prints a warning.
- **TPiS 1T address strap:** 0x0C or 0x0E (see section 2).
- **OPT4048 CRC:** data sheet Figure 8-12 lists `CRC_CHx` among the CRC inputs; a CRC cannot cover
  itself, so the driver uses the 4-bit counter there. Checked only with a hand-computed vector.
  A wrong guess shows as every sample counted in `GetCrcErrors()` and no values.
- **OPT4048 device ID:** the register description gives reset 0x0820, the field table DIDH
  0x821; both are accepted and the value read is logged.
- **OPT4048 XYZ matrix:** the application example's matrix, not a per-device calibration.
- **OPT3001 and OPT4048 board pull-up rail** (3.3 V or VCC).
- **MLX at 400 kHz** instead of the plan's 1 MHz (section 2). The pixel RAM may allow 1 MHz;
  this was not tried.
- **MLX90641 frame per subpage:** taken from data sheet rev 4, section 10.4 and the Melexis API,
  not seen on a chip.
- **MLX supply in regulator dropout** at VCC = 3.3 V (section 2).

## 6. Resource use

- **OPT3001, OPT4048, CaliPile:** no thread, no heap, no timers. Each object holds a
  `prosdk::I2C` (CaliPile two: the chip and the general call), a mutex and the cached values.
  Bus time per `Update()`: OPT3001 2 x 2-byte reads, OPT4048 16 + 2 bytes, CaliPile 19 bytes,
  plus the threshold write when it is due.
- **MLX90640:** one heap block of about 14 KiB (calibration parameters, 832-word EEPROM copy,
  834-word frame, two 768-float buffers) and a reader thread with a 12 KiB stack (the API keeps
  arrays of about 3 KiB on the stack). Bus time at 400 kHz: about 40 ms per subpage (1.7 KB),
  about 37 ms once for the EEPROM.
- **MLX90641:** about 5 KiB of heap and a 6 KiB thread stack. About 13 ms of bus time per
  subpage at 400 kHz.
- **Bus sharing:** at k8Hz an MLX90640 uses about 30 percent of a 400 kHz bus; k16Hz is the
  fastest rate the bus keeps up with. `ScopedI2C` holds one process-wide mutex for each Melexis
  API call, so two cameras (on any ports) never transfer at the same time.
- Transfers block the calling thread for their duration (Arduino I2C HAL, 50 ms timeout).
