# P1: Climate sensors

Package P1 of the Boardoza plan. It adds six I2C sensor classes in
`src/prosdk/drivers/environment/`:

| Board | Ozdisan id | Chip | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| LM75ADP | 1065559 | NXP LM75A | `prosdk::LM75A` | LM75ADP_Temperature_Sensor_Breakout_Board |
| HIH8131 | 1202709 | Honeywell HIH8131 (HumidIcon HIH8000 series) | `prosdk::HIH8000` | none (Ozdisan only) |
| TH09C | 1206510 | TH09C module, ENS210 register map | `prosdk::ENS210` | TH09C_Temperature_Humidity_Sensor_Breakout_Board |
| MS5611 | 1202433 | TE MS5611-01BA | `prosdk::MS5611` | MS5611_Barometric_Pressure_Sensor_Breakout_Board |
| HP303B | 1473941 | HopeRF HP303B (DPS310 register map) | `prosdk::DPS310` | HP303B_Barometric_Pressure_Sensor_Breakout_Board |
| BME688 | 1595256 | Bosch BME688 | `prosdk::BME688` | BME688_Environmental_Sensor_Breakout_Board |

Files:
- `src/prosdk/drivers/environment/LM75A.hpp` and `.cpp`, `HIH8000`, `ENS210`, `MS5611`, `DPS310`,
  `BME688` (each `.hpp` and `.cpp`)
- Tests (host only): `test/prosdk/drivers/LM75ATest.cpp`, `HIH8000Test.cpp`, `ENS210Test.cpp`,
  `MS5611Test.cpp`, `DPS310Test.cpp`, `BME688Test.cpp`
- Example: `examples/BoardozaClimate/BoardozaClimate.ino`

Sources (copies kept outside the repo):
- NXP LM75A product data sheet Rev. 04 (10 July 2007): sections 7.4 (Tables 5, 8, 9, 10) and 7.9.
- Honeywell technical note "I2C Communication with the Honeywell HumidIcon Digital
  Humidity/Temperature Sensors" 009061-2-EN (June 2012): sections 2.1 to 2.6, 3.0, 4.0, 5.0;
  HIH8000 series data sheet 009075-7 (Tables 2 and 6).
- TH09C data sheet V1.01: timing (Figure 9), I2C slave address, register map (Figures 22 to 33),
  readout registers, CRC-7, PART_ID access.
- TE MS5611-01BA03 data sheet (06/2017): ADC table, Figures 2 to 5 and 10 to 16; TE (MEAS)
  application note AN520 (CRC-4).
- HP303B data sheet V1.0 (2016-09-05): 4.4, 4.5, 4.9, 6.1, Tables 8, 9, 15, 16, 18, sections 8.3
  to 8.12.
- Bosch BME688 data sheet BST-BME688-DS000-03 revision 1.3: 3.5 to 3.7, Tables 13 to 17, 5.2,
  5.3, 6.2.
- The board READMEs and schematics of the five boards that have a repo.

No vendor code was used. The BME688 compensation is written from the data sheet formulas; Bosch's
BME68x SensorAPI was not copied (it was consulted only for the signedness of the calibration
parameters, see section 5), so NOTICE and `licenses/` do not change.

## 1. Wiring and hazards

**Power every board from 3.3 V when it is wired to the ESP32-S3.** The ESP32-S3 pins are not
5 V tolerant.

| Board | Supply | Host-side I2C lines | Address |
|---|---|---|---|
| LM75ADP | README says 5 V; the chip runs at 2.8 to 5.5 V | pull-ups not documented (no schematic in the repo): use 3.3 V | 0x48, pads A0-A2 select 0x48 to 0x4F |
| HIH8131 | chip 2.3 to 5.5 V | board not documented (**unverified**): use 3.3 V | 0x27 (chip default) |
| TH09C | LDO on the board | level shifters with pull-ups to VCC: 5 V on VCC puts 5 V on SDA/SCL | 0x43, fixed |
| MS5611 | TLV73333 LDO | 2N7002 shifters with 10k pull-ups to VCC: same hazard. CSB, SDO and PS on J2 are not shifted | 0x77 (CSB pulled down); CSB high gives 0x76 |
| HP303B | 3.3 V only (no regulator, no shifter) | SDA/SCL pull-ups behind JP1/JP2, open by default | 0x77; JP3 shorted (SDO low) gives 0x76 |
| BME688 | LDO on the board | shifters (10k / 4.7k) with pull-ups to VCC: same hazard | 0x77 with ADR unconnected (**unverified** board pull-up); ADR to GND gives 0x76 |

Notes:
- LM75ADP: pins VCC, SCL, SDA, OS, GND. OS is open-drain; the driver leaves it at its power-on
  setting (comparator, active low, 80 C / 75 C).
- HP303B: tie CSB to 3.3 V for I2C. With JP1/JP2 open the board has no pull-ups: on a bus with an
  MS5611, TH09C or BME688 board their pull-ups serve, alone short JP1 and JP2. The jumper
  functions were read from the schematic and are **unverified** on a board.
- BME688: leave nCSB unconnected. A low level on CSB switches the chip to SPI until the next
  power cycle (data sheet 6.1).
- The MS5611, HP303B and BME688 can only use 0x76 and 0x77, so at most two of them share a bus.
  The example puts the BME688 on I2C port 0.
- Every chip here supports 400 kHz. The example runs port 1 at 100 kHz, because six boards with
  their pull-ups and cables load the bus.

## 2. API

All six classes follow the hand-written driver pattern (`design/drivers.md`): the constructor
never throws, `IsConnected()` reports the last transfer, values are cached under a mutex, and
each class is a `TelemetryLoggable`. None starts a thread. Except for `LM75A`, the constructor does
not touch the bus. `Update()` is non-blocking: call it from a periodic method (for example
`RobotPeriodic()`). It advances a small state machine and never waits. After a failed transfer the
next start attempt is at least 1 s later. Before the first sample the getters return 0.

| Class | Constructor | Methods | Telemetry type, entries |
|---|---|---|---|
| `LM75A` | `(I2C::Port, int address = 0x48)` | `GetTemperature() -> celsius_t` (reads the chip), `SetShutdown(bool)`, `IsConnected()` | "Temperature Sensor": Temperature, Connected |
| `HIH8000` | `(I2C::Port, int address = 0x27)` | `Update()`, `GetHumidity()` (%RH), `GetTemperature()`, `IsStale()`, `IsConnected()` | "Humidity Sensor": Humidity, Temperature, Connected |
| `ENS210` | `(I2C::Port, int address = 0x43)` | `Update()`, `GetTemperature()`, `GetHumidity()`, `HasData()`, `GetCrcErrors()`, `IsConnected()` | "Humidity Sensor": Temperature, Humidity, CRC Errors, Connected |
| `MS5611` | `(I2C::Port, int address = 0x77, Osr osr = Osr::k4096)` | `Update()`, `GetPressure() -> pascal_t`, `GetTemperature()`, `GetAltitude(pascal_t seaLevel = 101325 Pa) -> meter_t`, `HasData()`, `IsConnected()`; static `Compensate()`, `PromCrc()` | "Barometer": Pressure, Temperature, Connected |
| `DPS310` | `(I2C::Port, int address = 0x77)` | `Update()`, `GetPressure()`, `GetTemperature()`, `GetAltitude(seaLevel)`, `HasData()`, `IsConnected()` | "Barometer": Pressure, Temperature, Connected |
| `BME688` | `(I2C::Port, int address = 0x77)` | `Update()`, `SetHeater(celsius_t, millisecond_t)`, `GetTemperature()`, `GetPressure()`, `GetHumidity()`, `GetGasResistance() -> ohm_t`, `IsGasValid()`, `HasData()`, `IsConnected()`; static compensation functions | "Environmental Sensor": Temperature, Pressure, Humidity, Gas Resistance, Gas Valid, Connected |

Humidity is a plain `double` in percent (prosdk units has no humidity unit). `GetAltitude()` uses
the international barometric formula h = 44330 m (1 - (p / p0)^(1 / 5.255)) on the last pressure,
and returns 0 m before the first sample.

## 3. Behaviour and defaults

**LM75A.** The constructor writes Conf = 0x00 (normal mode, power-on configuration), which also
ends a shutdown left by an earlier program. `GetTemperature()` reads the 2-byte Temp register:
the signed 16-bit word shifted right by 5, times 0.125 C. The chip converts every 100 ms. On a
failed read it returns the last good value. `SetShutdown(true)` writes Conf = 0x01.

**HIH8000.** `Update()` sends a Measurement Request (an empty write, `I2C::AddressOnly()`). A call
40 ms or more later fetches 4 bytes. Status 00 (valid): the sample is stored and the next request
goes out at once, so a 20 ms loop gives a sample about every 40 ms. Status 01 (stale): the fetch
repeats on the next call, and after 0.5 s without valid data a new request is sent. Status 10 or
11: a new request. Conversion: RH = counts * 100 / (2^14 - 2), T = counts * 165 / (2^14 - 2) - 40.
`IsStale()` is true before the first valid sample and when the last one is more than 1 s old. The
note gives the measurement cycle as 36.65 ms typical with no maximum; the 40 ms wait is based on
that, and an early fetch only costs a repeat (status 01).

**ENS210.** The first call writes SENS_RUN = 0x03 and SENS_START = 0x03 (T and RH continuous). The
next calls read SYS_STAT until SYS_ACTIVE is set (PART_ID is only readable while active), then
check PART_ID = 0x0210 and log a warning once if it differs (the driver continues). After that each
call reads T_VAL and H_VAL in one 6-byte burst (reading the first byte of each register latches
its 3 bytes, so a value cannot tear). Each value is
used only when its CRC-7 (polynomial 0x89, initial vector 0x7F, over the 16 data bits and the
valid bit) matches and its valid bit is set. CRC mismatches are counted in `GetCrcErrors()`.
T = data / 64 K, RH = data / 512 %. The chip makes a T and RH pair every 225 ms (238 ms max).
Low power (SYS_CTRL) is left at its default; in continuous mode the chip stays active.

**MS5611.** The first call sends RESET (0x1E). The next call at least 3 ms later (2.8 ms PROM
reload) reads the 8 PROM words and checks the CRC-4 (AN520) against the low nibble of word 7. On a
mismatch it logs a warning once and starts over (after 1 s). Then the calls alternate: start D2
(temperature), read it after the conversion time, start D1 (pressure), read it, compensate. Each
command is its own write, and each answer is a separate read, as in the data sheet's I2C figures.
Conversion times (max, plus 0.2 ms margin): 0.60, 1.17, 2.28, 4.54 and 9.04 ms for OSR 256 to 4096.
An ADC result of 0 (no finished conversion) restarts the same conversion. The compensation is the
data sheet's integer first-order formula plus the second-order terms below 20 C and below -15 C,
in 64-bit arithmetic with C++ truncating division.

**DPS310.** The first call reads the ID register 0x0D and requires PROD_ID (bits 3:0) = 0. The
HP303B reads 0x10, as the DPS310 does. Any other value logs a warning once and the driver retries
after 1 s, so another chip at the same address (for example an MS5611 or a BME688) is not
configured. Then it waits for COEF_RDY and SENSOR_RDY (40 ms and 12 ms max), reads the 18
coefficient bytes and COEF_SRCE, and configures:
- PRS_CFG = 0x33 (8 per second, 8 times oversampling);
- TMP_CFG = TMP_EXT from COEF_SRCE | 0x33. The temperature sensor matches the one the coefficients
  were made with (data sheet 8.12);
- CFG_REG = 0 (no result shift, which is only needed above 8 times);
- MEAS_CFG = 0x07 (continuous pressure and temperature).

Each measurement takes 14.8 ms (Table 16), so 8 * 2 * 14.8 ms = 237 ms of every second, under the
1 s limit (4.4). Each later call reads MEAS_CFG, and when TMP_RDY or PRS_RDY is set it reads the 6
result bytes. The compensation follows 4.9.1 and 4.9.2 with kT = kP = 7864320. Pressure is computed
only once a temperature exists.

**BME688.** The first call checks chip_id 0xD0 = 0x61 and variant_id 0xF0 = 0x01. A BME680
(variant 0x00, a different gas formula) or another chip logs a warning once and is retried after
1 s. It then reads the calibration (0x8A to 0xA0, 0xE1 to 0xEE, 0x00, 0x02) and writes these
registers one at a time (the I2C multi-byte write is register/data pairs, so single writes are
equivalent):
- ctrl_hum = 0x01 (humidity 1x);
- config = 0x00 (IIR filter off);
- ctrl_gas_0 = 0x00 (heater on);
- ctrl_gas_1 = 0x20 (run_gas, heater step 0);
- gas_wait_0;
- res_heat_0;
- ctrl_meas = 0x55 (temperature 2x, pressure 16x, forced mode).

These are the data sheet 3.5 quick-start settings, except for the heater. The default heater is
320 C for 150 ms, stored as gas_wait_0 = 0x66 (38 x 4 = 152 ms, the nearest step).

Each later call reads meas_status_0, and when new_data is set it reads field 0 (17 bytes from
0x1D), compensates and triggers the next measurement. Without new data for the heating time plus
1 s after a trigger it triggers again (a mode write during a running measurement only takes
effect at its end, data sheet 3.4, so an earlier re-trigger could never complete with a long
heater step).
- The heater code res_heat_0 uses the last measured temperature as the ambient (25 C before the
  first sample) and is rewritten only when it changes.
- `SetHeater()` limits the target to 200 to 400 C (the typical range in 3.6) and the time to 1 to
  4032 ms, and takes effect at the next trigger.
- Humidity is limited to 0 to 100 %.
- `IsGasValid()` requires both gas_valid_r and heat_stab_r (3.7).

The gas resistance is the raw value in ohms. Air-quality indices need Bosch BSEC (closed source)
and are not part of this driver.

## 4. Verification

Host tests (`prosdk_drivers_tests`, FakeI2CBus from F2):
- **LM75A:** rows of Table 10 as vectors (+127, +25, +0.125, -0.125, -25, -54.875 C),
  the ignored low bits, the Conf write in the constructor, shutdown, a NACK (last value kept,
  `IsConnected()` false) and a missing chip.
- **HIH8000:**
  - the empty-write request, the 40 ms wait, conversion at 50 %RH, 0 %RH and 125 C;
  - stale and command-mode status, `IsStale()`;
  - a NACK and recovery.
- **ENS210:**
  - the data sheet readout example FD 49 0B / 6C 2E F5 (CRC 0x05 and 0x7A, 295.953125 K, which
    the data sheet prints rounded as 296.0 K / 22.8 C, and 23.2109375 %RH);
  - a CRC error, waiting for SYS_ACTIVE;
  - the 1 s retry after a missing chip and a NACK.
- **MS5611:**
  - the data sheet example (C1 to C6 = 40127, 36924, 23317, 23282, 33464, 28312; D1 = 9085466,
    D2 = 8569150 give TEMP 2007 and P 100009);
  - two hand-computed second-order vectors (D2 = 8000000 gives -61 and 95989; D2 = 7500000 gives
    -2129 and 91910, the below -15 C branch);
  - the CRC-4: three vectors computed with the AN520 `crc4()` algorithm (AN520 prints no example
    vector), one of them showing that the CRC nibble itself is ignored;
  - the full command sequence on an emulated chip, OSR 256 timing, a lost conversion;
  - a bad PROM CRC, a missing chip and a NACK.
- **DPS310:** the HP303B data sheet has no worked example, so a hand-computed vector is used
  (coefficients c0 209, c1 -261, c00 80469, c10 -54769, c01 -2165, c11 1484, c20 -10715, c21 231,
  c30 -1554; Praw -2900000, Traw 2395000 give 25.0150604 C and 98469.776 Pa). The tests also cover
  the configuration registers, TMP_EXT from COEF_SRCE, waiting for COEF_RDY, pressure before
  temperature, a wrong product ID and a missing chip.
- **BME688:** the data sheet has no worked example, so the tests use a made-up calibration set
  with hand-computed values from the float formulas: 27.3330641 C, 92713.852 Pa, 73.4479114 %RH,
  1878899.08 ohm, and heater codes 111 (320 C at 25 C) and 106 (300 C at 27.33 C). The data sheet
  integer formulas, evaluated separately, give 27.33 C, 92715 Pa and 73.433 %RH, which agrees.
  The tests also check the configuration writes, the gas_wait encoding (0x66 for 150 ms, 0x59 for
  100 ms as in data sheet 3.5, 0xFF at the limit), the re-trigger timeout (heating time + 1 s), gas valid and humidity
  limiting, the BME680 variant, the retry and a NACK.

The test expectations were computed with a Python model of the formulas (C-style truncating
division for the MS5611). Run on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): the six test files pass 191
assertions in 19 test cases (LM75A 21/3, HIH8000 28/2, ENS210 26/3, MS5611 48/5, DPS310 25/3,
BME688 43/3). The `BoardozaClimate` example builds with arduino-cli (design/examples.md, Sizes).

## 5. Unverified

- **No board has been tested on hardware.** Every driver is checked only against data sheet
  vectors on the simulation HAL.
- **HIH8000:**
  - **zero-length write on the ESP32** (`AddressOnly()`, plan G9). If Arduino-ESP32 does not
    send it, the HIH8000 never gets a measurement request;
  - **HIH8131 board** (no repo, no schematic): its supply, pull-ups and connector are unknown.
    0x27 is the chip's factory default; Honeywell can ship other addresses;
  - **measurement time:** the 40 ms wait is based on the 36.65 ms typical value; no maximum is
    given.
- **LM75ADP board** pull-ups and the safety of 3.3 V on its VCC (README says 5 V; the chip allows
  2.8 V).
- **HP303B jumpers** JP1/JP2 (pull-ups) and JP3 (address), read from the schematic.
- **DPS310 temperature fix:** Infineon's own library writes an undocumented sequence to 0x0E,
  0x0F and 0x62 to fix the temperature reading of some DPS310 lots. The HP303B data sheet does not
  mention it, and this driver does not apply it. Whether the HP303B needs it is unknown.
- **BME688:**
  - **board ADR default:** 0x77 with ADR unconnected depends on a pull-up that could not be
    confirmed from the schematic;
  - **calibration signedness** (t1, p1, p10, h1, h2, h6 unsigned; the rest signed): the data
    sheet does not state it. It was taken from the parameter types in Bosch's public BME68x API;
  - **pressure formula:** the data sheet's float formula names var1_p, var2_p and var3_p, read
    here as var1, var2 and var3. The integer formula gives the same result within 2 Pa on the test
    vector;
  - **sample period:** about 0.2 s with the default heater (152 ms of heating after the T, P and H
    conversions). This is an estimate.
- **TH09C = ENS210:** the TH09C data sheet describes the ENS210 register map (PART_ID 0x0210,
  CRC-7, the same example readout) but does not print the manufacturer's name.

## 6. Resource use

- **No threads, no heap allocation, no timers.** Each object holds a `prosdk::I2C`, a
  `std::mutex` and a few cached values. The largest are `BME688` (the calibration struct) and
  `MS5611` (8 PROM words).
- **Bus time per `Update()` call**, as bytes on the bus including address bytes, about 90 us per
  byte at 100 kHz and 23 us at 400 kHz:

| Class | Steady state per call |
|---|---|
| `LM75A::GetTemperature()` | 5 bytes |
| `HIH8000` | 5 bytes (fetch) + 1 byte (request) every second call |
| `ENS210` | 9 bytes |
| `MS5611` | 8 bytes when a conversion is due (every call at a 20 ms loop and OSR 4096), else none |
| `DPS310` | 4 bytes (status), plus 9 bytes when a result is ready (16 per second) |
| `BME688` | 4 bytes (status), plus 20 bytes and the 3-byte trigger when a sample is ready (about 5 per second) |

  With all six on one 100 kHz bus that is about 3 to 4 ms of bus time per 20 ms loop. Transfers
  block the calling thread for their duration (Arduino I2C HAL, 50 ms timeout).
- **Start-up:** each driver needs 2 to 4 `Update()` calls before its first value, plus the chip's
  own first conversion: at most 238 ms (ENS210) or about 0.2 s (BME688).
