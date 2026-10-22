# P3: IMUs (I2C)

Package P3 of the Boardoza plan. It adds three IMU drivers in `src/prosdk/drivers/imu/`, written
like `prosdk::MPU6050`:

- `prosdk::BNO055` (`BNO055.hpp`, `.cpp`);
- `prosdk::BMI088` (`BMI088.hpp`, `.cpp`);
- `prosdk::LSM303AGR` (`LSM303AGR.hpp`, `.cpp`).

| Board | Ozdisan id | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|
| BNO055 absolute orientation sensor | 1206508 | `BNO055` | BNO055_Absolute_Orientation_Sensor_Breakout_Board |
| BMI088 6-axis IMU | 1598825 | `BMI088` | BMI088_6-Axis_IMU_Breakout_Board |
| LSM303AGR accelerometer + magnetometer | 1591109 | `LSM303AGR` | LSM303_Triple_Axis_Accelerometer_Magnetometer_Breakout_Board |

Sources (the copies in each repo's `assets/`):
- BNO055: Bosch BST-BNO055-DS000-18 rev 1.8.
- BMI088: Bosch BST-BMI088-DS000-19 rev 1.9.
- LSM303AGR: ST DocID027765 Rev 11.
- Board wiring: each repo's README and KiCad schematic, as read in the plan's research records.

Every register value in the code cites its datasheet section. Anything not checked against a
datasheet is marked **unverified** here and in the code. The vendor Arduino libraries were not
ported. The BNO055 library is CC BY-SA, and all three have bugs (see the research records).

## Common to the three classes

- Only prosdk `I2C` is used, with no Arduino or probot-core API, so the drivers run unchanged on
  the host sim HAL.
- Each one has the `OnboardIMU`-style getters that make sense for the chip:
  - `GetYaw`, `ResetYaw`, `GetRotation2d`, `GetRotation3d`, `GetQuaternion`;
  - `GetAngleX/Y/Z`, `GetAccelX/Y/Z`;
  - `GetGyroRateX/Y/Z` (not on LSM303AGR, which has no gyro);
  - `IsConnected`, `Update`.
- Axes are the WPILib robot axes: chip flat, Z up, X forward, Y left, yaw counter-clockwise
  positive. Other mountings need the caller to remap axes.
- Yaw is continuous: it does not wrap at +/-180 degrees, as on MPU6050.
- Roll is `atan2(gy, gz)` and pitch is `atan2(-gx, sqrt(gy^2 + gz^2))` of a gravity vector, the
  same formulas as MPU6050.
- Threading follows MPU6050. A reader thread starts through `ScopedThreadConfig` and calls
  `Update()` every `updatePeriod`. With `updatePeriod = 0` there is no thread, and the user calls
  `Update()`.
- The constructors do not throw. A missing or wrong chip prints one console line, and
  `IsConnected()` reports the identity check and the last read.
- Telemetry type is "Gyro", with the entries "Value" (yaw, degrees), "Roll" and "Pitch".
  BNO055 and BMI088 add "Rate"; LSM303AGR adds "Heading".
- The I2C devices are not gated by the HAL (I2C never is). These are sensors, so nothing needs
  gating.

### Wiring rules (all three boards)

- ESP32-S3 pins are 3.3 V only and **not 5 V tolerant**.
- All three boards accept VCC from 3.3 to 5.5 V, and each has a level shifter whose host side
  sits at VCC. **Power VCC from 3.3 V.** At 5 V the boards put 5 V on SDA and SCL (BNO055,
  through its 10k pull-ups to VCC) or drive their host-side lines at 5 V (BMI088 TXS0108E,
  LSM303AGR TXS0104E).
- The interrupt pins on the J2/J3 headers are 3.3 V logic and are not used by the drivers.
- The I2C bus is `prosdk::platform::SetI2CPins(port, sda, scl[, hz])`. Port 0 defaults to
  SDA 8 / SCL 9 at 400 kHz. BNO055 (4.6) and BMI088 (6.2) support standard and fast mode only.
  The LSM303AGR datasheet also lists faster modes, but its board's TXS0104E shifter is not rated
  for them here, so 400 kHz is the maximum for all three.

## 1. `prosdk::BNO055`

```cpp
enum class Mode : uint8_t { kIMU = 0x08, kCompass = 0x09, kM4G = 0x0A, kNDOFFmcOff = 0x0B, kNDOF = 0x0C };
struct CalibrationStatus { int system, gyro, accel, mag; };   // 0..3 each
using Offsets = std::array<uint8_t, 22>;                       // registers 0x55-0x6A
BNO055(I2C::Port port, int address = 0x28, Mode mode = Mode::kNDOF,
       units::millisecond_t updatePeriod = 10_ms, bool externalCrystal = true);
units::celsius_t GetTemperature();
CalibrationStatus GetCalibrationStatus();
std::optional<Offsets> GetOffsets();
bool SetOffsets(const Offsets&);
```

Board:
- J1 has VCC, SCL, SDA and GND. The SCL/SDA lines go through 2N7002 shifters, with 10k pull-ups
  to VCC on the host side, so the host adds no pull-ups.
- J2 ADR is pulled low on the board, which gives address 0x28. Tie ADR to 3.3 V for 0x29.
- PS0/PS1 are pulled low, which selects I2C. The UART mode is not supported.
- nRESET is not broken out.
- The board has a 32.768 kHz crystal.

What the driver does:
- **Configuration** (datasheet 3.3, 4.3, 5.5):
  1. Check that CHIP_ID (0x00) reads 0xA0.
  2. PAGE_ID = 0.
  3. OPR_MODE = CONFIG, then wait 20 ms (Table 3-6: 19 ms).
  4. PWR_MODE = normal.
  5. UNIT_SEL = 0x00: m/s^2, deg/s, degrees, deg C, Windows format. Its reset value is 0x80.
  6. If `externalCrystal` is set and CLK_SEL is still clear: SYS_TRIGGER = 0x80, then poll
     SYS_CLK_STATUS.ST_MAIN_CLK every 10 ms, waiting before the first read, until it clears (at
     most 1 s; the datasheet gives about 600 ms).
     After a restart without a power cycle, CLK_SEL is already set and this step is skipped. The
     datasheet allows CLK_SEL only with a crystal fitted (4.3.63), and says the chip falls back to
     its internal oscillator and clears CLK_SEL when the crystal fails (5.5). Pass `false` for
     breakouts without a crystal.
  7. OPR_MODE = the fusion mode, then wait 8 ms (Table 3-6: 7 ms).
- **No soft reset**. The chip is reconfigured from CONFIG mode instead, which avoids the 650 ms
  POR time.
- **Retry.** The chip needs up to 650 ms after power-on (Table 0-2). If the constructor finds no
  chip, or a wrong CHIP_ID, `Update()` retries the configuration every 100 ms. A failed read
  also triggers a reconfiguration: a chip that lost power comes back in CONFIG mode. The
  constructor blocks about 30 ms. The first configuration after power-on blocks up to 1 s
  longer while the crystal starts, which can also happen inside a retrying `Update()`.
- **Read.** One burst of 50 bytes, 0x08-0x39: accel, mag, gyro, Euler, quaternion, linear accel,
  gravity, TEMP, CALIB_STAT, ST_RESULT, INT_STA, SYS_CLK_STATUS, SYS_STATUS. A burst keeps the
  LSB/MSB pairs consistent (3.7).
- **Scaling** (3.6.5): accel and gravity 100 LSB per m/s^2; gyro 16 LSB per deg/s; Euler
  16 LSB per degree; TEMP int8 at 1 deg C per LSB.
- **Yaw** is the Euler heading (0x1A), negated, because the heading increases clockwise
  (Table 3-13). It is unwrapped into a continuous angle.
  - The first heading sets the yaw: the absolute heading from magnetic north in the absolute
    modes (kNDOF, kNDOFFmcOff, kCompass), or near 0 in kIMU and kM4G, where the heading is
    relative (Table 3-3).
  - After a reconfiguration (a retry, `GetOffsets`, `SetOffsets`) the next heading becomes the
    new reference, so the yaw does not jump. In the relative modes the chip's heading restarts
    after a mode change (assumed; the datasheet does not say it). Rotation during the CONFIG window (about 30 ms) is lost.
- **Roll and pitch** come from the fused gravity vector (0x2E-0x33), not from the Euler
  roll/pitch. Their axis convention is not tied to the robot axes in the datasheet, and their
  signs differ between the Windows and Android formats.
- The orientation is updated only while SYS_STATUS = 5 (fusion running). Otherwise the last
  orientation is kept, while accel, gyro and temperature still update.
- `GetGyroRate*` returns the chip's offset-compensated gyro data. It is 0 in kCompass and kM4G,
  which have no gyro.
- **Calibration** (3.11): the chip calibrates itself while it runs, and there is no `Calibrate()`.
  - `GetCalibrationStatus()` reports CALIB_STAT. Bits 7:6 are the system level, 5:4 gyro,
    3:2 accel and 1:0 mag; 3 means fully calibrated.
  - `GetOffsets()` reads the 22-byte profile in CONFIG mode and returns to the fusion mode. The
    datasheet allows it only after a full calibration; the driver does not check this.
  - `SetOffsets()` writes the profile in CONFIG mode, one byte per transfer in address order, so
    each block's last (MSB) byte is written last, as 3.6.4 requires.
  - The profile can be stored with prosdk Preferences by the user. The chip overwrites it when it
    recalibrates.
- Modes: `kNDOF` is the default (the plan's choice). Near motors and steel the magnetometer is
  disturbed, and the NDOF heading can jump while the magnetometer calibrates. For a drivetrain,
  `kIMU` (gyro + accel, relative heading) is usually the better choice.
- The chip's axis remap (AXIS_MAP_CONFIG/SIGN) is left at its defaults.

Verified from the datasheet: the address, CHIP_ID, the register addresses, page 0, the OPR_MODE
and PWR_MODE values, UNIT_SEL, the scaling, the heading direction, the switching times, the POR
time, the crystal procedure, the CALIB_STAT layout, SYS_STATUS 5, the profile registers and the
order they are written in.

Unverified:
- **Clock stretching.** The BNO055 stretches SCL (4.6). Whether the ESP32-S3 I2C (Arduino-ESP32
  3.3.10) handles it at 400 kHz within the 50 ms HAL timeout must be checked on hardware. The
  classic ESP32 had known problems. If it fails, run the port at 100 kHz.
- Gravity-vector signs: +Z when flat is assumed, as for the accelerometer. The datasheet does
  not state it for the gravity output.
- The gyro sign: positive CCW about Z is assumed from the right-handed axis figure (3.4).
- Whether OPR_MODE may be written while the crystal is still starting. The driver waits for
  ST_MAIN_CLK first, as 5.5 describes. The datasheet does not say how soon after the CLK_SEL
  write ST_MAIN_CLK is set; the driver waits 10 ms before its first poll (the same delay the
  common Arduino libraries use after CLK_SEL). If the bit is set later than that, the poll ends
  early and the mode is written during the crystal start.
- The board schematic reading: pull-downs on PS0/PS1/ADR, and the host-side pull-ups to VCC.

## 2. `prosdk::BMI088`

```cpp
BMI088(I2C::Port port, int accelAddress = 0x19, int gyroAddress = 0x69,
       units::millisecond_t updatePeriod = 5_ms);
bool Calibrate(int samples = 500);
units::celsius_t GetTemperature();
```

Board:
- **The board ships in SPI mode.** Short **JP1** to select I2C (it drives the PS pin).
- **JP2** open gives accel 0x19 / gyro 0x69, the defaults. JP2 shorted gives 0x18 / 0x68: pass
  them to the constructor.
- J1: SCK/SCL, MOSI/SDA, nCS2, nCS1, MISO, GND. J2: VCC, GND. J3: INT1, INT3 (not used).
- In I2C mode, **leave nCS1 and nCS2 unconnected**. The datasheet pin table (7.1) gives CSB1 at
  VDDIO or not connected, and CSB2 not connected. Never pulse nCS1: a rising edge on CSB1
  switches the accelerometer to SPI until the next power-on (section 3). Whether the TXS0108E
  keeps a floating nCS1 steady is unverified.
- SPI mode is not supported by this class. A constructor on `prosdk::SPI` (F1) could be added
  later; it is not in this package.

What the driver does:
- **Identity.** ACC_CHIP_ID (0x00 at the accel address) must read 0x1E, and GYRO_CHIP_ID
  (0x00 at the gyro address) 0x0F.
- **Accelerometer power-up** (section 3, 4.1.1, 5.3.20, 5.3.21). The accelerometer starts in
  I2C mode and in suspend; no SPI dummy read is needed.
  1. ACC_PWR_CTRL (0x7D) = 0x04 (on), then wait 450 us.
  2. ACC_PWR_CONF (0x7C) = 0x00 (active; the reset value 0x03 is suspend).
  3. ACC_CONF (0x40) = 0xA8: normal filter, 100 Hz ODR, 40 Hz cut-off (Table 8).
  4. ACC_RANGE (0x41) = 0x01: +/-6 g.
  Every configuration write is followed by 450 us, the idle time required between writes in
  suspend mode (6.2).
- **Gyroscope** (5.5): normal mode after power-on (4.1.2), and GYRO_LPM1 is not written.
  GYRO_RANGE (0x0F) = 0x01, +/-1000 deg/s, 32.768 LSB per deg/s. GYRO_BANDWIDTH (0x10) = 0x03,
  400 Hz ODR, 47 Hz filter.
- **No soft reset.** All configuration registers are written explicitly.
- **Read**, three bursts per update:
  - accel 0x12-0x17;
  - gyro 0x02-0x07;
  - temperature 0x22-0x23.
  Data are int16, LSB first.
  - Accel: mg = raw / 32768 * 1000 * 2^(range + 1) * 1.5 (5.3.4), which is 5461.33 LSB per g
    at +/-6 g.
  - Temperature (5.3.7): 11-bit two's complement = MSB * 8 + LSB / 32, at 0.125 deg C per LSB
    plus 23 deg C. The chip updates it every 1.28 s.
- **Yaw, roll, pitch and `Calibrate()`** are exactly as on MPU6050:
  - yaw integrates the bias-corrected Z rate and drifts slowly (no magnetometer);
  - roll and pitch use a complementary filter with a gyro weight of 0.98;
  - `Calibrate()` averages the gyro over 500 samples 2 ms apart (about 1 s), with the robot
    still, and blocks the caller.
- The sensing axes follow 8.3: +1 g on Z when the chip is flat.
- `IsConnected()` needs both chip IDs at construction and the last read. A missing chip is not
  retried, as on MPU6050. Both parts are ready within 30 ms of power-on (4.1).

Verified from the datasheet: the addresses, the chip IDs, the power-up sequence and its waits,
the register values, the scaling, the temperature formula (including the 23.375 and -40 deg C
table rows used in the test) and the axes.

Unverified: the TXS0108E auto-direction shifter with I2C at 400 kHz, the board's own pull-ups
(the schematic resistors were not traced), and the floating nCS lines in I2C mode.

## 3. `prosdk::LSM303AGR`

```cpp
LSM303AGR(I2C::Port port, units::millisecond_t updatePeriod = 10_ms);
units::radian_t GetHeading();                       // absolute, CCW from magnetic north, -pi..pi
units::tesla_t GetMagX(), GetMagY(), GetMagZ();     // hard-iron offset subtracted
void SetHardIronOffset(units::tesla_t x, units::tesla_t y, units::tesla_t z);
```

Board:
- J1: VCC, SDA, /CS_MAG, /CS_XL, SCL, GND. **Tie /CS_XL and /CS_MAG to VCC (3.3 V)** for I2C.
  The datasheet requires CS high for I2C. Whether the TXS0104E alone keeps the CS lines high
  when they are left open is unverified.
- The addresses are fixed: accel 0x19, mag 0x1E. Only one board fits on each I2C port.
- The schematic shows no discrete I2C pull-ups; the bus relies on the TXS0104E or the host.
  Whether that is enough at 400 kHz is unverified. Add 4.7k to 3.3 V if the bus is unreliable.
- J2 interrupts (XL1, XL2, MAG) are 3.3 V outputs and are not used.
- The chip supports only 3-wire SPI. SPI is not offered.

What the driver does:
- **Identity.** WHO_AM_I_A (0x0F at 0x19) must read 0x33, and WHO_AM_I_M (0x4F at 0x1E) 0x40
  (Table 26).
- **Configuration.** Both sensors are off after power-on: the CTRL_REG1_A ODR is power-down and
  CFG_REG_A_M MD = 11 is idle (Table 26 defaults). The driver writes:
  - CFG_REG_A_M = 0x8C: COMP_TEMP_EN (which "must be set to 1", Table 92), high resolution,
    100 Hz, continuous mode;
  - CFG_REG_B_M = 0x03: offset cancellation and the low-pass filter (ODR/4);
  - CFG_REG_C_M = 0x10: BDU;
  - CTRL_REG1_A = 0x57: 100 Hz, normal/HR, XYZ enabled;
  - CTRL_REG4_A = 0x98: BDU, +/-4 g, HR.
- **Read**: two bursts per update.
  - Accel at sub-address 0x28 | 0x80. The MSb of the sub-address enables auto-increment (6.1.1).
    The data is 12-bit, left-justified (>> 4), at 1.95 mg per LSB (Table 3, +/-4 g HR).
  - Mag at the plain sub-address 0x68, as ST's own driver does. The data is int16 at
    1.5 mgauss = 0.15 uT per LSB (Table 3).
- **Heading.** The heading is tilt-compensated with vectors. Down is the normalized negative
  accel vector. East = Down x Mag and North = East x Down are horizontal. The compass heading
  of X is atan2(East.x, North.x), clockwise, and `GetHeading()` returns its negation
  (CCW-positive). `GetYaw()` is the unwrapped heading relative to `ResetYaw()`. The first
  reading sets the yaw to the absolute heading. Roll and pitch come from the accelerometer.
- The heading is magnetic, not true north, and has no gyro behind it. It is noisy (about 3
  mgauss RMS, Table 3) and is disturbed by motors, magnets and steel.
- **Hard-iron calibration** is done in software. `SetHardIronOffset()` subtracts a fixed vector
  from every mag reading. To measure it on the finished robot: set the offset to 0, turn the
  robot through every orientation, and take (max + min) / 2 of `GetMagX/Y/Z` on each axis. The
  chip's own OFFSET_*_REG_M registers are not used. There is no soft-iron correction.
- No temperature getter. The datasheet gives 1 digit per deg C but no zero point, so only
  changes could be reported.

Verified from the datasheet: the addresses, both WHO_AM_I values, the register addresses and
defaults (Table 26), every configuration bit, the sensitivities, the HR left-justification, the
accel auto-increment bit, and the same axis directions for accel and mag (Figure 2).

Unverified:
- Whether the magnetometer also honours the auto-increment bit. The driver does not rely on it:
  it uses plain addresses, as ST's driver does, and the datasheet does not say that the mag
  auto-increments without the bit. Check on hardware that `GetMagY/Z` change.
- The sign of the heading on hardware. Figure 2 is a perspective drawing, and the driver assumes
  a right-handed X/Y/Z. Read literally (top view, X toward the lower right, Y toward the lower
  left, Z up) the drawing would be left-handed, so this is a real open point, not a formality.
  Check it: turning the robot left must increase `GetYaw()`, and tilting its left side up must
  give a positive `GetAngleX()`.
- The /CS and pull-up questions above.

## Resource use

| Class | Thread | Default period | Bus traffic per update | Bus share at 400 kHz |
|---|---|---|---|---|
| BNO055 | `bno055` | 10 ms | 1 transfer, 50 data bytes | about 12 % (about 1.2 ms) |
| BMI088 | `bmi088` | 5 ms | 3 transfers, 14 data bytes | about 10 % (about 0.5 ms) |
| LSM303AGR | `lsm303agr` | 10 ms | 2 transfers, 12 data bytes | about 4 % (about 0.4 ms) |

- Each class uses one thread (prosdk default stack and priority via `ScopedThreadConfig`) unless
  `updatePeriod` is 0.
- At 100 kHz the BNO055 alone takes about half the bus.
- Each class uses no GPIO, PWM, RMT or PCNT resources.
- BNO055 blocks its caller for about 30 ms in `GetOffsets`/`SetOffsets` and in each
  reconfiguration. The reader thread waits meanwhile.

## Tests

`prosdk_drivers_tests`, using `FakeI2CBus` (F2) with one device per address. Run on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): 145
assertions in 10 test cases pass (BNO055 91/4, BMI088 26/3, LSM303AGR 28/3).

- `BNO055Test.cpp`:
  - the configuration sequence (with and without the crystal, and CLK_SEL already set);
  - scaling and the sign of the heading (90 degrees clockwise gives -90 degrees yaw);
  - continuity across the 0/360 wrap and `ResetYaw`;
  - the SYS_STATUS gate and roll from the gravity vector;
  - the calibration profile read and write sequence;
  - retries on a missing or booting chip, and reconfiguration after a NACK without a yaw jump
    (simulated time).
- `BMI088Test.cpp`:
  - the power-up and configuration sequence;
  - NACK, wrong chip ID and the JP2 addresses;
  - scaling, temperature, bias calibration and yaw integration on simulated time;
  - the roll filter.
- `LSM303AGRTest.cpp`:
  - the configuration sequence, NACK and the wrong WHO_AM_I;
  - heading at known field vectors (0, +/-90 and 30 degrees, and 30 degrees of roll facing
    north and west);
  - yaw continuity across +/-180 degrees, `ResetYaw`, and the hard-iron offset.

## Files

- `src/prosdk/drivers/imu/BNO055.hpp`, `BNO055.cpp`, `BMI088.hpp`, `BMI088.cpp`,
  `LSM303AGR.hpp`, `LSM303AGR.cpp` (new)
- `test/prosdk/drivers/BNO055Test.cpp`, `BMI088Test.cpp`, `LSM303AGRTest.cpp` (new)
- `design/boardoza/P3-imu.md` (this file)

Shared edits for integration:
- a design/drivers.md link to this file, plus the three test entries in its "Tests" list;
- three design/API-DIFFERENCES.md section 7 rows (`BNO055`, `BMI088`, `LSM303AGR`: prosdk
  additions, no WPILib counterpart).
