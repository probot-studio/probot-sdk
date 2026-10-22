# prosdk drivers

Device drivers for the Probot kit on top of the prosdk HAL and wpilibc classes. Two sources:

- **Vendored** from `allwpilib/drivers` (module `tools/upstream/modules/drivers.py`, unchanged apart
  from the `wpi` -> `prosdk` renames): the 9 generated PWM motor controllers (`Koors40`,
  `PWMSparkFlex`, `PWMSparkMax`, `PWMTalonFX`, `PWMTalonSRX`, `PWMVenom`, `PWMVictorSPX`, `Spark`,
  `SparkMini`, in `prosdk/drivers/motor/`) and `GoBildaPinpoint` (`prosdk/drivers/odometry/`), with
  their upstream tests (`test/drivers`, executable `drivers_tests`). Excluded upstream files:
  `design/API-DIFFERENCES.md` section 4.
- **Hand-written** (`src/prosdk/drivers/`, tests in `test/prosdk/drivers`, executable
  `prosdk_drivers_tests`): `Servo` (the WPILib 2026 API, removed in 2027) and the drivers of the old
  Probot Lib 0.2.6 (BTS7960, VNH5019, MPU6050, ProHub PCA9685 and encoder stream), rewritten on prosdk
  classes, and the drivers for the Boardoza boards (section "Boardoza boards"). They never touch
  Arduino or probot-core APIs; all hardware access goes through prosdk classes (`PWM`,
  `DigitalOutput`, `DigitalInput`, `AnalogInput`, `Encoder`, `I2C`, `SerialPort`, `SPI`) and the
  platform HAL extensions (SPI, timed DIO, pulse I/O, six-step; `design/hal-esp32.md`), so they
  run unchanged on the host simulation HAL.

Every motor driver is a `MotorController` with `MotorSafety` (`Feed()` on every `SetThrottle()`),
inversion, `GetDescription()` and telemetry (`TelemetryLoggable`, type "Motor Controller", entry
"Value"), and has a `SimDevice` ("Throttle", after inversion) for tests. Background threads start
through `ScopedThreadConfig` (prosdk priority, below the probot-core loop).

Example: `examples/DriversShowcase` (one of each driver in a TimedRobot).

## Pins and output safety

- Motor and servo pins are GPIO numbers (any free GPIO, `design/hal-esp32.md`). The ProHub pins
  (I2C SDA 1 / SCL 2, encoder UART RX 10 / TX 11) are set with `prosdk::prohub::ConfigurePins()`
  before the first PCA9685, MPU6050 or ProHubEncoders is constructed (for example as the first
  member of the robot class).
- GPIO outputs (PWM, DIO, digital PWM) follow the HAL output gate: no pulses / duty 0 while the robot
  is not enabled, every pin driven low by the lock-free emergency-stop callback.
- I2C devices are not gated by the HAL (I2C is never gated). The PCA9685 driver gates itself
  (section "PCA9685"); the emergency stop has a limitation there.
- SPI is not gated either. Pulse transmitters (`PulseTransmitter`, IR, PYD1598 SERIN) are not
  gated while disabled, like LED strips; the emergency stop drives their pins low. The DRV2605L
  haptic driver and the GP8403 DAC are I2C outputs and are not gated: they set safe outputs at
  construction and in the destructor. The DRV8317H coasts while the gate is closed.

## Digital PWM (ESP32 HAL)

`BTS7960` and `VNH5019` need PWM at kHz rates, which the 50-400 Hz servo `PWM` class does not
give. They use `DigitalOutput::EnablePWM`/`UpdateDutyCycle`/`SetPWMRate`, which the ESP32 HAL now
implements on the LEDC peripheral (`src/prosdk/platform/hal/esp32/DIO.cpp`):

- 6 generators (one per PWM pin), **one rate for all of them**, 5 Hz to 40 kHz (default 500 Hz);
  resolution floor(log2(80 MHz / rate)) bits, at most 14 (12 bits at 20 kHz). Changing the rate
  re-attaches every active generator. Both H-bridge drivers set 20 kHz in their constructor, so the
  last constructed driver decides the rate for every digital PWM output.
- The output channel must be a DIO output (a `DigitalOutput`); each pin takes one generator.
- Gated: the duty is forced to 0 while the output gate is closed (the value set is kept and
  applies again on enable). The emergency stop drives the pins low; after it no generator can be
  attached.
- PPS mode and `DigitalOutput::Pulse` are not implemented (`HAL_HANDLE_ERROR`).

Budget: one BTS7960 uses 2 generators (`PWM_ON_INPUTS`) or 1 (`PWM_ON_ENABLE`), one VNH5019 uses 1.
Of the Boardoza drivers, `TwoPinHBridge` uses 2 per motor (the dual L9110S board 4), `VNH7070AS`
and `DRV8307` 1 each; `DRV8317H` uses MCPWM instead. Six generators drive at most three BTS7960
in the default wiring, six VNH5019, or three two-pin H-bridge motors. Every one of these drivers
sets 20 kHz (the VNH7070AS maximum, inside the DRV8307's 16-50 kHz), so mixing them keeps one
rate; any other `SetPWMRate` call changes the rate of every motor. The full budget of shared
units is in `design/boardoza.md`.

## Servo

`prosdk::Servo` keeps the WPILib 2026 API: `Set`/`Get` (0 to 1), `SetAngle`/`GetAngle` (0 to 180
degrees), `SetOffline`, `GetMaxAngle`/`GetMinAngle`, `GetChannel`, `SetBounds`. Defaults: 600 to
2400 us (HS-322HD, as WPILib). `SetBounds(max, deadbandMax, center, deadbandMin, min)` uses only max
and min (as 2026); 0 < min < max < 4096 us.

- **GPIO**: `Servo(int gpio)` on a prosdk `PWM` (MCPWM, 20 ms period).
- **PCA9685**: `Servo(PCA9685&, channel)`; the chip's period must be longer than the maximum pulse,
  so its frequency is at most about 400 Hz (`prohub::kMaxServoFrequency`); analog servos expect 50 Hz.
  On the ProHub the servo outputs are channels 12 to 15.
- `Set(NaN)` turns the output off like `SetOffline()`; while offline `Get()` returns 0.

## BTS7960 (IBT-2)

Assumed wiring: R_EN and L_EN bridged to one GPIO; module VCC to 5 V or 3.3 V (the inputs accept
3.3 V logic); R_IS/L_IS (current sense) not used.

- `Wiring::PWM_ON_INPUTS` (default): PWM on RPWM (forward) or LPWM (reverse), enable high while
  enabled. Throttle 0 brakes (both low sides on); `Disable()` coasts (enable low).
- `Wiring::PWM_ON_ENABLE` (old Probot Lib wiring): PWM on the bridged enable, direction on
  RPWM/LPWM (one high). Off-time and throttle 0 coast. The duty is 0 while the direction changes.
- Limits: 25 kHz maximum PWM (20 kHz set); no current sensing or fault input is read.

## VNH5019

Assumed wiring: INA, INB, PWM to GPIOs; ENA/ENB (EN/DIAG) optional, one GPIO for both or two.
EN/DIAG is an open-drain fault output of the chip that also enables the half bridges: when driven
by the ESP32 put a series resistor (for example 1 kOhm) in the line so that a fault pulling it low does
not short the GPIO. CS (current sense) is not read.

- Forward: INA high, INB low, PWM = throttle; reverse the opposite. The duty is 0 while INA/INB
  change.
- Throttle 0: INA = INB = low with PWM = brake strength (`SetBrakeMode(true)`, strength
  `SetBrakeStrength`, default 1: full brake) or 0 (coast). Pololu's brake convention.
- `Disable()`: PWM 0, INA/INB low, EN low.
- Limits: 20 kHz maximum PWM; the driver does not read faults (EN/DIAG is only driven).

## PCA9685 and ProHub

`prosdk::PCA9685` drives one NXP PCA9685 (16 channels, 12 bits) over prosdk `I2C`.

- **One frequency per chip**, 24 to 1526 Hz (prescale round(25 MHz / (4096 f)) - 1, internal
  oscillator; the actual frequency is `GetFrequency()`, and the oscillator's tolerance applies). Motors and servos on the same chip share it: the ProHub motor
  frequency is 872 Hz (`prohub::kMotorPwmFrequency`, as Probot Lib), a chip that also drives servos
  must run at 400 Hz or less (50 Hz for analog servos).
- **Buffered writes**: `SetDutyCycle`/`SetPulseWidth`/`SetOff` store the value; a thread (default
  every 10 ms, or `Update()` when the period is 0) writes the changed channels. All I2C access is
  under one mutex (the old driver wrote from several tasks without one). Output latency is up to
  one update period plus the bus time (about 0.15 ms per channel at 400 kHz).
- **Gate**: the thread writes ALL_LED_OFF (one I2C transfer) as soon as the robot is not enabled
  (`HAL_GetOutputsEnabled()`), after an emergency stop or after the program halted, and rewrites
  every channel when it is enabled again. `OutputsEnabled()` reports the gate as last applied.
- **Missing chip**: the constructor does not throw; it prints a console line and the thread retries
  the configuration every period. `IsConnected()` reports the last transfer.
- **Emergency-stop limitation**: probot-core's emergency-stop callback must be lock-free, and an I2C
  transfer takes the bus lock (which waits forever), so the PCA9685 is not written from it. The
  outputs go off within one update period through the gate above, but only if the I2C bus is free.
  If the user task that core deletes on an e-stop held the I2C bus lock at that moment, no further
  transfer happens and the PCA9685 outputs keep their last values. The HAL's "drive every output
  pin low" does not help either: the chip's OE pin is active-low. For a guaranteed stop use the kit's
  hardware e-stop (power cut). A lock-free fix needs a dedicated OE pin driven high by the e-stop
  (not on the current board).

`ProHubMotor(pca, port)`: motor ports 1 to 8 as Probot Lib (port 1 channels 0/1, port 2 3/4, port 3
6/7, port 4 9/10; ports 5 to 8 the same on a second PCA9685 at 0x41, planned). Positive throttle
drives the forward channel, negative the reverse channel, the other one fully off; throttle 0 and
`Disable()` turn both off (coast). The H-bridge chip behind the PCA9685 is not documented in the
old SDK; brake behaviour is therefore not offered.

`prosdk/drivers/ProHub.hpp` holds the board constants (pins, addresses, frequencies, channel map)
and `ConfigurePins()`.

## ProHub encoders

`ProHubEncoders` reads the encoder stream of the ProHub co-processor (8 quadrature counters) from
a `SerialPort` (default onboard port, RX 10 / TX 11, 921600 baud) on a thread (every 2 ms; the 1 KiB
UART buffer holds about 11 ms of stream). Frame, 108 bytes, little endian (Probot Lib 0.2.6 wire
format):

| Bytes | Field |
|---|---|
| 2 | header `AA 55` |
| 1 | sequence number |
| 4 | co-processor timestamp (us) |
| 4 | time since the previous frame (us) |
| 8 x 8 | total ticks per encoder (i64) |
| 8 x 4 | ticks since the previous frame per encoder (i32) |
| 1 | XOR of the 107 bytes before it |

Frames with a bad checksum are counted (`GetCrcErrors()`) and dropped; the parser resynchronises
on the next header. `IsConnected()` is false without a valid frame for 100 ms. The latest frame is
copied under a mutex (the old driver's double buffer could tear).

`ProHubEncoder(encoders, index)` gives one channel an `Encoder`-like API: `Get()` (ticks since
construction or `Reset()`), `GetDistance()`, `GetRate()` (from the last frame's delta ticks and dt),
`SetDistancePerPulse`, `SetReverseDirection`. Encoder i belongs to motor port i + 1. Ticks per
revolution depend on the motor (the co-processor counts quadrature edges as configured in its own
firmware).

## MPU6050

`prosdk::MPU6050` (address 0x68, 0x69 with AD0 high) with the getters of `OnboardIMU` (`GetYaw`,
`GetRotation2d/3d`, `GetQuaternion`, `GetAngleX/Y/Z`, `GetGyroRateX/Y/Z`, `GetAccelX/Y/Z`) plus
`GetTemperature`, `ResetYaw`, `Calibrate`, `IsConnected`.

- Mounting: chip flat, Z up, X forward, Y left (WPILib robot axes, counter-clockwise positive).
  Other mountings need the caller to remap axes.
- Ranges: +/-1000 deg/s (32.8 LSB per deg/s), +/-4 g, 44 Hz low-pass. Read every 5 ms by a thread
  (or `Update()` with period 0).
- Yaw integrates the bias-corrected Z rate. There is no magnetometer: yaw drifts (how fast depends on the chip,
  the calibration and temperature changes; nothing corrects it). Roll and pitch come from a
  complementary filter (gyro weight 0.98) with the accelerometer and do not drift.
- `Calibrate()` (500 samples, about 1 s) must run while the robot is still, for example in the robot
  constructor; it blocks the caller. Without it the raw bias (up to several deg/s) is integrated.
- WHO_AM_I 0x68 (MPU-6050) and the register-compatible 0x70/0x71/0x73 (MPU-6500/9250 family) are
  accepted; anything else prints a console line.

## GoBildaPinpoint and PWM motor controllers (vendored)

As upstream WPILib 2027. The PWM motor controllers run on prosdk `PWM` (MCPWM, gated); see
`design/hal-esp32.md` for the PWM channel limits. `GoBildaPinpoint` talks I2C (default address 0x31)
and needs its port's pins set, for example by `prohub::ConfigurePins()` when it shares the ProHub bus.

## Boardoza boards

Drivers for the Boardoza breakout boards. The board index, the shared resource budget and the
5 V hazards are in `design/boardoza.md`; each package document gives the wiring, API, behaviour
and what is verified. None of them has been tested on hardware.

| Package | Classes (`src/prosdk/drivers/...`) |
|---|---|
| [F1](boardoza/F1-spi.md) | `SPI` (bus/) |
| [F3](boardoza/F3-timed-io.md) | `DigitalShift`, `WriteWaveform`, `PulseTransmitter`, `PulseCapture` (bus/) |
| [P1](boardoza/P1-climate.md) | `LM75A`, `HIH8000`, `ENS210`, `MS5611`, `DPS310`, `BME688` (environment/) |
| [P2](boardoza/P2-light-thermal.md) | `OPT3001`, `OPT4048` (light/), `CaliPile`, `MLX90640`, `MLX90641` (thermal/) |
| [P3](boardoza/P3-imu.md) | `BNO055`, `BMI088`, `LSM303AGR` (imu/) |
| [P4](boardoza/P4-tof.md) | `VL53L7CX` (range/) |
| [P5](boardoza/P5-i2c-misc.md) | `DRV2605L` (haptic/), `PCF8563` (rtc/), `GP8403` (dac/), `I2CSwitch` (bus/), `NT3H2111` (memory/) |
| [P6](boardoza/P6-loadcell.md) | `NAU7802`, `HX711`, `LoadCell` (adc/) |
| [P7](boardoza/P7-gnss.md) | `UbloxGnss`, `NmeaGnss`, `GnssFix`, `nmea::`, `ubx::` (gnss/) |
| [P8](boardoza/P8-motors.md) | `TwoPinHBridge` (`L9110S`, `BA6208`), `VNH7070AS`, `DRV8307` (motor/) |
| [P9](boardoza/P9-storage.md) | `W25QFlash`, `W25NFlash` (memory/); `platform::MountSdCard` (`src/prosdk/platform/SdCard.hpp`) |
| [P10](boardoza/P10-tmag-leds.md) | `TMAG5170` (magnetic/), `ClockedAddressableLED` (led/) |
| [P11](boardoza/P11-lora.md) | `SX126x` (radio/) |
| [P12](boardoza/P12-pulse.md) | `IRTransceiver`, `OokReceiver` (radio/), `PYD1598` (sensor/) |
| [P13](boardoza/P13-eve.md) | `FT8xx`, `ft8xx::` panel presets (display/) |
| [P14](boardoza/P14-existing.md) | `AnalogCurrentSensor` (analog/); the boards that existing classes drive |
| [P15](boardoza/P15-drv8317h.md) | `DRV8317H` (motor/) |

Third-party code: the Melexis MLX90640/41 API (`thermal/melexis/`, Apache-2.0) and the ST VL53L7CX
ULD (`range/st/`, BSD-3-Clause), listed in `NOTICE`. The sim I2C changes that the I2C driver
tests rely on (device address, per-address NACK) are [F2](boardoza/F2-sim-i2c.md).

## Tests

- `drivers_tests`: the upstream tests (PWM motor controller pulse ranges, Pinpoint register
  protocol on the sim I2C).
- `prosdk_drivers_tests`: Servo (GPIO pulses, bounds, offline, PCA channel), PCA9685 (configuration
  sequence, prescale, buffered writes, gate off/on, ProHubMotor channels), BTS7960 (both
  wirings, brake/coast, inversion, disable), VNH5019 (direction, brake, EN pins), MPU6050 (fake
  register file: configuration, calibration, yaw integration on simulated time, tilt), ProHub
  encoders (frame parsing, resync, CRC errors, timeout, per-channel offset/reverse/rate).
  Boardoza (one `<Class>Test.cpp` each, on `FakeI2CBus`, `FakeSPIDevice` or the DIO/analog/pulse
  sims): `FakeI2CBus` itself, SPI, DigitalShift, PulseIO; LM75A, HIH8000, ENS210, MS5611, DPS310,
  BME688; OPT3001, OPT4048, CaliPile, MLX90640, MLX90641; BNO055, BMI088, LSM303AGR; VL53L7CX;
  DRV2605L, PCF8563, GP8403, I2CSwitch, NT3H2111; NAU7802, HX711, LoadCell; Nmea, Ubx,
  UbloxGnss, NmeaGnss; TwoPinHBridge, VNH7070AS, DRV8307, DRV8317H; W25QFlash, W25NFlash;
  TMAG5170, ClockedAddressableLED; SX126x; IRTransceiver, OokReceiver, PYD1598; FT8xx;
  AnalogCurrentSensor. Run on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check
  --test`): `prosdk_drivers_tests` passes 7,849 assertions in 198 test cases, of which the 45
  Boardoza test files are 7,657 assertions in 185 test cases; `drivers_tests` 275 assertions in
  23 test cases.
