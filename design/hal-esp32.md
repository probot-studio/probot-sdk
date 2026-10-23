# ESP32-S3 HAL backend

The WPILib 2027 HAL on the Probot board: `src/prosdk/platform/hal/esp32/*.cpp`, compiled into the
precompiled archive only (`tools/build/modules/hal.cmake`, module `hal_esp32`). The host build
uses the WPILib simulation backend instead (`host/hal/sim`). The contract is
`design/hal-esp32-functions.txt`; the reference implementation is allwpilib's systemcore backend
(pinned allwpilib `b68effc`). This file lists every hardware limit and every place where the
backend behaves differently from systemcore/roboRIO.

Target: ESP32-S3 (Arduino-ESP32 3.3.10, ESP-IDF 5.5), probot-core 0.4.x as the runtime and
Driver Station (design/PORT.md section 8).

## Channel model

- **Channel = GPIO number** for every pin API: DIO, PWM, Counter, Encoder, DutyCycle,
  AnalogInput, AddressableLED, and the prosdk extensions (SPI, timed DIO, pulse I/O, six-step).
  There are no fixed headers like the roboRIO's DIO 0-9.
- One table (`smartIoHandles`, a `DigitalHandleResource` indexed by GPIO, as systemcore's
  SmartIo) gives each pin exactly one owner. Allocating a pin twice fails with
  `HAL_RESOURCE_IS_ALLOCATED` and the previous allocation location.
- A pin is refused with a HAL error and a message when it:
  - does not exist (ESP32-S3: GPIO 0-21 and 26-48; 22-25 do not exist);
  - is USB D-/D+ (GPIO 19, 20: flashing and the USB console);
  - is UART0 TX/RX (GPIO 43, 44: `Serial`, the boot console);
  - belongs to the module's flash or PSRAM (`esp_gpio_is_reserved`; GPIO 26-32, and 33-37 on
    octal-PSRAM modules);
  - is reserved by probot-core: the pins named by its configuration (`NEOPIXEL_PIN`,
    `PROBOT_RSL_PIN`, `PROBOT_ESTOP_ENABLE_PIN`, `PROBOT_BATTERY_ADC_PIN`, the battery I2C pins),
    reported by the sketch-side bridge (`ReserveCorePins`);
  - is already attached to an Arduino peripheral (`perimanGetPinBusType`, e.g. a pin used by
    `Wire`, `Serial1`, `ledcAttach`, `analogRead` outside the HAL);
  - is the RX/TX pin of a serial port the HAL opened or the TX/RX pin of the started CAN bus:
    these claim their pins in `smartIoHandles` (`ClaimPin`, the owner shows in the error message)
    and release them on close (serial) or never (CAN runs until reboot). SPI bus and chip-select
    pins, the SD card's chip select, pulse I/O pins and six-step pins are claimed the same way.
- Free/Clean releases everything the HAL created: the hardware unit (MCPWM timer/generator, PCNT
  unit, capture channel, RMT channel, UART driver, TWAI driver), the Arduino peripheral-manager
  attachment, and the pin, which returns to `gpio_reset_pin` state (input with pull-up). After an
  emergency stop released output pins stay low instead.
- `HAL_GetNumDigitalChannels`, `HAL_GetNumPWMChannels` and `HAL_GetNumSmartIo` are 49 (GPIO 0-48,
  including the ones that do not exist); the other counts in Ports.cpp are hardware units that
  the pins share (table below).

## Capacities

| Resource | Count | Hardware | Notes |
|---|---|---|---|
| DIO | any free GPIO | GPIO matrix | inputs have the internal pull-up |
| PWM outputs | 12 | 2 MCPWM groups x 3 timers x 2 generators | the 2 outputs of a timer share its period |
| Counters + encoders | 4 together | 4 PCNT units | one unit each |
| Duty-cycle inputs | 6 | 2 MCPWM groups x 3 capture channels | |
| Analog inputs | 10 | ADC1, GPIO 1-10 | ADC2 refused (WiFi) |
| Addressable LED outputs + pulse transmitters | 3 together | RMT TX channels (4; core's status LED uses one) | 1024 LEDs shared buffer; one counter (`rmt::AcquireTxChannel`) for both kinds |
| Pulse capture inputs | 4 | RMT RX channels | |
| I2C ports | 2 | I2C controllers 0 and 1 | port 0 = `Wire`'s bus |
| Serial ports | 2 | UART1 (kOnboard), UART2 (kMXP) | USB ports not available |
| CAN buses | 1 | TWAI | external transceiver, classic CAN |
| SPI buses | 2 | SPI2 (`kPort0`), SPI3 (`kPort1`); SPI0/1 are the flash | no default pins (`SetSPIPins`) |
| SPI chip selects | 6 + 3 | hardware CS of SPI2 and SPI3 | a device without chip select takes the whole bus |
| Six-step motors | 2 | 1 MCPWM timer/operator pair each | each motor leaves 2 fewer of the 12 PWM outputs |
| Notifiers | unlimited | one thread, software alarms | |
| Interrupts | 0 | | WPILib 2027 has no interrupt HAL; only the six-step extension uses GPIO interrupts (the GPIO ISR service) |
| Digital PWM generators | 6 | LEDC, on DIO outputs | one rate for all, 5 Hz to 40 kHz; gated |
| DIO pulses | 0 | | stubs (systemcore does not implement them either) |
| CTRE PCM, REV PH, PDP/PDH | 0 | | not supported (below) |

## Output safety

- **Gate**: servo/ESC PWM pulses only leave the chip while probot-core's `outputsEnabled()` is
  true (RUN phase, Driver Station connected, no loop deadline miss, no e-stop). While it is false
  every PWM output is forced low (no pulses: ESCs and servos see "no signal" and stop). The gate
  is re-evaluated on core's user task before every loop iteration and every 10 ms by the
  `prosdk-gate` esp_timer, so outputs stop within about 10 ms of a disable even if the user loop
  hangs. Pulse values set while disabled are kept and apply on the next enable.
  Digital PWM generators (DIO) are gated the same way: their duty is 0 while the gate is
  closed (the value set is kept). Six-step motors coast while the gate is closed.
- **Emergency stop**: core deletes the user task and calls the framework e-stop hook on a fresh
  task. The backend's callback is lock-free except for the six-step step: `sixstep::OnEmergencyStop`
  runs first, takes the Hall interrupt's spinlock (held by every holder for a few register writes
  only), coasts every six-step motor and latches a flag so that neither the interrupt nor a
  setter drives a phase again. Then every HAL output pin (PWM, DIO outputs, LED data lines, pulse
  transmitter pins, six-step phase pins) is routed back to plain GPIO and driven low, then CAN
  periodic frames are stopped. New pulse transmitters, pulse trains and six-step motors are
  refused afterwards (`HAL_INCOMPATIBLE_STATE`). After
  that, new PWM/DIO-output/LED allocations and PWM period changes (they recreate the MCPWM
  generator, which would reconnect the pin) fail with `HAL_INCOMPATIBLE_STATE`, PWM outputs stay
  forced low whatever their pulse value, and DIO writes are ignored until reboot (a write racing
  the e-stop is undone). Deviation from the roboRIO: DIO outputs are also driven low (the roboRIO
  keeps DIO values on disable); "every HAL output" stops on an e-stop.
- **Not gated**: DIO output levels while merely disabled (as on the roboRIO), LED strips (they show
  robot state), pulse transmitters while merely disabled (an IR command or a sensor configuration
  word is not an actuator output), I2C, SPI, serial and CAN. Neither the gate nor the emergency
  stop stops SPI; no Boardoza SPI board drives an actuator. CAN TX is never forced low: a dominant TX line jams the bus;
  after an e-stop periodic frames stop and new sends fail with
  `HAL_ERR_CANSessionMux_NotAllowed`.
- **Brownout** is reported only (`HAL_GetBrownedOut`); outputs are not cut by hardware as on the
  roboRIO. With probot-core R12 an accepted `SetBrownoutVoltages` also sets core's low-voltage
  threshold (Power).

## Modules

### DIO (DIO.cpp)
- Input: `GPIO_MODE_INPUT` with the internal pull-up (an open switch reads true, as on
  roboRIO/systemcore). Output: starts low, `GPIO_MODE_INPUT_OUTPUT` (the level reads back),
  no pull.
- Direction change through `HAL_SetDIODirection`; changing to output after an e-stop is refused.
- Digital PWM generators (`DigitalOutput::EnablePWM`; systemcore stubs them) on the LEDC
  peripheral through the Arduino LEDC API: 6 generators, each on one DIO output
  (`HAL_SetDigitalPWMOutputChannel` refuses a pin that is not an allocated DIO output or already
  has a generator). One rate for all generators (`HAL_SetDigitalPWMRate`, 5 Hz to 40 kHz,
  default 500 Hz): resolution floor(log2(80 MHz / rate)) bits, at most 14; a rate change
  re-attaches every active generator. Duty 0..1 (NaN = 0), forced to 0 while the output gate is
  closed (applied with the PWM gate, before every loop iteration and every 10 ms). Freeing the
  generator or the DIO, or turning the DIO into an input, detaches it (pin low). After an
  emergency stop the pins are low like every DIO output and no generator can be attached.
- `HAL_SetDigitalPWMPPS` and `HAL_Pulse*` report `HAL_HANDLE_ERROR` with a message (systemcore
  stubs them too).

### PWM (PWM.cpp)
- MCPWM timers at 1 MHz (1 us per tick), up-counting; output high at timer zero, low at the
  comparator. Pulse range 0..4095 us; 0 = always low (disabled), 0xFFFF = always high.
- Default period 20 ms (50 Hz). `HAL_SetPWMOutputPeriod`: 0 = 5 ms, 1 and 2 = 10 ms, 3 = 20 ms
  (systemcore's multipliers 1x/2x/4x of 5 ms), others `HAL_PARAMETER_OUT_OF_RANGE`.
- The two generators of a timer share its period: a period change moves the output to a timer
  with that period or to a free timer, and fails with `HAL_NO_AVAILABLE_RESOURCES` (keeping the
  old period) when none is left. In practice: up to 12 outputs at one period, 6 period groups.
- Comparator updates are latched at timer zero (no glitched pulses).

### Counters and encoders (PulseCounter.cpp, Counter.cpp, Encoder.cpp)
- 4 PCNT units in total, shared by counters and encoders. 32-bit counts (the 16-bit hardware
  counter is extended on overflow through watch points at +32767/-32768).
- Glitch filter: pulses shorter than 1 us are dropped. Internal pull-ups on the inputs.
- Encoder: any two distinct free GPIOs as A/B (systemcore requires fixed pairs). Hardware
  decoding is always x4; 1x/2x are scaled like systemcore (`HAL_GetEncoderEncodingScale`).
  `HAL_GetEncoderFPGAIndex` returns the A GPIO.
- Rate: an esp_timer (`prosdk-pcnt`) samples every unit every 5 ms; the rate is the count change
  over the rate window, default 50 ms, settable 5..300 ms (`HAL_SetEncoderRateWindow`,
  `HAL_SetCounterRateWindow`). "Stopped" means a rate of 0 over the window. Maximum count rate is limited
  by the 1 us filter (about 500 kHz per channel).
- Counter: one input, counts rising (default) or falling edges.

### Duty cycle (DutyCycle.cpp)
- MCPWM capture channels, 3 per group, 6 in total, on any GPIO; both edges captured in an
  interrupt with the group's capture timer (80 MHz on the ESP32-S3; the resolution is read back
  from the driver).
- Frequency = 1 / (rising-to-rising period); output = high time / period; high time in ns.
- No edge for 100 ms: frequency 0, output 1.0 or 0.0 from the pin level, high time 0.
- The capture ISR writes into one of 6 static slots and checks the slot's generation (packed into
  the callback context) first; Free clears the generation, deletes the channel and waits for a
  running callback to finish before the slot is reused, so a late interrupt never touches freed
  memory.

### Analog inputs (AnalogInput.cpp)
- Channel = GPIO 1..10 (ADC1 channels 0..9). ADC2 (GPIO 11..20) cannot be read while WiFi is on;
  the Driver Station link is WiFi, so it is refused with the reason in the message.
- 12-bit, 12 dB attenuation (about 0..3.1 V). `HAL_GetAnalogVoltage` uses Arduino's calibrated
  `analogReadMilliVolts`; raw <-> volts uses the eFuse curve-fitting calibration (fallback:
  linear 3.1 V full scale, with a console note). No oversampling/averaging engine and no analog
  triggers/accumulators (WPILib 2027 has none in the HAL).

### Addressable LEDs (AddressableLED.cpp)
- WS2812/WS2812B timing (800 kHz) on RMT TX at 10 MHz; up to 3 outputs on any GPIO.
- One shared buffer of 1024 LEDs as on systemcore; each output sends its own `[start,
  start + length)` slice. `HAL_SetAddressableLEDData` converts to the requested wire order and
  sends every output with a non-zero length; it waits up to 100 ms for the previous frame of an
  output. A full 1024-LED frame takes about 31 ms.

### I2C (I2C.cpp)
- Port 0 is controller 0, the bus `Wire` uses. If `Wire` (or probot-core's INA battery sensor)
  already started it, the HAL shares it as is (pins and clock unchanged); otherwise the HAL starts
  it on `prosdk::platform::SetI2CPins(0, ...)` pins (default SDA 8, SCL 9, 400 kHz).
- Port 1 needs `prosdk::platform::SetI2CPins(1, sda, scl[, hz])` first.
- Transfers go through Arduino's I2C HAL (serialized per bus, 50 ms timeout) and return 0 on
  success, -1 on failure (NACK, timeout, short read), like systemcore. Combined write-read uses a
  repeated start.
- The HAL deinitializes a controller on the last close only if it started it.

### SPI (SPI.cpp)
- `prosdk::platform::hal` C++ API (`prosdk/platform/hal/SPI.hpp`: `OpenSpiDevice`,
  `ConfigureSpiDevice`, `SpiTransfer`, `CloseSpiDevice`) behind `prosdk::SPI`; WPILib 2027 has
  no SPI HAL. Bus 0 = SPI2, bus 1 = SPI3; pins from `prosdk::platform::SetSPIPins(bus, sck,
  mosi, miso)` (no defaults; `miso` -1 for a transmit-only bus).
- The bus starts lazily on its first device (no `Initialize*` hook): `spi_bus_initialize` with
  `SPI_DMA_CH_AUTO` and `max_transfer_sz` 4092, then one `spi_bus_add_device` per device
  (`queue_size` 1). The bus is freed when its last user leaves.
- Bus and CS pins are claimed with `ClaimPin` (handle type `VENDOR`). Chip select is the
  peripheral's hardware CS: 6 on SPI2, 3 on SPI3. A device without chip select must be alone on
  its bus: opening it on a bus with any user fails, and so does the reverse.
- Transfers are full duplex and serialized by one mutex per bus. Pieces of at most 64 bytes are
  polled, longer ones use the interrupt and DMA, and a segment over 4092 bytes is split. A
  transaction of several pieces acquires the bus and keeps CS active
  (`SPI_TRANS_CS_KEEP_ACTIVE`) until its last piece. Segments without transmit data send from a
  4092-byte DMA-capable zero buffer, allocated once. A transmit-only bus adds its devices with
  `SPI_DEVICE_NO_DUMMY`.
- Not gated (see Output safety).
- **SD card** (`prosdk::platform::MountSdCard`, `src/prosdk/platform/SdCard.cpp`): the card joins
  a bus through `detail::RetainSpiBus` (`esp32/SpiInternal.hpp`) and counts as one bus user,
  claims its chip select (`ClaimPin`, `VENDOR`, owner "SD card"), enables the MISO pull-up and
  lets `esp_vfs_fat_sdspi_mount` add its device. It stays mounted until the program ends. Mount it
  before other devices on the same bus talk. `RetainSpiBus` is refused on a bus that has a
  device without chip select. Details: `design/boardoza/P9-storage.md`.

### Serial (SerialPort.cpp)
- kOnboard = UART1 (default RX 15 / TX 16), kMXP = UART2 (no default pins:
  `prosdk::platform::SetSerialPins(1, rx, tx)`). kUSB1/kUSB2: `HAL_SERIAL_PORT_NOT_FOUND`
  (UART0/USB are the console). A UART that `Serial1`/`Serial2` already opened is refused. The RX
  and TX pins are claimed like DIO pins while the port is open (DIO/PWM on them is refused).
- 1 KB RX and TX buffers, fixed when the port opens: `HAL_SetSerialReadBufferSize` /
  `HAL_SetSerialWriteBufferSize` (`SerialPort::Set*BufferSize`) validate the handle and change
  nothing. Defaults 9600 8N1, no flow control, read timeout 0.
- Supported: baud rate, data bits 5..8, parity none/odd/even (mark/space refused), stop bits
  1/1.5/2, read timeout, termination character (reads stop after it), write buffer flush on
  access or explicit flush, bytes received, clear. Not supported: hardware or software flow
  control (`HAL_SetSerialFlowControl` accepts only "none"), `HAL_GetSerialFD` (-1).

### CAN (CAN.cpp, CANAPI.cpp)
- TWAI (bus 0 only) with an external transceiver. Pins and bit rate from
  `prosdk::platform::SetCanConfig(tx, rx, bitrate)`: 125, 250 or 500 kbit/s or 1 Mbit/s (the
  default). The bus starts on the first CAN call; without a configuration CAN calls fail with a
  message. Starting claims the TX and RX pins (DIO/PWM on them is refused); the bus is never
  stopped, so the claim lasts until reboot.
- Classic CAN only: 0..8 data bytes; CAN FD frames are refused. 11-bit and 29-bit ids, RTR.
- One `prosdk-can` thread (FreeRTOS priority 3, the prosdk thread default) receives frames (latest frame per id for
  `HAL_ReadCAN*`, plus stream sessions with overrun reporting), sends periodic frames and recovers
  from bus-off. RX queue 32 frames, TX queue 16.
- No FRC CAN heartbeat is sent, so FRC CAN motor controllers (Talon, Spark) stay disabled; they
  are not part of the Probot kit.
- `HAL_CAN_GetCANStatus`: TX/RX error counters, bus-off count, TX-full count; bus utilization is
  not measured (0).

### Timed DIO (BitBang.cpp)
- `prosdk/platform/hal/BitBang.hpp`: `ShiftIn(clockGpio, dataGpio, bits, highNs, lowNs, ...)`
  and `WriteWaveform(gpio, spans, ...)`, behind `prosdk::DigitalShift` and
  `prosdk::WriteWaveform`. The pins are GPIO numbers of allocated DIOs in the right direction
  (wpilibc does not expose DIO handles).
- Pin access through `gpio_ll` registers, waits are busy loops on the CPU cycle counter. A
  critical section masks interrupts for each clock-high phase of `ShiftIn` (at most 20 us) and
  for each whole `WriteWaveform` (at most 20 us); low phases run with interrupts enabled. The
  caller's thread busy-waits for the whole call.
- Emergency stop: nothing is written afterwards; a clock high that raced it is driven low again
  and the call fails.

### Pulse I/O (PulseIO.cpp)
- `prosdk/platform/hal/PulseIO.hpp`: `OpenPulseTx`, `SendPulses`, `WaitPulsesSent`,
  `OpenPulseRx(gpio, glitchUs, idleUs, ...)`, `ReadPulses`, `GetDroppedPulses`, `ClosePulse`,
  behind `prosdk::PulseTransmitter` and `prosdk::PulseCapture`.
- TX: one RMT TX channel per transmitter (from the 3 shared with addressable LEDs), 1 MHz
  resolution, 48-symbol block, queue depth 1, copy encoder; optional carrier from the RMT
  (`rmt_apply_carrier`, 2 kHz-1 MHz). The pin is registered as a HAL output pin, so the
  emergency stop drives it low. Not gated while disabled.
- RX: one RMT RX channel per capture (4), 1 MHz resolution, 96-symbol buffer with partial
  receive; the glitch filter and idle time map to `signal_range_min_ns` / `signal_range_max_ns`.
  The receive-done interrupt copies symbols into a lock-free 256-pulse ring and re-arms the
  receiver. The 4 capture slots are static (about 5.5 KiB) and never freed, so a late interrupt
  cannot touch freed memory.
- Details: `design/boardoza/F3-timed-io.md`.

### Six-step commutation (SixStep.cpp)
- `prosdk/platform/hal/SixStep.hpp`: `OpenSixStep(pins, pwmHz, ...)`, `SetSixStepTable`,
  `SetSixStepOutput(duty, brake)`, `GetSixStepFeedback`, `CloseSixStep`, behind
  `prosdk::DRV8317H` (3x PWM mode: INHx carries the PWM, INLx enables a phase). At most 2 motors
  (`kMaxSixStepMotors`), PWM 1-100 kHz.
- PWM: one MCPWM timer (10 MHz) with one operator, comparator and generator per motor, taken from
  the pairs the servo PWM outputs use. The generator's output signal is routed through the GPIO
  matrix to the INHx of the switching phase; the other INHx pins are held low.
- Commutation: a GPIO interrupt (the GPIO ISR service, shared with Arduino `attachInterrupt`) on
  every edge of the 3 Hall pins reads the Halls, looks up a precomputed table and writes up to 6
  GPIO registers, in an order where every intermediate state is Hi-Z or a subset of the target.
  The duty and mode setters re-read the Halls, which recovers a missed edge at standstill.
- Gate: the motor coasts while the output gate is closed (applied from the 10 ms gate timer and
  `UpdateOutputGate`). Emergency stop: see Output safety. Close: the motor coasts, the MCPWM
  objects are deleted and the pins are reset with a pull-down (a pull-up would turn a high side
  on in 3x mode).
- Details: `design/boardoza/P15-drv8317h.md`.

### Driver Station (DriverStation.cpp)
- The Driver Station is probot-core's browser Driver Station; the backend reads it through the
  CoreOps table. A pre-cycle callback on core's user task (every loop iteration, 20 ms) takes a
  new snapshot when a packet arrived or the control word changed and wakes the new-data events.
- Control word: robot mode from core's phase (AUTO_* autonomous, TELEOP_* teleop, otherwise the
  mode selected on the Driver Station); enabled = `outputsEnabled()`; e-stop latched; DS attached
  = a Driver Station client is connected; FMS never. UTILITY_INIT/UTILITY_RUN map to WPILib's
  utility mode.
- Opmodes (core-requests.md R7): `HAL_SetOpModeOptions` replaces core's opmode list. The WPILib
  opmode id (robot mode in bits 56-57, name hash below) is the core option id; the mode is
  autonomous, teleop or utility (other modes are skipped with a console warning); name, group and
  description are cut to core's limits (63/63/255 bytes, at a UTF-8 boundary) and the list to 64
  options, each with a console warning; text and background colors are dropped (the browser DS
  has no colored entries). A refused list returns `HAL_NO_AVAILABLE_RESOURCES` and the previous
  list stays. The control word's opmode is core's `activeOpModeId()` (latched at INIT) while an
  INIT or RUN phase is active and `selectedOpModeId()` while idle, so `OpModeRobot` constructs the
  selected opmode while disabled; an id whose mode differs from the control word's robot mode
  reports no opmode. There is no "first registered option" fallback.
- Joysticks (prosdk/platform/hal/CoreJoystick.cpp): core gamepad port n = HAL joystick n for
  ports 0-5 (`ds::rawGamepad(0..5)`, core R8; with an older core only port 0 has data). A port
  without a frame, or one core neutralized (stale), is a disconnected joystick. Core neutralizes
  a port whose frames stop for `PROBOT_INPUT_TIMEOUT_MS` (500 ms: an unplugged gamepad, a
  backgrounded Driver Station tab) without a new packet, so the pump re-reads the connected
  ports every cycle and drops a lost one at once; the last stick values are never kept. The raw browser
  gamepad is remapped through the snapshot's mapping profile to WPILib's Gamepad (SDL) layout:
  buttons FACE_DOWN, FACE_RIGHT, FACE_LEFT, FACE_UP, BACK, GUIDE, START, LEFT_STICK, RIGHT_STICK,
  LEFT_BUMPER, RIGHT_BUMPER, DPAD_UP/DOWN/LEFT/RIGHT (0..14); axes LEFT_X, LEFT_Y, RIGHT_X,
  RIGHT_Y (Y down positive), LEFT_TRIGGER, RIGHT_TRIGGER (0..1, analog: core's trigger value with
  its deadzone; the raw axis value is the undeadzoned analog button value scaled to 0..32767).
  POV 0 is core's 8-way POV; the D-pad buttons follow it (a diagonal presses both directions,
  opposite directions cancel). Descriptor: name = core's gamepad name (UTF-8, at most 63 bytes,
  empty when core has none); a gamepad with the browser's standard mapping is a gamepad of type
  `HIDType::STANDARD`, any other one a generic HID (not a gamepad, type UNKNOWN; the profile
  remap still applies). No touchpads, rumble or LEDs (`supportedOutputs` 0).
- No alliance station, match info or game data: the Probot DS has none to send (deferred to core
  request R18, design/core-requests.md). Match time from core (-1 when unknown).
- `HAL_WriteDisplayAnsi` (`DriverStationDisplay`): the screen as plain text in the telemetry table
  `DriverStationDisplay` (`prosdk/platform/dashboard/DashboardDisplay.hpp`; the page has no display
  panel, core request R19), not the console and not the DataLog telemetry copy. The terminal is
  linked into every sketch (the HAL's display hook is a virtual call, about 2-3 KB of flash).
- `HAL_GetSystemTimeValid` (`RobotController::IsSystemTimeValid`) = core's `ds::systemTimeValid()`
  (core-requests.md R10, CoreOps `systemTimeValid`): true once the Driver Station page has set the
  robot's clock to the browser's UTC time. DataLogManager then renames its log to
  `WPILIB_<UTC date>_<time>.wpilog` and writes `systemTime` entries. With a core without R10 it is
  the old heuristic: true once the wall clock is past 2024-01-01 (only if the sketch sets the time).
- `HAL_SendError`/console lines go to the Driver Station console (`platform::ConsoleWrite`) with a
  level (core-requests.md R9, `console::write`): HAL errors and ReportError are errors,
  ReportWarning and the HAL's warnings (calibration, priorities, timer gating) warnings, all other
  lines info. With a core without R9 the text goes to the console without its level. A console
  write from an ISR is dropped.

### Power (Power.cpp)
- `HAL_GetVinVoltage` = core's battery reading (0 without a configured battery sensor:
  `PROBOT_BATTERY_ADC_PIN` or `PROBOT_BATTERY_INA`). Brownout from it with hysteresis (defaults
  6.75 V / 7.25 V, settable 4..13 V; never browned out below 0.5 V, i.e. no sensor).
- Core's low-voltage flag (R12 `io::battery::setLowVoltage()` / `isLowVoltage()`, core's default
  off): an accepted `HAL_SetBrownoutVoltages` (`RobotController::SetBrownoutVoltages`, in range)
  sets its threshold to the brownout voltage, so core's flag flips at the voltage where
  `RobotController::IsBrownedOut()` enters brownout (core's flag has no recovery voltage). A
  refused call (`HAL_PARAMETER_OUT_OF_RANGE`) changes nothing. The WPILib default (6.75 V) is not
  forwarded: WPILib's brownout is a controller-rail concept, core's flag a low-battery warning
  (core's README suggests a 3S LiPo, where 6.75 V is far below any cutoff), and forwarding it at
  HAL initialization would turn core's flag on in every sketch and overwrite a threshold the
  sketch set itself. Without the call core's threshold stays off or at the sketch's own
  `setLowVoltage()`. Without R12 nothing is forwarded.
- Battery current: core's `io::battery::currentAmps()` (R12: declared in every configuration, NaN
  without an INA sensor, before its first sample or after a failed read) has no HAL function.
  WPILib 2027 has no robot input current (`HAL_GetVinCurrent` and `RobotController::
  GetInputCurrent` are gone; systemcore reports total current only through a PowerDistribution,
  which this HAL does not support: `HAL_InitializePowerDistribution` fails, every other PD call
  reports `HAL_HANDLE_ERROR` and 0). A sketch reads it from core directly and checks
  `std::isnan()`; logged through prosdk telemetry or a `DataLog` a NaN is stored as NaN (the
  dashboard store drops repeats of the same bytes).
- 3.3 V rail (the module's regulator, no monitoring): `HAL_GetUserVoltage3V3` 3.3 and
  `HAL_GetUserActive3V3` true, `HAL_GetUserCurrentFaults3V3` 0 and `HAL_ResetUserCurrentFaults`
  succeed (fixed values with `HAL_SUCCESS`); `HAL_GetUserCurrent3V3` and
  `HAL_SetUserRailEnabled3V3` report `HAL_HANDLE_ERROR` (no current sensor, the rail cannot be
  switched). `HAL_GetCPUTemp` = the chip's temperature sensor.

### Threads (Threads.cpp)
- WPILib's Linux real-time priorities map to FreeRTOS: 0 -> 1 (core's user task level),
  1..39 -> 2, 40..99 -> 3. Reading back: <= 1 -> 0, 2 -> 1, >= 3 -> 40. Nothing may run at or
  above core's system loop (4).
- `HAL_SetThreadPriority` on a `std::thread::native_handle()` uses `pthread_setschedparam`; the
  current thread uses `vTaskPrioritySet`. Out of range: `HAL_THREAD_PRIORITY_RANGE_ERROR`.

### Notifier (Notifier.cpp)
- systemcore's implementation: one SafeThread runs all notifiers; it asks for WPILib priority 40,
  which maps to FreeRTOS priority 3. Time base: `HAL_GetMonotonicTime` (esp_timer, since boot).

### Other
- IMU: the Probot board has no onboard IMU; reads report `HAL_HANDLE_ERROR` with a message, yaw
  getters return 0.
- Extensions: no dynamic loading; `HAL_LoadExtensions` loads nothing, the in-process registry
  works. Usage reporting: accepted and dropped.
- SimDevice and HALSIM_*: inert stubs as on systemcore (SimStubs.cpp, SimDevice.cpp), so the
  device `*Sim` classes link but do nothing on the robot.
- Runtime type `HAL_RUNTIME_SYSTEMCORE` (the closest real-robot type); serial number from the
  WiFi MAC. `HAL_GetComments` returns an empty string (SystemCore reads the comments of its robot
  image; the probot has none), so `RobotController::GetComments()` is `""`.
- `HAL_GetTeamNumber` is always 0: a Probot has no team number; the robot is found through its own
  WiFi access point, not a team-number address.
- `HAL_GetRSLState` = core's `ds::rslState()` (R12: the signal light core draws is lit; blinking
  in a RUN phase that may move the robot, solid on otherwise, tracked with or without
  `PROBOT_RSL_PIN`). `HAL_GetCommsDisableCount` = core's `ds::commsDisableCount()` (times the DS
  timeout stopped an enabled robot since boot, PB-E303; a timeout during INIT and PB-E304 are not
  counted). With a core without R12 both report `HAL_HANDLE_ERROR` (as on systemcore).
- Framework install: `<prosdk.h>` installs prosdk's hooks into probot-core at static
  initialization. If another framework already holds core's hooks (`probot::framework::install`
  returns false), prosdk prints an error to the console at once and again from `HAL_Initialize`,
  and does not abort: the sketch keeps running, but prosdk's per-cycle work (Driver Station
  snapshots, robot loop) never runs and PWM outputs stay off (the output gate also requires the
  install, since the e-stop and pre-cycle hooks that guard it are not called). A sketch that does
  not include `<prosdk.h>` gets the same `HAL_Initialize` error.

## Not supported

- CTRE PCM, REV PH, CTRE PDP and REV PDH (`CANDevices.cpp`): `Initialize*` fails with a message,
  every other call reports `HAL_HANDLE_ERROR`. They need the FRC CAN heartbeat and are not in
  the kit.
- DIO pulse outputs and digital PWM PPS mode (stubs, as on systemcore).
- Serial flow control, ADC2, CAN FD, joystick rumble/LEDs/touchpads, alliance/match data (core R18).

## Completeness

`tools/build/hal_link_test.py --out DIR --compile` generates a sketch that takes the address of
all 733 functions in `design/hal-esp32-functions.txt` (697 backend + 36 common) and uses
`GetDefaultDriverStationImpl` and both `SimDevice` constructors, then links it with probot-core
(huge_app partition). A missing definition is an `undefined reference` link error.
`examples/HalSmoke` exercises DIO, PWM, Encoder, AnalogInput, AddressableLED and Notifier
through the HAL C API on hardware. The prosdk C++ extensions (SPI, timed DIO, pulse I/O,
six-step) are not in that list; they are covered by the host tests of the classes that use them
and by the Boardoza examples, and none has run on hardware yet.
