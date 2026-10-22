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
  AnalogInput, AddressableLED. There are no fixed headers like the roboRIO's DIO 0-9.
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
    `Wire`, `Serial1`, `ledcAttach`, `analogRead` outside the HAL).
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
| Addressable LED outputs | 3 | RMT TX channels (4; core's status LED uses one) | 1024 LEDs shared buffer |
| I2C ports | 2 | I2C controllers 0 and 1 | port 0 = `Wire`'s bus |
| Serial ports | 2 | UART1 (kOnboard), UART2 (kMXP) | USB ports not available |
| CAN buses | 1 | TWAI | external transceiver, classic CAN |
| Notifiers | unlimited | one thread, software alarms | |
| Interrupts | 0 | | WPILib 2027 has no interrupt HAL |
| Digital PWM, pulses | 0 | | stubs (systemcore does not implement them either) |
| CTRE PCM, REV PH, PDP/PDH | 0 | | not supported (below) |

## Output safety

- **Gate**: servo/ESC PWM pulses only leave the chip while probot-core's `outputsEnabled()` is
  true (RUN phase, Driver Station connected, no loop deadline miss, no e-stop). While it is false
  every PWM output is forced low (no pulses: ESCs and servos see "no signal" and stop). The gate
  is re-evaluated on core's user task before every loop iteration and every 10 ms by the
  `prosdk-gate` esp_timer, so outputs stop within about 10 ms of a disable even if the user loop
  hangs. Pulse values set while disabled are kept and apply on the next enable.
- **Emergency stop**: core deletes the user task and calls the framework e-stop hook on a fresh
  task. The backend's callback is lock-free: every HAL output pin (PWM, DIO outputs, LED data
  lines) is routed back to plain GPIO and driven low, then CAN periodic frames are stopped. After
  that, new PWM/DIO-output/LED allocations fail with `HAL_INCOMPATIBLE_STATE` and DIO writes are
  ignored until reboot. Deviation from the roboRIO: DIO outputs are also driven low (the roboRIO
  keeps DIO values on disable); "every HAL output" stops on an e-stop.
- **Not gated**: DIO outputs while merely disabled (as on the roboRIO), LED strips (they show
  robot state), I2C, serial and CAN. CAN TX is never forced low: a dominant TX line jams the bus;
  after an e-stop periodic frames stop and new sends fail with
  `HAL_ERR_CANSessionMux_NotAllowed`.
- **Brownout** is reported only (`HAL_GetBrownedOut`); outputs are not cut by hardware as on the
  roboRIO.

## Modules

### DIO (DIO.cpp)
- Input: `GPIO_MODE_INPUT` with the internal pull-up (an open switch reads true, as on
  roboRIO/systemcore). Output: starts low, `GPIO_MODE_INPUT_OUTPUT` (the level reads back),
  no pull.
- Direction change through `HAL_SetDIODirection`; changing to output after an e-stop is refused.
- `HAL_AllocateDigitalPWM`, `HAL_Pulse*` and friends report `HAL_HANDLE_ERROR` with a message
  (systemcore stubs them too).

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

### Serial (SerialPort.cpp)
- kOnboard = UART1 (default RX 15 / TX 16), kMXP = UART2 (no default pins:
  `prosdk::platform::SetSerialPins(1, rx, tx)`). kUSB1/kUSB2: `HAL_SERIAL_PORT_NOT_FOUND`
  (UART0/USB are the console). A UART that `Serial1`/`Serial2` already opened is refused.
- 1 KB RX and TX buffers. Defaults 9600 8N1, no flow control, read timeout 0.
- Supported: baud rate, data bits 5..8, parity none/odd/even (mark/space refused), stop bits
  1/1.5/2, read timeout, termination character (reads stop after it), write buffer flush on
  access or explicit flush, bytes received, clear. Not supported: hardware or software flow
  control (`HAL_SetSerialFlowControl` accepts only "none"), `HAL_GetSerialFD` (-1).

### CAN (CAN.cpp, CANAPI.cpp)
- TWAI (bus 0 only) with an external transceiver. Pins and bit rate from
  `prosdk::platform::SetCanConfig(tx, rx, bitrate)`: 125, 250 or 500 kbit/s or 1 Mbit/s (the
  default). The bus starts on the first CAN call; without a configuration CAN calls fail with a
  message.
- Classic CAN only: 0..8 data bytes; CAN FD frames are refused. 11-bit and 29-bit ids, RTR.
- One `prosdk-can` thread (FreeRTOS priority 3, the prosdk thread default) receives frames (latest frame per id for
  `HAL_ReadCAN*`, plus stream sessions with overrun reporting), sends periodic frames and recovers
  from bus-off. RX queue 32 frames, TX queue 16.
- No FRC CAN heartbeat is sent, so FRC CAN motor controllers (Talon, Spark) stay disabled; they
  are not part of the Probot kit.
- `HAL_CAN_GetCANStatus`: TX/RX error counters, bus-off count, TX-full count; bus utilization is
  not measured (0).

### Driver Station (DriverStation.cpp)
- The Driver Station is probot-core's browser Driver Station; the backend reads it through the
  CoreOps table. A pre-cycle callback on core's user task (every loop iteration, 20 ms) takes a
  new snapshot when a packet arrived or the control word changed and wakes the new-data events.
- Control word: robot mode from core's phase (AUTO_* autonomous, TELEOP_* teleop, otherwise the
  mode selected on the Driver Station); enabled = `outputsEnabled()`; e-stop latched; DS attached
  = a Driver Station client is connected; FMS never. Opmode: the first option registered with
  `HAL_SetOpModeOptions` for the robot mode (core selects only auto/teleop; the core opmode list,
  core-requests.md R7, is not wired yet).
- Joysticks: core gamepad port n = HAL joystick n (up to 6). The raw browser gamepad is remapped
  through core's active mapping profile to WPILib's Gamepad (SDL) layout: buttons FACE_DOWN,
  FACE_RIGHT, FACE_LEFT, FACE_UP, BACK, GUIDE, START, LEFT_STICK, RIGHT_STICK, LEFT_BUMPER,
  RIGHT_BUMPER, DPAD_UP/DOWN/LEFT/RIGHT (0..14); axes LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y (Y down
  positive), LEFT_TRIGGER, RIGHT_TRIGGER (0 or 1: core has trigger buttons only); POV 0 from the
  D-pad. Descriptor: gamepad, type "standard", name = the mapping profile's name. No touchpads,
  rumble or LEDs.
- No alliance station, match info or game data. Match time from core (-1 when unknown).
- `HAL_SendError`/console lines go to the Driver Station console (`platform::ConsoleWrite`).

### Power (Power.cpp)
- `HAL_GetVinVoltage` = core's battery reading (0 without a configured battery sensor:
  `PROBOT_BATTERY_ADC_PIN` or `PROBOT_BATTERY_INA`). Brownout from it with hysteresis (defaults
  6.75 V / 7.25 V, settable 4..13 V; never browned out below 0.5 V, i.e. no sensor).
- 3.3 V rail: fixed 3.3 V and active; current, fault counts and enabling it are not available
  (`HAL_HANDLE_ERROR`). `HAL_GetCPUTemp` = the chip's temperature sensor.

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
  WiFi MAC.

## Not supported

- CTRE PCM, REV PH, CTRE PDP and REV PDH (`CANDevices.cpp`): `Initialize*` fails with a message,
  every other call reports `HAL_HANDLE_ERROR`. They need the FRC CAN heartbeat and are not in
  the kit.
- Digital PWM generators and pulse outputs (stubs, as on systemcore).
- Serial flow control, ADC2, CAN FD, joystick rumble/LEDs/touchpads, alliance/match data.

## Completeness

`tools/build/hal_link_test.py --out DIR --compile` generates a sketch that takes the address of
all 731 functions in `design/hal-esp32-functions.txt` (695 backend + 36 common) and uses
`GetDefaultDriverStationImpl` and both `SimDevice` constructors, then links it with probot-core
(huge_app partition). A missing definition is an `undefined reference` link error.
`examples/HalSmoke` exercises DIO, PWM, Encoder, AnalogInput, AddressableLED and Notifier
through the HAL C API on hardware.
