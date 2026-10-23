# P14: analog current sensors and the boards that use existing classes

Package P14 of the Boardoza plan. It adds one class, `prosdk::AnalogCurrentSensor`, for the
INA250A1 and ACS758 boards. It also documents every board in plan section 1 that has the status
`existing`, `doc_only` or `not_applicable`: which prosdk class to use, how to wire it to the
ESP32-S3, the I2C address where there is one, and the hazards.

Sources:
- The Boardoza GitHub repos: README, KiCad schematic and PCB, example sketches.
- The manufacturer datasheets bundled in those repos, read with pdftotext.
- The Ozdisan product pages.

Anything not checked against a datasheet or a schematic is marked **unverified**.

ESP32-S3 rules that apply to every row (design/hal-esp32.md):
- Pins are 3.3 V only and **not 5 V tolerant**.
- Analog inputs are ADC1 only: GPIO 1-10, readable from about 0 to 3.1 V. ADC2 is refused
  because the Driver Station link runs over WiFi.
- `DigitalInput` always enables the internal pull-up, so an open line reads true.
- The HAL refuses these pins:
  - GPIO 19/20 (USB) and 43/44 (UART0, the console);
  - flash and PSRAM pins: 26-32 (33-37 are not refused, even on octal-PSRAM modules: see
    design/hal-esp32.md "Other ESP32-S3 boards");
  - the probot-core pins: `NEOPIXEL_PIN` (3), RSL, e-stop and the battery pins;
  - pins already in use by an Arduino peripheral.
- I2C port 0 defaults to SDA 1 / SCL 2 (the Pulse J3, section 3.1) at 400 kHz. Other pins need
  `platform::SetI2CPins`.
- SPI bus 0 defaults to SCK 17 / MOSI 15 / MISO 16 (the Pulse J5). Other pins need
  `platform::SetSPIPins`.
- Serial ports:
  - `SerialPort::Port::ONBOARD` is UART1, on RX 10 / TX 11 (the Pulse J4 F1/F2) by default.
  - `MXP` is UART2 and needs `platform::SetSerialPins(1, rx, tx)`.
  - There is no hardware flow control.
- CAN is TWAI, classic CAN only. Call `platform::SetCanConfig(tx, rx, bitrate)` before opening
  it, and use an external 3.3 V-logic transceiver.
- Digital PWM (`DigitalOutput::EnablePWM`) has 6 LEDC generators sharing **one** rate. The
  BTS7960 and VNH5019 drivers set it to 20 kHz (plan G10).
- `AddressableLED`: 3 outputs and 1024 LEDs shared between them, GRB by default.

## 1. `prosdk::AnalogCurrentSensor` (INA250A1, ACS758LCB-050B)

Files:
- `src/prosdk/drivers/analog/AnalogCurrentSensor.hpp`
- `src/prosdk/drivers/analog/AnalogCurrentSensor.cpp`
- `test/prosdk/drivers/AnalogCurrentSensorTest.cpp` (tag `[analogcurrent]`)

### API

```cpp
AnalogCurrentSensor(int adcGpio, double voltsPerAmp, double zeroVolts, double dividerRatio = 1.0);
static AnalogCurrentSensor INA250A1(int adcGpio, double refVolts, double dividerRatio = 1.0);
static AnalogCurrentSensor ACS758LCB050B(int adcGpio, double dividerRatio, double vccVolts = 5.0);

units::ampere_t GetCurrent() const;   // (sensor V - zero V) / voltsPerAmp
units::volt_t GetVoltage() const;     // ADC pin V x dividerRatio
void CalibrateZero(int samples = 100); // average of `samples` reads, at least 1, no delay
units::volt_t GetZeroVoltage() const;
int GetChannel() const;
// TelemetryLoggable: "Value" = amps, "Voltage" = sensor volts, type "Current Sensor"
```

- `dividerRatio` is the sensor voltage per ADC pin voltage, (Rtop + Rbottom) / Rbottom. Use 1
  when there is no divider. A value below 1, or a non-finite value, is replaced by 1 with a
  console warning.
- If `voltsPerAmp` is 0 or not finite, the sensor prints a warning and always reads 0 A.
- The ACS758 preset prints a warning when `vccVolts` is outside the 3.0-5.5 V supply range
  (for example 3300 passed as millivolts). The values are still used.
- A negative `voltsPerAmp` flips the sign, for a sensor wired the other way round.
- A non-finite `zeroVolts` becomes 0.
- Resources: one ADC1 pin through `AnalogInput`. No thread and no timer. Each `GetCurrent()`
  is one ADC read; the class does no averaging. Usage is reported as `"IO", gpio`.

### INA250A1 board (Boardoza INA250A1 breakout)

- Transfer function: VOUT = ILOAD x 200 mV/A + VREF.
  - Source: TI SBOS511C section 7.4.1, equation 1; the A1 gain is from Table 7-1.
  - The 2 mOhm shunt is inside the chip. Positive current flows IN+ -> IN- (J2).
  - Ratings (datasheet): 15 A continuous. VS 2.7-36 V. The output swings from GND + 25 mV to
    VS - 0.1 V.
- Board header J1, traced from the schematic: 1 VS, 2 REF, 3 OUT, 4 GND.
  - **REF is routed only to J1 pin 2; there is no tie on the board.** The user must wire it.
  - For a unidirectional sensor, tie REF to GND and use `refVolts` 0.
  - For a bidirectional sensor, drive REF from a low-impedance reference and pass its voltage.
  - The datasheet requires a buffered REF source (section 7.4.1). A bare resistive divider on
    REF is not enough.
- Wiring: power VS from **3.3 V**. OUT then stays below about 3.2 V, and the ADC reads up to
  about 3.1 V, which is 15 A with REF at 0 V. If VS is 5 V, OUT can reach 4.9 V and needs a
  divider; pass its ratio.
- OUT cannot swing below GND + 25 mV typ (50 mV max; datasheet 6.5, RL = 10 kOhm). With REF
  grounded, currents below about 0.125-0.25 A read high, and negative currents read about 0 A.
  **Do not call `CalibrateZero()` with REF grounded**: it would store that swing limit as the
  zero and offset every reading by up to -0.25 A. With REF at a mid-rail reference,
  `CalibrateZero()` is fine.

```cpp
prosdk::AnalogCurrentSensor ina = prosdk::AnalogCurrentSensor::INA250A1(/*adcGpio=*/4, /*refVolts=*/0.0);
double amps = ina.GetCurrent().value();
```

### ACS758LCB-050B board (Boardoza ACS758 breakout)

- Sensitivity: 40 mV/A at VCC = 5 V, with the zero at VCC/2 (VIOUT(QBI)). Source: the Allegro
  ACS758 datasheet, X050B performance characteristics. Range +/-50 A, bidirectional.
- Positive current flows IP+ -> IP- (chip terminal 4 -> 5), which drives VOUT above VCC/2
  (datasheet). The datasheet marks the ACS758xCB family "Not for New Design".
- Ratiometry, meaning the sensitivity and zero scale with VCC, is only specified for VCC
  4.5-5.5 V. The chip runs from 3-5.5 V. The preset scales 40 mV/A x VCC/5 for any VCC; **below
  4.5 V that scaling is unverified**. In that case, call `CalibrateZero()` and measure the
  sensitivity with a known current, then use the general constructor.
- Datasheet limits: RLOAD at least 4.7 kOhm, noise about 10 mV, offset +/-5 mV. Offset alone is
  about +/-0.13 A at 40 mV/A, so call `CalibrateZero()` at start-up with no current flowing.
- Board header J1: 1 VCC, 2 VOUT, 3 GND. The board adds R1 100 Ohm and C2 10 nF on the output.
- **At VCC = 5 V, VOUT spans 0.5-4.5 V and needs a divider** before the ESP32-S3 pin. For
  example, 10 kOhm (top) over 20 kOhm (bottom) is ratio 1.5, giving 0.33-3.0 V at the pin.
  Keep the divider total at 4.7 kOhm or more (RLOAD).
- At VCC = 3.3 V the preset gives 1.65 V +/- 26.4 mV/A, about 0.33-2.97 V over +/-50 A, which
  fits the ADC with no divider. That scaling is the unverified ratiometric extrapolation above.
- The current path is the board's high-current terminals. Size the wiring for the current.

```cpp
prosdk::AnalogCurrentSensor acs = prosdk::AnalogCurrentSensor::ACS758LCB050B(/*adcGpio=*/5, /*divider=*/1.5);
acs.CalibrateZero();  // motor off
double amps = acs.GetCurrent().value();
```

### Other sensors through the general constructor

Any sensor with a linear output works with the general constructor. For example, the CS pin of
both VNH5019 boards (section 2.5): `AnalogCurrentSensor cs{gpio, 0.143, 0.0};`.

## 2. Boards with the `existing` status

Each entry gives the pins (board header -> host), the class, a snippet and the hazards. The GPIO
numbers in the snippets are examples: use any free pin that the HAL accepts.

### 2.1 Sensors and simple I/O

**MQ4 methane sensor** (MQ4_Methane_Gas_Sensor_Breakout_Board)
- Header: 1 +5V, 2 AO, 3 DO, 4 GND.
- Class: `AnalogInput` on AO and/or `DigitalInput` on DO. The optional `MQGasSensor` helper is
  **not** built (plan decision).
- Hazards:
  - The heater needs 5 V at about 150-190 mA, so do not power it from the 3.3 V rail.
  - AO and DO are 5 V signals. AO needs a divider into ADC1; 10k/10k (ratio 2) keeps 5 V at
    2.5 V. DO needs a level shifter or a divider.
  - The sensor needs a preheat time. The load resistor RL on the board is **unverified**, so a
    ppm conversion needs calibration.
```cpp
prosdk::AnalogInput gas{6};          // AO through a 2:1 divider
double aoVolts = gas.GetVoltage() * 2.0;
```

**Vibration motor** (Vibration_Motor_Breakout_Board)
- J1: 1 VDD (3.3-5 V), 2 I/O (active high, drives a 2N7002 low-side switch), 3 GND. The motor
  draws about 80 mA from VDD.
- Class: `DigitalOutput`. For variable strength, `EnablePWM` works, but it shares the one LEDC
  rate (G10).
```cpp
prosdk::DigitalOutput buzz{12};
buzz.Set(true);
```

**ADXL345 accelerometer** (ADXL345_Triple_Axis_Accelerometer_Breakout_Board)
- J1: 1 GND, 2 SDA, 3 SCL, 4 +5V (3.3-5 V).
- Side header: CS (must be high for I2C), SDO/ADDR, INT1, INT2.
- **I2C address: 0x53.** The board grounds SDO. The class default is 0x1D, so pass 0x53.
  - Source: the datasheet and the library's address. The board strap itself is unverified,
    because the repo has no schematic.
  - If SDO is tied high, the address is 0x1D.
- Class: `ADXL345_I2C`.
```cpp
prosdk::ADXL345_I2C accel{prosdk::I2C::Port::PORT_0, /*range=*/4, /*address=*/0x53};
```
- Hazard: whether the board has SDA/SCL pull-ups and pulls on CS/SDO is unverified. The chip has
  no internal pulls on CS or SDO. If reads fail, tie CS to VCC.

**BS801C capacitive touch** (BS801C_Capacitive_Touch_Sensor_Breakout_Board)
- Pins: 1 VCC, 2 KOUT, 3 GND. KOUT is open-drain and active low.
- Class: `DigitalInput`. Its internal pull-up is enough, and a touch reads false.
- Behaviour: response is about 100 ms. The chip releases a touch held for about 40 s.
- Power: from 3.3 V.
```cpp
prosdk::DigitalInput touch{13};
bool touched = !touch.Get();
```

**SM351LT Hall switch** (SM351LT_Hall_Effect_Sensor_Breakout_Board)
- Pins: 1 VCC (3.3-5 V, with an LDO on the board), 2 OUT, 3 GND.
- OUT is a push-pull output at the 3.3 V rail of the on-board LDO (Honeywell datasheet: "push-pull
  output which does not require a pull-up"). The plan table's "open-drain" is wrong; either way,
  `DigitalInput` works.
- Class: `DigitalInput`.
- Behaviour:
  - Polarity: OUT goes **LOW with a magnet present** and HIGH when it is removed (datasheet
    notice under Table 1). The board schematic shows no inverter on OUTPUT (LDO, LEDs and
    resistors only).
  - The output is sampled about every 100 ms (sleep time 30-180 ms), so it is not a speed sensor.
  - The field must be parallel to the package.
```cpp
prosdk::DigitalInput hall{14};
bool magnet = !hall.Get();
```

**Water level sensor** (Water_Sensor_Analog_Water_Level_Detection_Breakout_Board)
- J1: 1 VCC, 2 SGN, 3 GND.
- Class: `AnalogInput` on an ADC1 pin.
- Power: from 3.3 V, so SGN stays within the ADC range.
- Hazards:
  - Leaving it powered all the time corrodes the traces by electrolysis. Optionally, power VCC
    from a `DigitalOutput` and switch it on only to read.
  - The dry and wet levels need calibration.
```cpp
prosdk::DigitalOutput waterPower{21};
prosdk::AnalogInput water{7};
waterPower.Set(true);  /* settle */  double level = water.GetVoltage();  waterPower.Set(false);
```

**Analog-to-PWM converter** (Analog_To_PWM_Converter_Breakout_Board)
- Headers:
  - J1: 1 VCC, 2 INPUT, 3 GND.
  - J2: V_OUT, GND. This is the schematic order; the README differs.
  - J3: VCC, GND.
- Output: about 20 kHz push-pull at VCC. Power it from **3.3 V** if the ESP32-S3 reads the
  output.
- Class: `DutyCycle` (MCPWM capture; one of 6 duty-cycle inputs).
  `GetOutput()` returns the duty cycle, from 0 to 1.
```cpp
prosdk::DutyCycle pwmIn{17};
double duty = pwmIn.GetOutput();
```
- Unverified: the duty-cycle resolution at 20 kHz, and whether the frequency is stable.

### 2.2 Isolation and relays

**PC817C 2CH / 4CH optocoupler level shifters** (PC817C_2CH_/4CH_Isolated_Level_Shifter...)
- Output: non-inverting. The opto transistor is an emitter follower with a pulldown.
- Maximum frequency: about 1.5 kHz (Boardoza's figure, unverified). **Not for encoders.**
- 2CH pins:
  - J1: VIN, GND (output side). J2/J3: IN+, IN-. J4/J5: OUT, GND.
  - 332 Ohm input resistor, 1.8 kOhm pulldown.
- 4CH pins:
  - J1: VIN, GND. J2-J5: inputs. J6-J9: outputs.
  - 560 Ohm input and 2.2 kOhm pulldown on the PCB. The schematic shows other values, so
    which are fitted is unverified.
- Usage: as an input to the ESP32-S3, use `DigitalInput` on OUT, with **VIN = 3.3 V** so that
  OUT is at most 3.3 V. As an output from the ESP32-S3, use `DigitalOutput` on IN+.
- The bundled datasheet is the UTC UPC817, an equivalent part. Which part is fitted is
  unverified.
```cpp
prosdk::DigitalInput isolatedIn{18};   // OUT, VIN = 3.3 V
```

**CPC1008N DC solid-state relay** (CPC1008N_Solid_State_Relay_Breakout_Board)
- Pins: J1: 1 SIGNAL (active high), 2 GND. J2: OUTA, OUTB (the switch).
- Class: `DigitalOutput`.
- Ratings: 100 V, 150 mA load. On within 2 ms, off within 1 ms.
- Hazards:
  - The 1 kOhm input resistor gives about 2 mA of LED current at 3.3 V, which is marginal
    against the datasheet's on-current. Test that it switches reliably.
  - The board has no pulldown, so the relay state is undefined while the pin floats at boot.
  - Avoid the strapping pins 0, 3, 45 and 46.
```cpp
prosdk::DigitalOutput relay{38};
relay.Set(true);
```

**CPC1945Y AC solid-state relay** (CPC1945Y_AC_Solid_State_Relay_Breakout_Board, repo only)
- Pins: J1: 1 CONTROL, 2 GND. J2: AC load x2.
- Class: `DigitalOutput`.
- Ratings: 20-120 Vac, 1 A, zero-cross switching. AC loads only; it does not switch DC.
- **Mains hazard. Not rated for 230 V mains.** Keep it off robots.
- The input current is about 2.1 mA at 3.3 V, against 5 mA guaranteed (datasheet). Switching
  from 3.3 V is unverified; use a transistor driver.

### 2.3 Motor drivers (VNH5019)

Both boards use `prosdk::VNH5019(ina, inb, pwm[, ena, enb])` (drivers/motor/VNH5019.hpp).
- PWM runs at 20 kHz, which is the VNH5019 datasheet maximum, on the shared LEDC rate.
- Each board uses 1 LEDC generator out of 6.
- Current sense: the CS pin gives about 0.143 V/A.
  - Derived from the datasheet K factor of about 7000 (spread +/-12-19 %) and the 1 kOhm R on
    the board.
  - It is valid only while the high side conducts, so average or filter it.
  - The board puts a 10 kOhm / 33 nF low-pass filter on CS.
  - The ADC reads up to about 21 A.

**VNH5019 w/ Enc Elite** (VNH5019ATR_w-Enc_Elite_Breakout_Board)
- Control headers:
  - J1: 1 VCC, 2 INA, 3 ENA, 4 PWM, 5 GND. **J1 pin 1 is the motor supply (VIN). Do NOT
    connect it to 3V3.**
  - J6: 1 VDD (host 3.3 V), 2 CS, 3 ENB, 4 INB, 5 GND.
- Power and motor: J2: OUTB, OUTA. J3: VIN (5.5-24 V), GND.
- Encoder headers:
  - J4 ENCODER_IN: VDD, C, B, A, GND.
  - J5 ENCODER_OUT: VDD, A, B, C, GND. These outputs go through LVC buffers powered from VDD.
- Hazard: keep **VDD at 3.3 V**. J4 VDD is the same net as VDD, so a 5 V encoder powered from
  J4 would make the buffered outputs 5 V. Power a 5 V encoder separately; the encoder inputs
  tolerate 5 V.
- ENA and ENB have 4.7 kOhm pull-ups to VDD and 1 kOhm series resistors, so the 3-argument
  constructor works. CS_DIS is tied low, so current sense is always on.
- Channel C (index or third Hall sensor) is unverified.
```cpp
prosdk::VNH5019 motor{/*ina=*/39, /*inb=*/40, /*pwm=*/41};
prosdk::Encoder enc{/*A_OUT=*/47, /*B_OUT=*/48};
prosdk::AnalogCurrentSensor cs{/*adcGpio=*/2, 0.143, 0.0};
```

**VNH5019 module** (VNH5019_H_Bridge_Motor_Driver_Module, repo only)
- J1, in board order:
  - 1 VCC: internal VIN, **leave it open**.
  - 2 PWM: 10 kOhm pulldown.
  - 3 INB.
  - 4 ENB.
  - 5 VDD: host 3.3 V. This is the pull-up rail for ENA and ENB.
  - 6 CS.
  - 7 ENA.
  - 8 INA.
  - 9 GND, 10 GND.
- Power and motor: J2: OUTB, OUTA. J3: VIN (5.5-24 V), GND.
- The module has the same EN pull-ups and 1 kOhm series resistors, and the same CS network, as
  the Enc Elite board. The 3-argument constructor works.
```cpp
prosdk::VNH5019 motor{/*ina=*/39, /*inb=*/40, /*pwm=*/41};
```

### 2.4 Serial and CAN transceivers

**RS485 3V3 transceiver** (RS485_3V3_Transceiver_Breakout_Board)
- Pins:
  - J1/J2: 1 GND, 2 TXD, 3 RXD, 4 VCC (3.3 V).
  - Bus: COM, B, A.
  - JP1: probably the 120 Ohm termination (unverified).
- Class: `SerialPort`.
- Direction switching is automatic, so there is no DE pin to drive.
- Unverified: whether TXD/RXD are named from the host's side or the transceiver's. If no data
  arrives, swap them. Local echo of transmitted bytes is possible.
```cpp
prosdk::SerialPort rs485{115200, prosdk::SerialPort::Port::ONBOARD};  // RX 10, TX 11
```

**RS485 5V transceiver** (RS485_5V_Transceiver_Breakout_Board)
- Same pinout and class as the 3V3 board, with a 5 V supply.
- **RXD is a 5 V output and needs a divider** before the ESP32-S3 RX pin.

**ATA6561 CAN transceiver** (ATA6561_CAN_Transceiver_Breakout_Board, repo only)
- Pins:
  - J1: 1 5V, 2 STB, 3 TXD, 4 RXD, 5 GND.
  - J2: CANL, GND, CANH.
  - JP1: termination.
  - S1: VIO select. **Set it to 3.3 V.**
- STB high or floating means standby, so drive it **LOW** to talk.
- Classes: `CAN` + `DigitalOutput` (STB).
```cpp
prosdk::platform::SetCanConfig(/*tx=*/4, /*rx=*/5, 500000);
prosdk::DigitalOutput stb{6};
stb.Set(false);
prosdk::CAN can{prosdk::CANPort::CAN_S0, /*deviceId=*/1};
```

**TPT71050 isolated CAN FD transceiver** (TPT71050_Isolated_CAN_FD_Transceiver_Breakout_Board)
- Pins:
  - J1: 1 VCC1 (3.3 V), 2 TX (to the host TWAI TX), 3 RX, 4 GND1.
  - J2: VCC2 (5 V), GND2.
  - J3: CANL, CANH.
  - JP1: termination.
- Class: `CAN`, with `SetCanConfig` as for the ATA6561 above.
- TWAI is **classic CAN only**: the transceiver's CAN FD rates are not used.
- Real isolation needs a separate isolated 5 V supply on J2. Feeding J2 from the robot's 5 V
  defeats the isolation.

**CY7C65213 USB-UART bridge** (CY7C65213_USB_to_UART_Breakout_Board)
- Pins:
  - J5: 1 VCC (5 V), 2 TX (bridge output, to the ESP RX), 3 RX, 4 GND.
  - J4: the VCCIO jumper. **It must be on 3V3.**
- Class: `SerialPort`. This makes a PC link over UART1 or UART2.
- The default is 115200 8N1; the PC side sets the baud rate.
- The HAL has no RTS/CTS flow control.
```cpp
prosdk::platform::SetSerialPins(1, /*rx=*/17, /*tx=*/18);
prosdk::SerialPort pc{115200, prosdk::SerialPort::Port::MXP};
```

### 2.5 Radio modules (user serial devices; not the robot link)

The robot's Driver Station link is probot-core's WiFi AP. These modules are only extra serial
devices for user code. A second 2.4 GHz radio near the robot can interfere with the AP.

| Board | Pins | Class / protocol | Hazards |
|---|---|---|---|
| FC41D (Quectel_FC41D_Wi-Fi_And_Bluetooth_Module_Breakout_Board) | J4: 1 VIN 5 V, 2 RX (from host TX), 3 TX, 4 RST, 5 WKP, 6 GND | `SerialPort`, AT commands, 115200 (unverified); `DigitalOutput` RST | The SPI and I2C headers are not supported. It duplicates the core WiFi. |
| ESP-01M (ESP-01M_Wi-Fi_Module) | 1 5V, 2 RST, 3 TX, 4 RX, 5 IO0 (high = run), 6 GND | `SerialPort`, AT firmware, 115200 | About 300 mA peaks. RF interference with the core AP. The firmware version is unverified. |
| MDBT50Q (MDBT50Q_Bluetooth5.2_Module) | J1: 1 3V3, 2 UART_TX, 3 UART_RX, 4 CTS, 5 RTS, 6 GND. J2: ADC, UART_PD, FLASH_DEFAULT, RESET, WAKEUP | `SerialPort` 9600 8N1; `DigitalOutput` UART_PD **driven LOW** | Wait 250 ms between commands; AT+RESET saves the settings. No flow control in the HAL. It is a bare nRF52833, so the behaviour depends on the firmware. |
| MDBT42TV (MDBT42TV_AT_Bluetooth_5_1_Module_Breakout_Board) | J1: 1 3V3, 2 RX, 3 TX, 4 RESET, 5 PD (**drive LOW**), 6 GND. J2: FLASH DEFAULT, WAKEUP | `SerialPort` 9600, AT | Peripheral role only. No flow control. |
| RFM6601W (RFM6601W_LoRa_Wireless_Breakout_Board) | J1: 1 GND, 2 TX, 3 RX, 4 SWD CLK, 5 SWD DATA, 6 RESET (active low), 7 VDD 3.3-5 V | `SerialPort` 9600, LoRaWAN AT (CJOIN/DTRX); `DigitalOutput` RESET | The AT command set is unverified, and so is the TX/RX direction. +2 dBm, EU868. Low data rate: not a control link. |

```cpp
prosdk::SerialPort ble{9600, prosdk::SerialPort::Port::ONBOARD};
ble.Write(std::string_view{"AT\r\n"});
```

### 2.6 LEDs and buses

**WS2812B 8x8 matrix and the flexible 8x8** (8x8_ARGB_Display_WS2812B_LED_Matrix..., 8x8_ARGB_Flexible...)
- Pins: VDD, DI (DO on the output side), GND.
- Class: `AddressableLED`. Each board has 64 LEDs, out of the 1024 shared across 3 outputs.
  Colour order is GRB.
- Index: row-major, y * 8 + x, according to the examples. Whether the rows run in a serpentine
  is unverified.
- Hazards:
  - Full white on 5 V draws about 2.3 A per board. Power the matrix from the 5 V rail, not
    from the controller.
  - A 3.3 V data line is below VIH (0.7 x VDD) when VDD is 5 V. Use a 74AHCT125 level shifter.
```cpp
prosdk::AddressableLED matrix{42};
matrix.SetLength(64);
std::array<prosdk::AddressableLED::LEDData, 64> px{};
matrix.SetData(px);
```

**I2C hub** (I2C_Hub_I2C_Bus_Distribution_Module, repo only)
- Pins: 1 VCC, 2 SCL, 3 SDA, 4 GND on each port. Note the order: SCL comes before SDA.
- It is passive and has no pull-ups. Every device on it must have a unique address.
- Class: `I2C` (or the device class) on one port.

### 2.7 Power boards with an enable pin

The EN pins below can stay on their jumpers, which needs no code. To switch a rail from the
robot, use `DigitalOutput`.

| Board | Pins | Use | Hazards |
|---|---|---|---|
| TPS630702 (TPS630702_Buck-Boost_Converter_Breakout_Board) | J1: VIN 2-16 V, GND. J2: PG (open-drain), GND. J3: VOUT, GND. JP1 EN (default HIGH). JP3 VSEL | Remove JP1 to drive EN from a `DigitalOutput`. Read PG with `DigitalInput` (internal pull-up). | JP3 selects 3.3 V or the 5/7.5/9 V pads. **Never feed 7.5 or 9 V into the ESP32-S3.** The PG pull-up rail is unverified. |
| TPS63030 (TPS63030_Buck-Boost_Converter_Breakout_Board) | J3 EN jumper (1 MOhm pull-up to VIN). VIN 1.8-5.5 V; fixed 3.3 V output | Remove the jumper to drive EN from a `DigitalOutput`. | The 1 MOhm pull-up goes to VIN (up to 5.5 V). Through 1 MOhm that is only microamps into a 3.3 V GPIO, which is not expected to harm it (unverified). Drive the pin as an output, LOW to disable. |
| TPS63020 eval (TPS63020_Buck_Boost_Converter_Evaluation_Board) | J2 EN jumper. J3 PS/SYNC. JP1-5 select VOUT | Drive EN from a `DigitalOutput` (VIH 1.2 V). | The 5 V output setting must not feed a 3.3 V rail. PG is not broken out. |
| TPP60308 (TPP60308_60V_Buck_Breakout_Board, repo only) | J1: 1 VCC 5-60 V, 2 EN (active high, internal pull-up, abs max 8.4 V), 3 CLK, 4 GND. J2: 5 V out, GND | `DigitalOutput` on EN, or leave it open (enabled). | The input can be up to 60 V. A 10k/13k EN divider is possible but unverified. **Measure the EN level before wiring a GPIO.** |

```cpp
prosdk::DigitalOutput railEnable{10};
railEnable.Set(true);
```

### 2.8 Audio amplifiers

prosdk has no audio source. The ESP32-S3 has no DAC, and `DigitalOutput` PWM tones would change
the one shared LEDC rate that the motor drivers use (G10). So the audio comes from an external
source. Tones are only an option on a robot with no LEDC motor drivers.

**FM8002A** (FM8002A_Audio_Amplifier_Module)
- Pins:
  - J1: 1 VDD, 2 AUDIO IN, 3 SHUTDOWN, 4 GND.
  - J2: the speaker.
- SHUTDOWN has a 10 kOhm pulldown. HIGH turns the amplifier off; it takes 100 ms to wake.
- Class: `DigitalOutput` (SHUTDOWN).
- Unverified: whether 3.3 V counts as high when VDD is 5 V.
- The output is BTL: **never ground either speaker lead.**

**TPA3111** (TPA3111_Mono_Audio_Amplifier_Breakout_Board, repo only)
- J3: PVCC 8-28 V (26 V recommended maximum).
- SD is pulled up to PVCC: **never connect a GPIO directly**. Drive SD low through an NPN
  transistor or NMOS, using `DigitalOutput` on its gate or base.
- FAULT: an open-drain output on the JP5 pins with no pull-up on the board, so read it with
  `DigitalInput` (internal pull-up).
- Gain is set by jumpers; the default is 36 dB.

## 3. Boards with the `doc_only` status

### 3.1 Pulse S32-S3 (ESP32-S3_Pulse_Development_Board): the Probot main board

- Status: the Probot main board and the default prosdk target (Tuna, 2026-10-23, "for now"). The
  `prosdk::platform` bus defaults are its J3/J4/J5 headers, the sketch FQBN is
  `esp32:esp32:esp32s3:FlashSize=4M,PartitionScheme=huge_app,PSRAM=enabled,CDCOnBoot=cdc`
  (README.md "Board", design/hal-esp32.md "Boards"). Not yet run on a board.
- Module: ESP32-S3-WROOM-1-N4R2, with 4 MB flash and 2 MB **quad** PSRAM. GPIO 33-37 are
  therefore free; only 26-32 are flash/PSRAM. The README's "Technical Specifications" says
  "Flash Memory: 8MB", but its "Model" line and the module datasheet in the same repo (ordering
  table: N4R2 = 4 MB quad flash, 2 MB quad PSRAM) say 4 MB. The FQBN uses FlashSize=4M, which
  works either way; `esptool.py flash_id` on the first board settles it.
- USB-C: native USB on GPIO 19/20 (README: "Power + Programming"; no USB-UART bridge chip is
  visible on the board render). Unverified: there is no schematic; `lsusb` on the first board
  should show Espressif 303a:1001 (USB-Serial-JTAG). Also to check on the first board: core's
  `Serial` and prosdk's stderr both reach USB-C without interleaving or loss, and core keeps
  running (DS timeout stop included) with a serial monitor opened and then closed while the cable
  stays in: with the 5 ms TX timeout (core #46 and the bridge, Arduino-ESP32 3.3.9 or later) a
  stalled write should return within about 100 ms, and with a reading host no text should be lost
  (design/hal-esp32.md "Boards").
- Source: the repo README and the board silkscreen (Pulse_S32-S3_Front/Back.png). There is no
  schematic in the repo.

| Header | Pins (1 -> n) | prosdk use |
|---|---|---|
| J1 BAT | BAT-, BAT+ | Battery input. The battery voltage pin is unknown, so no ADC reading. |
| J3 I2C | 1 GND, 2 IO2 = SCL, 3 IO1 = SDA, 4 +3V3 | I2C port 0 default (SDA 1, SCL 2): no `SetI2CPins` needed. |
| J4 UART | 1 GND, 2 TX IO43, 3 RX IO44, 4 IO11 (F2), 5 IO10 (F1), 6 +3V3 | IO43/44 are UART0, the console, and the HAL refuses them. `Port::ONBOARD` defaults to RX 10 (F1) / TX 11 (F2): no `SetSerialPins` needed. |
| J5 SPI | 1 GND, 2 IO17 CLK, 3 IO7 CS, 4 IO16 MISO, 5 IO15 MOSI, 6 +3V3 | SPI bus 0 default (SCK 17, MOSI 15, MISO 16): no `SetSPIPins` needed; `SPI(SPI::Port::kPort0, /*cs=*/7)`. |
| J6 | GND, IO38, IO37, IO36, IO35, +3V3 | GPIO (35-37 are free on quad PSRAM). |
| J7 | GND, IO42, IO41, IO40, IO39, +3V3 | GPIO. These are also the JTAG pins. |
| J8 | GND, IO45, IO46, IO47, IO48, +3V3 | GPIO. IO45/IO46 are strapping pins: do not drive or load them at boot. |
| On board | RGB LED on IO3; BOOT button on IO0; USB-C native on 19/20 | IO3 is probot-core's `NEOPIXEL_PIN` (3), so core drives this LED and the HAL refuses the pin. IO0 is a strapping pin. 19/20 are refused. |

- ADC1 pins on the headers: IO1, IO2, IO7 and IO10.
- Using IO1/IO2 for I2C leaves IO7 and IO10 for analog.
- probot-core's INA battery sensor (`PROBOT_BATTERY_INA`) starts `Wire` on SDA 1 / SCL 2 (J3) by
  default since core #46 (core request R21-a), and prosdk I2C port 0 shares that bus. A core
  before #46 starts it on the variant pins 8/9 unless `PROBOT_BATTERY_INA_SDA`/`SCL` are set; 8/9
  are not on a header, so with such a core define `PROBOT_BATTERY_INA_SDA 1` and
  `PROBOT_BATTERY_INA_SCL 2` (prosdk's `CoreBridge.hpp` warns otherwise).

### 3.2 Coprocessors and closed boards

| Board | Use from the robot | Notes |
|---|---|---|
| Data Logger (Data_Logger_Multi_Sensor_Logging_Module) | A separate MCU. User firmware sends the data over UART on its J5 (IO16/IO17), which the robot reads with `SerialPort`. | Classic ESP32-WROOM-32D with a BNO055 (0x28), MS5611 (on SPI per the schematic), SAM-M8Q and SD. Supply 3-16 V. **Do not master its J4**: that is the board's own I2C bus. |
| Blue Pill (STM32F103) | Coprocessor over `SerialPort` or I2C, or over `CAN` with a transceiver on each side. | 3.3 V logic. The protocol is user firmware. |
| Black Pill (STM32F411) | Coprocessor over `SerialPort` or I2C. | The F411 has no CAN. 3.3 V logic. |
| NSP2340 VoiceIC (NSP2340T16L_VoiceIC_Development_Board) | None. | J1 I2C: VCC, SCL, SDA, GND (level shifted). J3: BP02, INT. J4: speaker (BTL). The I2C address and command protocol are not public. |
| VL53L7CX eval (VL53L7CX_Multizone_ToF_Depth_Sensing_Eval_Board) | None. | An STM32 with closed firmware. J1: GND, SWCLK, SWDIO, TX, RX, GND. Power is USB-C only. The UART protocol is unknown. Use the VL53L7CX-1..4 boards (plan P4) instead. |
| E-paper 24-pin driver (E_Paper_Driver_24_Pin_E_Paper_Display_Driver_Breakout_Board) | None now. A future SPI driver could follow F1. | J1: 1 3.3V, 2 GND, 3 DIN, 4 SCK, 5 nCS, 6 D/C, 7 RST, 8 BSY. The board has no controller, so the commands depend on the panel. JP1 bridges BS to GND (4-wire SPI). |
| MP3 player (Ozdisan 1202713) | None. | See 3.3. |

### 3.3 MP3 player: chip identification result

- Sources checked:
  - The Ozdisan product page for 1202713. It says "No Datasheet" and gives no part numbers.
  - The two product images (`BOARDOZA MP3 PLAYER.jpg` and its thumbnail).
- No Boardoza repo exists for this board.
- What the image shows:
  - It is a **3D CAD render, not a photograph**, and the chip bodies carry no markings.
  - A 24-pin SSOP, which is probably the decoder.
  - An 8-pin SOIC, which is probably the audio amplifier.
  - Two 8-pin headers, passives, and a card slot or connector at the bottom edge.
- **Result: the chip is not identified, so the board stays doc_only.**
- A 24-pin SSOP decoder with an SOIC-8 amplifier matches the DFPlayer-class layout (YX5200 or
  a similar part), but this is **unverified**. It needs the marking read from a physical board.
- If a physical board shows a YX5200, JQ8400 or similar UART player, document it as a
  `SerialPort` device (9600 8N1 is the usual DFPlayer default, unverified) and add a row to the
  board index.

## 4. Boards with the `not_applicable` status

These boards have no signal for prosdk to control. They are listed here so that the board index
is complete. Where a regulator board has a VOUT sense pad, its rail voltage can be read with an
`AnalogInput` through a divider. That is optional, and no class is needed.

| Board(s) | Kind | Reason no class |
|---|---|---|
| AMS1117-3V3, AMS1117-5V | LDO | No control pin |
| SPX1117-ADJ, SPX1117-3V3 | LDO | No control pin |
| NCP3334A-3V3, NCP3334A-5V | LDO | No control pin |
| TPS70302 | Dual-output LDO (3.3 V + 1.2/1.8 V) | No control pin in the README |
| TPS563203-5V, TPS563203-3V3 | Buck | No control pin |
| TPS62130A-3V3 | Buck | No control pin |
| TPS63020-5V, TPS63020-3V3 (Ozdisan only) | Fixed buck-boost | No documentation; no control pin known |
| XL6019, MT3608, MC34063 | Boost | No control pin |
| XL6007 | Symmetrical supply | No control pin |
| TC7660S-Neg, TC7660S-Sym | Charge pump | No control pin |
| TP4056, MCP73831 | Li-ion charger | Standalone. The status LEDs could be read as open-drain inputs, but this is not documented by the board (unverified). |
| BQ24650 (Ozdisan only) | MPPT solar charger | Standalone |
| M032KG-EDK, NuMaker-M032KG, NuDuo-M4 | Nuvoton MCU boards | A separate MCU. It can only be a user coprocessor over `SerialPort` or I2C, with user firmware. |
| Chameleon (5 Ozdisan ids) | MCU main board + 4 MCU cards | Same as above |
| Boarduino R3, R3-C, R4 Minima | Arduino-style MCU boards | Same as above. Their 5 V I/O needs level shifting. |
| LoRa Omni 5.8 dBi, LoRa Yagi 9 dBi | Antennas | Passive |
| FFC/FPC adapter | Cable adapter | Passive |

## 5. Verified and unverified facts

Verified from datasheets and schematics:
- INA250A1: 200 mV/A and VOUT = I x G + VREF (SBOS511C 7.4.1 eq. 1, Table 7-1). REF has no
  tie on the Boardoza board; it goes to J1 pin 2 only (schematic).
- ACS758LCB-050B: 40 mV/A and zero at VCC/2 at VCC = 5 V. Ratiometry is specified for VCC
  4.5-5.5 V. RLOAD at least 4.7 kOhm.
- VNH5019 boards:
  - Both: the 1 kOhm CS resistor, the EN pull-ups, and the 1 kOhm series resistors (PCB
    netlist).
  - The Enc Elite J1 pin 1 is VIN.
- ADXL345: 0x53 with SDO grounded, 0x1D with SDO high (datasheet).
- SM351LT: push-pull output, LOW with a magnet present (Honeywell datasheet); no inverter on
  the board (schematic).
- ATA6561: STB high means standby (datasheet).
- Pulse header map: README and silkscreen. Not a schematic.

Unverified:
- The ACS758 sensitivity below VCC = 4.5 V (the preset scales it ratiometrically).
- The MP3 player chip.
- The ADXL345 board's SDO strap, CS pull and I2C pull-ups.
- The PC817C 4CH resistor values; the 1.5 kHz figure.
- The RS485 TX/RX naming direction; the JP1 termination.
- The FC41D, ESP-01M and RFM6601W baud rates and AT command sets.
- The TPS630702 PG pull-up rail; the TPP60308 EN divider.
- Whether FM8002A SHUTDOWN accepts a 3.3 V high when VDD is 5 V.
- The DutyCycle resolution at 20 kHz (analog-to-PWM).
- The WS2812 matrix row order.
- The VNH5019 channel C meaning.
- Whether the Pulse I2C pins clash with the probot-core battery I2C pins.
- The AnalogCurrentSensor test ran on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`):
  `[analogcurrent]` passes 28 assertions in 6 test cases. Nothing has run on hardware.

## 6. Shared edits for the integration step

- `design/drivers.md`: add `AnalogCurrentSensor` to the hand-written drivers list, with a link to
  `design/boardoza/P14-existing.md`.
- `design/API-DIFFERENCES.md` section 7: add one row, `prosdk::AnalogCurrentSensor`, described
  as "prosdk addition (no WPILib equivalent); linear analog current sensor with INA250A1 /
  ACS758LCB-050B presets".
- `design/boardoza.md` board index: one row per board from sections 2-4 of this file, linking
  here.
- Optional: the comment in `drivers.cmake` that lists the hand-written drivers, if it names each
  one. The sources are picked up by the glob.
