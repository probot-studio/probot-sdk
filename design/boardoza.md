# Boardoza boards in prosdk

Boardoza makes breakout boards that Ozdisan sells and that are published as open hardware
(`github.com/Boardoza`). This file is the index: every board, the prosdk class that drives it or
the reason there is none, the shared resource budget, and the 5 V hazards. The details of each
driver (wiring, API, behaviour, what is verified) are in the package documents under
`design/boardoza/`.

**No Boardoza board has been tested on hardware.** Every driver was written from datasheets,
board schematics and READMEs, and tested on the host simulation with fake devices. Facts that
could not be checked are marked unverified in the package documents.

## Packages

| Package | Content |
|---|---|
| [F1](boardoza/F1-spi.md) | `prosdk::SPI` (WPILib 2026 `frc::SPI` shape), ESP32 `spi_master` backend, host sim |
| [F2](boardoza/F2-sim-i2c.md) | Sim I2C: device address and per-address NACK injection (HAL patch 0004), `FakeI2CBus` |
| [F3](boardoza/F3-timed-io.md) | Timed DIO (`DigitalShift`, `WriteWaveform`) and RMT pulse I/O (`PulseTransmitter`, `PulseCapture`) |
| [P1](boardoza/P1-climate.md) | `LM75A`, `HIH8000`, `ENS210`, `MS5611`, `DPS310`, `BME688` |
| [P2](boardoza/P2-light-thermal.md) | `OPT3001`, `OPT4048`, `CaliPile`, `MLX90640`, `MLX90641` (Melexis API vendored) |
| [P3](boardoza/P3-imu.md) | `BNO055`, `BMI088`, `LSM303AGR` |
| [P4](boardoza/P4-tof.md) | `VL53L7CX` (ST ULD vendored) |
| [P5](boardoza/P5-i2c-misc.md) | `DRV2605L`, `PCF8563`, `GP8403`, `I2CSwitch`, `NT3H2111` |
| [P6](boardoza/P6-loadcell.md) | `NAU7802`, `HX711`, `LoadCell` |
| [P7](boardoza/P7-gnss.md) | `UbloxGnss`, `NmeaGnss`, `GnssFix`, NMEA and UBX parsers |
| [P8](boardoza/P8-motors.md) | `TwoPinHBridge` (`L9110S`, `BA6208`), `VNH7070AS`, `DRV8307` |
| [P9](boardoza/P9-storage.md) | `W25QFlash`, `W25NFlash`, `platform::MountSdCard` |
| [P10](boardoza/P10-tmag-leds.md) | `TMAG5170`, `ClockedAddressableLED` |
| [P11](boardoza/P11-lora.md) | `SX126x` |
| [P12](boardoza/P12-pulse.md) | `IRTransceiver`, `OokReceiver`, `PYD1598` |
| [P13](boardoza/P13-eve.md) | `FT8xx` and panel presets |
| [P14](boardoza/P14-existing.md) | `AnalogCurrentSensor`; how to use the boards that existing classes drive; doc-only and not-applicable boards |
| [P15](boardoza/P15-drv8317h.md) | `DRV8317H` on the platform six-step HAL extension (MCPWM, Hall interrupts) |

Examples: `examples/BoardozaClimate`, `BoardozaThermalCamera`, `BoardozaMultizoneToF`,
`BoardozaLoadCell`, `BoardozaGnss`, `BoardozaSdLog`, `BoardozaRotaryArgb`, `BoardozaLoRa`,
`BoardozaEveDisplay` (design/examples.md).

## Board table

Support:

- **driver**: a new prosdk class for this chip.
- **wrapper**: a thin prosdk class or function over an existing one.
- **existing**: existing prosdk classes drive it; P14 shows how.
- **doc_only**: no prosdk class; the reason is given, and P14 documents the board.
- **not_applicable**: nothing for software to control (power, MCU, antenna or passive boards).

Ozdisan ids are the catalog's product ids; "-" means the board has no Ozdisan entry (repo only).
A repo "-" means the board has no Boardoza repo (Ozdisan only). I2C addresses are 7-bit.

| Board | Ozdisan ids | Repo (`Boardoza_` prefix omitted) | Support | prosdk class, or the reason there is none | Note | Doc |
|---|---|---|---|---|---|---|
| LM75ADP | 1065559 | LM75ADP_Temperature_Sensor_Breakout_Board | driver | `prosdk::LM75A` | I2C 0x48-0x4F | [P1](boardoza/P1-climate.md) |
| HIH8131 | 1202709 | - | driver | `prosdk::HIH8000` | I2C 0x27 (chip default) | [P1](boardoza/P1-climate.md) |
| TH09C | 1206510 | TH09C_Temperature_Humidity_Sensor_Breakout_Board | driver | `prosdk::ENS210` | I2C 0x43, CRC-7 | [P1](boardoza/P1-climate.md) |
| MS5611 | 1202433 | MS5611_Barometric_Pressure_Sensor_Breakout_Board | driver | `prosdk::MS5611` | I2C 0x77/0x76 | [P1](boardoza/P1-climate.md) |
| HP303B | 1473941 | HP303B_Barometric_Pressure_Sensor_Breakout_Board | driver | `prosdk::DPS310` | I2C 0x77 (SDO floating/high), 0x76 (SDO low); ID reg 0x0D reads 0x10 (HP303B datasheet 8.10, verified) | [P1](boardoza/P1-climate.md) |
| BME688 | 1595256 | BME688_Environmental_Sensor_Breakout_Board | driver | `prosdk::BME688` | I2C 0x77/0x76; gas heater profile | [P1](boardoza/P1-climate.md) |
| OPT3001 | 1590925 | OPT3001_Ambient_Light_Sensor_Breakout_Board | driver | `prosdk::OPT3001` | I2C 0x44-0x47 | [P2](boardoza/P2-light-thermal.md#opt3001-no-thread) |
| OPT4048 | 1598828 | OPT4048_High_Precision_Color_Sensor_Breakout_Board | driver | `prosdk::OPT4048` | I2C 0x44-0x47; device ID 0x0820 or 0x0821 accepted | [P2](boardoza/P2-light-thermal.md#opt4048-no-thread) |
| TPiS-1S-1385 | 1473945 | Excelitas_TPiS_1S_1385_IR_Presence_Motion_Detection_Breakout | driver | `prosdk::CaliPile` | I2C 0x0C-0x0F (A0, A1); general-call reload at 0x00 before any access (G9, unverified on hardware) | [P2](boardoza/P2-light-thermal.md#calipile-no-thread) |
| TPiS-1T-1386 | 1202719 | TPiS_1T_1386_Non_Contact_Temperature_Sensor_Breakout_Board | driver | `prosdk::CaliPile` | I2C 0x0C (unverified, could be 0x0E); same general-call reload as 1S | [P2](boardoza/P2-light-thermal.md#calipile-no-thread) |
| MLX90640 | 1590924 | MLX90640_Thermal_Camera_Breakout_Board | driver | `prosdk::MLX90640` | I2C 0x33 at 400 kHz; Melexis API (Apache-2.0); reader thread | [P2](boardoza/P2-light-thermal.md#mlx90640-and-mlx90641-reader-thread) |
| MLX90641 | - | MLX90641_Thermal_Camera_Breakout_Board | driver | `prosdk::MLX90641` | repo only; I2C 0x33 at 400 kHz; Melexis API (Apache-2.0); reader thread | [P2](boardoza/P2-light-thermal.md#mlx90640-and-mlx90641-reader-thread) |
| BNO055 | 1206508 | BNO055_Absolute_Orientation_Sensor_Breakout_Board | driver | `prosdk::BNO055` | I2C 0x28/0x29; heading negated (CCW+) | [P3](boardoza/P3-imu.md#1-prosdkbno055) |
| BMI088 | 1598825 | BMI088_6-Axis_IMU_Breakout_Board | driver | `prosdk::BMI088` | I2C only: short JP1 (board default is SPI); accel 0x19 / gyro 0x69, or 0x18 / 0x68 with JP2 shorted | [P3](boardoza/P3-imu.md#2-prosdkbmi088) |
| LSM303AGR | 1591109 | LSM303_Triple_Axis_Accelerometer_Magnetometer_Breakout_Board | driver | `prosdk::LSM303AGR` | accel 0x19, mag 0x1E | [P3](boardoza/P3-imu.md#3-prosdklsm303agr) |
| VL53L7CX-1 | 1577675 | VL53L7CX-1_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board | driver | `prosdk::VL53L7CX` | I2C 0x29 (SetAddress to move); ST ULD (BSD-3-Clause); boards -1..-4 differ only in supply and level shifting | [P4](boardoza/P4-tof.md) |
| VL53L7CX-2 | 1577676 | VL53L7CX-2_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board | driver | `prosdk::VL53L7CX` | duplicate of -1 (3.3-5 V, SDA/SCL shifted) | [P4](boardoza/P4-tof.md) |
| VL53L7CX-3 | 1577677 | VL53L7CX-3_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board | driver | `prosdk::VL53L7CX` | duplicate of -1 (SDA/SCL/INT shifted) | [P4](boardoza/P4-tof.md) |
| VL53L7CX-4 | 1577678 | VL53L7CX-4_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board | driver | `prosdk::VL53L7CX` | duplicate of -1 (5 V supply, 3.3 V signals) | [P4](boardoza/P4-tof.md) |
| DRV2605L | 1564792 | DRV2605L_Haptic_Driver_Breakout_Board | driver | `prosdk::DRV2605L` | I2C 0x5A; pull-ups to VCC | [P5](boardoza/P5-i2c-misc.md#2-prosdkdrv2605l) |
| PCF8563 | - | PCF8563T_RTC_I2C_Breakout_Board | driver | `prosdk::PCF8563` | repo only; I2C 0x51 | [P5](boardoza/P5-i2c-misc.md#3-prosdkpcf8563) |
| GP8403 | 1595259 | GP8403_Dual_Channel_I2C_DAC_Breakout_Board | driver | `prosdk::GP8403` | I2C 0x58-0x5F; VOUT up to 10 V; pull-ups to VCC | [P5](boardoza/P5-i2c-misc.md#4-prosdkgp8403) |
| TPT29545 | 1591106 | TPT29545_I2C_Bus_Switch_Breakout_Board | driver | `prosdk::I2CSwitch` (TPT29545) | I2C 0x70-0x73 | [P5](boardoza/P5-i2c-misc.md#5-prosdki2cswitch) |
| NAU7802 | 1206517 | NAU7802_24bit_Dual_Channel_ADC_Breakout_Board | driver | `prosdk::NAU7802` + `prosdk::LoadCell` | I2C 0x2A | [P6](boardoza/P6-loadcell.md) |
| HX711 | 1206514 | HX711_Load_Cell_Amplifier_Breakout_Board | driver | `prosdk::HX711` + `prosdk::LoadCell` | DIO clock and data through DigitalShift (F3) | [P6](boardoza/P6-loadcell.md) |
| SAM-M8Q | 1206512 | Ublox_Sam_M8Q_GNSS_Module_Breakout_Board | driver | `prosdk::UbloxGnss` | UBX NAV-PVT over UART or I2C 0x42 | [P7](boardoza/P7-gnss.md#3-prosdkubloxgnss) |
| SAM-M10Q | - | SAM-M10Q_Multi_GNSS_Positioning_Breakout_Board | driver | `prosdk::UbloxGnss` | repo only; M10 config via CFG-VALSET | [P7](boardoza/P7-gnss.md#3-prosdkubloxgnss) |
| GP-02 | 1206509 | GP-02_GNSS_Satellite_Navigation_Breakout_Board | driver | `prosdk::NmeaGnss` | UART NMEA 0183 | [P7](boardoza/P7-gnss.md#4-prosdknmeagnss) |
| L9110S | 1564784 | L9110S_Dual_DC_Motor_Driver_Breakout_Board | driver | `prosdk::TwoPinHBridge` (alias L9110S) | 2 pins per motor on digital PWM | [P8](boardoza/P8-motors.md#l9110s-board-wiring) |
| BA6208 | 1065566 | BA6208_Motor_Driver_Breakout_Board | driver | `prosdk::TwoPinHBridge` (alias BA6208) | maximum PWM frequency unverified | [P8](boardoza/P8-motors.md#ba6208-board-wiring) |
| DRV8307 | 1473936 | - | driver | `prosdk::DRV8307` | Ozdisan only; internal Hall commutation; PWM 16-50 kHz (20 kHz default) | [P8](boardoza/P8-motors.md#3-prosdkdrv8307) |
| DRV8317H | 1473938 | DRV8317H_Three_Phase_BLDC_Driver_Breakout_Board | driver | `prosdk::DRV8317H` | Hall six-step on MCPWM (platform six-step API); MODE jumper J10 in 3x mode | [P15](boardoza/P15-drv8317h.md) |
| W25Q40 | 1206520 | W25Q40CLSNIG_SPI_Flash_Memory_Breakout_Board | driver | `prosdk::W25QFlash` | SPI NOR | [P9](boardoza/P9-storage.md#2-prosdkw25qflash) |
| W25N01GV | 1598830 | W25N01GVZEIG_Serial_SLC_NAND_Flash_Breakout_Board | driver | `prosdk::W25NFlash` | SPI NAND, raw page/block API; supply from 3.3 V | [P9](boardoza/P9-storage.md#3-prosdkw25nflash) |
| TMAG5170 | 1564797 | TMAG5170_3D_Hall_Effect_Sensor_Breakout_Board | driver | `prosdk::TMAG5170` | SPI 32-bit frames, CRC4; at 5 V the board drives MISO at 5 V | [P10](boardoza/P10-tmag-leds.md#2-prosdktmag5170) |
| Phoenix4 | 1564796 | Phoenix4_Modular_ARGB_LED_Breakout_Board | driver | `prosdk::ClockedAddressableLED` | 4 LEDs | [P10](boardoza/P10-tmag-leds.md#3-prosdkclockedaddressableled) |
| Phoenix6 | - | Phoenix6_Modular_ARGB_LED_Breakout_Board | driver | `prosdk::ClockedAddressableLED` | repo only; 6 LEDs | [P10](boardoza/P10-tmag-leds.md#3-prosdkclockedaddressableled) |
| Phoenix12 | 1564794 | Phoenix12_Modular_ARGB_LED_Breakout_Board | driver | `prosdk::ClockedAddressableLED` | 12 LEDs | [P10](boardoza/P10-tmag-leds.md#3-prosdkclockedaddressableled) |
| Phoenix24 | 1564800 | Phoenix24_Modular_ARGB_LED_Breakout_Board | driver | `prosdk::ClockedAddressableLED` | 24 LEDs | [P10](boardoza/P10-tmag-leds.md#3-prosdkclockedaddressableled) |
| Phoenix12-WENC | - | Phoenix12_WENC_ARGB_Encoder_Breakout_Board | driver | `prosdk::ClockedAddressableLED` + `prosdk::Encoder` + `prosdk::DigitalInput` (button) | repo only | [P10](boardoza/P10-tmag-leds.md#4-encoder-and-button-rotary-wenc-boards) |
| Phoenix24-WENC | - | Phoenix24_WENC_ARGB_Encoder_Breakout_Board | driver | `prosdk::ClockedAddressableLED` + `prosdk::Encoder` + `prosdk::DigitalInput` (button) | repo only | [P10](boardoza/P10-tmag-leds.md#4-encoder-and-button-rotary-wenc-boards) |
| RotaryEncoder-ARGB | - | Rotary_Encoder_ARGB_Control_Breakout_Board | driver | `prosdk::ClockedAddressableLED` + `prosdk::Encoder` + `prosdk::DigitalInput` (button) | repo only; 20 PPR | [P10](boardoza/P10-tmag-leds.md#4-encoder-and-button-rotary-wenc-boards) |
| E22-900M22S | - | E22-900M22S_Long_Range_LoRa_Breakout_Board | driver | `prosdk::SX126x` | repo only; SPI + BUSY + DIO1 + NRST, RXEN/TXEN | [P11](boardoza/P11-lora.md#2-prosdksx126x) |
| IrDA-38kHz | 1595255 | IrDA_Transceiver_Infrared_Wireless_Communication_Breakout_Board | driver | `prosdk::IRTransceiver` | RMT carrier TX and pulse capture (F3); NEC protocol | [P12](boardoza/P12-pulse.md#prosdkirtransceiver) |
| CMT2210LH | 1564803 | CMT2210LH_RF_Receiver_Breakout_Board | driver | `prosdk::OokReceiver` | RMT pulse capture (F3); 3.3 V only | [P12](boardoza/P12-pulse.md#prosdkookreceiver) |
| PYD1598 | 1473943 | - | driver | `prosdk::PYD1598` | Ozdisan only; wake-up mode (SERIN config, DL event); timing from the datasheet, unverified | [P12](boardoza/P12-pulse.md#prosdkpyd1598) |
| FT800 | 1473939 | FT800_TFT_Driver_Breakout_Board | driver | `prosdk::FT8xx` | SPI; display list and coprocessor; give it its own bus unless MISO release is verified | [P13](boardoza/P13-eve.md) |
| FT810 | 1473940 | FT810_TFT_Driver_Breakout_Board | driver | `prosdk::FT8xx` | SPI; as FT800 | [P13](boardoza/P13-eve.md) |
| FT800+TFT bundle | 1585930, 1585931, 1585933 | - | driver | `prosdk::FT8xx` (ft8xx::kBoardozaFT800Bundle) | 480x272 preset (unverified); no touch | [P13](boardoza/P13-eve.md) |
| FT800+TFT bundle, capacitive touch | 1585932 | - | driver | `prosdk::FT8xx` (ft8xx::kBoardozaFT800Bundle) | 480x272 preset (unverified); the capacitive touch is not supported | [P13](boardoza/P13-eve.md) |
| FT810+TFT bundle, resistive touch | 1585934, 1585936 | - | driver | `prosdk::FT8xx` (ft8xx::kBoardozaFT810Bundle) | 800x480 preset (unverified); resistive touch | [P13](boardoza/P13-eve.md) |
| FT810+TFT bundle | 1585935 | - | driver | `prosdk::FT8xx` (ft8xx::kBoardozaFT810Bundle) | 800x480 preset (unverified); no touch | [P13](boardoza/P13-eve.md) |
| FT810+TFT bundle, capacitive touch | 1585938 | - | driver | `prosdk::FT8xx` (ft8xx::kBoardozaFT810Bundle) | 800x480 preset (unverified); the capacitive touch is not supported | [P13](boardoza/P13-eve.md) |
| NT3H2111 | 1206521 | - | driver | `prosdk::NT3H2111` | Ozdisan only; I2C 0x55 (NXP datasheet rev 3.0, table 15); board pinout and I2C header unverified | [P5](boardoza/P5-i2c-misc.md#6-prosdknt3h2111) |
| VNH7070AS | 1598829 | VNH7070AS_Bidirectional_DC_Motor_Driver_Breakout_Board | wrapper | `prosdk::VNH7070AS` | VNH5019 semantics plus SEL0/CS; fPWM max 20 kHz; CS reaches 5-7 V in a fault | [P8](boardoza/P8-motors.md#2-prosdkvnh7070as) |
| MicroSD | 1202712 | MicroSD_Breakout_Board | wrapper | `prosdk::platform::MountSdCard` | SD over SPI, FAT16/FAT32; mount before other devices on the bus talk | [P9](boardoza/P9-storage.md#4-prosdkplatformmountsdcard) |
| INA250A1 | 1564788 | INA250A1_Precision_Current_Sense_Breakout_Board | wrapper | `prosdk::AnalogCurrentSensor` (INA250A1 preset) | 200 mV/A around REF, on an ADC1 pin | [P14 1](boardoza/P14-existing.md#1-prosdkanalogcurrentsensor-ina250a1-acs758lcb-050b) |
| ACS758 | 1598826 | ACS758LCB-050B_Hall_Effect_Current_Sensor_Breakout_Board | wrapper | `prosdk::AnalogCurrentSensor` (ACS758LCB050B preset) | 40 mV/A at VCC = 5 V, VCC/2 offset; a divider is needed at 5 V; at 3.3 V use CalibrateZero and a measured sensitivity | [P14 1](boardoza/P14-existing.md#1-prosdkanalogcurrentsensor-ina250a1-acs758lcb-050b) |
| MQ4 | 1065558 | MQ4_Methane_Gas_Sensor_Breakout_Board | existing | `prosdk::AnalogInput` / `prosdk::DigitalInput` | AO and DO are 5 V signals; the MQGasSensor helper is not built | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| Vibration-Motor | 1202717 | Vibration_Motor_Breakout_Board | existing | `prosdk::DigitalOutput` (EnablePWM for strength) | - | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| ADXL345 | 1473929 | ADXL345_Triple_Axis_Accelerometer_Breakout_Board | existing | `prosdk::ADXL345_I2C` (address 0x53) | board SDO low -> 0x53, not the class default 0x1D | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| BS801C | 1473948 | BS801C_Capacitive_Touch_Sensor_Breakout_Board | existing | `prosdk::DigitalInput` | - | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| SM351LT | 1206518 | SM351LT_Hall_Effect_Sensor_Breakout_Board | existing | `prosdk::DigitalInput` | push-pull output, active low: reads LOW with a magnet present | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| PC817C-2CH | 1564771 | PC817C_2CH_Isolated_Level_Shifter_Breakout_Board | existing | `prosdk::DigitalInput` / `prosdk::DigitalOutput` | - | [P14 2.2](boardoza/P14-existing.md#22-isolation-and-relays) |
| PC817C-4CH | 1564772 | PC817C_4CH_Isolated_Level_Shifter_Breakout_Board | existing | `prosdk::DigitalInput` / `prosdk::DigitalOutput` | - | [P14 2.2](boardoza/P14-existing.md#22-isolation-and-relays) |
| CPC1008N | 1564787 | CPC1008N_Solid_State_Relay_Breakout_Board | existing | `prosdk::DigitalOutput` | - | [P14 2.2](boardoza/P14-existing.md#22-isolation-and-relays) |
| CPC1945Y | - | CPC1945Y_AC_Solid_State_Relay_Breakout_Board | existing | `prosdk::DigitalOutput` | repo only; mains hazard | [P14 2.2](boardoza/P14-existing.md#22-isolation-and-relays) |
| Water-Sensor | 1591130 | Water_Sensor_Analog_Water_Level_Detection_Breakout_Board | existing | `prosdk::AnalogInput` | - | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| Analog-to-PWM | 1591110 | Analog_To_PWM_Converter_Breakout_Board | existing | `prosdk::DutyCycle` | about 20 kHz output; power it from 3.3 V | [P14 2.1](boardoza/P14-existing.md#21-sensors-and-simple-io) |
| VNH5019-Enc | 1567453 | VNH5019ATR_w-Enc_Elite_Breakout_Board | existing | `prosdk::VNH5019` + `prosdk::Encoder` + `prosdk::AnalogInput` (CS) | J1 pin 1 is VIN | [P14 2.3](boardoza/P14-existing.md#23-motor-drivers-vnh5019) |
| VNH5019-Module | - | VNH5019_H_Bridge_Motor_Driver_Module | existing | `prosdk::VNH5019` | - | [P14 2.3](boardoza/P14-existing.md#23-motor-drivers-vnh5019) |
| RS485-3V3 | 1474138 | RS485_3V3_Transceiver_Breakout_Board | existing | `prosdk::SerialPort` | no DE pin to drive | [P14 2.4](boardoza/P14-existing.md#24-serial-and-can-transceivers) |
| RS485-5V | 1474139 | RS485_5V_Transceiver_Breakout_Board | existing | `prosdk::SerialPort` | RXD is a 5 V signal | [P14 2.4](boardoza/P14-existing.md#24-serial-and-can-transceivers) |
| ATA6561 | - | ATA6561_CAN_Transceiver_Breakout_Board | existing | `prosdk::CAN` + `prosdk::DigitalOutput` (STBY) | repo only | [P14 2.4](boardoza/P14-existing.md#24-serial-and-can-transceivers) |
| TPT71050 | 1591107 | TPT71050_Isolated_CAN_FD_Transceiver_Breakout_Board | existing | `prosdk::CAN` + `prosdk::DigitalOutput` (STB) | TWAI is classic CAN only | [P14 2.4](boardoza/P14-existing.md#24-serial-and-can-transceivers) |
| CY7C65213 | 1564762 | CY7C65213_USB_to_UART_Breakout_Board | existing | `prosdk::SerialPort` | PC link over UART1/2 | [P14 2.4](boardoza/P14-existing.md#24-serial-and-can-transceivers) |
| FC41D | 1065569 | Quectel_FC41D_Wi-Fi_And_Bluetooth_Module_Breakout_Board | existing | `prosdk::SerialPort` (AT commands) | robot WiFi is the DS link; external module only as user serial device | [P14 2.5](boardoza/P14-existing.md#25-radio-modules-user-serial-devices-not-the-robot-link) |
| ESP-01M | 1202706 | ESP-01M_Wi-Fi_Module | existing | `prosdk::SerialPort` (AT commands) | - | [P14 2.5](boardoza/P14-existing.md#25-radio-modules-user-serial-devices-not-the-robot-link) |
| MDBT50Q | 1202711 | MDBT50Q_Bluetooth5.2_Module | existing | `prosdk::SerialPort` (firmware-dependent) | bare nRF52 module: needs its own firmware | [P14 2.5](boardoza/P14-existing.md#25-radio-modules-user-serial-devices-not-the-robot-link) |
| MDBT42TV | 1564767 | MDBT42TV_AT_Bluetooth_5_1_Module_Breakout_Board | existing | `prosdk::SerialPort` (AT commands) | - | [P14 2.5](boardoza/P14-existing.md#25-radio-modules-user-serial-devices-not-the-robot-link) |
| RFM6601W | 1202716 | RFM6601W_LoRa_Wireless_Breakout_Board | existing | `prosdk::SerialPort` (AT commands) | - | [P14 2.5](boardoza/P14-existing.md#25-radio-modules-user-serial-devices-not-the-robot-link) |
| WS2812-8x8 | 1591129 | 8x8_ARGB_Display_WS2812B_LED_Matrix_Breakout_Board | existing | `prosdk::AddressableLED` | 64 of the 1024 shared LEDs; data line needs a level shifter | [P14 2.6](boardoza/P14-existing.md#26-leds-and-buses) |
| WS2812-Flex-8x8 | 1591128 | 8x8_ARGB_Flexible_Display_WS2812B_Breakout_Board | existing | `prosdk::AddressableLED` | - | [P14 2.6](boardoza/P14-existing.md#26-leds-and-buses) |
| I2C-Hub | - | I2C_Hub_I2C_Bus_Distribution_Module | existing | `prosdk::I2C` (passive hub) | repo only | [P14 2.6](boardoza/P14-existing.md#26-leds-and-buses) |
| TPS630702 | 1206511 | TPS630702_Buck-Boost_Converter_Breakout_Board | existing | `prosdk::DigitalOutput` (EN) + `prosdk::DigitalInput` (PG, open drain, pull-up) or none | power board with EN and power-good; never feed the 7.5/9 V outputs to the ESP32-S3 | [P14 2.7](boardoza/P14-existing.md#27-power-boards-with-an-enable-pin) |
| TPS63030 | 1206513 | TPS63030_Buck-Boost_Converter_Breakout_Board | existing | `prosdk::DigitalOutput` (EN) or none | - | [P14 2.7](boardoza/P14-existing.md#27-power-boards-with-an-enable-pin) |
| TPS63020-Eval | 1564779 | TPS63020_Buck_Boost_Converter_Evaluation_Board | existing | `prosdk::DigitalOutput` (EN) or none | - | [P14 2.7](boardoza/P14-existing.md#27-power-boards-with-an-enable-pin) |
| FM8002A | 1590923 | FM8002A_Audio_Amplifier_Module | existing | `prosdk::DigitalOutput` (EN/SD pin) | no audio source in prosdk (no DAC; PWM tones would change the shared LEDC rate) | [P14 2.8](boardoza/P14-existing.md#28-audio-amplifiers) |
| TPA3111 | - | TPA3111_Mono_Audio_Amplifier_Breakout_Board | existing | `prosdk::DigitalOutput` (SD) | repo only; SD is pulled up to PVCC; same audio-source note as FM8002A | [P14 2.8](boardoza/P14-existing.md#28-audio-amplifiers) |
| TPP60308 | - | TPP60308_60V_Buck_Breakout_Board | existing | `prosdk::DigitalOutput` (EN) or none | repo only; EN active high; input up to 60 V, measure the EN level before wiring a GPIO | [P14 2.7](boardoza/P14-existing.md#27-power-boards-with-an-enable-pin) |
| SenseTech-RFM68LCW | - | SenseTech-Competition-2026 | doc_only | none until a hardware test shows that it answers the SX126x command set | SenseTech aggregate repo; its other folders are boards listed in their own rows | [P11](boardoza/P11-lora.md) |
| ESP32-S3-Pulse | 1473942 | ESP32-S3_Pulse_Development_Board | doc_only | board profile doc (pins, I2C/UART defaults) | it is a possible robot controller itself | [P14 3.1](boardoza/P14-existing.md#31-esp32-s3-pulse-esp32-s3_pulse_development_board-possible-robot-controller) |
| Data-Logger | 1473935 | Data_Logger_Multi_Sensor_Logging_Module | doc_only | SerialPort doc (separate MCU) | - | [P14 3.2](boardoza/P14-existing.md#32-coprocessors-and-closed-boards) |
| Blue-Pill | - | STM32F103_Blue_Pill_Development_Board | doc_only | coprocessor over SerialPort/I2C/CAN, doc | - | [P14 3.2](boardoza/P14-existing.md#32-coprocessors-and-closed-boards) |
| NSP2340-VoiceIC | 1202690 | NSP2340T16L_VoiceIC_Development_Board | doc_only | none: control protocol not public | - | [P14 3.2](boardoza/P14-existing.md#32-coprocessors-and-closed-boards) |
| VL53L7CX-Eval | 1577674 | VL53L7CX_Multizone_ToF_Depth_Sensing_Eval_Board | doc_only | none: closed STM32 firmware; use VL53L7CX-1..4 | - | [P14 3.2](boardoza/P14-existing.md#32-coprocessors-and-closed-boards) |
| MP3-Player | 1202713 | - | doc_only | none: chip not identified (Ozdisan render only, no datasheet) | Ozdisan only | [P14 3.3](boardoza/P14-existing.md#33-mp3-player-chip-identification-result) |
| E-Paper-24pin | 1595253 | E_Paper_Driver_24_Pin_E_Paper_Display_Driver_Breakout_Board | doc_only | none: the panel is unknown (an SPI driver is possible now that prosdk::SPI exists) | - | [P14 3.2](boardoza/P14-existing.md#32-coprocessors-and-closed-boards) |
| Black-Pill | 1564775 | STM32F411_Black_Pill_Development_Board | doc_only | coprocessor over SerialPort or I2C, doc (the F411 has no CAN) | - | [P14 3.2](boardoza/P14-existing.md#32-coprocessors-and-closed-boards) |
| AMS1117-3V3 | 1473930 | AMS1117-3V3_Voltage_Regulator_Breakout_Board | not_applicable | voltage regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| AMS1117-5V | 1473931 | AMS1117-5V_Voltage_Regulator_Breakout_Board | not_applicable | voltage regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TP4056 | 1473952 | TP4056_Lithium_Battery_Charger_Breakout_Board | not_applicable | battery charger | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| MCP73831 | 1206515 | MCP73831_Lithium_Battery_Charger_Breakout_Board | not_applicable | battery charger | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| BQ24650 | 1473934 | - | not_applicable | MPPT solar charger (in SenseTech repo only) | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TPS70302 | 1564780 | TPS70302_Dual_Output_Voltage_Regulator_Breakout_Board | not_applicable | dual-output LDO with no control pin (the Ozdisan id is matched by its outputs, inferred) | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| NCP3334A-3V3 | 1564768 | NCP3334A_3V3_Low_Dropout_Voltage_Regulator_Breakout_Board | not_applicable | LDO | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| NCP3334A-5V | 1564769 | NCP3334A_5V_Low_Dropout_Voltage_Regulator_Breakout_Board | not_applicable | LDO | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TC7660S-Neg | 1564776 | TC7660S_Negative_Doubler_Breakout_Board | not_applicable | charge pump | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TC7660S-Sym | - | TC7660S_Symmetrical_Voltage_Regulator_Breakout_Board | not_applicable | charge pump | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TPS563203-5V | 1564777 | TPS563203_5V_Buck_Regulator_Module | not_applicable | buck regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TPS563203-3V3 | - | TPS563203_3V3_Buck_Regulator_Breakout_Board | not_applicable | buck regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TPS62130A-3V3 | 1564778 | TPS62130A_3V3_3A_Buck_Converter_Breakout_Board | not_applicable | buck regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TPS63020-5V | 1580501 | - | not_applicable | fixed buck-boost (Ozdisan only) | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| TPS63020-3V3 | 1580502 | - | not_applicable | fixed buck-boost (Ozdisan only) | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| XL6019 | - | XL6019E1_DC_DC_Boost_Breakout_Board | not_applicable | boost regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| XL6007 | - | XL6007_Symmetrical_5V_Power_Supply_Module | not_applicable | symmetrical supply | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| MT3608 | 1595254 | MT3608_3V3_to_5V_Boost_Converter_Breakout_Board | not_applicable | boost regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| MC34063 | 1591108 | MC34063AD_DC_DC_Boost_Converter_Module | not_applicable | boost regulator | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| SPX1117-ADJ | - | SPX1117_Adjustable_Voltage_Regulator_Breakout_Board | not_applicable | LDO | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| SPX1117-3V3 | 1595258 | SPX1117-3V3_LDO_Regulator_Breakout_Board | not_applicable | LDO | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| M032KG-EDK | - | M032KG_Educational_Development_Kit | not_applicable | MCU education kit | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| NuMaker-M032KG | 1208247 | NuMaker_M032KG_Evaluation_Module | not_applicable | MCU evaluation board | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| Chameleon | 1563007, 1563008, 1563009, 1563010, 1563011 | - | not_applicable | MCU main board + 4 MCU cards | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| Boarduino | 1473946, 1473947, 1473944 | - | not_applicable | Arduino-style MCU boards R3, R3-C, R4 Minima | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| NuDuo-M4 | 1600090 | - | not_applicable | MCU board | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| LoRa-Omni-5.8dBi | 726441 | 5.8dbi_LoRa_Omni_Antenna | not_applicable | antenna | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| LoRa-Yagi-9dBi | 859867 | - | not_applicable | antenna | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |
| FFC-Adapter | - | FFC_FPC_Adapter_Flexible_Cable_Breakout_Board | not_applicable | passive cable adapter | - | [P14 4](boardoza/P14-existing.md#4-boards-with-the-not_applicable-status) |

Coverage, checked on 2026-10-22 with a script that reads this table: the Ozdisan Boardoza catalog
(114 product ids) and the Boardoza GitHub organisation (111 repos, `SenseTech-Competition-2026`
included) were both listed, and every id and every repo appears in exactly one row. The table
has 128 rows: 54 driver, 4 wrapper, 32 existing, 9 doc_only, 29 not_applicable. A row can carry
several ids (the FT8xx bundles, Chameleon, Boarduino), and several rows use one class (the
VL53L7CX-1..4 boards).

Corrections against the implementation plan, applied above:

- DRV8317H is a driver now (P15, `prosdk::DRV8317H`), not blocked.
- The RFM68LCW in the SenseTech repo stays doc_only until a hardware test shows that it answers the
  SX126x command set.
- SM351LT: push-pull output, LOW with a magnet present (the plan said open drain).
- NT3H2111: address 0x55 is from the NXP datasheet (rev 3.0, table 15); the board pinout and
  whether it exposes I2C are still unverified.
- MLX90640/41 run at 400 kHz, not 1 MHz. HIH8131 uses 0x27, the chip default. OPT4048 accepts
  device ID 0x0820 and 0x0821. TPiS 1T: 0x0C, unverified (0x0E is possible).
- The FT8xx bundles are split by panel preset and touch; capacitive touch is not supported.
- MQ4: the optional `MQGasSensor` helper is not built.

## Resource budget

The ESP32-S3 units that Boardoza drivers share with each other and with the rest of the HAL
(design/hal-esp32.md "Capacities"). Opening one more than the count fails with
`HAL_NO_AVAILABLE_RESOURCES` or a constructor error that names the unit.

| Resource | Count | Boardoza users | Rules |
|---|---|---|---|
| RMT TX channels | 3 (4 on the S3; probot-core's status LED uses one) | `AddressableLED` outputs (WS2812 boards), `PulseTransmitter`: `IRTransceiver` 1 (with a TX pin), `PYD1598` 1 (SERIN) | One counter for LED outputs and pulse transmitters together (`rmt::AcquireTxChannel`) |
| RMT RX channels | 4 | `PulseCapture`: `IRTransceiver` 1 (with an RX pin), `OokReceiver` 1 | |
| LEDC digital PWM generators | 6, **one rate for all** | `TwoPinHBridge` 2 per motor (the dual L9110S board 4), `VNH7070AS` 1, `DRV8307` 1, `BTS7960` 1-2, `VNH5019` 1, `DigitalOutput::EnablePWM` (vibration motor) | Every motor driver sets 20 kHz in its constructor, the VNH7070AS maximum and inside the DRV8307's 16-50 kHz. The last driver constructed sets the rate for all; anything that sets another rate (PWM tones) changes every motor's rate |
| PCNT units | 4, counters and encoders together | `Encoder` on the Rotary/WENC boards and the VNH5019 encoder board, `DRV8307` HALLOUT (optional) | |
| MCPWM timer/operator pairs | 6 (12 servo PWM outputs) | `DRV8317H`: 1 pair per motor, at most 2 motors | Each DRV8317H leaves 2 fewer of the 12 servo outputs |
| MCPWM capture channels | 6 | `DutyCycle` (Analog-to-PWM board) | |
| ADC1 pins | 10 (GPIO 1-10) | `AnalogCurrentSensor` (INA250A1, ACS758), `VNH7070AS` CS, `VNH5019` CS, MQ4 AO, water sensor | ADC2 is refused (WiFi) |
| I2C ports | 2 | every I2C board | Port 0 is `Wire`'s bus, which probot-core's battery sensor may use. Port 1 needs `SetI2CPins(1, ...)`. A driver with a reader thread must not sit behind an `I2CSwitch` |
| SPI buses | 2: `kPort0` = SPI2 (6 chip selects), `kPort1` = SPI3 (3) | `W25QFlash`, `W25NFlash`, SD card, `TMAG5170`, `SX126x`, `FT8xx`, `ClockedAddressableLED` | **A device without chip select must be alone on its bus**: the clocked LEDs take a whole bus (normally `kPort1`) and every CS device shares the other. Give `FT8xx` its own bus unless its MISO release is verified. Mount the SD card before other devices on its bus talk; it counts as one bus user |
| UARTs | 2 (`kOnboard` UART1, `kMXP` UART2) | GNSS over UART, RS-485, USB-UART, AT radio modules, coprocessors | |
| CAN | 1 (TWAI, classic CAN) | ATA6561, TPT71050 | No CAN FD |
| GPIO interrupts | the GPIO ISR service | `DRV8317H`: 3 Hall pins per motor | Shared with Arduino `attachInterrupt`; there is no interrupt HAL otherwise |

### I2C addresses

Addresses that collide on one port. Put the colliding boards on different ports, move one with
its address pins, or use the TPT29545 switch (`I2CSwitch`).

| Address | Boards |
|---|---|
| 0x00 (general call) | `CaliPile` (TPiS 1S/1T) sends the reload 0x00, 0x04 to every device on the port. Whether other boards react to it is unverified (G9) |
| 0x19 | BMI088 accelerometer (default), LSM303AGR accelerometer. Short BMI088 JP2 for 0x18 |
| 0x29 | VL53L7CX (default), BNO055 with ADR high. `VL53L7CX::SetAddress` moves a sensor |
| 0x33 | MLX90640, MLX90641 |
| 0x44-0x47 | OPT3001, OPT4048 |
| 0x5A | DRV2605L, GP8403 at A2-A0 = 010 |
| 0x76, 0x77 | MS5611, DPS310 (HP303B), BME688: at most two of them per port |

### Threads

Drivers with long frames or continuous reads start a thread through `ScopedThreadConfig` (prosdk
priority, below the probot-core loop). The default stack is `PROSDK_THREAD_STACK_SIZE` (4096
bytes).

| Thread | Driver | Stack | Notes |
|---|---|---|---|
| `bno055`, `bmi088`, `lsm303agr` | IMUs | default | `updatePeriod` 0: no thread, the caller calls `Update()` |
| `mlx90640` | `MLX90640` | 12 KiB | the Melexis API keeps 3 KiB arrays on the stack |
| `mlx90641` | `MLX90641` | 6 KiB | |
| `vl53l7cx` | `VL53L7CX` | 6 KiB | starts at the first call, uploads the 84 KiB firmware |
| `ublox-gnss`, `nmea-gnss` | GNSS | default | `updatePeriod` 0: no thread |
| `sx126x` | `SX126x` | default | polls BUSY and DIO1 every 2 ms |
| `ft8xx` | `FT8xx` | default | brings the chip up, feeds the command FIFO |

A robot that uses many of these at once has not been measured: the stack and priority budget
under probot-core is a hardware check (plan section 7).

### Memory

- `VL53L7CX`: the ST firmware image is about 84 KiB of flash in every sketch that uses the class.
- Pulse capture: about 5.5 KiB of static RAM for the 4 capture slots, used or not.
- SPI: a 4092-byte DMA-capable zero buffer on the first read-only segment.
- `FT8xx`: up to 2 x 8 KiB of heap for the frame under construction and the pending frame.
- The archive and sketch sizes have not been measured since these drivers were added
  (design/size.md).

## 5 V hazards

The ESP32-S3 pins are 3.3 V only. Many Boardoza boards accept 5 V and then put 5 V on a line that
reaches the controller. Rules, with the board facts from the package documents:

- **I2C pull-ups to VCC.** Most I2C boards pull SDA and SCL up to their supply pin (directly or
  through a level shifter), so powering them from 5 V puts 5 V on the controller's pins: TH09C,
  MS5611, BME688, TPiS 1S/1T, MLX90640/41, BNO055, BMI088, LSM303AGR, DRV2605L, GP8403, TPT29545,
  NAU7802, SAM-M8Q, SAM-M10Q and VL53L7CX-2/-3 (INT too on the -3). Power them from 3.3 V.
  The VL53L7CX-4 keeps its signals at 3.3 V on a 5 V supply.
- **Analog outputs.** MQ4 AO and DO are 5 V signals (AO through a divider, DO through a level
  shifter). The ACS758 at 5 V needs a divider. VNH7070AS CS reaches 5-7 V in a fault. GP8403
  VOUT goes up to 10 V: never wire it back to an ADC pin.
- **Data lines.** HX711 DOUT swings to its DVDD pin. The IR transceiver's RXD runs at VCC.
  RS485-5V RXD is 5 V. The GNSS boards' level shifters (GP-02 UART, SAM-M8Q/M10Q) may pull the host lines to 5 V. TMAG5170 at 5 V drives MISO at 5 V. W25N01GV: supply from 3.3 V (its
  TXB0106 may drive MISO at 5 V). The clocked LED boards drive DOUT/COUT at VDD, and the
  Rotary/WENC encoder and button pull-ups go to VCC. DRV8307 open-drain outputs may be pulled up
  to 5 V. WS2812 boards need a level shifter on the data line.
- **3.3 V only.** CMT2210LH. PYD1598: 3.6 V absolute maximum. HP303B. VL53L7CX-1.
- **5 V supply, 3.3 V signals.** MicroSD (5 V on pin 6, card lines at 3.3 V). FT8xx boards (5 V
  in; J1 pin 1, 3V3, is an output: never feed it). VL53L7CX-4.
- **Motor and power boards.** L9110S: R1-R4 pull the inputs up to the motor VCC. VNH5019 encoder
  board: J1 pin 1 is VIN. DRV8317H: Hall power in the "Ext" position can put 5 V on the Hall
  lines, and J3-2 is a 5 V input. TPA3111: SD is pulled up to PVCC. TPS630702: never feed its
  7.5 or 9 V output settings to the controller. TPP60308: input up to 60 V; measure the EN level
  before wiring a GPIO. CPC1945Y: mains voltage on the load side.

## Open points

- Hardware tests of every driver (none done). G9: whether the general call to 0x00 works on the
  ESP32-S3 I2C controller (both TPiS boards depend on it) and whether a zero-length write starts
  an HIH8131 measurement.
- Board facts marked unverified in the package documents: TPiS 1T address, FT8xx bundle panel
  timings and part numbers, NT3H2111 board pinout, BA6208 and L9110S PWM limits, MP3 player chip.
- The RFM68LCW: a hardware test against the SX126x command set.
- The thread, stack and I2C bus-time budget with several drivers running at once.
