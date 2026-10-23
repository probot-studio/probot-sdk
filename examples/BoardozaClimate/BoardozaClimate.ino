// BoardozaClimate: the Boardoza climate boards in a TimedRobot (design/boardoza/P1-climate.md).
// LM75ADP (temperature, prosdk::LM75A), HIH8131 (humidity and temperature, prosdk::HIH8000),
// TH09C (humidity and temperature, prosdk::ENS210), MS5611 and HP303B (barometers, prosdk::MS5611
// and prosdk::DPS310) and BME688 (gas, pressure, humidity, temperature, prosdk::BME688). Every
// reading goes to the Driver Station (/Telemetry). Use any subset of the boards.
//
// Power every board from 3.3 V (VCC to the controller's 3V3, GND to GND). The MS5611, TH09C and
// BME688 boards have level shifters whose host-side pull-ups go to VCC, so 5 V there puts 5 V on
// the ESP32-S3 pins, which are not 5 V tolerant. The HP303B board is 3.3 V only. The LM75ADP and
// HIH8131 board pull-ups are not documented; keep them at 3.3 V too.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   I2C port 1, SDA -> GPIO 17, SCL -> GPIO 16, 100 kHz:
//     LM75ADP  0x48 (address pads A0-A2 open)
//     HIH8131  0x27
//     TH09C    0x43 (fixed)
//     MS5611   0x77 (CSB low, the board default)
//     HP303B   0x76: short JP3 (SDO low) and tie CSB to 3.3 V. Its SDA/SCL pull-ups (JP1/JP2)
//              are open by default; the MS5611 and TH09C boards pull the bus up, alone short JP1
//              and JP2. (Jumper functions read from the schematic, unverified on a board.)
//   I2C port 0 (Wire's bus, GPIO 1 SDA / 2 SCL on J3 unless Wire runs elsewhere):
//     BME688   0x77 with ADR unconnected (board pull-up, unverified; ADR to GND gives 0x76).
//              Leave nCSB unconnected: a low level on it switches the chip to SPI.
// The MS5611, HP303B and BME688 can only use 0x76 and 0x77, so the third one needs the second bus.
//
// Readings start within a second of power-up; the BME688 gas resistance needs a few minutes of
// heater operation before it settles.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/environment/BME688.hpp>
#include <prosdk/drivers/environment/DPS310.hpp>
#include <prosdk/drivers/environment/ENS210.hpp>
#include <prosdk/drivers/environment/HIH8000.hpp>
#include <prosdk/drivers/environment/LM75A.hpp>
#include <prosdk/drivers/environment/MS5611.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    m_lm75a.GetTemperature();  // one read; the others are non-blocking state machines
    m_hih8131.Update();
    m_th09c.Update();
    m_ms5611.Update();
    m_hp303b.Update();
    m_bme688.Update();

    prosdk::telemetry::Log("LM75ADP", m_lm75a);
    prosdk::telemetry::Log("HIH8131", m_hih8131);
    prosdk::telemetry::Log("TH09C", m_th09c);
    prosdk::telemetry::Log("MS5611", m_ms5611);
    prosdk::telemetry::Log("MS5611/Altitude m", m_ms5611.GetAltitude().value());
    prosdk::telemetry::Log("HP303B", m_hp303b);
    prosdk::telemetry::Log("HP303B/Altitude m", m_hp303b.GetAltitude().value());
    prosdk::telemetry::Log("BME688", m_bme688);
  }

 private:
  // first member: the bus pins before the drivers open the buses
  bool m_pinsConfigured =
      prosdk::platform::SetI2CPins(1, /*sda=*/17, /*scl=*/16, /*frequencyHz=*/100000);

  prosdk::LM75A m_lm75a{prosdk::I2C::Port::PORT_1};                      // 0x48
  prosdk::HIH8000 m_hih8131{prosdk::I2C::Port::PORT_1};                  // 0x27
  prosdk::ENS210 m_th09c{prosdk::I2C::Port::PORT_1};                     // 0x43
  prosdk::MS5611 m_ms5611{prosdk::I2C::Port::PORT_1};                    // 0x77
  prosdk::DPS310 m_hp303b{prosdk::I2C::Port::PORT_1, /*address=*/0x76};  // JP3 shorted
  prosdk::BME688 m_bme688{prosdk::I2C::Port::PORT_0};                    // 0x77
};

PROSDK_ROBOT_MAIN(Robot)
