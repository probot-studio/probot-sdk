// BoardozaLoadCell: the Boardoza load-cell boards in a TimedRobot (design/boardoza/P6-loadcell.md).
// NAU7802 (24-bit I2C ADC, prosdk::NAU7802) and HX711 (24-bit ADC on two GPIOs, prosdk::HX711),
// each with a prosdk::LoadCell for tare and scale. Mass and raw counts go to the Driver Station
// (/Telemetry). Use one board or both.
//
// Power both boards from 3.3 V. The NAU7802 board pulls SDA/SCL up to its supply pin (labelled
// 3.3V), and the HX711's DOUT swings to its VDD pin, so 5 V there puts 5 V on ESP32-S3 pins,
// which are not 5 V tolerant.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   NAU7802  J2: 1 GND, 2 SDA -> GPIO 17, 3 SCL -> GPIO 18, 4 3.3V. Address 0x2A (fixed).
//            Load cell on J1 (channel 1): 1 WHT (A-), 2 GRN (A+), 3 BLK (E-), 4 RED (E+).
//   HX711    J1: 1 VCC -> 3.3 V, 2 VDD -> 3.3 V, 3 DAT -> GPIO 4, 4 CLK -> GPIO 5, 5 GND.
//            Load cell on J3: 1 RED (E+), 2 BLK (E-), 3 WHT (A-), 4 GRN (A+).
//            JP1 selects the rate: bridge 1-2 for 80 SPS or 2-3 for 10 SPS (open by default,
//            unverified; an open RATE pin floats).
//
// Calibration: start with the cells unloaded (each tares over its first 10 samples), put a known
// mass m on a cell, read "<cell>/Counts" on the Driver Station and set its kCountsPerKg below to
// Counts / m.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/adc/HX711.hpp>
#include <prosdk/drivers/adc/LoadCell.hpp>
#include <prosdk/drivers/adc/NAU7802.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>

namespace {

// Placeholders: counts per kilogram of your cells (see Calibration above).
constexpr double kNau7802CountsPerKg = 100000.0;
constexpr double kHx711CountsPerKg = 100000.0;

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    m_nau7802.CalibrateInternal();  // blocks briefly; after power-up (data sheet 8.6)
    m_nauCell.SetScale(kNau7802CountsPerKg);
    m_nauCell.Tare();
    m_hxCell.SetScale(kHx711CountsPerKg);
    m_hxCell.Tare();
  }

  void RobotPeriodic() override {
    m_nauCell.Update();  // each polls its ADC once; neither waits for a conversion
    m_hxCell.Update();

    prosdk::telemetry::Log("NAU7802", m_nau7802);
    prosdk::telemetry::Log("NAU7802 cell", m_nauCell);
    prosdk::telemetry::Log("HX711", m_hx711);
    prosdk::telemetry::Log("HX711 cell", m_hxCell);
  }

 private:
  // first member: the bus pins before the driver opens the bus
  bool m_pinsConfigured =
      prosdk::platform::SetI2CPins(1, /*sda=*/17, /*scl=*/18, /*frequencyHz=*/400000);

  prosdk::NAU7802 m_nau7802{prosdk::I2C::Port::PORT_1};  // x128, 80 SPS, channel 1
  prosdk::LoadCell m_nauCell{[this] { return m_nau7802.Update(); }};
  prosdk::HX711 m_hx711{/*dataGpio=*/4, /*clockGpio=*/5};  // channel A, gain 128
  prosdk::LoadCell m_hxCell{[this] { return m_hx711.Update(); }};
};

PROSDK_ROBOT_MAIN(Robot)
