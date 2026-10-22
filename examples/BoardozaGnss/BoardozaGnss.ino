// BoardozaGnss: the Boardoza GNSS boards in a TimedRobot (design/boardoza/P7-gnss.md). A SAM-M8Q
// or SAM-M10Q board is read over I2C with prosdk::UbloxGnss (UBX NAV-PVT), a GP-02 board over a
// UART with prosdk::NmeaGnss (NMEA). Position, speed, heading and satellites go to the Driver
// Station (/Telemetry). Use one board or both.
//
// Power every board from 3.3 V (VCC pin of its connector to the controller's 3V3, GND to GND).
// At 5 V the level shifters on the boards may pull the host-side lines to 5 V, and the ESP32-S3
// pins are not 5 V tolerant.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   SAM-M8Q  J2: SDA -> GPIO 17, SCL -> GPIO 18   (pins: 1 GND, 2 SDA, 3 SCL, 4 VCC)
//   SAM-M10Q J2: SDA -> GPIO 17, SCL -> GPIO 18   (pins: 1 VCC, 2 SCL, 3 SDA, 4 GND)
//   GP-02    J1: TX -> GPIO 15 (host RX), RX -> GPIO 16 (host TX)   (pins: 1 VCC, 2 RX, 3 TX, 4 GND)
//            connect the active antenna to J2 before powering the board.
// I2C port 1 runs at 100 kHz: the SAM-M10Q allows at most 320 kHz.
//
// A u-blox board can use its UART (J1) instead: replace m_ublox with
//   prosdk::UbloxGnss m_ublox{prosdk::SerialPort::Port::MXP};   // 9600 baud, UART2
// after prosdk::platform::SetSerialPins(1, rx, tx). Keep NAV-PVT at 5 Hz or below at 9600 baud.
//
// The first fix outdoors takes about 25 s (cold start; the boards have no backup battery).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/gnss/NmeaGnss.hpp>
#include <prosdk/drivers/gnss/UbloxGnss.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>

namespace {

bool ConfigureGnssPins() {
  bool ok = prosdk::platform::SetI2CPins(1, /*sda=*/17, /*scl=*/18, /*frequencyHz=*/100000);
  ok &= prosdk::platform::SetSerialPins(0, /*rx=*/15, /*tx=*/16);
  return ok;
}

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() { m_ublox.SetRate(2_Hz); }

  void RobotPeriodic() override {
    prosdk::telemetry::Log("u-blox", m_ublox);
    prosdk::telemetry::Log("u-blox/Connected", m_ublox.IsConnected());
    prosdk::telemetry::Log("u-blox/Configured", m_ublox.IsConfigured());
    prosdk::telemetry::Log("GP-02", m_gp02);
    prosdk::telemetry::Log("GP-02/Connected", m_gp02.IsConnected());

    const prosdk::GnssFix fix = m_ublox.GetFix();
    if (fix.valid) {
      prosdk::telemetry::Log("u-blox/UTC second", fix.utc.second);
    }
  }

 private:
  // first member: the bus pins before the drivers open the buses
  bool m_pinsConfigured = ConfigureGnssPins();

  prosdk::UbloxGnss m_ublox{prosdk::I2C::Port::PORT_1};               // address 0x42
  prosdk::NmeaGnss m_gp02{prosdk::SerialPort::Port::ONBOARD, 9600};  // UART1
};

PROSDK_ROBOT_MAIN(Robot)
