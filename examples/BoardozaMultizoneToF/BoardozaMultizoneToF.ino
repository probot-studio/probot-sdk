// BoardozaMultizoneToF: a Boardoza VL53L7CX-1, -2, -3 or -4 board (ST VL53L7CX multizone
// time-of-flight sensor) in a TimedRobot with prosdk::VL53L7CX (design/boardoza/P4-tof.md). The 64
// zone distances (8x8, 15 frames per second) and the nearest valid distance go to the Driver
// Station (/Telemetry).
//
// Power (the ESP32-S3 pins are not 5 V tolerant):
//   VL53L7CX-1      3.3 V only, on its 3V3 pin.
//   VL53L7CX-2, -3  3.3 V on the 5V pin: their level shifters pull SDA and SCL (and INT on the -3)
//                   up to that supply, so 5 V there puts 5 V on the controller's pins.
//   VL53L7CX-4      3.3 V or 5 V on the 5V pin; its signals stay at 3.3 V.
//
// Wiring (J2: 1 supply, 2 GND, 3 SDA, 4 SCL, 5 RST, 6 INT, 7 LPn; change the pins to match your
// robot, none may be a probot-core pin):
//   SDA -> GPIO 17, SCL -> GPIO 18. RST, INT and LPn stay unconnected (pulled on the board).
// I2C port 1 runs at 400 kHz; the driver needs at least that (1 MHz also works for the sensor).
//
// After the first IsReady() call the driver uploads the sensor firmware (84 KiB, about 2 s).
//
// Two boards on one bus: wire each LPn pin to a GPIO and give each sensor its own address, e.g.
//   prosdk::VL53L7CX m_left{prosdk::I2C::Port::PORT_1, /*lpnChannel=*/38, /*address=*/0x30,
//                           prosdk::VL53L7CX::Resolution::k8x8};
//   prosdk::VL53L7CX m_right{prosdk::I2C::Port::PORT_1, /*lpnChannel=*/39, /*address=*/0x31,
//                            prosdk::VL53L7CX::Resolution::k8x8};

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <array>
#include <cstdint>

#include <prosdk/drivers/range/VL53L7CX.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    prosdk::telemetry::Log("ToF", m_tof);
    if (!m_tof.IsReady()) {
      return;
    }
    std::array<int16_t, prosdk::VL53L7CX::kMaxZones> distances{};
    std::array<uint8_t, prosdk::VL53L7CX::kMaxZones> status{};
    int zones = m_tof.GetDistances(distances, status);  // both from the same frame
    int nearest = -1;  // mm; -1: no valid zone
    for (int zone = 0; zone < zones; ++zone) {
      bool valid = status[zone] == 5 || status[zone] == 9;
      if (valid && (nearest < 0 || distances[zone] < nearest)) {
        nearest = distances[zone];
      }
    }
    prosdk::telemetry::Log("ToF/Nearest mm", nearest);
  }

 private:
  // first member: the bus pins before the driver opens the bus
  bool m_pinsConfigured =
      prosdk::platform::SetI2CPins(1, /*sda=*/17, /*scl=*/18, /*frequencyHz=*/400000);

  prosdk::VL53L7CX m_tof{prosdk::I2C::Port::PORT_1, /*lpnChannel=*/-1,
                         prosdk::VL53L7CX::kDefaultAddress, prosdk::VL53L7CX::Resolution::k8x8};
};

PROSDK_ROBOT_MAIN(Robot)
