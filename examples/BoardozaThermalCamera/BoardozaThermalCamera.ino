// BoardozaThermalCamera: a Boardoza MLX90640 thermal camera board in a TimedRobot
// (design/boardoza/P2-light-thermal.md). prosdk::MLX90640 reads the 32x24 frames on its own
// thread and converts them to degrees Celsius; the ambient, minimum and maximum temperatures and
// the full frame (768 values, row by row) go to the Driver Station (/Telemetry).
//
// Power the board from 3.3 V (VCC pin of its connector to the controller's 3V3, GND to GND). At
// 5 V the board's pull-ups may pull SDA and SCL to 5 V, and the ESP32-S3 pins are not 5 V tolerant.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   SDA -> GPIO 17, SCL -> GPIO 18
// I2C port 1 runs at 400 kHz, the highest speed the MLX90640 datasheet allows for EEPROM access
// (section 8, note 5). At 8 Hz one subpage takes about 40 ms of bus time, so keep other sensors
// on this port few and slow.
//
// For the 16x12 MLX90641 board, replace MLX90640 with MLX90641 (same address 0x33, same API).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <array>
#include <span>

#include <prosdk/drivers/thermal/MLX90640.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    prosdk::telemetry::Log("Thermal", m_camera);  // ambient, minimum, maximum, frames, connected

    // log the frame only when a new one is complete (8 Hz refresh rate: 4 frames per second)
    uint32_t count = m_camera.GetFrameCount();
    if (count != m_lastFrame && m_camera.GetFrame(m_frame)) {
      m_lastFrame = count;
      prosdk::telemetry::Log("Thermal/Frame", std::span<const float>{m_frame});
    }
  }

 private:
  // first member: the bus pins before the driver opens the bus
  bool m_pinsConfigured =
      prosdk::platform::SetI2CPins(1, /*sda=*/17, /*scl=*/18, /*frequencyHz=*/400000);

  prosdk::MLX90640 m_camera{prosdk::I2C::Port::PORT_1};  // address 0x33, 8 Hz
  std::array<float, prosdk::MLX90640::kPixelCount> m_frame{};
  uint32_t m_lastFrame = 0;
};

PROSDK_ROBOT_MAIN(Robot)
