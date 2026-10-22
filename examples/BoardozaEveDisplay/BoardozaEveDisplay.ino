// BoardozaEveDisplay: a status screen on a Boardoza FT810 (or FT800) TFT driver board with
// prosdk::FT8xx (design/boardoza/P13-eve.md). The screen shows the battery voltage as a number and
// a gauge, the robot state and a button; touching the button counts presses. The display state
// goes to the Driver Station (/Telemetry/Display).
//
// Wiring (change the pins to match your robot; none may be a probot-core pin). SPI uses the SPI2
// IO_MUX pins of the ESP32-S3:
//   J6: 1 CS -> GPIO 10, 2 MISO -> GPIO 13, 3 MOSI -> GPIO 11, 4 SCK -> GPIO 12
//   J1: 1 3V3 -> leave open (an output of the board), 2 5V -> 5 V, 3 GND -> GND,
//       4 PD -> GPIO 4, 5 INT -> GPIO 5 (optional; pass -1 when it is not wired)
// The board needs 5 V (about 150 mA plus the backlight; a 7 inch panel draws more). Its logic is
// 3.3 V, and the host lines go through 5 V tolerant buffers, so the ESP32-S3 pins connect
// directly. Never feed 3.3 V into the J1 3V3 pin. The panel plugs into the 40-pin FPC socket.
//
// Panel: kBoardozaFT810Bundle (800x480) for the Ozdisan FT810 bundles, kBoardozaFT800Bundle
// (480x272) for the FT800 bundles. Other panels need their own FT8xxPanel timing.
//
// Touch: only the resistive-touch panels (FT810 5 inch TFT-1 and 7 inch TFT-2 bundles) report
// touches; the "no touch" and "cap. touch" bundles do not. Resistive touch needs one calibration
// per panel: set kCalibrateTouch to true, tap the three dots at start-up, and the Driver Station
// shows the six values under Display/Transform. Copy them into kTouchTransform and set
// kCalibrateTouch back to false. Never calibrate a panel without resistive touch: the
// calibration screen would wait for taps forever.
//
// An e-stop drives every DIO output low, PD included: the display goes dark and stays dark until
// the robot restarts.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

#include <prosdk/driverstation/RobotState.hpp>
#include <prosdk/drivers/display/FT8xx.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>
#include <prosdk/system/RobotController.hpp>

namespace {

constexpr int kSpiBus = 0;  // SPI2
constexpr int kSck = 12;
constexpr int kMosi = 11;
constexpr int kMiso = 13;
constexpr int kChipSelect = 10;
constexpr int kPowerDown = 4;
constexpr int kInterrupt = 5;

constexpr bool kCalibrateTouch = false;
constexpr std::array<int32_t, 6> kTouchTransform{};  // from Display/Transform A-F; all 0: unset

constexpr uint8_t kButtonTag = 1;
constexpr int kRedrawLoops = 5;  // every 5th 20 ms loop: 10 frames per second

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    if (kCalibrateTouch) {
      m_display.CalibrateTouch();
    } else if (kTouchTransform != std::array<int32_t, 6>{}) {
      m_display.SetTouchTransform(kTouchTransform);
    }
  }

  void RobotPeriodic() override {
    prosdk::telemetry::Log("Display", m_display);
    if (auto transform = m_display.GetTouchTransform(); transform && kCalibrateTouch) {
      for (int i = 0; i < 6; ++i) {
        prosdk::telemetry::Log(std::string{"Display/Transform "} + static_cast<char>('A' + i),
                               (*transform)[i]);
      }
    }

    // count a press when the button's tag appears
    const bool pressed = m_display.GetTouchTag() == kButtonTag;
    if (pressed && !m_wasPressed) {
      ++m_presses;
    }
    m_wasPressed = pressed;

    if (++m_loops < kRedrawLoops || !m_display.IsReady() || m_display.IsCalibrating()) {
      return;
    }
    m_loops = 0;
    Draw(pressed);
  }

 private:
  void Draw(bool pressed) {
    const int w = m_display.GetPanel().width;
    const int h = m_display.GetPanel().height;
    const double volts = prosdk::RobotController::GetBatteryVoltage().value();
    const bool enabled = prosdk::RobotState::IsEnabled();

    m_display.ClearColorRGB(10, 20, 40);
    m_display.Clear();
    m_display.ColorRGB(255, 255, 255);
    m_display.Text(w / 2, 30, 30, prosdk::FT8xx::kOptCenter, enabled ? "ENABLED" : "DISABLED");

    // battery: millivolts as a number and a 0-15 V gauge
    m_display.Text(w / 4, h / 2 - 90, 28, prosdk::FT8xx::kOptCenter, "Battery mV");
    m_display.Number(w / 4, h / 2 - 50, 31, prosdk::FT8xx::kOptCenter,
                     static_cast<int32_t>(volts * 1000.0));
    m_display.Gauge(w / 4, h / 2 + 60, h / 5, 0, 5, 3,
                    std::clamp(static_cast<int>(volts * 10.0), 0, 150), 150);

    // a touch button (Tag) and the press count
    m_display.FgColor(pressed ? 0x00A040 : 0x003870);
    m_display.Tag(kButtonTag);
    m_display.Button(w / 2 + 20, h / 2 - 40, w / 2 - 60, 80, 29, 0, "Press");
    m_display.Tag(0);
    m_display.Number(3 * w / 4, h / 2 + 80, 31, prosdk::FT8xx::kOptCenter, m_presses);

    // a red dot where the screen is touched
    if (auto touch = m_display.GetTouch()) {
      m_display.ColorRGB(255, 40, 40);
      m_display.PointSize(8);
      m_display.Begin(prosdk::FT8xx::kPoints);
      m_display.Vertex2f(touch->x, touch->y);
      m_display.End();
    }
    m_display.Swap();
  }

  // first member: the bus pins before the driver opens the bus
  bool m_pinsConfigured = prosdk::platform::SetSPIPins(kSpiBus, kSck, kMosi, kMiso);

  prosdk::FT8xx m_display{prosdk::SPI::kPort0, kChipSelect, kPowerDown,
                          prosdk::ft8xx::kBoardozaFT810Bundle, kInterrupt};
  int m_loops = 0;
  int m_presses = 0;
  bool m_wasPressed = false;
};

PROSDK_ROBOT_MAIN(Robot)
