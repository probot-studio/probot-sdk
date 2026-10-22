// BoardozaRotaryArgb: the Boardoza Rotary Encoder ARGB board in a TimedRobot
// (design/boardoza/P10-tmag-leds.md). Turning the knob moves a lit LED around the ring of 14
// clocked LEDs (prosdk::ClockedAddressableLED); pressing it changes the colour. The detent count
// and the button go to the Driver Station (/Telemetry).
//
// Power the board from 3.3 V (VCC pin to the controller's 3V3, GND to GND). The encoder and
// button pull-ups go to VCC: at 5 V the A, B and SW lines would be 5 V, and the ESP32-S3 pins
// are not 5 V tolerant. 14 LEDs at full white take about 840 mA, so the brightness is kept low.
//
// Wiring (J1 pin: board signal -> controller; change the pins to match your robot, none may be
// a probot-core pin):
//   1 VCC  -> 3V3          2 SW   -> GPIO 38       3 DOUT, 4 COUT: not connected (chaining)
//   5 CIN  -> GPIO 39 (SPI3 SCK)                   6 DIN  -> GPIO 40 (SPI3 MOSI)
//   7 B    -> GPIO 42      8 A    -> GPIO 41       9 GND  -> GND
// The LEDs need SPI bus 1 (SPI3) to themselves: put every SPI device with a chip select on bus 0.
//
// The encoder (EC1101S) gives 20 quadrature cycles per turn, one per detent: 4 counts per detent
// with the x4 decoding. Which LED is index 0 on the ring, and which way counts up, is not
// verified: use SetReverseDirection(true) or another offset if the pointer runs backwards.
// The Phoenix12/24-WENC boards work the same way with 12 or 24 LEDs (their encoder's counts per
// detent are not verified).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <array>
#include <cmath>

#include <prosdk/drivers/led/ClockedAddressableLED.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/discrete/DigitalInput.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/platform/HalConfig.hpp>

namespace {

constexpr int kLedCount = 14;
constexpr int kDetentsPerTurn = 20;
// red, green, blue; the button steps through them
constexpr std::array<std::array<int, 3>, 3> kColors{{{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}};

bool ConfigureLedPins() {
  return prosdk::platform::SetSPIPins(1, /*sck=*/39, /*mosi=*/40, /*miso=*/-1);
}

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    m_knob.SetDistancePerPulse(1.0 / 4);  // distance in detents
    m_leds.SetBrightness(4);              // of 31
  }

  void RobotPeriodic() override {
    const bool pressed = !m_button.Get();  // active low
    if (pressed && !m_wasPressed) {
      m_colorIndex = (m_colorIndex + 1) % static_cast<int>(kColors.size());
    }
    m_wasPressed = pressed;

    const long detents = std::lround(m_knob.GetDistance());
    // the detent's position on the turn, mapped to the nearest LED
    const long step = ((detents % kDetentsPerTurn) + kDetentsPerTurn) % kDetentsPerTurn;
    const int lit = static_cast<int>((step * kLedCount + kDetentsPerTurn / 2) / kDetentsPerTurn) %
                    kLedCount;
    m_buffer.fill(prosdk::AddressableLED::LEDData{});
    const auto& color = kColors[m_colorIndex];
    m_buffer[lit].SetRGB(color[0], color[1], color[2]);
    m_leds.SetData(m_buffer);

    prosdk::telemetry::Log("Knob/Detents", static_cast<double>(detents));
    prosdk::telemetry::Log("Knob/Pressed", pressed);
  }

 private:
  // first member: the SPI3 pins before the LEDs open the bus
  bool m_pinsConfigured = ConfigureLedPins();

  prosdk::ClockedAddressableLED m_leds{prosdk::SPI::kPort1, kLedCount};
  prosdk::Encoder m_knob{/*A=*/41, /*B=*/42};
  prosdk::DigitalInput m_button{38};

  std::array<prosdk::AddressableLED::LEDData, kLedCount> m_buffer;
  int m_colorIndex = 0;
  bool m_wasPressed = false;
};

PROSDK_ROBOT_MAIN(Robot)
