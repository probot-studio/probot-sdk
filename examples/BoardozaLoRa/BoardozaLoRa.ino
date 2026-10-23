// BoardozaLoRa: two robots (or a robot and a second controller) exchange short text messages over
// the Boardoza E22-900M22S LoRa board with prosdk::SX126x (design/boardoza/P11-lora.md). The robot
// listens all the time and sends "probot <n>" now and then; the last message received, its RSSI
// and SNR go to the Driver Station (/Telemetry). Flash the same sketch on both ends.
//
// Regulatory: the default settings use 868.1 MHz at 14 dBm (25 mW) in the EU 868.0-868.6 MHz
// sub-band, where a transmitter may be on air at most 1 % of the time. The sketch waits 100 times
// a packet's air time (at least 5 s) before the next send. Other bands and countries have other
// rules (US 902-928 MHz: FCC Part 15); check the local rules before you change the frequency or
// the power, and never transmit without an antenna on the SMA connector.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin). SPI bus 0 on the
// Probot main board's J5 header (Boardoza Pulse S32-S3; the HAL's default bus 0 pins):
//   J1: 1 VCC -> 3.3 V (or 5 V: the board regulates to 3.3 V), 2 MISO -> GPIO 16,
//       3 MOSI -> GPIO 15, 4 SCK -> GPIO 17, 5 nCS -> GPIO 7, 6 GND -> GND
//   J2: 1 NRST -> GPIO 36, 2 BUSY -> GPIO 38, 3 DIO1 -> GPIO 37
//   J3: 1 RXEN -> GPIO 35, 2 TXEN -> GPIO 39, 3 DIO2 unconnected
// The signal pins are 3.3 V: never drive them from 5 V logic. RXEN and TXEN must be wired: the
// board leaves the module's RF switch undriven otherwise. To let the chip drive TXEN instead,
// bridge TXEN to DIO2 on J3, pass -1 for txEn and set config.dio2AsRfSwitch = true.
// At 22 dBm the module draws about 120 mA while sending: power VCC from a supply that has it.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <stdint.h>

#include <span>
#include <string>

#include <prosdk/drivers/radio/SX126x.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>
#include <prosdk/system/Timer.hpp>

namespace {

constexpr int kSpiBus = 0;  // SPI2
constexpr int kSck = 17;
constexpr int kMosi = 15;
constexpr int kMiso = 16;
constexpr int kChipSelect = 7;
constexpr int kBusy = 38;
constexpr int kDio1 = 37;
constexpr int kReset = 36;
constexpr int kRxEn = 35;
constexpr int kTxEn = 39;

bool ConfigureLoRaPins() {
  return prosdk::platform::SetSPIPins(kSpiBus, kSck, kMosi, kMiso);
}

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    m_radioReady = m_radio.Begin() && m_radio.StartReceive();
    m_sendTimer.Start();
  }

  void RobotPeriodic() override {
    while (auto packet = m_radio.Poll()) {
      m_lastMessage.assign(packet->data.begin(), packet->data.end());
      prosdk::telemetry::Log("LoRa/Last message", std::string_view{m_lastMessage});
      prosdk::telemetry::Log("LoRa/Last RSSI (dBm)", packet->rssi);
      prosdk::telemetry::Log("LoRa/Last SNR (dB)", packet->snr);
    }

    if (m_radioReady && m_radio.IsTxDone() && m_sendTimer.HasElapsed(m_sendPeriod)) {
      const std::string text = "probot " + std::to_string(m_count);
      const std::span<const uint8_t> bytes{reinterpret_cast<const uint8_t*>(text.data()),
                                           text.size()};
      if (m_radio.Send(bytes)) {
        ++m_count;
        // 1 % duty cycle: on air for t, then silent for 99 t
        m_sendPeriod = 100.0 * m_radio.GetTimeOnAir(static_cast<int>(text.size()));
        if (m_sendPeriod < prosdk::units::second_t{5.0}) {
          m_sendPeriod = prosdk::units::second_t{5.0};
        }
        m_sendTimer.Reset();
      }
    }

    prosdk::telemetry::Log("LoRa", m_radio);
    prosdk::telemetry::Log("LoRa/Ready", m_radioReady);
  }

 private:
  // first member: the bus pins before the radio opens the bus
  bool m_pinsConfigured = ConfigureLoRaPins();

  // default Config: 868.1 MHz, SF7, 125 kHz, 4/5, 14 dBm, TCXO 1.8 V, private sync word
  prosdk::SX126x m_radio{prosdk::SPI::kPort0, kChipSelect, kBusy, kDio1, kReset, kRxEn, kTxEn};
  bool m_radioReady = false;
  prosdk::Timer m_sendTimer;
  prosdk::units::second_t m_sendPeriod{5.0};
  int m_count = 0;
  std::string m_lastMessage;
};

PROSDK_ROBOT_MAIN(Robot)
