// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NAU7802 driver (NAU7802.hpp). Registers: Nuvoton NAU7802 data sheet Revision 1.7, sections 8.6
// (calibration), 8.8 (reset and power-down), 9.1 (power-on sequencing), 10 (register map) and
// 11.1 to 11.15 (register details).

#include "prosdk/drivers/adc/NAU7802.hpp"

#include <chrono>
#include <thread>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegPuCtrl = 0x00;
constexpr int kRegCtrl1 = 0x01;
constexpr int kRegCtrl2 = 0x02;
// 3 bytes, MSB first; reading 0x12 latches all 24 bits (11.9). That the read also clears CR is
// not stated in the data sheet (unverified): CR may instead stay set until the next conversion.
constexpr int kRegAdcoB2 = 0x12;
constexpr int kRegAdc = 0x15;     // write: ADC registers (11.10)
constexpr int kRegRevision = 0x1F;

// PU_CTRL bits (11.1)
constexpr uint8_t kPuRR = 0x01;     // register reset (level: 1 enters, 0 leaves reset)
constexpr uint8_t kPuPUD = 0x02;    // power up digital
constexpr uint8_t kPuPUA = 0x04;    // power up analog
constexpr uint8_t kPuPUR = 0x08;    // power-up ready (read only)
constexpr uint8_t kPuCS = 0x10;     // cycle start: no conversion until set (9.1 step 5)
constexpr uint8_t kPuCR = 0x20;     // conversion ready (read only)
constexpr uint8_t kPuAVDDS = 0x80;  // AVDD from the internal LDO
constexpr uint8_t kPuRunning = kPuAVDDS | kPuCS | kPuPUA | kPuPUD;

// CTRL1 (11.2): VLDO in bits 5:3, 101 = 3.0 V. AVDD must not exceed DVDD (7.2), and DVDD must be
// 0.3 V above the LDO output (section 3, Features); the board feeds DVDD 3.3 V. The LDO drives
// at least 10 mA at 3.0 V (section 3). CRP = 0 (DRDY active high), DRDY_SEL = 0 (conversion ready).
constexpr uint8_t kLdo3V0 = 0x05 << 3;

// CTRL2 (11.3): CHS bit 7, CRS bits 6:4, CAL_ERR bit 3 (read only), CALS bit 2, CALMOD 1:0
// (00 = internal offset calibration)
constexpr uint8_t kCtrl2Ch2 = 0x80;
constexpr uint8_t kCtrl2CalErr = 0x08;
constexpr uint8_t kCtrl2Cals = 0x04;

// REG0x15 = 0x30: REG_CHPS = 11, the only valid setting (11.10; 9.1 step 4b)
constexpr uint8_t kAdcChopperOff = 0x30;

// REG0x1F bits 3:0: chip revision ID, 1111 (11.15.3)
constexpr uint8_t kRevisionMask = 0x0F;
constexpr uint8_t kRevisionId = 0x0F;

// PUR comes "after about 200 microseconds" (9.1 step 3); 50 polls give at least 10 ms.
constexpr auto kPowerUpPollInterval = std::chrono::microseconds{200};
constexpr int kPowerUpPolls = 50;

// The data sheet gives no calibration time; 2 s covers several conversions even at 10 SPS
// (unverified).
constexpr auto kCalibrationPollInterval = std::chrono::milliseconds{1};
constexpr int kCalibrationPolls = 2000;

// Conversions dropped after a gain, rate, channel or calibration change. The chip itself holds
// CR back for 6 conversions after a reset or power-up (8.8); for these changes the data sheet
// gives no number, so the same 6 is used (unverified).
constexpr int kSettleConversions = 6;

constexpr double kSetupRetryTime = 1.0;  // s between setup attempts after a failure

int32_t SignExtend24(const uint8_t* p) {
  int32_t v = (static_cast<int32_t>(p[0]) << 16) | (p[1] << 8) | p[2];
  return (v & 0x800000) ? v - 0x1000000 : v;
}

}  // namespace

NAU7802::NAU7802(I2C::Port port) : m_i2c{port, kAddress} {
  m_lastSetupTime = Timer::GetMonotonicTimestamp().value();
  if (!Setup()) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] NAU7802 0x{:02X} on I2C port {}: no answer or no power-up\n",
                           kAddress, static_cast<int>(port));
  }
  util::ReportUsage("NAU7802", kAddress, "");
}

// Power-on sequence (9.1). m_mutex held, or called from the constructor.
bool NAU7802::Setup() {
  bool aborted = m_i2c.Write(kRegPuCtrl, kPuRR);
  aborted = aborted || m_i2c.Write(kRegPuCtrl, kPuPUD);
  bool ready = false;
  for (int i = 0; !aborted && !ready && i < kPowerUpPolls; ++i) {
    std::this_thread::sleep_for(kPowerUpPollInterval);
    uint8_t pu = 0;
    aborted = m_i2c.Read(kRegPuCtrl, 1, &pu);
    ready = !aborted && (pu & kPuPUR) != 0;
  }
  if (!ready) {
    m_connected = false;
    return false;
  }
  uint8_t revision = 0;
  aborted = m_i2c.Read(kRegRevision, 1, &revision);
  if (!aborted && (revision & kRevisionMask) != kRevisionId) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] NAU7802 0x{:02X}: unexpected revision ID 0x{:02X}\n",
                           kAddress, revision);
  }
  // the LDO voltage before the analog part (and with it the LDO) powers up
  aborted = aborted || WriteCtrl1();
  aborted = aborted || m_i2c.Write(kRegPuCtrl, kPuAVDDS | kPuPUA | kPuPUD);
  aborted = aborted || m_i2c.Write(kRegAdc, kAdcChopperOff);
  aborted = aborted || WriteCtrl2(0);
  aborted = aborted || m_i2c.Write(kRegPuCtrl, kPuRunning);
  m_connected = !aborted;
  m_discard = 0;  // the chip holds CR back for the first 6 conversions itself (8.8)
  return m_connected;
}

bool NAU7802::WriteCtrl1() {
  return m_i2c.Write(kRegCtrl1, kLdo3V0 | m_gainCode);
}

bool NAU7802::WriteCtrl2(uint8_t extra) {
  uint8_t value = static_cast<uint8_t>((m_channel == 2 ? kCtrl2Ch2 : 0) | (m_rateCode << 4) | extra);
  return m_i2c.Write(kRegCtrl2, value);
}

void NAU7802::SetGain(int gain) {
  int code = -1;
  for (int c = 0; c < 8; ++c) {
    if (gain == (1 << c)) {
      code = c;
    }
  }
  if (code < 0) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "NAU7802 gain {} (valid: 1, 2, 4, 8, 16, 32, 64, 128)", gain);
  }
  std::scoped_lock lock{m_mutex};
  if (m_gainCode != code) {
    m_discard = kSettleConversions;
  }
  m_gainCode = static_cast<uint8_t>(code);
  m_connected = !WriteCtrl1();
}

void NAU7802::SetSampleRate(int samplesPerSecond) {
  int code;
  switch (samplesPerSecond) {  // CRS (11.3); 100, 101 and 110 are not listed
    case 10: code = 0; break;
    case 20: code = 1; break;
    case 40: code = 2; break;
    case 80: code = 3; break;
    case 320: code = 7; break;
    default:
      throw WPILIB_MakeError(err::ParameterOutOfRange,
                             "NAU7802 sample rate {} (valid: 10, 20, 40, 80, 320)",
                             samplesPerSecond);
  }
  std::scoped_lock lock{m_mutex};
  if (m_rateCode != code) {
    m_discard = kSettleConversions;
  }
  m_rateCode = static_cast<uint8_t>(code);
  m_connected = !WriteCtrl2(0);
}

void NAU7802::SetChannel(int channel) {
  if (channel != 1 && channel != 2) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "NAU7802 channel {} (valid: 1, 2)",
                           channel);
  }
  std::scoped_lock lock{m_mutex};
  if (m_channel != channel) {
    m_discard = kSettleConversions;
  }
  m_channel = channel;
  m_connected = !WriteCtrl2(0);
}

bool NAU7802::CalibrateInternal() {
  std::scoped_lock lock{m_mutex};
  // CALS = 1 starts it; CALS reads 1 until done, then CAL_ERR tells the result (8.6, 11.3)
  if (WriteCtrl2(kCtrl2Cals)) {
    m_connected = false;
    return false;
  }
  m_discard = kSettleConversions;
  for (int i = 0; i < kCalibrationPolls; ++i) {
    std::this_thread::sleep_for(kCalibrationPollInterval);
    uint8_t ctrl2 = 0;
    if (m_i2c.Read(kRegCtrl2, 1, &ctrl2)) {
      m_connected = false;
      return false;
    }
    if ((ctrl2 & kCtrl2Cals) == 0) {
      m_connected = true;
      return (ctrl2 & kCtrl2CalErr) == 0;
    }
  }
  platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                         "[prosdk] NAU7802 0x{:02X}: calibration did not finish\n", kAddress);
  return false;
}

bool NAU7802::IsReady() {
  std::scoped_lock lock{m_mutex};
  uint8_t pu = 0;
  m_connected = !m_i2c.Read(kRegPuCtrl, 1, &pu) && (pu & kPuPUR) != 0;
  return m_connected && (pu & kPuCR) != 0;
}

std::optional<int32_t> NAU7802::Update() {
  std::scoped_lock lock{m_mutex};
  uint8_t pu = 0;
  if (m_i2c.Read(kRegPuCtrl, 1, &pu)) {
    m_connected = false;
    return std::nullopt;
  }
  if ((pu & kPuPUR) == 0) {
    // not powered up: missing at construction, or reset by a supply dip (a reset clears PUD,
    // 11.1). Set it up again, at most once per second; the calibration is lost.
    m_connected = false;
    double now = Timer::GetMonotonicTimestamp().value();
    if (now - m_lastSetupTime >= kSetupRetryTime) {
      m_lastSetupTime = now;
      if (Setup()) {
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] NAU7802 0x{:02X}: set up again; calibrate again\n",
                               kAddress);
      }
    }
    return std::nullopt;
  }
  m_connected = true;
  if ((pu & kPuCR) == 0) {
    return std::nullopt;
  }
  uint8_t buf[3];
  if (m_i2c.Read(kRegAdcoB2, sizeof(buf), buf)) {
    m_connected = false;
    return std::nullopt;
  }
  if (m_discard > 0) {
    --m_discard;
    return std::nullopt;
  }
  m_raw = SignExtend24(buf);  // two's complement (11.9)
  return m_raw;
}

int32_t NAU7802::GetRaw() const {
  std::scoped_lock lock{m_mutex};
  return m_raw;
}

bool NAU7802::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void NAU7802::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Raw", m_raw);
  table.Log("Channel", m_channel);
  table.Log("Connected", m_connected);
}

std::string_view NAU7802::GetTelemetryType() const {
  return "ADC";
}
