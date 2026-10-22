// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TMAG5170 driver (TMAG5170.hpp). Texas Instruments TMAG5170 datasheet SBASAF4 (September 2021):
// 6.5 (TADCT0 17522 at TSENS_T0 25 C, TADCRES 60 LSB per C, typical), 6.8 (SCK 10 MHz maximum),
// 7.5.1.1 equation 1 (field), 7.5.1.2 equation 3 (temperature), 7.5.1.4 equation 7 (angle),
// 7.5.2.1 (SDI latched on the rising SCK edge, SDO changes on the falling edge: SPI mode 0),
// 7.5.2.3 Figure 7-10 (SDI frame), 7.5.2.4.1 Figure 7-11 (SDO frame and status bits), 7.5.2.5
// (CRC; command 0x0F000407 turns it off), 7.6 Tables 7-4 to 7-26 (registers).
//
// SDI frame, MSB first: bit 31 read (1) or write (0), bits 30..24 address, bits 23..8 data
// (ignored on a read), bits 7..4 CMD3..CMD0 (0: no conversion trigger, status bits as default),
// bits 3..0 CRC. A read answers in the same frame (DATA_TYPE 0, regular 32-bit read): bits
// 31..24 STAT11..STAT4, bits 23..8 the register, bits 7..4 STAT3..STAT0, bits 3..0 CRC.
// STAT11 PREV_CRC_STAT, STAT10 CFG_RESET, STAT3 ERROR_STAT. A frame with a wrong CRC is ignored
// by the chip, which inverts the last CRC bit of its answer, so the answer's CRC check fails.

#include "prosdk/drivers/magnetic/TMAG5170.hpp"

#include <algorithm>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegDeviceConfig = 0x00;
constexpr int kRegSensorConfig = 0x01;
constexpr int kRegSystemConfig = 0x02;
constexpr int kRegXResult = 0x09;
constexpr int kRegYResult = 0x0A;
constexpr int kRegZResult = 0x0B;
constexpr int kRegTempResult = 0x0C;
constexpr int kRegAfeStatus = 0x0D;  // CFG_RESET (bit 15) clears on read
constexpr int kRegTestConfig = 0x0F;
constexpr int kRegAngleResult = 0x13;

constexpr uint32_t kReadBit = 0x80000000;
constexpr uint32_t kStatusCfgReset = 0x40000000;  // STAT10 of an answer

// DEVICE_CONFIG (Table 7-6): CONV_AVG [14:12], MAG_TEMPCO [9:8] 0 (no magnet compensation),
// OPERATING_MODE [6:4] 2 (active, continuous conversion), T_CH_EN bit 3
constexpr uint16_t kOperatingModeContinuous = 2 << 4;
constexpr uint16_t kTemperatureChannelOn = 1 << 3;
// SENSOR_CONFIG (Table 7-7): ANGLE_EN [15:14], SLEEPTIME [13:10] 0, MAG_CH_EN [9:6] 7 (X, Y, Z),
// Z_RANGE [5:4], Y_RANGE [3:2], X_RANGE [1:0]
constexpr uint16_t kMagChannelsXyz = 7 << 6;
// SYSTEM_CONFIG (Table 7-8) 0: DATA_TYPE regular 32-bit reads, conversion trigger by SPI command,
// no diagnostics, no limit checks
constexpr uint16_t kSystemConfig = 0x0000;
// TEST_CONFIG (Table 7-21) 0: CRC_DIS 0 (CRC on), OSC_CNT_CTL 0
constexpr uint16_t kTestConfig = 0x0000;

// range code of X_RANGE/Y_RANGE/Z_RANGE (Table 7-7)
uint16_t RangeCode(TMAG5170::Range range) {
  switch (range) {
    case TMAG5170::Range::kLow:
      return 1;  // +/-25 mT (A1), +/-75 mT (A2)
    case TMAG5170::Range::kHigh:
      return 2;  // +/-100 mT (A1), +/-300 mT (A2)
    default:
      return 0;  // +/-50 mT (A1), +/-150 mT (A2)
  }
}

// ANGLE_EN (Table 7-7): 1 X-Y, 2 Y-Z, 3 X-Z
uint16_t AngleCode(TMAG5170::AnglePlane plane) {
  switch (plane) {
    case TMAG5170::AnglePlane::kYZ:
      return 2;
    case TMAG5170::AnglePlane::kXZ:
      return 3;
    default:
      return 1;
  }
}

double RangeMillitesla(TMAG5170::Range range, bool isA2) {
  double a1 = 50.0;  // A1: 25, 50, 100 mT; A2: 75, 150, 300 mT
  if (range == TMAG5170::Range::kLow) {
    a1 = 25.0;
  } else if (range == TMAG5170::Range::kHigh) {
    a1 = 100.0;
  }
  return isA2 ? 3.0 * a1 : a1;
}

// equation 1: B = signed result / 2^16 x 2 x range
double FieldMillitesla(uint16_t raw, double rangeMillitesla) {
  return static_cast<int16_t>(raw) * rangeMillitesla / 32768.0;
}

}  // namespace

TMAG5170::TMAG5170(SPI::Port bus, int csGpio, int clockHz) : m_spi{bus, csGpio}, m_csGpio{csGpio} {
  m_spi.SetMode(SPI::kMode0);
  m_spi.SetClockRate(std::clamp(clockHz, SPI::kMinClockRate, kMaxClockRate));
  bool connected = false;
  const char* problem = "no answer";
  {
    std::scoped_lock lock{m_mutex};
    // An earlier program may have turned the CRC off (CRC_DIS, kept until power-down). This frame
    // has a valid CRC, so the chip takes it either way; its answer is not checked, because
    // whether SDO carries a CRC while CRC is off is not stated (unverified).
    const uint32_t frame = (uint32_t{kRegTestConfig} << 24) | (uint32_t{kTestConfig} << 8);
    const uint32_t withCrc = frame | Crc4(frame);
    uint8_t tx[4] = {static_cast<uint8_t>(withCrc >> 24), static_cast<uint8_t>(withCrc >> 16),
                     static_cast<uint8_t>(withCrc >> 8), static_cast<uint8_t>(withCrc)};
    m_spi.Write(tx, sizeof(tx));
    connected = ApplyConfigLocked() && m_lastFrameOk;
    if (m_lastFrameOk) {
      problem = m_versionOk ? "configuration not accepted" : "unknown version (TEST_CONFIG VER)";
    }
  }
  if (!connected) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] TMAG5170 on SPI bus {} CS {}: {}\n", static_cast<int>(bus),
                           csGpio, problem);
  }
  util::ReportUsage("TMAG5170", csGpio, "");
}

uint8_t TMAG5170::Crc4(uint32_t frame) {
  // the serial form of 7.5.2.5 (example code next to Figure 7-13)
  const uint32_t padded = frame & 0xFFFFFFF0u;
  uint8_t crc = 0x0F;
  for (int i = 31; i >= 0; --i) {
    const uint8_t inv = static_cast<uint8_t>(((padded >> i) & 1u) ^ ((crc >> 3) & 1u));
    crc = static_cast<uint8_t>(((crc << 1) & 0x0E) | inv);  // crc[3:1] = crc[2:0], crc[0] = inv
    crc ^= static_cast<uint8_t>(inv << 1);                   // crc[1] = crc[0] ^ inv
  }
  return crc & 0x0F;
}

// m_mutex held
bool TMAG5170::TransferLocked(uint32_t frame, uint32_t* response) {
  frame = (frame & 0xFFFFFFF0u) | Crc4(frame);
  uint8_t tx[4] = {static_cast<uint8_t>(frame >> 24), static_cast<uint8_t>(frame >> 16),
                   static_cast<uint8_t>(frame >> 8), static_cast<uint8_t>(frame)};
  uint8_t rx[4] = {};
  if (m_spi.Transaction(tx, rx, sizeof(tx)) != static_cast<int>(sizeof(tx))) {
    m_lastFrameOk = false;
    return false;
  }
  const uint32_t answer = (uint32_t{rx[0]} << 24) | (uint32_t{rx[1]} << 16) |
                          (uint32_t{rx[2]} << 8) | rx[3];
  if (Crc4(answer) != (answer & 0x0F)) {
    ++m_crcErrors;
    m_lastFrameOk = false;
    return false;
  }
  if (answer & kStatusCfgReset) {
    m_resetSeen = true;
  }
  m_lastFrameOk = true;
  *response = answer;
  return true;
}

// m_mutex held
bool TMAG5170::ReadRegisterLocked(int reg, uint16_t* value) {
  uint32_t answer = 0;
  if (!TransferLocked(kReadBit | (static_cast<uint32_t>(reg) << 24), &answer)) {
    return false;
  }
  *value = static_cast<uint16_t>(answer >> 8);
  return true;
}

// m_mutex held
bool TMAG5170::WriteRegisterLocked(int reg, uint16_t value) {
  uint32_t answer = 0;
  return TransferLocked((static_cast<uint32_t>(reg) << 24) | (uint32_t{value} << 8), &answer);
}

// m_mutex held
bool TMAG5170::ApplyConfigLocked() {
  m_configured = false;
  uint16_t afe = 0;
  uint16_t test = 0;
  if (!ReadRegisterLocked(kRegAfeStatus, &afe)) {  // clears CFG_RESET
    return false;
  }
  m_resetSeen = false;
  if (!ReadRegisterLocked(kRegTestConfig, &test)) {
    return false;
  }
  const int version = (test >> 4) & 0x3;  // VER: 0 A1, 1 A2, 2 and 3 reserved
  m_versionOk = version == 0 || version == 1;
  m_isA2 = version == 1;
  if (!m_versionOk) {
    return false;
  }
  const uint16_t range = RangeCode(m_range);
  const auto sensor = static_cast<uint16_t>((AngleCode(m_plane) << 14) | kMagChannelsXyz |
                                            (range << 4) | (range << 2) | range);
  const auto device = static_cast<uint16_t>((static_cast<uint16_t>(m_averaging) << 12) |
                                            kOperatingModeContinuous | kTemperatureChannelOn);
  // the mode last, so that the conversions start with the new sensor configuration
  bool ok = WriteRegisterLocked(kRegSensorConfig, sensor) &&
            WriteRegisterLocked(kRegSystemConfig, kSystemConfig) &&
            WriteRegisterLocked(kRegDeviceConfig, device);
  // a write whose frame the chip saw with a wrong CRC is ignored: read everything back
  uint16_t sensorRead = 0;
  uint16_t systemRead = 0;
  uint16_t deviceRead = 0;
  ok = ok && ReadRegisterLocked(kRegSensorConfig, &sensorRead) &&
       ReadRegisterLocked(kRegSystemConfig, &systemRead) &&
       ReadRegisterLocked(kRegDeviceConfig, &deviceRead);
  m_configured = ok && sensorRead == sensor && systemRead == kSystemConfig && deviceRead == device;
  if (m_configured) {
    m_resetSeen = false;  // the read-back shows the configuration in place
  }
  return m_configured;
}

// m_mutex held
void TMAG5170::EnsureConfiguredLocked() {
  if (m_configured && !m_resetSeen) {
    return;
  }
  // not configured yet (no answer at construction) or the chip reset since (CFG_RESET)
  const bool wasReset = m_configured;
  if (ApplyConfigLocked() && wasReset) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] TMAG5170 on SPI bus {} CS {}: chip reset, configured again\n",
                           static_cast<int>(m_spi.GetPort()), m_csGpio);
  }
}

bool TMAG5170::Configure(Range range, Averaging averaging, AnglePlane plane) {
  std::scoped_lock lock{m_mutex};
  m_range = range;
  m_averaging = averaging;
  m_plane = plane;
  return ApplyConfigLocked();
}

TMAG5170::Field TMAG5170::GetField() {
  std::scoped_lock lock{m_mutex};
  EnsureConfiguredLocked();
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t z = 0;
  // a reset seen in these frames means the values came from an unconfigured chip
  if (m_configured && ReadRegisterLocked(kRegXResult, &x) &&
      ReadRegisterLocked(kRegYResult, &y) && ReadRegisterLocked(kRegZResult, &z) &&
      !m_resetSeen) {
    const double range = RangeMillitesla(m_range, m_isA2);
    m_field.x = units::millitesla_t{FieldMillitesla(x, range)};
    m_field.y = units::millitesla_t{FieldMillitesla(y, range)};
    m_field.z = units::millitesla_t{FieldMillitesla(z, range)};
  }
  return m_field;
}

units::celsius_t TMAG5170::GetTemperature() {
  std::scoped_lock lock{m_mutex};
  EnsureConfiguredLocked();
  uint16_t raw = 0;
  if (m_configured && ReadRegisterLocked(kRegTempResult, &raw) && !m_resetSeen) {
    // equation 3 with the typical TSENS_T0, TADCT0 and TADCRES of 6.5 (no per-chip calibration)
    m_temperature = 25.0 + (static_cast<double>(raw) - 17522.0) / 60.0;
  }
  return units::celsius_t{m_temperature};
}

units::degree_t TMAG5170::GetAngle() {
  std::scoped_lock lock{m_mutex};
  EnsureConfiguredLocked();
  uint16_t raw = 0;
  if (m_configured && ReadRegisterLocked(kRegAngleResult, &raw) && !m_resetSeen) {
    // equation 7: bits 12..4 whole degrees, bits 3..0 sixteenths; bits 15..13 are 0
    m_angle = (raw & 0x1FFF) / 16.0;
  }
  return units::degree_t{m_angle};
}

units::millitesla_t TMAG5170::GetRange() const {
  std::scoped_lock lock{m_mutex};
  return units::millitesla_t{RangeMillitesla(m_range, m_isA2)};
}

bool TMAG5170::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_versionOk && m_configured && m_lastFrameOk;
}

int TMAG5170::GetCrcErrors() const {
  std::scoped_lock lock{m_mutex};
  return m_crcErrors;
}

void TMAG5170::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("X", m_field.x.value());
  table.Log("Y", m_field.y.value());
  table.Log("Z", m_field.z.value());
  table.Log("Angle", m_angle);
  table.Log("Temperature", m_temperature);
  table.Log("Connected", m_versionOk && m_configured && m_lastFrameOk);
  table.Log("CRC Errors", m_crcErrors);
}

std::string_view TMAG5170::GetTelemetryType() const {
  return "Magnetic Sensor";
}
