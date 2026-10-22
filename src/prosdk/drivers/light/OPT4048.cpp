// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// OPT4048 driver (OPT4048.hpp). Registers: TI OPT4048 data sheet SBOSA84 (December 2022),
// section 8.4.1: channel outputs 0h-7h (Figures 8-10 to 8-25), configuration Ah (8.4.1.11,
// reset 3208h), INT and burst configuration Bh (8.4.1.12, reset 8011h, I2C_BURST = 1 so the
// 16-byte read auto-increments, 8.3.4.7.2.2), device ID 11h (8.4.1.14). All registers are 16 bits,
// MSB first. Conversion: section 8.3.4.5 equations 1-9; the XYZ matrix is the example of section
// 9.2.4, equation 16 (an application example, not part of the TI component specification).
// Conversion time codes: Table 8-4. Addresses: section 8.3.4.7.1.1.

#include "prosdk/drivers/light/OPT4048.hpp"

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegChannel0 = 0x00;  // 8 registers: MSB and LSB of channels 0 to 3
constexpr int kRegConfig = 0x0A;
constexpr int kRegDeviceId = 0x11;

// QWAKE = 0, bit 14 = 0, RANGE[13:10] = 12 automatic, CONVERSION_TIME[9:6] = 8 (100 ms per
// channel), OPERATING_MODE[5:4] = 3 continuous, LATCH[3] = 1 (reset value), INT_POL = 0,
// FAULT_COUNT = 0
constexpr uint16_t kConfig = 0x3238;
constexpr uint16_t kConfigModeMask = 0x0030;  // OPERATING_MODE; 0 = power-down, the reset state

// Register 11h: DIDH[11:0]. The register header says reset 820h, the field table says DIDH = 21h
// and DIDL = 8h, which does not fit 2 bits. Accept 820h and 821h (unverified which one the chip
// reads).
constexpr uint16_t kDeviceIdMask = 0x0FFF;

// Equation 16 (rows: channels 0 to 3; columns: X, Y, Z, lux)
constexpr double kMatrix[4][4] = {
    {2.34892992e-4, -1.89652390e-5, 1.20811684e-5, 0.0},
    {4.07467441e-5, 1.98958202e-4, -1.58848115e-5, 2.15e-3},
    {9.28619404e-5, -1.69739553e-5, 6.74021520e-4, 0.0},
    {0.0, 0.0, 0.0, 0.0},
};

bool ReadWord(I2C& i2c, int reg, uint16_t& value) {
  uint8_t buf[2];
  if (i2c.Read(reg, 2, buf)) {
    return false;
  }
  value = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
  return true;
}

bool WriteWord(I2C& i2c, int reg, uint16_t value) {
  uint8_t buf[3] = {static_cast<uint8_t>(reg), static_cast<uint8_t>(value >> 8),
                    static_cast<uint8_t>(value)};
  return !i2c.WriteBulk(buf, 3);
}

int Bit(uint32_t value, int bit) {
  return static_cast<int>((value >> bit) & 1u);
}

}  // namespace

OPT4048::OPT4048(I2C::Port port, int address) : m_i2c{port, address} {
  if (!Configure()) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] OPT4048 0x{:02X} on I2C port {}: no answer or wrong ID\n",
                           address, static_cast<int>(port));
  }
  util::ReportUsage("OPT4048", address, "");
}

bool OPT4048::Configure() {
  uint16_t id = 0;
  bool ok = ReadWord(m_i2c, kRegDeviceId, id);
  if (ok) {
    uint16_t didh = id & kDeviceIdMask;
    ok = didh == 0x0820 || didh == 0x0821;
    if (!m_idLogged) {  // once: Update() retries a wrong chip every call
      m_idLogged = true;
      platform::ConsolePrint(ok ? platform::ConsoleLevel::kInfo : platform::ConsoleLevel::kWarning,
                             "[prosdk] OPT4048 0x{:02X}: device ID register 0x{:04X}\n",
                             m_i2c.GetDeviceAddress(), id);
    }
  }
  ok = ok && WriteWord(m_i2c, kRegConfig, kConfig);
  std::scoped_lock lock{m_mutex};
  m_configured = ok;
  m_connected = ok;
  return ok;
}

OPT4048::Channel OPT4048::DecodeChannel(uint16_t msbRegister, uint16_t lsbRegister) {
  Channel channel;
  channel.exponent = static_cast<uint8_t>(msbRegister >> 12);
  // equation 1: MANTISSA = (RESULT_MSB[11:0] << 8) + RESULT_LSB[7:0]
  channel.mantissa = (static_cast<uint32_t>(msbRegister & 0x0FFF) << 8) | (lsbRegister >> 8);
  channel.counter = static_cast<uint8_t>((lsbRegister >> 4) & 0x0F);
  channel.crc = static_cast<uint8_t>(lsbRegister & 0x0F);
  return channel;
}

uint8_t OPT4048::ChannelCrc(const Channel& channel) {
  // Figure 8-12 with R = mantissa, E = exponent and C = the sample counter. The datasheet writes
  // CRC_CHx where C stands; a CRC cannot cover itself, so the counter is meant (unverified).
  uint32_t r = channel.mantissa;
  uint32_t e = channel.exponent;
  uint32_t c = channel.counter;
  int x0 = 0;
  for (int i = 0; i < 4; ++i) {
    x0 ^= Bit(e, i) ^ Bit(c, i);
  }
  for (int i = 0; i < 20; ++i) {
    x0 ^= Bit(r, i);
  }
  int x1 = Bit(c, 1) ^ Bit(c, 3) ^ Bit(e, 1) ^ Bit(e, 3);
  for (int i = 1; i < 20; i += 2) {
    x1 ^= Bit(r, i);
  }
  int x2 = Bit(c, 3) ^ Bit(r, 3) ^ Bit(r, 7) ^ Bit(r, 11) ^ Bit(r, 15) ^ Bit(r, 19) ^ Bit(e, 3);
  int x3 = Bit(r, 3) ^ Bit(r, 11) ^ Bit(r, 19);
  return static_cast<uint8_t>(x0 | (x1 << 1) | (x2 << 2) | (x3 << 3));
}

void OPT4048::Update() {
  bool configured;
  {
    std::scoped_lock lock{m_mutex};
    configured = m_configured;
  }
  if (!configured && !Configure()) {
    return;
  }
  uint16_t config = 0;
  bool ok = ReadWord(m_i2c, kRegConfig, config);
  if (ok && (config & kConfigModeMask) == 0) {
    // powered down: the chip lost power and came back with its reset value
    ok = WriteWord(m_i2c, kRegConfig, kConfig);
  }
  uint8_t buf[16];
  ok = ok && !m_i2c.Read(kRegChannel0, sizeof(buf), buf);
  if (!ok) {
    std::scoped_lock lock{m_mutex};
    m_connected = false;
    return;
  }

  double out[4] = {0.0, 0.0, 0.0, 0.0};  // X, Y, Z, lux (equation 5)
  bool crcOk = true;
  for (int ch = 0; ch < 4; ++ch) {
    auto msb = static_cast<uint16_t>((buf[4 * ch] << 8) | buf[4 * ch + 1]);
    auto lsb = static_cast<uint16_t>((buf[4 * ch + 2] << 8) | buf[4 * ch + 3]);
    Channel channel = DecodeChannel(msb, lsb);
    crcOk = crcOk && ChannelCrc(channel) == channel.crc;
    // equation 3: ADC_CODES = MANTISSA << EXPONENT (at most 28 bits)
    double adc = static_cast<double>(static_cast<uint64_t>(channel.mantissa) << channel.exponent);
    for (int k = 0; k < 4; ++k) {
      out[k] += adc * kMatrix[ch][k];
    }
  }

  std::scoped_lock lock{m_mutex};
  m_connected = true;
  if (!crcOk) {
    ++m_crcErrors;
    return;
  }
  m_xyz = XYZ{out[0], out[1], out[2]};
  m_lux = out[3];  // equation 9: channel 1 times 2.15e-3
}

OPT4048::XYZ OPT4048::GetXYZ() const {
  std::scoped_lock lock{m_mutex};
  return m_xyz;
}

units::lux_t OPT4048::GetLux() const {
  std::scoped_lock lock{m_mutex};
  return units::lux_t{m_lux};
}

OPT4048::Chromaticity OPT4048::GetCIExy() const {
  std::scoped_lock lock{m_mutex};
  double sum = m_xyz.x + m_xyz.y + m_xyz.z;
  if (sum <= 0.0) {
    return Chromaticity{};
  }
  return Chromaticity{m_xyz.x / sum, m_xyz.y / sum};  // equations 6 and 7
}

uint32_t OPT4048::GetCrcErrors() const {
  std::scoped_lock lock{m_mutex};
  return m_crcErrors;
}

bool OPT4048::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void OPT4048::LogTo(telemetry::TelemetryTable& table) const {
  Chromaticity cie = GetCIExy();
  std::scoped_lock lock{m_mutex};
  table.Log("X", m_xyz.x);
  table.Log("Y", m_xyz.y);
  table.Log("Z", m_xyz.z);
  table.Log("Lux", m_lux);
  table.Log("CIE x", cie.x);
  table.Log("CIE y", cie.y);
  table.Log("CRC Errors", static_cast<int64_t>(m_crcErrors));
  table.Log("Connected", m_connected);
}

std::string_view OPT4048::GetTelemetryType() const {
  return "Color Sensor";
}
