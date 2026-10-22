// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// GP8403 driver (GP8403.hpp). Protocol: GP8403 datasheet (StanStrong translation) sections 3.3.3
// (VOUT0 at register 0x02: DATA low byte then high byte, the low 4 bits of the low byte ignored),
// 3.3.4 (VOUT1 at 0x04), 3.3.5 (both from 0x02, 4 data bytes) and 3.3.6 (range at 0x01: 0x00 =
// 0-5 V, 0x11 = 0-10 V). The register figures are images; the byte layout was cross-checked with
// the Boardoza GP8403 library. The base address 0x58 comes from the board silkscreen and the
// Boardoza library (the datasheet's address figure is an image): unverified against the chip.

#include "prosdk/drivers/dac/GP8403.hpp"

#include <algorithm>
#include <cmath>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kRegRange = 0x01;
constexpr uint8_t kRegVout0 = 0x02;  // VOUT1 at 0x04
constexpr uint8_t kRange5V = 0x00;
constexpr uint8_t kRange10V = 0x11;

}  // namespace

GP8403::GP8403(I2C::Port port, int address, Range range)
    : m_i2c{port, address}, m_address{address}, m_range{range} {
  std::scoped_lock lock{m_mutex};
  uint16_t zero[kNumChannels] = {0, 0};
  bool ok = WriteCodes(0, kNumChannels, zero) && WriteRange();
  if (!ok) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] GP8403 0x{:02X}: no answer on I2C port {}\n", address,
                           static_cast<int>(port));
  }
  util::ReportUsage("GP8403", address, "");
}

GP8403::~GP8403() {
  std::scoped_lock lock{m_mutex};
  uint16_t zero[kNumChannels] = {0, 0};
  WriteCodes(0, kNumChannels, zero);
}

void GP8403::CheckChannel(int channel) const {
  if (channel < 0 || channel >= kNumChannels) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "GP8403 channel {} (valid: 0 to {})",
                           channel, kNumChannels - 1);
  }
}

// m_mutex held.
double GP8403::FullScale() const {
  return m_range == Range::k0to10V ? 10.0 : 5.0;
}

// m_mutex held.
uint16_t GP8403::CodeFor(units::volt_t voltage) const {
  double fraction = voltage.value() / FullScale();
  if (!(fraction > 0.0)) {
    fraction = 0.0;  // also NaN
  } else if (fraction > 1.0) {
    fraction = 1.0;
  }
  return static_cast<uint16_t>(std::lround(fraction * kMaxCode));
}

// m_mutex held. WPILib I2C calls return true when the transfer was aborted.
bool GP8403::WriteRange() {
  bool ok = !m_i2c.Write(kRegRange, m_range == Range::k0to10V ? kRange10V : kRange5V);
  m_connected = ok;
  m_rangeWritten = ok;
  return ok;
}

// m_mutex held. Writes codes[0 .. count) to channels first .. first + count in one transfer.
bool GP8403::WriteCodes(int first, int count, const uint16_t* codes) {
  uint8_t buf[1 + 2 * kNumChannels];
  buf[0] = static_cast<uint8_t>(kRegVout0 + 2 * first);
  for (int i = 0; i < count; ++i) {
    uint16_t word = static_cast<uint16_t>(codes[i] << 4);  // left-justified 12 bits
    buf[1 + 2 * i] = static_cast<uint8_t>(word & 0xFF);
    buf[2 + 2 * i] = static_cast<uint8_t>(word >> 8);
  }
  bool ok = !m_i2c.WriteBulk(buf, 1 + 2 * count);
  m_connected = ok;
  for (int i = 0; i < count; ++i) {
    m_written[first + i] = ok && codes[i] == m_codes[first + i];
  }
  return ok;
}

// m_mutex held. Both outputs 0 V, the range, then the stored codes: the order that never
// applies a code with the wrong range (an output never jumps to twice its value).
bool GP8403::WriteAll() {
  uint16_t zero[kNumChannels] = {0, 0};
  WriteCodes(0, kNumChannels, zero);
  return WriteRange() && WriteCodes(0, kNumChannels, m_codes.data());
}

void GP8403::SetRange(Range range) {
  std::scoped_lock lock{m_mutex};
  units::volt_t volts[kNumChannels];
  for (int i = 0; i < kNumChannels; ++i) {
    volts[i] = units::volt_t{m_codes[i] * FullScale() / kMaxCode};
  }
  m_range = range;
  for (int i = 0; i < kNumChannels; ++i) {
    m_codes[i] = CodeFor(volts[i]);
  }
  WriteAll();
}

GP8403::Range GP8403::GetRange() const {
  std::scoped_lock lock{m_mutex};
  return m_range;
}

void GP8403::SetVoltage(int channel, units::volt_t voltage) {
  CheckChannel(channel);
  std::scoped_lock lock{m_mutex};
  uint16_t code = CodeFor(voltage);
  if (code == m_codes[channel] && m_written[channel]) {
    return;
  }
  m_codes[channel] = code;
  if (!m_rangeWritten) {
    WriteAll();
    return;
  }
  WriteCodes(channel, 1, &code);
}

void GP8403::SetBoth(units::volt_t voltage) {
  std::scoped_lock lock{m_mutex};
  uint16_t code = CodeFor(voltage);
  if (code == m_codes[0] && code == m_codes[1] && m_written[0] && m_written[1]) {
    return;
  }
  m_codes = {code, code};
  if (!m_rangeWritten) {
    WriteAll();
    return;
  }
  WriteCodes(0, kNumChannels, m_codes.data());
}

units::volt_t GP8403::GetVoltage(int channel) const {
  CheckChannel(channel);
  std::scoped_lock lock{m_mutex};
  return units::volt_t{m_codes[channel] * FullScale() / kMaxCode};
}

bool GP8403::IsConnected() const {
  return m_connected;
}

int GP8403::GetAddress() const {
  return m_address;
}

void GP8403::LogTo(telemetry::TelemetryTable& table) const {
  double v0, v1;
  {
    std::scoped_lock lock{m_mutex};
    v0 = m_codes[0] * FullScale() / kMaxCode;
    v1 = m_codes[1] * FullScale() / kMaxCode;
  }
  table.Log("Voltage 0", v0);
  table.Log("Voltage 1", v1);
  table.Log("Connected", m_connected.load());
}

std::string_view GP8403::GetTelemetryType() const {
  return "DAC";
}
