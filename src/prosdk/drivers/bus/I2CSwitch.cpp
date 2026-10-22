// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// I2CSwitch driver (I2CSwitch.hpp). Protocol: 3PEAK TPT29545 datasheet CA20241229A1, "Control
// Register" (one byte, the last byte of a write is kept) and table 2 (bit n = channel n, 0 after
// power-up or reset). The address figure (figure 4) is an image; the 0x70-0x73 range comes from
// the Boardoza board README.

#include "prosdk/drivers/bus/I2CSwitch.hpp"

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

I2CSwitch::I2CSwitch(I2C::Port port, int address, int channels)
    : m_i2c{port, address}, m_address{address}, m_channels{channels}, m_channelMask{0} {
  if (channels < 1 || channels > kMaxChannels) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "I2CSwitch channels {} (valid: 1 to {})",
                           channels, kMaxChannels);
  }
  m_channelMask = static_cast<uint8_t>((1u << channels) - 1);
  if (!Select(0)) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] I2CSwitch 0x{:02X}: no answer on I2C port {}\n", address,
                           static_cast<int>(port));
  }
  util::ReportUsage("I2CSwitch", address, "");
}

bool I2CSwitch::Select(uint8_t mask) {
  if ((mask & ~m_channelMask) != 0) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "I2CSwitch mask 0x{:02X} (valid: channels 0 to {})", mask,
                           m_channels - 1);
  }
  std::scoped_lock lock{m_mutex};
  // a single byte, no register address (WPILib I2C returns true when aborted)
  bool ok = !m_i2c.WriteBulk(&mask, 1);
  m_connected = ok;
  if (ok) {
    m_selected = mask;
  }
  return ok;
}

bool I2CSwitch::SelectOnly(int channel) {
  if (channel < 0 || channel >= m_channels) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "I2CSwitch channel {} (valid: 0 to {})",
                           channel, m_channels - 1);
  }
  return Select(static_cast<uint8_t>(1u << channel));
}

std::optional<uint8_t> I2CSwitch::GetSelected() {
  std::scoped_lock lock{m_mutex};
  uint8_t control = 0;
  bool ok = !m_i2c.ReadOnly(1, &control);
  m_connected = ok;
  if (!ok) {
    return std::nullopt;
  }
  return static_cast<uint8_t>(control & m_channelMask);
}

int I2CSwitch::GetChannels() const {
  return m_channels;
}

int I2CSwitch::GetAddress() const {
  return m_address;
}

bool I2CSwitch::IsConnected() const {
  return m_connected;
}

void I2CSwitch::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Selected", static_cast<int32_t>(m_selected.load()));
  table.Log("Connected", m_connected.load());
}

std::string_view I2CSwitch::GetTelemetryType() const {
  return "I2C Switch";
}
