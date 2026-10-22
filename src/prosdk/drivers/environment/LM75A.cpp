// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// LM75A driver (LM75A.hpp). Registers: NXP LM75A product data sheet Rev. 04 (10 July 2007),
// section 7.4 (Table 5 registers, Table 8 Conf, Table 9 Temp) and 7.9 (power-up defaults).

#include "prosdk/drivers/environment/LM75A.hpp"

#include <stdint.h>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegTemp = 0x00;  // 2 bytes, MSByte first
constexpr int kRegConf = 0x01;

constexpr uint8_t kConfNormal = 0x00;  // normal mode, comparator, OS active low, fault queue 1
constexpr uint8_t kConfShutdown = 0x01;

}  // namespace

LM75A::LM75A(I2C::Port port, int address) : m_i2c{port, address} {
  // also clears a shutdown left from an earlier program; Table 8 power-on value
  m_connected = !m_i2c.Write(kRegConf, kConfNormal);
  if (!m_connected) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] LM75A 0x{:02X} on I2C port {}: no answer\n", address,
                           static_cast<int>(port));
  }
  util::ReportUsage("LM75A", address, "");
}

units::celsius_t LM75A::GetTemperature() {
  uint8_t buf[2];
  bool ok = !m_i2c.Read(kRegTemp, sizeof(buf), buf);
  std::scoped_lock lock{m_mutex};
  m_connected = ok;
  if (ok) {
    // D10..D0 in the top 11 bits, two's complement, 0.125 C per LSB (Table 9): the signed
    // 16-bit word divided by 256
    auto word = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    m_temperature = (word >> 5) * 0.125;
  }
  return units::celsius_t{m_temperature};
}

void LM75A::SetShutdown(bool shutdown) {
  bool ok = !m_i2c.Write(kRegConf, shutdown ? kConfShutdown : kConfNormal);
  std::scoped_lock lock{m_mutex};
  m_connected = ok;
}

bool LM75A::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void LM75A::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Temperature", m_temperature);
  table.Log("Connected", m_connected);
}

std::string_view LM75A::GetTelemetryType() const {
  return "Temperature Sensor";
}
