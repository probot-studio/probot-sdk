// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// OPT3001 driver (OPT3001.hpp). Registers: TI OPT3001 data sheet SBOS681C (revised November 2017),
// section 7.6.1.1: result 00h (7.6.1.1.1, equation 3, Table 9), configuration 01h (7.6.1.1.2,
// Table 10, reset C810h), manufacturer ID 7Eh = 5449h (7.6.1.1.5, Table 14), device ID
// 7Fh = 3001h (7.6.1.1.6, Table 15). All registers are 16 bits, MSB first. Addresses: Table 1.

#include "prosdk/drivers/light/OPT3001.hpp"

#include <stdint.h>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegResult = 0x00;
constexpr int kRegConfig = 0x01;
constexpr int kRegManufacturerId = 0x7E;
constexpr int kRegDeviceId = 0x7F;
constexpr uint16_t kManufacturerId = 0x5449;  // "TI"
constexpr uint16_t kDeviceId = 0x3001;

// RN[15:12] = 1100b automatic full-scale, CT[11] = 1 800 ms, M[10:9] = 10b continuous,
// L[4] = 1 latched (reset value), POL, ME, FC = 0 (Table 10)
constexpr uint16_t kConfig = 0xCC10;
constexpr uint16_t kConfigModeMask = 0x0600;  // M[10:9]; 00b = shutdown, the power-up state

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

}  // namespace

OPT3001::OPT3001(I2C::Port port, int address) : m_i2c{port, address} {
  if (!Configure()) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] OPT3001 0x{:02X} on I2C port {}: no answer or wrong ID\n",
                           address, static_cast<int>(port));
  }
  util::ReportUsage("OPT3001", address, "");
}

bool OPT3001::Configure() {
  uint16_t manufacturer = 0;
  uint16_t device = 0;
  bool ok = ReadWord(m_i2c, kRegManufacturerId, manufacturer) &&
            ReadWord(m_i2c, kRegDeviceId, device) && manufacturer == kManufacturerId &&
            device == kDeviceId && WriteWord(m_i2c, kRegConfig, kConfig);
  std::scoped_lock lock{m_mutex};
  m_configured = ok;
  m_connected = ok;
  return ok;
}

double OPT3001::ResultToLux(uint16_t result) {
  int exponent = result >> 12;
  int mantissa = result & 0x0FFF;
  return 0.01 * static_cast<double>(1 << exponent) * mantissa;
}

void OPT3001::Update() {
  bool configured;
  {
    std::scoped_lock lock{m_mutex};
    configured = m_configured;
  }
  if (!configured && !Configure()) {
    return;
  }
  uint16_t config = 0;
  uint16_t result = 0;
  bool ok = ReadWord(m_i2c, kRegConfig, config);
  if (ok && (config & kConfigModeMask) == 0) {
    // shut down: the chip lost power and came back with its reset value
    ok = WriteWord(m_i2c, kRegConfig, kConfig);
  }
  ok = ok && ReadWord(m_i2c, kRegResult, result);
  std::scoped_lock lock{m_mutex};
  m_connected = ok;
  if (ok) {
    m_lux = ResultToLux(result);
  }
}

units::lux_t OPT3001::GetLux() const {
  std::scoped_lock lock{m_mutex};
  return units::lux_t{m_lux};
}

bool OPT3001::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void OPT3001::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Lux", m_lux);
  table.Log("Connected", m_connected);
}

std::string_view OPT3001::GetTelemetryType() const {
  return "Light Sensor";
}
