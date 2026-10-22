// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// MS5611 driver (MS5611.hpp). MS5611-01BA03 data sheet (TE, 06/2017): ADC table (conversion
// times), Figure 2 (first order compensation), Figure 3 (second order), Figure 4 (commands),
// Figure 5 (2.8 ms PROM reload after reset), "Conversion sequence" (ADC read gives 0 when no
// finished conversion is waiting), Figures 10 to 15 (I2C framing: every command is its own write,
// the answer is a separate read) and Figure 16 (PROM map). CRC-4: Application Note AN520.

#include "prosdk/drivers/environment/MS5611.hpp"

#include <cmath>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kCmdReset = 0x1E;
constexpr uint8_t kCmdConvertD1 = 0x40;  // + 2 * OSR index
constexpr uint8_t kCmdConvertD2 = 0x50;
constexpr uint8_t kCmdAdcRead = 0x00;
constexpr uint8_t kCmdPromRead = 0xA0;  // + 2 * word address

constexpr double kRetryTime = 1.0;    // s between attempts after a failure
constexpr double kReloadTime = 0.003;  // s; 2.8 ms (Figure 5)

// conversion time, max, per OSR (ADC table), plus a little margin
constexpr double kConversionTime[] = {0.0006, 0.00117, 0.00228, 0.00454, 0.00904};
constexpr double kConversionMargin = 0.0002;  // s

}  // namespace

MS5611::MS5611(I2C::Port port, int address, Osr osr)
    : m_i2c{port, address}, m_address{address}, m_osr{osr} {
  util::ReportUsage("MS5611", address, "");
}

int MS5611::PromCrc(std::span<const uint16_t, 8> prom) {
  uint16_t remainder = 0;
  for (int i = 0; i < 16; ++i) {
    uint16_t word = prom[i / 2];
    if (i / 2 == 7) {
      word &= 0xFF00;  // the CRC byte counts as 0
    }
    remainder ^= (i % 2 == 0) ? (word >> 8) : (word & 0xFF);
    for (int bit = 0; bit < 8; ++bit) {
      remainder = (remainder & 0x8000) ? static_cast<uint16_t>((remainder << 1) ^ 0x3000)
                                       : static_cast<uint16_t>(remainder << 1);
    }
  }
  return (remainder >> 12) & 0x0F;
}

void MS5611::Compensate(std::span<const uint16_t, 8> prom, uint32_t d1, uint32_t d2,
                        int32_t& temperature, int32_t& pressure) {
  int64_t c1 = prom[1];
  int64_t c2 = prom[2];
  int64_t c3 = prom[3];
  int64_t c4 = prom[4];
  int64_t c5 = prom[5];
  int64_t c6 = prom[6];
  // first order (Figure 2)
  int64_t dT = static_cast<int64_t>(d2) - c5 * 256;
  int64_t temp = 2000 + dT * c6 / 8388608;  // 2^23
  int64_t off = c2 * 65536 + c4 * dT / 128;
  int64_t sens = c1 * 32768 + c3 * dT / 256;
  // second order (Figure 3)
  if (temp < 2000) {
    int64_t t2 = dT * dT / 2147483648LL;  // 2^31
    int64_t low = (temp - 2000) * (temp - 2000);
    int64_t off2 = 5 * low / 2;
    int64_t sens2 = 5 * low / 4;
    if (temp < -1500) {
      int64_t veryLow = (temp + 1500) * (temp + 1500);
      off2 += 7 * veryLow;
      sens2 += 11 * veryLow / 2;
    }
    temp -= t2;
    off -= off2;
    sens -= sens2;
  }
  temperature = static_cast<int32_t>(temp);
  pressure = static_cast<int32_t>((static_cast<int64_t>(d1) * sens / 2097152 - off) / 32768);
}

bool MS5611::Command(uint8_t command) {
  return m_i2c.WriteBulk(&command, 1);
}

bool MS5611::ReadAdc(uint32_t& value) {
  uint8_t buf[3];
  if (Command(kCmdAdcRead) || m_i2c.ReadOnly(sizeof(buf), buf)) {
    return true;
  }
  value = (static_cast<uint32_t>(buf[0]) << 16) | (buf[1] << 8) | buf[2];
  return false;
}

// m_mutex held.
void MS5611::Fail(double now) {
  m_connected = false;
  m_state = State::kReset;
  m_stateTime = now;
}

void MS5611::Update() {
  double now = Timer::GetMonotonicTimestamp().value();
  int osr = static_cast<int>(m_osr);
  uint8_t convertD1 = static_cast<uint8_t>(kCmdConvertD1 + 2 * osr);
  uint8_t convertD2 = static_cast<uint8_t>(kCmdConvertD2 + 2 * osr);
  double conversionTime = kConversionTime[osr] + kConversionMargin;
  std::scoped_lock lock{m_mutex};
  switch (m_state) {
    case State::kReset: {
      if (m_everTried && now - m_stateTime < kRetryTime) {
        return;
      }
      m_everTried = true;
      if (Command(kCmdReset)) {
        Fail(now);
        return;
      }
      m_connected = true;
      m_state = State::kLoadProm;
      m_stateTime = now;
      return;
    }
    case State::kLoadProm: {
      if (now - m_stateTime < kReloadTime) {
        return;
      }
      for (int i = 0; i < 8; ++i) {
        uint8_t buf[2];
        if (Command(static_cast<uint8_t>(kCmdPromRead + 2 * i)) ||
            m_i2c.ReadOnly(sizeof(buf), buf)) {
          Fail(now);
          return;
        }
        m_prom[i] = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
      }
      if (PromCrc(m_prom) != (m_prom[7] & 0x0F)) {
        if (!m_warnedCrc) {
          m_warnedCrc = true;
          platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                                 "[prosdk] MS5611 0x{:02X} on I2C port {}: PROM CRC mismatch\n",
                                 m_address, static_cast<int>(m_i2c.GetPort()));
        }
        Fail(now);
        return;
      }
      if (Command(convertD2)) {
        Fail(now);
        return;
      }
      m_state = State::kReadD2;
      m_stateTime = now;
      return;
    }
    case State::kReadD2:
    case State::kReadD1: {
      if (now - m_stateTime < conversionTime) {
        return;
      }
      bool readingD2 = m_state == State::kReadD2;
      uint32_t value = 0;
      if (ReadAdc(value)) {
        Fail(now);
        return;
      }
      m_connected = true;
      if (value == 0) {
        // no finished conversion (read too early, or the command was lost): convert again
        if (Command(readingD2 ? convertD2 : convertD1)) {
          Fail(now);
        } else {
          m_stateTime = now;
        }
        return;
      }
      if (readingD2) {
        m_d2 = value;
      } else {
        int32_t temperature = 0;
        int32_t pressure = 0;
        Compensate(m_prom, value, m_d2, temperature, pressure);
        m_temperature = temperature / 100.0;
        m_pressure = pressure;
        m_hasData = true;
      }
      if (Command(readingD2 ? convertD1 : convertD2)) {
        Fail(now);
        return;
      }
      m_state = readingD2 ? State::kReadD1 : State::kReadD2;
      m_stateTime = now;
      return;
    }
  }
}

units::pascal_t MS5611::GetPressure() const {
  std::scoped_lock lock{m_mutex};
  return units::pascal_t{m_pressure};
}

units::celsius_t MS5611::GetTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_temperature};
}

units::meter_t MS5611::GetAltitude(units::pascal_t seaLevel) const {
  std::scoped_lock lock{m_mutex};
  if (!m_hasData) {
    return units::meter_t{0.0};  // not 44330 m from a pressure of 0
  }
  return units::meter_t{44330.0 * (1.0 - std::pow(m_pressure / seaLevel.value(), 1.0 / 5.255))};
}

bool MS5611::HasData() const {
  std::scoped_lock lock{m_mutex};
  return m_hasData;
}

bool MS5611::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void MS5611::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Pressure", m_pressure);
  table.Log("Temperature", m_temperature);
  table.Log("Connected", m_connected);
}

std::string_view MS5611::GetTelemetryType() const {
  return "Barometer";
}
