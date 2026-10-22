// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ENS210 driver (ENS210.hpp). Registers and timing: TH09C data sheet V1.01: Figure 9 (timing),
// Figure 22 (register overview), Figures 23 to 33 (PART_ID, SYS_STAT, SENS_RUN, SENS_START,
// T_VAL, H_VAL), "The Sensor Readout Registers" (valid flag, CRC-7, double buffering), "Computing
// CRC-7" (polynomial x^7 + x^3 + 1, initial vector 0x7F, 17-bit payload) and "Reading PART_ID,
// DIE_REV and UID" (PART_ID is readable only while the device is active).

#include "prosdk/drivers/environment/ENS210.hpp"

#include <stdint.h>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegPartId = 0x00;     // 2 bytes, little endian, active state only
constexpr int kRegSysStat = 0x11;    // bit 0 SYS_ACTIVE
constexpr int kRegSensRun = 0x21;    // bit 1 H_RUN, bit 0 T_RUN (1 = continuous)
constexpr int kRegSensStart = 0x22;  // bit 1 H_START, bit 0 T_START
constexpr int kRegTVal = 0x30;       // T_VAL 0x30..0x32, H_VAL 0x33..0x35

constexpr uint8_t kBothSensors = 0x03;

constexpr double kRetryTime = 1.0;   // s between start attempts after a failure
constexpr double kActiveTime = 0.1;  // s; booting takes 1.2 ms max (Figure 9)

// CRC-7 of a 17-bit payload: the payload followed by the 7-bit initial vector, divided by the
// polynomial 0x89 (x^7 + x^3 + 1); the remainder is the CRC.
uint32_t Crc7(uint32_t payload) {
  uint32_t value = ((payload & 0x1FFFF) << 7) | 0x7F;
  for (int bit = 16 + 7; bit >= 7; --bit) {
    if (value & (1u << bit)) {
      value ^= 0x89u << (bit - 7);
    }
  }
  return value & 0x7F;
}

// One T_VAL or H_VAL register (3 bytes, little endian): bits 15:0 data, bit 16 valid,
// bits 23:17 CRC. Returns false when the value is not valid; sets crcError on a CRC mismatch.
bool Decode(const uint8_t* bytes, int& data, bool& crcError) {
  uint32_t raw = bytes[0] | (static_cast<uint32_t>(bytes[1]) << 8) |
                 (static_cast<uint32_t>(bytes[2]) << 16);
  crcError = Crc7(raw & 0x1FFFF) != (raw >> 17);
  data = static_cast<int>(raw & 0xFFFF);
  return !crcError && (raw & 0x10000) != 0;
}

}  // namespace

ENS210::ENS210(I2C::Port port, int address) : m_i2c{port, address}, m_address{address} {
  util::ReportUsage("ENS210", address, "");
}

// m_mutex held.
void ENS210::Fail(double now) {
  m_connected = false;
  m_state = State::kStart;
  m_stateTime = now;
}

void ENS210::Update() {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  switch (m_state) {
    case State::kStart: {
      if (m_everTried && now - m_stateTime < kRetryTime) {
        return;
      }
      m_everTried = true;
      // continuous T and RH; writes to SENS_* take effect between measurements
      if (m_i2c.Write(kRegSensRun, kBothSensors) || m_i2c.Write(kRegSensStart, kBothSensors)) {
        Fail(now);
        return;
      }
      m_connected = true;
      m_state = State::kWaitActive;
      m_stateTime = now;
      return;
    }
    case State::kWaitActive: {
      uint8_t stat;
      if (m_i2c.Read(kRegSysStat, 1, &stat)) {
        Fail(now);
        return;
      }
      m_connected = true;
      if ((stat & 0x01) == 0) {
        if (now - m_stateTime > kActiveTime) {
          m_state = State::kStart;  // start again (after kRetryTime)
          m_stateTime = now;
        }
        return;
      }
      uint8_t id[2];
      if (m_i2c.Read(kRegPartId, sizeof(id), id)) {
        Fail(now);
        return;
      }
      int partId = id[0] | (id[1] << 8);
      if (partId != kPartId && !m_warnedPartId) {
        m_warnedPartId = true;
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] ENS210 0x{:02X} on I2C port {}: PART_ID 0x{:04X}, "
                               "expected 0x{:04X}\n",
                               m_address, static_cast<int>(m_i2c.GetPort()), partId, kPartId);
      }
      m_state = State::kRunning;
      m_stateTime = now;
      return;
    }
    case State::kRunning: {
      // one burst over T_VAL and H_VAL; reading the first byte of each latches that register's
      // 3 bytes (double buffering), so neither value can tear
      uint8_t buf[6];
      if (m_i2c.Read(kRegTVal, sizeof(buf), buf)) {
        Fail(now);
        return;
      }
      m_connected = true;
      int tData = 0;
      int hData = 0;
      bool tCrcError = false;
      bool hCrcError = false;
      bool tValid = Decode(buf, tData, tCrcError);
      bool hValid = Decode(buf + 3, hData, hCrcError);
      m_crcErrors += (tCrcError ? 1 : 0) + (hCrcError ? 1 : 0);
      if (tValid) {
        m_temperature = tData / 64.0 - 273.15;  // 1/64 K per LSB (Figure 32)
      }
      if (hValid) {
        m_humidity = hData / 512.0;  // 1/512 %RH per LSB (Figure 33)
      }
      if (tValid && hValid) {
        m_hasData = true;
      }
      return;
    }
  }
}

units::celsius_t ENS210::GetTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_temperature};
}

double ENS210::GetHumidity() const {
  std::scoped_lock lock{m_mutex};
  return m_humidity;
}

bool ENS210::HasData() const {
  std::scoped_lock lock{m_mutex};
  return m_hasData;
}

int ENS210::GetCrcErrors() const {
  std::scoped_lock lock{m_mutex};
  return m_crcErrors;
}

bool ENS210::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void ENS210::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Temperature", m_temperature);
  table.Log("Humidity", m_humidity);
  table.Log("CRC Errors", m_crcErrors);
  table.Log("Connected", m_connected);
}

std::string_view ENS210::GetTelemetryType() const {
  return "Humidity Sensor";
}
