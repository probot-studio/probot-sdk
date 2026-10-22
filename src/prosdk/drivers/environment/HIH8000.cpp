// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// HIH8000 driver (HIH8000.hpp). Protocol: Honeywell technical note "I2C Communication with the
// Honeywell HumidIcon Digital Humidity/Temperature Sensors" 009061-2-EN (June 2012): section 2.3
// (Measurement Request = address + write bit, no data), 2.5 (4-byte Data Fetch, Figure 4), 2.6
// (status bits, Table 1), 3.0 (measurement cycle typically 36.65 ms), 4.0 and 5.0 (conversion).
// The same formulas are in the HIH8000 series data sheet 009075-7, Table 2 note 1.

#include "prosdk/drivers/environment/HIH8000.hpp"

#include <stdint.h>

#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// The note gives the cycle as typically 36.65 ms with no maximum (so 40 ms is unverified); a fetch
// that comes too early reads status "stale" and is simply repeated on the next Update().
constexpr double kMeasurementTime = 0.040;  // s
constexpr double kRetryTime = 0.5;          // s without valid data before a new request
constexpr double kStaleAge = 1.0;           // s
constexpr double kFullScale = 16382.0;      // 2^14 - 2 counts

constexpr int kStatusNormal = 0;  // S1 S0 = 00: valid data not fetched before
constexpr int kStatusStale = 1;   // 01: already fetched, or the cycle is not finished

}  // namespace

HIH8000::HIH8000(I2C::Port port, int address) : m_i2c{port, address} {
  util::ReportUsage("HIH8000", address, "");
}

// m_mutex held.
bool HIH8000::Request(double now) {
  m_connected = !m_i2c.AddressOnly();  // Measurement Request: address + write, then stop
  m_requested = m_connected;
  m_requestTime = now;
  return m_connected;
}

void HIH8000::Update() {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  if (!m_requested) {
    Request(now);  // a failure shows in IsConnected(); the next Update() tries again
    return;
  }
  if (now - m_requestTime < kMeasurementTime) {
    return;
  }
  uint8_t buf[4];
  if (m_i2c.ReadOnly(sizeof(buf), buf)) {
    m_connected = false;
    m_requested = false;  // request again next time
    return;
  }
  m_connected = true;
  int status = buf[0] >> 6;
  if (status == kStatusNormal) {
    // humidity: 14 bits after the status bits; temperature: 14 bits, the low 2 bits of byte 4
    // are "do not care" (Figure 4)
    int humidityCounts = ((buf[0] & 0x3F) << 8) | buf[1];
    int temperatureCounts = (buf[2] << 6) | (buf[3] >> 2);
    m_humidity = humidityCounts * 100.0 / kFullScale;
    m_temperature = temperatureCounts * 165.0 / kFullScale - 40.0;
    m_sampleTime = now;
    Request(now);
  } else if (status != kStatusStale || now - m_requestTime > kRetryTime) {
    // command mode (10) or not used (11), or no new data for too long: request again
    m_requested = false;
  }
}

double HIH8000::GetHumidity() const {
  std::scoped_lock lock{m_mutex};
  return m_humidity;
}

units::celsius_t HIH8000::GetTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_temperature};
}

bool HIH8000::IsStale() const {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  return m_sampleTime < 0.0 || now - m_sampleTime > kStaleAge;
}

bool HIH8000::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void HIH8000::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Humidity", m_humidity);
  table.Log("Temperature", m_temperature);
  table.Log("Connected", m_connected);
}

std::string_view HIH8000::GetTelemetryType() const {
  return "Humidity Sensor";
}
