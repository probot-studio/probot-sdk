// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NMEA GNSS driver (NmeaGnss.hpp). Fix flags as in the u-blox M10 Interface Description
// UBX-21035062 R03 section 2.5.5: RMC status A = valid, GGA quality 0 = no fix, 6 = dead
// reckoning, GSA navMode 1 = no fix, 2 = 2D, 3 = 3D.

#include "prosdk/drivers/gnss/NmeaGnss.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <format>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr double kMetersPerSecondPerKnot = 1852.0 / 3600.0;  // 1 knot = 1 nautical mile per hour
constexpr int kGgaDeadReckoning = 6;
constexpr double kWarnAfterSilence = 5.0;  // s from the start without a valid sentence

}  // namespace

NmeaGnss::NmeaGnss(SerialPort::Port port, int baudRate, units::millisecond_t updatePeriod)
    : m_name{std::format("serial port {} ({} baud)", static_cast<int>(port), baudRate)},
      m_updatePeriod{updatePeriod} {
  if (m_updatePeriod > 0_ms) {
    m_serial.emplace(baudRate, port);
    m_serial->SetTimeout(0_s);
    util::ReportUsage("NmeaGnss", static_cast<int>(port), "");
    platform::ScopedThreadConfig threadConfig{{.name = "nmea-gnss"}};
    m_thread = std::thread([this] { Run(); });
  }
}

NmeaGnss::~NmeaGnss() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void NmeaGnss::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  std::array<char, 256> chunk;
  double start = Timer::GetMonotonicTimestamp().value();
  bool checkedSilence = false;
  while (!m_stop) {
    int available = m_serial->GetBytesReceived();
    while (available > 0 && !m_stop) {
      int n = m_serial->Read(chunk.data(),
                             available < static_cast<int>(chunk.size())
                                 ? available
                                 : static_cast<int>(chunk.size()));
      if (n <= 0) {
        break;
      }
      Feed({reinterpret_cast<const uint8_t*>(chunk.data()), static_cast<size_t>(n)});
      available -= n;
    }
    if (!checkedSilence && Timer::GetMonotonicTimestamp().value() - start > kWarnAfterSilence) {
      checkedSilence = true;
      if (GetSentenceCount() == 0) {
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] NmeaGnss on {}: no valid NMEA sentence within 5 s "
                               "({} dropped; check the wiring and the baud rate)\n",
                               m_name, GetChecksumErrors());
      }
    }
    std::this_thread::sleep_for(period);
  }
}

void NmeaGnss::Feed(std::span<const uint8_t> bytes) {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock parse{m_parseMutex};
  for (uint8_t b : bytes) {
    if (m_reader.Push(b)) {
      Handle(m_reader.Body(), now);
    }
  }
}

void NmeaGnss::Handle(std::string_view body, double now) {
  std::scoped_lock lock{m_mutex};
  m_lastSentence = now;
  GnssFix& fix = m_fix;
  if (auto rmc = nmea::ParseRmc(body)) {
    fix.valid = rmc->valid;
    if (rmc->hasTime) {
      fix.utc.hour = rmc->hour;
      fix.utc.minute = rmc->minute;
      fix.utc.second = rmc->second;
    }
    fix.utc.timeValid = rmc->hasTime;
    if (rmc->hasDate) {
      fix.utc.year = rmc->year;
      fix.utc.month = rmc->month;
      fix.utc.day = rmc->day;
    }
    fix.utc.dateValid = rmc->hasDate;
    if (rmc->valid && !std::isnan(rmc->latitude) && !std::isnan(rmc->longitude)) {
      fix.latitude = units::degree_t{rmc->latitude};
      fix.longitude = units::degree_t{rmc->longitude};
    }
    if (!std::isnan(rmc->speedKnots)) {
      fix.groundSpeed = units::meters_per_second_t{rmc->speedKnots * kMetersPerSecondPerKnot};
    }
    if (!std::isnan(rmc->courseDegrees)) {
      fix.heading = units::degree_t{rmc->courseDegrees};
    }
    fix.timestamp = units::second_t{now};
  } else if (auto gga = nmea::ParseGga(body)) {
    fix.valid = gga->quality > 0;
    m_deadReckoning = gga->quality == kGgaDeadReckoning;
    if (gga->numSatellites >= 0) {
      fix.numSatellites = gga->numSatellites;
    }
    fix.hdop = gga->hdop;
    if (fix.valid && !std::isnan(gga->latitude) && !std::isnan(gga->longitude)) {
      fix.latitude = units::degree_t{gga->latitude};
      fix.longitude = units::degree_t{gga->longitude};
    }
    if (fix.valid && !std::isnan(gga->altitudeMsl)) {
      fix.heightMsl = units::meter_t{gga->altitudeMsl};
    }
    fix.timestamp = units::second_t{now};
  } else if (auto gsa = nmea::ParseGsa(body)) {
    m_gsaMode = gsa->navMode;
  } else {
    return;  // another sentence (GSV, VTG, TXT, ...)
  }
  if (!fix.valid) {
    fix.fixType = GnssFixType::kNoFix;
  } else if (m_deadReckoning) {
    fix.fixType = GnssFixType::kDeadReckoning;
  } else {
    fix.fixType = m_gsaMode == 3 ? GnssFixType::k3D : GnssFixType::k2D;
  }
}

GnssFix NmeaGnss::GetFix() const {
  std::scoped_lock lock{m_mutex};
  return m_fix;
}

bool NmeaGnss::IsConnected() const {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  return m_lastSentence >= 0.0 && now - m_lastSentence <= units::second_t{kTimeout}.value();
}

uint32_t NmeaGnss::GetSentenceCount() const {
  std::scoped_lock parse{m_parseMutex};
  return m_reader.GetSentenceCount();
}

uint32_t NmeaGnss::GetChecksumErrors() const {
  std::scoped_lock parse{m_parseMutex};
  return m_reader.GetErrorCount();
}

void NmeaGnss::LogTo(telemetry::TelemetryTable& table) const {
  GnssFix fix = GetFix();
  table.Log("Valid", fix.valid);
  table.Log("Fix Type", static_cast<int32_t>(fix.fixType));
  table.Log("Latitude", fix.latitude.value());
  table.Log("Longitude", fix.longitude.value());
  table.Log("Height MSL", fix.heightMsl.value());
  table.Log("Ground Speed", fix.groundSpeed.value());
  table.Log("Heading", fix.heading.value());
  table.Log("Satellites", static_cast<int32_t>(fix.numSatellites));
  table.Log("HDOP", fix.hdop);
}

std::string_view NmeaGnss::GetTelemetryType() const {
  return "GNSS";
}
