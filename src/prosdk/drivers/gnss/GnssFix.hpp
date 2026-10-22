// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// One GNSS position solution, as reported by prosdk::UbloxGnss (UBX NAV-PVT) and
// prosdk::NmeaGnss (NMEA RMC/GGA/GSA). design/boardoza/P7-gnss.md.

#pragma once

#include <stdint.h>

#include <limits>

#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"

namespace prosdk {

/** Kind of position fix (the UBX NAV-PVT fixType values). */
enum class GnssFixType : uint8_t {
  kNoFix = 0,
  kDeadReckoning = 1,
  k2D = 2,
  k3D = 3,
  kGnssDeadReckoning = 4,
  kTimeOnly = 5,
};

/** UTC date and time of a fix. */
struct GnssTime {
  int year = 0;          ///< four digits
  int month = 0;         ///< 1 to 12
  int day = 0;           ///< 1 to 31
  int hour = 0;          ///< 0 to 23
  int minute = 0;        ///< 0 to 59
  double second = 0.0;   ///< 0 to 60 (60 during a leap second), with the fraction
  bool dateValid = false;
  bool timeValid = false;
};

/**
 * One position solution. Latitude and longitude are WGS84 degrees (north and east positive).
 * heading is the course over ground in degrees clockwise from true north (the compass
 * convention, not the counter-clockwise WPILib one); it is meaningless while the receiver is not
 * moving. Fields a source does not report are NaN.
 */
struct GnssFix {
  /// True when the receiver reports a usable position (UBX gnssFixOK, NMEA status "A").
  bool valid = false;
  GnssFixType fixType = GnssFixType::kNoFix;
  prosdk::units::degree_t latitude{0.0};
  prosdk::units::degree_t longitude{0.0};
  /// Height above mean sea level.
  prosdk::units::meter_t heightMsl{0.0};
  prosdk::units::meters_per_second_t groundSpeed{0.0};
  prosdk::units::degree_t heading{0.0};
  /// Satellites used in the solution.
  int numSatellites = 0;
  /// Horizontal accuracy estimate (UBX only; NaN from NMEA).
  prosdk::units::meter_t hAcc{std::numeric_limits<double>::quiet_NaN()};
  /// Horizontal dilution of precision (NMEA GGA only; NaN from UBX).
  double hdop = std::numeric_limits<double>::quiet_NaN();
  GnssTime utc;
  /// Monotonic time (prosdk::Timer::GetMonotonicTimestamp) when the solution was received.
  prosdk::units::second_t timestamp{0.0};
};

}  // namespace prosdk
