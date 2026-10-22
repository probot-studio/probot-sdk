// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NMEA 0183 sentence reader and the GGA, RMC and GSA decoders (no HAL; used by prosdk::NmeaGnss,
// design/boardoza/P7-gnss.md). Format from the u-blox M8 Receiver Description UBX-13003221 R28
// section 31.1 and the u-blox M10 SPG 5.10 Interface Description UBX-21035062 R03 section 2
// (both describe NMEA 0183 4.10, which is what GNSS receivers in general output):
//
//   $<talker 2><type 3>,<field>,<field>,...*<checksum 2 hex>\r\n
//
// The checksum is the XOR of every character between '$' and '*' (M8 section 31.1.1). Any
// two-letter talker is accepted (GP, GN, GL, GA, GB, BD, ...); the decoders look at the sentence
// type only. Latitude and longitude are (d)ddmm.mmmm plus a hemisphere letter (M10 section 2.5.3).

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace prosdk::nmea {

/// Value of a number field that is empty or missing.
inline constexpr double kNoValue = std::numeric_limits<double>::quiet_NaN();

/** XOR checksum of a sentence body (the characters between '$' and '*'). */
uint8_t Checksum(std::string_view body);

/** Builds a complete sentence "$<body>*hh\r\n" (for tests and simulators). */
std::string Encode(std::string_view body);

/**
 * The sentence type of a body: the last three characters of its address field ("GGA" for
 * "GNGGA,..."), or an empty view for a proprietary or malformed address.
 */
std::string_view SentenceType(std::string_view body);

/** GGA: time, position and fix data (M10 section 2.7.5). Unreported numbers are NaN or -1. */
struct Gga {
  bool hasTime = false;
  int hour = 0;
  int minute = 0;
  double second = 0.0;
  double latitude = kNoValue;         ///< degrees, north positive (NaN when empty)
  double longitude = kNoValue;        ///< degrees, east positive (NaN when empty)
  int quality = 0;         ///< 0 no fix, 1 autonomous, 2 differential, 4/5 RTK, 6 dead reckoning
  int numSatellites = -1;  ///< satellites used
  double hdop = kNoValue;
  double altitudeMsl = kNoValue;      ///< m above mean sea level
  double geoidSeparation = kNoValue;  ///< m, ellipsoid minus mean sea level
};

/** RMC: recommended minimum data (M10 section 2.7.17). Unreported numbers are NaN. */
struct Rmc {
  bool hasTime = false;
  int hour = 0;
  int minute = 0;
  double second = 0.0;
  bool hasDate = false;
  int year = 0;            ///< 2000 + the two-digit year
  int month = 0;
  int day = 0;
  bool valid = false;      ///< status "A" (data valid); "V" is invalid
  double latitude = kNoValue;
  double longitude = kNoValue;
  double speedKnots = kNoValue;
  double courseDegrees = kNoValue;    ///< course over ground, clockwise from true north
  char posMode = '\0';     ///< mode indicator (NMEA 2.3 and later), '\0' when absent
};

/** GSA: DOP and active satellites (M10 section 2.7.13). */
struct Gsa {
  int navMode = 0;  ///< 1 no fix, 2 2D, 3 3D
  double pdop = kNoValue;
  double hdop = kNoValue;
  double vdop = kNoValue;
};

/** Decodes a GGA body; nullopt for another type or too few fields. */
std::optional<Gga> ParseGga(std::string_view body);
/** Decodes an RMC body; nullopt for another type or too few fields. */
std::optional<Rmc> ParseRmc(std::string_view body);
/** Decodes a GSA body; nullopt for another type or too few fields. */
std::optional<Gsa> ParseGsa(std::string_view body);

/**
 * Splits a byte stream into sentences and checks their checksums. Sentences may span Push calls;
 * bytes outside a sentence (for example interleaved binary messages) are skipped.
 */
class Reader {
 public:
  /// Longest accepted body (characters between '$' and '*'); NMEA allows 79, high-precision
  /// receiver modes write a few more.
  static constexpr size_t kMaxBodyLength = 120;

  /**
   * Adds one byte. Returns true when it ended a sentence whose checksum matched; Body() holds that
   * sentence until the next Push.
   */
  bool Push(uint8_t byte);

  /** The body of the last complete sentence (valid after Push returned true). */
  std::string_view Body() const;

  /** Sentences that passed the checksum. */
  uint32_t GetSentenceCount() const { return m_sentences; }
  /** Sentences dropped: checksum mismatch, missing checksum, too long or a stray control byte. */
  uint32_t GetErrorCount() const { return m_errors; }

 private:
  std::array<char, kMaxBodyLength + 3> m_buf{};  // body plus "*hh"
  size_t m_len = 0;
  size_t m_bodyLen = 0;
  bool m_inSentence = false;
  uint32_t m_sentences = 0;
  uint32_t m_errors = 0;
};

}  // namespace prosdk::nmea
