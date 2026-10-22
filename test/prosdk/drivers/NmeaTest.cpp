// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/gnss/Nmea.hpp"

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace prosdk;

namespace {

// Example sentences of the u-blox M10 SPG 5.10 Interface Description UBX-21035062 R03,
// sections 2.7.5 (GGA) and 2.5.3 (RMC); their printed checksums are correct.
constexpr std::string_view kSpecGga =
    "$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B\r\n";
constexpr std::string_view kSpecRmc =
    "$GNRMC,014230.00,A,4722.80340,N,00831.68218,E,0.000,,120477,,,A,V*14\r\n";

std::string_view BodyOf(std::string_view sentence) {
  return sentence.substr(1, sentence.find('*') - 1);
}

// Pushes every byte; returns the bodies of the sentences the reader accepted.
std::vector<std::string> PushAll(nmea::Reader& reader, std::string_view bytes) {
  std::vector<std::string> out;
  for (char c : bytes) {
    if (reader.Push(static_cast<uint8_t>(c))) {
      out.emplace_back(reader.Body());
    }
  }
  return out;
}

}  // namespace

TEST_CASE("NMEA checksum matches the u-blox examples", "[gnss][nmea]") {
  CHECK(nmea::Checksum(BodyOf(kSpecGga)) == 0x5B);
  CHECK(nmea::Checksum(BodyOf(kSpecRmc)) == 0x14);
  CHECK(nmea::Encode(BodyOf(kSpecGga)) == kSpecGga);
  CHECK(nmea::Encode("GPTXT,01") == "$GPTXT,01*62\r\n");
  CHECK(nmea::SentenceType("GNGGA,1,2") == "GGA");
  CHECK(nmea::SentenceType("BDGSA,A,3") == "GSA");
  CHECK(nmea::SentenceType("PUBX,00,1") == "");
  CHECK(nmea::SentenceType("GGA,1") == "");
}

TEST_CASE("NMEA GGA decoding", "[gnss][nmea]") {
  auto g = nmea::ParseGga(BodyOf(kSpecGga));
  REQUIRE(g.has_value());
  CHECK(g->hasTime);
  CHECK(g->hour == 9);
  CHECK(g->minute == 27);
  CHECK(g->second == Catch::Approx(25.0));
  CHECK(g->latitude == Catch::Approx(47.0 + 17.11399 / 60.0));
  CHECK(g->longitude == Catch::Approx(8.0 + 33.91590 / 60.0));
  CHECK(g->quality == 1);
  CHECK(g->numSatellites == 8);
  CHECK(g->hdop == Catch::Approx(1.01));
  CHECK(g->altitudeMsl == Catch::Approx(499.6));
  CHECK(g->geoidSeparation == Catch::Approx(48.0));

  // no fix: empty fields give NaN
  auto empty = nmea::ParseGga("GNGGA,,,,,,0,00,99.99,,,,,,");
  REQUIRE(empty.has_value());
  CHECK_FALSE(empty->hasTime);
  CHECK(empty->quality == 0);
  CHECK(empty->numSatellites == 0);
  CHECK(std::isnan(empty->latitude));
  CHECK(std::isnan(empty->longitude));
  CHECK(std::isnan(empty->altitudeMsl));

  // an out-of-range number is treated as missing
  auto garbled = nmea::ParseGga("GNGGA,,,,,,1,99999999999,1.0,,,,,,");
  REQUIRE(garbled.has_value());
  CHECK(garbled->numSatellites == -1);
  CHECK(garbled->quality == 1);

  CHECK_FALSE(nmea::ParseGga("GPRMC,1,A").has_value());  // another type
  CHECK_FALSE(nmea::ParseGga("GPGGA,1,2,3").has_value());  // too few fields
}

TEST_CASE("NMEA RMC decoding", "[gnss][nmea]") {
  auto r = nmea::ParseRmc(BodyOf(kSpecRmc));
  REQUIRE(r.has_value());
  CHECK(r->valid);
  CHECK(r->hour == 1);
  CHECK(r->minute == 42);
  CHECK(r->second == Catch::Approx(30.0));
  // the interface description's converted values: 47.38005667 N, 8.52803633 E
  CHECK(r->latitude == Catch::Approx(47.38005667).margin(1e-8));
  CHECK(r->longitude == Catch::Approx(8.52803633).margin(1e-8));
  CHECK(r->speedKnots == Catch::Approx(0.0));
  CHECK(std::isnan(r->courseDegrees));
  CHECK(r->hasDate);
  CHECK(r->day == 12);
  CHECK(r->month == 4);
  CHECK(r->year == 2077);  // two-digit years are read as 20yy
  CHECK(r->posMode == 'A');

  // southern and western hemispheres, invalid status
  auto sw = nmea::ParseRmc("GPRMC,235959.50,V,3352.12800,S,15112.55800,W,1.5,270.0,010126,,,N,V");
  REQUIRE(sw.has_value());
  CHECK_FALSE(sw->valid);
  CHECK(sw->latitude == Catch::Approx(-(33.0 + 52.128 / 60.0)));
  CHECK(sw->longitude == Catch::Approx(-(151.0 + 12.558 / 60.0)));
  CHECK(sw->second == Catch::Approx(59.5));
  CHECK(sw->courseDegrees == Catch::Approx(270.0));
  CHECK(sw->year == 2026);
  CHECK(sw->posMode == 'N');

  // NMEA 2.1 sentence: no mode indicator
  auto old = nmea::ParseRmc("GPRMC,120000,A,0100.000,N,00200.000,E,0,0,010100,,");
  REQUIRE(old.has_value());
  CHECK(old->posMode == '\0');
  CHECK(old->latitude == Catch::Approx(1.0));
  CHECK(old->longitude == Catch::Approx(2.0));
}

TEST_CASE("NMEA GSA decoding", "[gnss][nmea]") {
  auto g = nmea::ParseGsa("GNGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54,1");
  REQUIRE(g.has_value());
  CHECK(g->navMode == 3);
  CHECK(g->pdop == Catch::Approx(1.94));
  CHECK(g->hdop == Catch::Approx(1.18));
  CHECK(g->vdop == Catch::Approx(1.54));
  auto none = nmea::ParseGsa("GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99");  // NMEA 2.3: no systemId
  REQUIRE(none.has_value());
  CHECK(none->navMode == 1);
}

TEST_CASE("NMEA reader handles split input, garbage and bad checksums", "[gnss][nmea]") {
  nmea::Reader reader;
  std::string stream = "\x01\xFFgarbage";
  stream += kSpecGga;
  std::string bad{kSpecRmc};
  bad[10] = '9';  // corrupt one character: checksum mismatch
  stream += bad;
  stream += "$GPTXT,no checksum\r\n";
  stream += kSpecRmc;

  // split at every possible point into two pushes
  for (size_t split = 0; split <= stream.size(); ++split) {
    nmea::Reader r;
    auto first = PushAll(r, std::string_view{stream}.substr(0, split));
    auto second = PushAll(r, std::string_view{stream}.substr(split));
    first.insert(first.end(), second.begin(), second.end());
    REQUIRE(first.size() == 2);
    CHECK(first[0] == BodyOf(kSpecGga));
    CHECK(first[1] == BodyOf(kSpecRmc));
    CHECK(r.GetSentenceCount() == 2);
    CHECK(r.GetErrorCount() == 2);
  }

  // lower-case checksum digits are accepted
  CHECK(PushAll(reader, "$GPXXX,1,2*4c\r\n").size() == 1);

  // an overlong line is dropped, the next sentence is read
  std::string longLine = "$GP" + std::string(200, 'A') + "\r\n";
  CHECK(PushAll(reader, longLine).empty());
  CHECK(reader.GetErrorCount() == 1);
  CHECK(PushAll(reader, kSpecGga).size() == 1);

  // a '$' inside a sentence restarts it
  CHECK(PushAll(reader, "$GPGGA,12$").empty());
  CHECK(reader.GetErrorCount() == 2);
  CHECK(PushAll(reader, std::string{kSpecRmc}.substr(1)).size() == 1);
}
