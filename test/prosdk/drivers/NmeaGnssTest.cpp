// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/gnss/NmeaGnss.hpp"

#include <cmath>
#include <span>
#include <string>
#include <string_view>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/simulation/SimHooks.hpp"

using namespace prosdk;

namespace {

void Feed(NmeaGnss& gnss, std::string_view text) {
  gnss.Feed({reinterpret_cast<const uint8_t*>(text.data()), text.size()});
}

std::string Sentence(std::string_view body) {
  return nmea::Encode(body);
}

}  // namespace

TEST_CASE("NmeaGnss builds a fix from RMC, GGA and GSA", "[gnss][nmea]") {
  sim::PauseTiming();
  NmeaGnss gnss{SerialPort::Port::ONBOARD, NmeaGnss::kDefaultBaudRate, 0_ms};
  CHECK_FALSE(gnss.IsConnected());
  CHECK_FALSE(gnss.GetFix().valid);
  CHECK(gnss.GetFix().fixType == GnssFixType::kNoFix);
  CHECK(gnss.GetTelemetryType() == "GNSS");

  // receiver searching: no position yet
  Feed(gnss, Sentence("GNRMC,,V,,,,,,,,,,N,V"));
  Feed(gnss, Sentence("GNGGA,,,,,,0,00,99.99,,,,,,"));
  CHECK(gnss.IsConnected());
  CHECK_FALSE(gnss.GetFix().valid);
  CHECK(gnss.GetFix().fixType == GnssFixType::kNoFix);
  CHECK(gnss.GetFix().numSatellites == 0);

  // fix: an epoch as the GP-02 sends it, split over several reads at odd places
  std::string epoch =
      Sentence("GNGGA,104512.000,4100.49536,N,02858.70526,E,1,09,0.92,37.5,M,36.8,M,,") +
      Sentence("GNGSA,A,3,05,13,15,18,20,23,24,,,,,,1.53,0.92,1.22,1") +
      Sentence("GPGSV,3,1,10,05,57,090,32,13,45,288,30,15,37,215,28,18,62,029,35,1") +
      Sentence("GNRMC,104512.000,A,4100.49536,N,02858.70526,E,12.50,271.30,221026,,,A,V") +
      Sentence("GNVTG,271.30,T,,M,12.50,N,23.15,K,A");
  for (size_t at = 0; at < epoch.size(); at += 17) {
    Feed(gnss, std::string_view{epoch}.substr(at, 17));
  }
  CHECK(gnss.GetChecksumErrors() == 0);
  CHECK(gnss.GetSentenceCount() == 7);

  GnssFix fix = gnss.GetFix();
  CHECK(fix.valid);
  CHECK(fix.fixType == GnssFixType::k3D);
  CHECK(fix.latitude.value() == Catch::Approx(41.0 + 0.49536 / 60.0));
  CHECK(fix.longitude.value() == Catch::Approx(28.0 + 58.70526 / 60.0));
  CHECK(fix.heightMsl.value() == Catch::Approx(37.5));
  CHECK(fix.groundSpeed.value() == Catch::Approx(12.5 * 1852.0 / 3600.0));
  CHECK(fix.heading.value() == Catch::Approx(271.3));
  CHECK(fix.numSatellites == 9);
  CHECK(fix.hdop == Catch::Approx(0.92));
  CHECK(std::isnan(fix.hAcc.value()));
  CHECK(fix.utc.timeValid);
  CHECK(fix.utc.dateValid);
  CHECK(fix.utc.year == 2026);
  CHECK(fix.utc.month == 10);
  CHECK(fix.utc.day == 22);
  CHECK(fix.utc.hour == 10);
  CHECK(fix.utc.minute == 45);
  CHECK(fix.utc.second == Catch::Approx(12.0));

  // 2D fix; dead reckoning (GGA quality 6)
  Feed(gnss, Sentence("GNGSA,A,2,05,13,15,,,,,,,,,,2.10,1.40,1.56,1"));
  Feed(gnss, Sentence("GNGGA,104513.000,4100.49536,N,02858.70526,E,1,03,1.40,37.5,M,36.8,M,,"));
  CHECK(gnss.GetFix().fixType == GnssFixType::k2D);
  Feed(gnss, Sentence("GNGGA,104514.000,4100.50000,N,02858.70526,E,6,00,99.99,37.5,M,36.8,M,,"));
  CHECK(gnss.GetFix().valid);
  CHECK(gnss.GetFix().fixType == GnssFixType::kDeadReckoning);
  CHECK(gnss.GetFix().latitude.value() == Catch::Approx(41.0 + 0.5 / 60.0));
  Feed(gnss, Sentence("GNRMC,104514.000,A,4100.50000,N,02858.70526,E,0.00,,221026,,,E,V"));
  CHECK(gnss.GetFix().fixType == GnssFixType::kDeadReckoning);  // until the next GGA

  // fix lost: the last position is kept, valid drops
  Feed(gnss, Sentence("GNRMC,104515.000,V,,,,,,,221026,,,N,V"));
  fix = gnss.GetFix();
  CHECK_FALSE(fix.valid);
  CHECK(fix.fixType == GnssFixType::kNoFix);
  CHECK(fix.latitude.value() == Catch::Approx(41.0 + 0.5 / 60.0));
  CHECK(fix.utc.second == Catch::Approx(15.0));

  // a sentence with a bad checksum is dropped
  std::string bad = Sentence("GNRMC,104516.000,A,0100.00000,N,00200.00000,E,0.00,,221026,,,A,V");
  bad[bad.find('*') + 1] = bad[bad.find('*') + 1] == '0' ? '1' : '0';
  Feed(gnss, bad);
  CHECK(gnss.GetChecksumErrors() == 1);
  CHECK_FALSE(gnss.GetFix().valid);

  // silent receiver
  sim::StepTiming(1900_ms);
  CHECK(gnss.IsConnected());
  sim::StepTiming(200_ms);
  CHECK_FALSE(gnss.IsConnected());
  sim::ResumeTiming();
}

TEST_CASE("NmeaGnss without GSA reports a valid fix as 2D", "[gnss][nmea]") {
  NmeaGnss gnss{SerialPort::Port::ONBOARD, NmeaGnss::kDefaultBaudRate, 0_ms};
  Feed(gnss, Sentence("GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,"));
  CHECK(gnss.GetFix().valid);
  CHECK(gnss.GetFix().fixType == GnssFixType::k2D);
  CHECK(gnss.GetFix().heightMsl.value() == Catch::Approx(499.6));
  CHECK_FALSE(gnss.GetFix().utc.dateValid);
}
