// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// probot-core R18 match information (CoreOps ABI 9 matchInfoSeq/matchInfo) -> the HAL's alliance
// station, game data and match info (prosdk/platform/hal/CoreMatchInfo.hpp), the conversion and
// change check the ESP32 Driver Station backend's pump runs every cycle. Own executable: it
// installs a core table with both fields null (CoreOps is installed once per process), the
// fallback of a core without R18. The other cases pass fake tables directly (not installed).
// When the probot-core clone has R18 at configure time (PROSDK_TEST_PROBOT_CORE_MATCH), one more
// case reads core's own probot::ds::matchInfoSeq()/matchInfo() the way CoreBridge.hpp does, with
// the values set through core's Driver Station form parser (applyMatchForm).

#include <cstdint>
#include <cstring>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/hal/CoreMatchInfo.hpp"

#if PROSDK_TEST_PROBOT_CORE_MATCH
#include <probot/ds.hpp>

// core's Arduino test stub keeps its clock here
unsigned long _test_millis_now = 0;
#endif

namespace pf = prosdk::platform;

namespace {

// a fake core: the values and the counter, and how often each entry was called
pf::CoreMatchInfo gInfo;
uint32_t gSeq = 0;
int gSeqCalls = 0;
int gInfoCalls = 0;
// a change the Driver Station makes while the values are read (after the counter)
bool gChangeWhileReading = false;

uint32_t FakeSeq() {
  ++gSeqCalls;
  return gSeq;
}
void FakeInfo(pf::CoreMatchInfo* out) {
  ++gInfoCalls;
  *out = gInfo;
  if (gChangeWhileReading) {
    gChangeWhileReading = false;
    gInfo.station = 1;
    ++gSeq;
  }
}

pf::CoreOps FakeOps(uint32_t (*seq)(), void (*info)(pf::CoreMatchInfo*)) {
  pf::CoreOps ops{};
  ops.abiVersion = pf::kCoreOpsAbiVersion;
  ops.size = sizeof(pf::CoreOps);
  ops.matchInfoSeq = seq;
  ops.matchInfo = info;
  return ops;
}

void ResetFake() {
  std::memset(&gInfo, 0, sizeof(gInfo));
  gSeq = 0;
  gSeqCalls = 0;
  gInfoCalls = 0;
  gChangeWhileReading = false;
}

pf::CoreMatchData Zeroed() {
  pf::CoreMatchData data;
  std::memset(&data, 0, sizeof(data));
  return data;
}

void CheckEmpty(const pf::CoreMatchData& data) {
  CHECK(data.station == HAL_ALLIANCE_STATION_UNKNOWN);
  CHECK(data.gameData.gameData[0] == '\0');
  CHECK(data.info.eventName[0] == '\0');
  CHECK(data.info.matchType == HAL_MATCH_TYPE_NONE);
  CHECK(data.info.matchNumber == 0);
  CHECK(data.info.replayNumber == 0);
}

}  // namespace

TEST_CASE("CoreMatchInfo: a core without R18 leaves the match data empty", "[prosdk][hal]") {
  static const bool installed = [] {
    pf::CoreOps ops = FakeOps(nullptr, nullptr);  // what CoreBridge.hpp fills without R18
    return pf::InstallCoreOps(&ops);
  }();
  REQUIRE(installed);
  const pf::CoreOps& ops = pf::GetCoreOps();
  uint32_t seq = 0;
  pf::CoreMatchData data = Zeroed();
  for (int i = 0; i < 3; ++i) {
    CHECK_FALSE(pf::RefreshCoreMatchData(ops, &seq, &data));
  }
  CHECK(seq == 0);
  CheckEmpty(data);
  // the defaults themselves convert to the same empty values
  pf::CoreMatchInfo info;
  std::memset(&info, 0x5A, sizeof(info));
  ops.matchInfo(&info);
  pf::ConvertCoreMatchInfo(info, &data);
  CheckEmpty(data);
}

TEST_CASE("CoreMatchInfo: values and numbering", "[prosdk][hal]") {
  ResetFake();
  const pf::CoreOps ops = FakeOps(FakeSeq, FakeInfo);
  struct Case {
    uint8_t station;
    HAL_AllianceStationID expected;
  };
  const Case stations[] = {
      {0, HAL_ALLIANCE_STATION_UNKNOWN}, {1, HAL_ALLIANCE_STATION_RED_1},
      {2, HAL_ALLIANCE_STATION_RED_2},   {3, HAL_ALLIANCE_STATION_RED_3},
      {4, HAL_ALLIANCE_STATION_BLUE_1},  {5, HAL_ALLIANCE_STATION_BLUE_2},
      {6, HAL_ALLIANCE_STATION_BLUE_3},  {7, HAL_ALLIANCE_STATION_UNKNOWN},
      {255, HAL_ALLIANCE_STATION_UNKNOWN},
  };
  uint32_t seq = 0;
  pf::CoreMatchData data = Zeroed();
  for (const Case& c : stations) {
    gInfo.station = c.station;
    ++gSeq;
    REQUIRE(pf::RefreshCoreMatchData(ops, &seq, &data));
    CHECK(data.station == c.expected);
  }
  const HAL_MatchType types[] = {HAL_MATCH_TYPE_NONE, HAL_MATCH_TYPE_PRACTICE,
                                 HAL_MATCH_TYPE_QUALIFICATION, HAL_MATCH_TYPE_ELIMINATION,
                                 HAL_MATCH_TYPE_NONE};
  for (uint8_t t = 0; t < 5; ++t) {
    gInfo.type = t;
    ++gSeq;
    REQUIRE(pf::RefreshCoreMatchData(ops, &seq, &data));
    CHECK(data.info.matchType == types[t]);
  }

  gInfo.station = 5;
  gInfo.type = 2;
  gInfo.matchNumber = 65535;
  gInfo.replayNumber = 255;
  std::strcpy(gInfo.eventName, "Istanbul Regional");
  std::strcpy(gInfo.gameData, "LRL");
  ++gSeq;
  REQUIRE(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(seq == gSeq);
  CHECK(data.station == HAL_ALLIANCE_STATION_BLUE_2);
  CHECK(data.info.matchType == HAL_MATCH_TYPE_QUALIFICATION);
  CHECK(data.info.matchNumber == 65535);
  CHECK(data.info.replayNumber == 255);
  CHECK(std::string{data.info.eventName} == "Istanbul Regional");
  CHECK(std::string{data.gameData.gameData} == "LRL");
}

TEST_CASE("CoreMatchInfo: values are read only when the counter changes", "[prosdk][hal]") {
  ResetFake();
  const pf::CoreOps ops = FakeOps(FakeSeq, FakeInfo);
  uint32_t seq = 0;
  pf::CoreMatchData data = Zeroed();
  // never set since boot: counter 0, nothing read
  CHECK_FALSE(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(gInfoCalls == 0);
  CheckEmpty(data);

  // set on the Driver Station (STOPPED)
  gInfo.station = 4;
  std::strcpy(gInfo.gameData, "B");
  gSeq = 1;
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(gInfoCalls == 1);
  CHECK(data.station == HAL_ALLIANCE_STATION_BLUE_1);
  CHECK(std::string{data.gameData.gameData} == "B");
  // the pump checks every cycle: one counter read each, no copy of the values
  for (int i = 0; i < 100; ++i) {
    CHECK_FALSE(pf::RefreshCoreMatchData(ops, &seq, &data));
  }
  CHECK(gInfoCalls == 1);
  CHECK(gSeqCalls == 102);

  // changed again between autonomous and TeleOp (TRANSITION): seen on the next check
  std::strcpy(gInfo.gameData, "RLR");
  gSeq = 2;
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(std::string{data.gameData.gameData} == "RLR");
  CHECK(data.station == HAL_ALLIANCE_STATION_BLUE_1);

  // a change while the values are read: the counter was read first, so the next check sees it
  gChangeWhileReading = true;
  gInfo.station = 6;
  gSeq = 3;
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(seq == 3);
  CHECK(data.station == HAL_ALLIANCE_STATION_BLUE_3);
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(seq == 4);
  CHECK(data.station == HAL_ALLIANCE_STATION_RED_1);

  // cleared on the Driver Station: the counter moves, the values are empty again
  std::memset(&gInfo, 0, sizeof(gInfo));
  gSeq = 5;
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CheckEmpty(data);
}

TEST_CASE("CoreMatchInfo: text is cut at 64 bytes on a UTF-8 boundary", "[prosdk][hal]") {
  pf::CoreMatchInfo in;
  std::memset(&in, 0, sizeof(in));
  pf::CoreMatchData data;

  // exactly 64 bytes: kept
  std::memset(in.gameData, 'g', 64);
  std::memset(in.eventName, 'e', 64);
  pf::ConvertCoreMatchInfo(in, &data);
  CHECK(std::string{data.gameData.gameData} == std::string(64, 'g'));
  CHECK(std::string{data.info.eventName} == std::string(64, 'e'));

  // 65 bytes without a NUL (a core that broke its limit): the first 64
  std::memset(in.gameData, 'g', sizeof(in.gameData));
  std::memset(in.eventName, 'e', sizeof(in.eventName));
  pf::ConvertCoreMatchInfo(in, &data);
  CHECK(std::string{data.gameData.gameData} == std::string(64, 'g'));
  CHECK(std::string{data.info.eventName} == std::string(64, 'e'));

  // a two-byte character across byte 64: dropped whole
  std::memset(in.gameData, 'g', 63);
  in.gameData[63] = '\xC3';  // U+00E7 (c with cedilla) = C3 A7
  in.gameData[64] = '\xA7';
  pf::ConvertCoreMatchInfo(in, &data);
  CHECK(std::string{data.gameData.gameData} == std::string(63, 'g'));
  // a character that ends at byte 64: kept
  std::memset(in.gameData, 'g', 62);
  in.gameData[62] = '\xC3';
  in.gameData[63] = '\xA7';
  in.gameData[64] = '\0';
  pf::ConvertCoreMatchInfo(in, &data);
  CHECK(std::string{data.gameData.gameData} == std::string(62, 'g') + "\xC3\xA7");

  // CopyCoreMatchInfo (the bridge's copy of core's MatchInfo) cuts the same way
  struct CoreLike {
    uint8_t station = 2;
    uint8_t type = 3;
    uint16_t matchNumber = 12;
    uint8_t replayNumber = 1;
    char eventName[65];
    char gameData[65];
  } core;
  std::memset(core.eventName, 'x', sizeof(core.eventName));
  std::strcpy(core.gameData, "L");
  pf::CoreMatchInfo copy;
  pf::CopyCoreMatchInfo(core, &copy);
  CHECK(copy.station == 2);
  CHECK(copy.type == 3);
  CHECK(copy.matchNumber == 12);
  CHECK(copy.replayNumber == 1);
  CHECK(std::string{copy.eventName} == std::string(64, 'x'));
  CHECK(std::string{copy.gameData} == "L");
  // a two-byte character across byte 64 of core's array: dropped whole, also through the HAL
  std::memset(core.gameData, 'g', 63);
  core.gameData[63] = '\xC3';
  core.gameData[64] = '\xA7';
  pf::CopyCoreMatchInfo(core, &copy);
  CHECK(std::string{copy.gameData} == std::string(63, 'g'));
  pf::ConvertCoreMatchInfo(copy, &data);
  CHECK(std::string{data.gameData.gameData} == std::string(63, 'g'));
  // a longer core array (a core that raised its limit): cut at 64 bytes on a boundary
  struct WiderCore {
    uint8_t station = 0;
    uint8_t type = 0;
    uint16_t matchNumber = 0;
    uint8_t replayNumber = 0;
    char eventName[80];
    char gameData[80];
  } wide;
  std::memset(wide.eventName, 0, sizeof(wide.eventName));
  std::memset(wide.gameData, 0, sizeof(wide.gameData));
  std::string event;
  for (int i = 0; i < 25; ++i) {
    event += "\xE2\x82\xAC";  // U+20AC, three bytes: 21 fit in 64 (63 bytes)
  }
  std::memcpy(wide.eventName, event.data(), event.size());
  pf::CopyCoreMatchInfo(wide, &copy);
  CHECK(std::string{copy.eventName} == event.substr(0, 63));
}

#if PROSDK_TEST_PROBOT_CORE_MATCH
namespace {
// what CoreBridge.hpp installs for a core with R18 (match_probe)
uint32_t CoreSeq() { return probot::ds::matchInfoSeq(); }
void CoreInfo(pf::CoreMatchInfo* out) { pf::CopyCoreMatchInfo(probot::ds::matchInfo(), out); }

// the Driver Station's POST /match: the form validated as a whole, then stored
bool DriverStationSets(const char* form) {
  probot::ds::MatchInfo next = probot::ds::matchInfo();
  if (probot::robot::applyMatchForm(form, next) != nullptr) {
    return false;
  }
  probot::robot::matchStore().set(next);
  return true;
}
}  // namespace

TEST_CASE("CoreMatchInfo: probot-core's own match store", "[prosdk][hal]") {
  static_assert(PROBOT_HAS_DS_MATCH_INFO == 1);
  probot::robot::matchStore().resetForTest();
  const pf::CoreOps ops = FakeOps(CoreSeq, CoreInfo);
  uint32_t seq = 0;
  pf::CoreMatchData data = Zeroed();
  // boot: never set
  CHECK(probot::ds::matchInfoSeq() == 0);
  CHECK_FALSE(pf::RefreshCoreMatchData(ops, &seq, &data));
  CheckEmpty(data);

  REQUIRE(DriverStationSets(
      "station=5&type=2&match=12&replay=3&event=Istanbul%20Regional&gd=LRL"));
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(seq == probot::ds::matchInfoSeq());
  CHECK(data.station == HAL_ALLIANCE_STATION_BLUE_2);
  CHECK(data.info.matchType == HAL_MATCH_TYPE_QUALIFICATION);
  CHECK(data.info.matchNumber == 12);
  CHECK(data.info.replayNumber == 3);
  CHECK(std::string{data.info.eventName} == "Istanbul Regional");
  CHECK(std::string{data.gameData.gameData} == "LRL");

  // the same values again: core's counter does not move, nothing is re-read
  REQUIRE(DriverStationSets("station=5&gd=LRL"));
  CHECK_FALSE(pf::RefreshCoreMatchData(ops, &seq, &data));

  // core allows a change between autonomous and TeleOp (TRANSITION, nothing pending)
  CHECK(probot::core::canChangeMatchInfo(probot::robot::Phase::TRANSITION,
                                         probot::robot::Status::STOP, false));
  CHECK_FALSE(probot::core::canChangeMatchInfo(probot::robot::Phase::TELEOP_INIT,
                                               probot::robot::Status::STOP, false));
  REQUIRE(DriverStationSets("station=1&gd=%C3%A7%C3%A7"));  // two U+00E7, red 1
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(data.station == HAL_ALLIANCE_STATION_RED_1);
  CHECK(std::string{data.gameData.gameData} == "\xC3\xA7\xC3\xA7");
  CHECK(std::string{data.info.eventName} == "Istanbul Regional");  // kept

  // 64 bytes of UTF-8 (32 two-byte characters) pass through whole; core refuses 65
  std::string form = "gd=";
  std::string expected;
  for (int i = 0; i < 32; ++i) {
    form += "%C3%A7";
    expected += "\xC3\xA7";
  }
  REQUIRE(DriverStationSets(form.c_str()));
  CHECK(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(std::string{data.gameData.gameData} == expected);
  CHECK_FALSE(DriverStationSets((form + "x").c_str()));
  CHECK_FALSE(pf::RefreshCoreMatchData(ops, &seq, &data));
  CHECK(std::string{data.gameData.gameData} == expected);
  probot::robot::matchStore().resetForTest();
}
#endif
