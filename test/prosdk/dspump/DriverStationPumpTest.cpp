// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// The ESP32 Driver Station backend (src/prosdk/platform/hal/esp32/DriverStation.cpp) on the host:
// its pump (Poll, run by the platform's pre-cycle callbacks) against a fake core table, through
// the HAL's own Driver Station functions (impl/hal FIRSTDriverStation.cpp, pointed at the backend
// with ForceDsInstance). The backend's GetDefaultDriverStationImpl is renamed for this executable
// (the host's sim backend defines the real one), and esp32::Log is a stub below. Own executable:
// it installs the fake core table (CoreOps is installed once per process).
//
// Checked: a match info change (CoreOps ABI 9 counter) makes a new snapshot without a packet,
// control-word or joystick change; the HAL getters see it only after HAL_RefreshDSData; the
// values are read once per change; a gamepad Home button (R20 mapping) reaches GUIDE.

#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/DriverStation.h"
#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/hal/cpp/MrcLibDs.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/hal/CoreJoystick.hpp"

namespace pf = prosdk::platform;

// the backend under test (renamed GetDefaultDriverStationImpl, see the CMake target)
namespace prosdk::hal {
MrcLibDs* GetEsp32DriverStationForTest();
namespace init {
void InitializeDriverStationPump();
}  // namespace init
namespace esp32 {
void Log(std::string_view text, platform::ConsoleLevel level) {
  platform::ConsoleWrite(std::string{text} + "\n", level);
}
}  // namespace esp32
}  // namespace prosdk::hal

namespace {

// fake core: attached and stopped; packets, match info and gamepad 0 set by the tests
std::atomic<uint32_t> gPacketSeq{1};
std::atomic<uint32_t> gMatchSeq{0};
std::atomic<int> gMatchReads{0};
pf::CoreMatchInfo gMatch{};
bool gHasPad = false;
pf::CoreGamepad gPad{};

uint8_t Phase() { return pf::kCorePhaseStopped; }
uint8_t Mode() { return pf::kCoreModeTeleop; }
bool True() { return true; }
bool False() { return false; }
float MatchTime() { return -1.0f; }
uint32_t PacketSeq() { return gPacketSeq.load(); }
uint32_t MatchSeq() { return gMatchSeq.load(); }
void MatchInfo(pf::CoreMatchInfo* out) {
  gMatchReads.fetch_add(1);
  *out = gMatch;
}
bool RawGamepad(int port, pf::CoreGamepad* out) {
  if (port != 0 || !gHasPad) {
    return false;
  }
  *out = gPad;
  return true;
}

bool Install() {
  pf::CoreOps ops{};
  ops.abiVersion = pf::kCoreOpsAbiVersion;
  ops.size = sizeof(pf::CoreOps);
  ops.phase = Phase;
  ops.selectedMode = Mode;
  ops.outputsEnabled = False;
  ops.isEStopped = False;
  ops.isAttached = True;
  ops.matchTimeSeconds = MatchTime;
  ops.packetSeq = PacketSeq;
  ops.rawGamepad = RawGamepad;
  ops.matchInfoSeq = MatchSeq;
  ops.matchInfo = MatchInfo;
  return pf::InstallCoreOps(&ops);
}

// one core pass: the pre-cycle callbacks (the pump among them)
void Pass() {
  pf::detail::PreCycle();
}

HAL_AllianceStationID Station() {
  int32_t status = 0;
  HAL_AllianceStationID station = HAL_GetAllianceStation(&status);
  CHECK(status == 0);
  return station;
}

std::string GameData() {
  HAL_GameData data;
  std::memset(&data, 0x5A, sizeof(data));
  CHECK(HAL_GetGameData(&data) == 0);
  return std::string{reinterpret_cast<const char*>(data.gameData)};
}

HAL_MatchInfo Info() {
  HAL_MatchInfo info;
  std::memset(&info, 0x5A, sizeof(info));
  CHECK(HAL_GetMatchInfo(&info) == 0);
  return info;
}

}  // namespace

TEST_CASE("Driver Station pump: match info reaches the HAL getters", "[prosdk][hal][ds]") {
  REQUIRE(Install());
  prosdk::hal::ForceDsInstance(prosdk::hal::GetEsp32DriverStationForTest());
  prosdk::hal::init::InitializeDriverStationPump();

  // first pass: a snapshot (attached, stopped), no match info yet (counter 0, nothing read)
  Pass();
  CHECK(HAL_RefreshDSData());
  CHECK(Station() == HAL_ALLIANCE_STATION_UNKNOWN);
  CHECK(GameData().empty());
  CHECK(Info().matchType == HAL_MATCH_TYPE_NONE);
  CHECK(gMatchReads == 0);
  // nothing changed: no new snapshot
  Pass();
  CHECK_FALSE(HAL_RefreshDSData());

  // the Driver Station sets the match info while the robot is stopped: same packet, same
  // control word, no gamepad; the next pass alone makes a new snapshot
  gMatch.station = 5;  // Blue 2
  gMatch.type = 2;     // qualification
  gMatch.matchNumber = 12;
  gMatch.replayNumber = 1;
  std::strcpy(gMatch.eventName, "Probot \xC3\x87" "ukurova");  // UTF-8 kept
  std::strcpy(gMatch.gameData, "LRL");
  gMatchSeq = 1;
  CHECK(Station() == HAL_ALLIANCE_STATION_UNKNOWN);  // not before the pass
  Pass();
  CHECK(gMatchReads == 1);
  CHECK(Station() == HAL_ALLIANCE_STATION_UNKNOWN);  // not before HAL_RefreshDSData
  CHECK(HAL_RefreshDSData());
  CHECK(Station() == HAL_ALLIANCE_STATION_BLUE_2);
  CHECK(GameData() == "LRL");
  HAL_MatchInfo info = Info();
  CHECK(std::string{reinterpret_cast<const char*>(info.eventName)} ==
        "Probot \xC3\x87" "ukurova");
  CHECK(info.matchType == HAL_MATCH_TYPE_QUALIFICATION);
  CHECK(info.matchNumber == 12);
  CHECK(info.replayNumber == 1);

  // unchanged counter: never read again, no new snapshot
  for (int i = 0; i < 20; ++i) {
    Pass();
  }
  CHECK(gMatchReads == 1);
  CHECK_FALSE(HAL_RefreshDSData());
  CHECK(Station() == HAL_ALLIANCE_STATION_BLUE_2);

  // a clear (TRANSITION or STOPPED): read once more, the getters follow after the refresh
  gMatch = pf::CoreMatchInfo{};
  gMatchSeq = 2;
  Pass();
  CHECK(gMatchReads == 2);
  CHECK(HAL_RefreshDSData());
  CHECK(Station() == HAL_ALLIANCE_STATION_UNKNOWN);
  CHECK(GameData().empty());
  CHECK(Info().eventName[0] == '\0');

  // gamepad 0 with core's R20 mapping: the Home button (16) is GUIDE through the same pump
  std::memset(&gPad, 0, sizeof(gPad));
  gPad.seq = 1;
  gPad.axisCount = 4;
  gPad.buttonCount = 17;
  gPad.buttons = 1u << 16;
  gPad.pov = -1;
  gPad.standardMapping = 1;
  gPad.mapping = pf::CoreGamepadMapping{"standard", 0, 1, 2, 3, 1, 1, 0, 1, 2, 3, 4, 5, 6, 7,
                                        8, 9, 16, 10, 11, 0, -1, 12, 13, 14, 15};
  gHasPad = true;
  gPacketSeq = 2;  // the packet that carried it
  Pass();
  CHECK(gMatchReads == 2);
  CHECK(HAL_RefreshDSData());
  HAL_JoystickButtons buttons;
  std::memset(&buttons, 0, sizeof(buttons));
  CHECK(HAL_GetJoystickButtons(0, &buttons) == 0);
  CHECK((buttons.available >> pf::kCoreJoystickGuide & 1u) == 1u);
  CHECK((buttons.buttons >> pf::kCoreJoystickGuide & 1u) == 1u);
  CHECK((buttons.buttons & ~(uint64_t{1} << pf::kCoreJoystickGuide)) == 0);
}
