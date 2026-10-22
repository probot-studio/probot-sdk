// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// prosdk::platform R6/R7/R9/R10 accessors (CoreOps.hpp): the no-core defaults, then the opmode
// list sanitizing, the leveled console and the wall-clock flag against a fake probot-core table. One TEST_CASE because InstallCoreOps is once per
// process (no other host test installs a table); Catch2 re-runs it per SECTION, so the no-core
// part runs only on the first pass.

#include <cstdint>
#include <ctime>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"

namespace pf = prosdk::platform;

namespace {

struct Captured {
  uint64_t id;
  uint8_t mode;
  std::string name, group, description;
};

std::vector<Captured> gList;
std::string gConsole;
uint32_t gPeriod = 20;

bool FakeSetOptions(const pf::CoreOpModeOption* options, size_t count) {
  gList.clear();
  for (size_t i = 0; i < count; ++i) {
    gList.push_back({options[i].id, options[i].mode, options[i].name, options[i].group,
                     options[i].description});
  }
  return true;
}
bool FakeSetPeriod(uint32_t ms) {
  if (ms > pf::kCoreLoopPeriodMaxMs) {
    return false;
  }
  gPeriod = ms == 0 ? 20 : ms;
  return true;
}
uint32_t FakePeriod() { return gPeriod; }
uint32_t FakeOverruns() { return 3; }
uint64_t FakeSelected() { return 0x0200000000000042ull; }
uint64_t FakeActive() { return 0x0100000000000007ull; }
bool FakeUtility() { return true; }
void FakeConsole(const char*, size_t) {}
void CaptureConsole(std::string_view text) { gConsole.append(text); }
std::vector<std::pair<uint8_t, std::string>> gCoreConsole;
void FakeConsoleLevel(uint8_t level, const char* text, size_t len) {
  gCoreConsole.emplace_back(level, std::string{text, len});
}
bool gTimeValid = false;
bool FakeSystemTimeValid() { return gTimeValid; }

pf::CoreOpModeOption Opt(uint64_t id, uint8_t mode, const char* name, const char* group = "",
                         const char* description = "") {
  return {id, mode, name, group, description};
}

}  // namespace

TEST_CASE("CoreOps R6/R7 accessors: defaults, then a fake core", "[prosdk][platform]") {
  if (!pf::HasCoreOps()) {
    // ---- no core installed (first pass) ----
    CHECK_FALSE(pf::FrameworkInstalled());
    CHECK(pf::LoopPeriodMs() == pf::kCoreLoopPeriodDefaultMs);
    CHECK(pf::SetLoopPeriodMs(5));
    CHECK(pf::LoopPeriodMs() == 5);
    CHECK_FALSE(pf::SetLoopPeriodMs(1001));
    CHECK(pf::LoopPeriodMs() == 5);
    CHECK(pf::SetLoopPeriodMs(0));
    CHECK(pf::LoopPeriodMs() == pf::kCoreLoopPeriodDefaultMs);
    CHECK(pf::LoopOverruns() == 0);
    auto one = Opt(1, pf::kCoreModeTeleop, "Drive");
    CHECK(pf::SetOpModeOptions(&one, 1));
    CHECK(pf::SetOpModeOptions(nullptr, 0));
    CHECK(pf::SelectedOpModeId() == 0);
    CHECK(pf::ActiveOpModeId() == 0);
    CHECK_FALSE(pf::UtilityAvailable());
    // no core: the clock counts as set once it is past 2024-01-01 (true on a host)
    CHECK(pf::SystemTimeValid() == (std::time(nullptr) >= 1704067200));

    // ---- fake core (unset entries fall back to the defaults) ----
    pf::CoreOps ops{};
    ops.abiVersion = pf::kCoreOpsAbiVersion;
    ops.size = sizeof(pf::CoreOps);
    ops.consoleWrite = FakeConsole;
    ops.setLoopPeriodMs = FakeSetPeriod;
    ops.loopPeriodMs = FakePeriod;
    ops.loopOverruns = FakeOverruns;
    ops.setOpModeOptions = FakeSetOptions;
    ops.selectedOpModeId = FakeSelected;
    ops.activeOpModeId = FakeActive;
    ops.utilityAvailable = FakeUtility;
    ops.consoleWriteLevel = FakeConsoleLevel;
    ops.systemTimeValid = FakeSystemTimeValid;
    REQUIRE(pf::InstallCoreOps(&ops));

    // R9: the console reaches core's leveled console with its level (one call per write)
    pf::ConsoleWrite("core info\n");
    pf::ConsoleWrite("core warning\n", pf::ConsoleLevel::kWarning);
    pf::ConsoleWrite("core error\n", pf::ConsoleLevel::kError);
    pf::ConsoleWrite("");  // nothing to send
    REQUIRE(gCoreConsole.size() == 3);
    CHECK(gCoreConsole[0] == std::pair<uint8_t, std::string>{0, "core info\n"});
    CHECK(gCoreConsole[1] == std::pair<uint8_t, std::string>{1, "core warning\n"});
    CHECK(gCoreConsole[2] == std::pair<uint8_t, std::string>{2, "core error\n"});
  }
  REQUIRE(pf::HasCoreOps());
  pf::SetConsoleSink(CaptureConsole);

  CHECK(pf::SetLoopPeriodMs(10));
  CHECK(pf::LoopPeriodMs() == 10);
  CHECK(pf::LoopOverruns() == 3);
  CHECK(pf::SelectedOpModeId() == 0x0200000000000042ull);
  CHECK(pf::ActiveOpModeId() == 0x0100000000000007ull);
  CHECK(pf::UtilityAvailable());

  // R10: the wall-clock flag is core's
  gTimeValid = false;
  CHECK_FALSE(pf::SystemTimeValid());
  gTimeValid = true;
  CHECK(pf::SystemTimeValid());

  SECTION("invalid options are dropped, long strings truncated at a UTF-8 boundary") {
    std::string longName(70, 'a');
    longName[62] = '\xc3';  // two-byte character straddling the 63-byte limit
    longName[63] = '\xa7';
    std::string longDesc(300, 'd');
    std::vector<pf::CoreOpModeOption> in{
        Opt(0, pf::kCoreModeTeleop, "zero id"),
        Opt(5, 9, "bad mode"),
        Opt(6, pf::kCoreModeAuto, ""),
        Opt(7, pf::kCoreModeAuto, longName.c_str(), "Auto", longDesc.c_str()),
        Opt(7, pf::kCoreModeTeleop, "duplicate"),
        Opt(8, pf::kCoreModeUtility, "Calibrate"),
    };
    gConsole.clear();
    REQUIRE(pf::SetOpModeOptions(in.data(), in.size()));
    REQUIRE(gList.size() == 2);
    CHECK(gList[0].id == 7);
    CHECK(gList[0].mode == pf::kCoreModeAuto);
    CHECK(gList[0].name == std::string(62, 'a'));
    CHECK(gList[0].group == "Auto");
    CHECK(gList[0].description.size() == pf::kCoreOpModeMaxDescription);
    CHECK(gList[1].id == 8);
    CHECK(gList[1].mode == pf::kCoreModeUtility);
    CHECK(gConsole.find("id 0") != std::string::npos);
    CHECK(gConsole.find("duplicate") != std::string::npos);
  }

  SECTION("more than 64 options are capped with a warning") {
    std::vector<std::string> names;
    for (int i = 0; i < 70; ++i) {
      names.push_back("Op" + std::to_string(i));
    }
    std::vector<pf::CoreOpModeOption> in;
    for (int i = 0; i < 70; ++i) {
      in.push_back(Opt(100 + i, pf::kCoreModeTeleop, names[i].c_str()));
    }
    gConsole.clear();
    REQUIRE(pf::SetOpModeOptions(in.data(), in.size()));
    CHECK(gList.size() == pf::kCoreOpModeMaxOptions);
    CHECK(gList.back().id == 100 + pf::kCoreOpModeMaxOptions - 1);
    CHECK(gConsole.find("at most 64") != std::string::npos);
  }

  SECTION("nullptr clears the list") {
    gList.push_back({1, 0, "x", "", ""});
    REQUIRE(pf::SetOpModeOptions(nullptr, 5));
    CHECK(gList.empty());
  }

  pf::SetConsoleSink(nullptr);
}
