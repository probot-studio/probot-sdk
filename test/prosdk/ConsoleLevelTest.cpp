// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Console levels (prosdk/platform/Console.hpp, core request R9): ReportError and HAL errors are
// errors; ReportWarning, MathShared warnings and the telemetry/tunables warning reporters are
// warnings; everything else is info. A text-only sink still gets every write.

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/HAL.h"
#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"

namespace pf = prosdk::platform;

namespace {

std::vector<std::pair<pf::ConsoleLevel, std::string>> gLines;
std::string gText;

void Leveled(pf::ConsoleLevel level, std::string_view text) {
  gLines.emplace_back(level, std::string{text});
}
void TextOnly(std::string_view text) { gText += text; }

// The level of the one captured line that contains `part`.
pf::ConsoleLevel LevelOf(std::string_view part) {
  const std::pair<pf::ConsoleLevel, std::string>* found = nullptr;
  for (const auto& line : gLines) {
    if (line.second.find(part) != std::string::npos) {
      REQUIRE(found == nullptr);
      found = &line;
    }
  }
  REQUIRE(found != nullptr);
  return found->first;
}

}  // namespace

TEST_CASE("Console writes carry a level", "[prosdk][console]") {
  REQUIRE(HAL_Initialize());
  gLines.clear();
  pf::SetLeveledConsoleSink(Leveled);

  pf::ConsoleWrite("plain line\n");
  pf::ConsoleWrite("explicit warning\n", pf::ConsoleLevel::kWarning);
  pf::ConsolePrint("printed {}\n", 1);
  pf::ConsolePrint(pf::ConsoleLevel::kError, "printed error {}\n", 2);
  CHECK(LevelOf("plain line") == pf::ConsoleLevel::kInfo);
  CHECK(LevelOf("explicit warning") == pf::ConsoleLevel::kWarning);
  CHECK(LevelOf("printed 1") == pf::ConsoleLevel::kInfo);
  CHECK(LevelOf("printed error 2") == pf::ConsoleLevel::kError);

  // ReportError / ReportWarning through the HAL (HAL_SendError, hal/0002)
  WPILIB_ReportError(prosdk::err::Error, "console level test error {}", 7);
  WPILIB_ReportError(prosdk::warn::Warning, "console level test warning {}", 8);
  CHECK(LevelOf("console level test error 7") == pf::ConsoleLevel::kError);
  CHECK(LevelOf("console level test warning 8") == pf::ConsoleLevel::kWarning);

  // MathShared default (wpimath/0002)
  prosdk::math::MathSharedStore::ReportError("math level error {}", 1);
  prosdk::math::MathSharedStore::ReportWarning("math level warning {}", 2);
  CHECK(LevelOf("math level error 1") == pf::ConsoleLevel::kError);
  CHECK(LevelOf("math level warning 2") == pf::ConsoleLevel::kWarning);

  // telemetry and tunables warning reporters without a robot (telemetry/0001, tunables/0001;
  // on the robot RobotBase turns them into alerts)
  prosdk::telemetry::TelemetryRegistry::ReportWarning("/level/test", "telemetry level warning");
  prosdk::tunables::TunableRegistry::ReportWarning("tunable level warning");
  CHECK(LevelOf("telemetry level warning") == pf::ConsoleLevel::kWarning);
  CHECK(LevelOf("tunable level warning") == pf::ConsoleLevel::kWarning);

  // a text-only sink replaces the leveled one and gets the text without the level
  gText.clear();
  pf::SetConsoleSink(TextOnly);
  size_t before = gLines.size();
  pf::ConsoleWrite("text sink line\n", pf::ConsoleLevel::kError);
  CHECK(gText == "text sink line\n");
  CHECK(gLines.size() == before);

  pf::SetConsoleSink(nullptr);
}
