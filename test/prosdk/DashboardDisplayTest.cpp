// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DriverStationDisplay on the dashboard (prosdk/platform/dashboard/DashboardDisplay.hpp): the
// screen as plain text in the telemetry table DriverStationDisplay.

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/dashboard/DashboardDisplay.hpp"
#include "prosdk/telemetry/MockTelemetryBackend.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"

namespace {

using Mock = prosdk::telemetry::MockTelemetryBackend;
namespace dash = prosdk::platform::dashboard;

constexpr std::string_view kClear = "\033[0m\033[2J\033[H";

std::optional<std::string> Text(const Mock& mock) {
  auto value = mock.GetLastValue<Mock::LogStringValue>("/DriverStationDisplay/text");
  if (!value) {
    return std::nullopt;
  }
  return value->value;
}

size_t TextLogs(const Mock& mock) {
  size_t n = 0;
  for (const auto& action : mock.GetActions()) {
    if (action.path == "/DriverStationDisplay/text") {
      ++n;
    }
  }
  return n;
}

}  // namespace

TEST_CASE("DriverStationDisplay frames become one telemetry text", "[prosdk][dashboard]") {
  prosdk::telemetry::TelemetryRegistry::Reset();
  auto mock = std::make_shared<Mock>();
  prosdk::telemetry::TelemetryRegistry::RegisterBackend("", mock);
  dash::detail::ResetDisplayForTest();

  // a line-mode frame as DriverStationDisplay::UpdateLines() builds it
  dash::WriteDisplay(std::string{kClear} + "Runtime : 1.0\033[0m\n\033[31mLoop : 5\033[0m\n");
  CHECK(Text(*mock) == "Runtime : 1.0\nLoop : 5");
  auto type = mock->GetLastValue<Mock::LogStringValue>("/DriverStationDisplay/.type");
  REQUIRE(type);
  CHECK(type->value == "DriverStationDisplay");

  // the next frame replaces it; an unchanged frame is not logged again
  dash::WriteDisplay(std::string{kClear} + "Runtime : 1.2\033[0m\n");
  CHECK(Text(*mock) == "Runtime : 1.2");
  size_t logs = TextLogs(*mock);
  dash::WriteDisplay(std::string{kClear} + "Runtime : 1.2\033[0m\n");
  CHECK(TextLogs(*mock) == logs);

  // raw mode: a clear, then writes that append
  dash::WriteDisplay(kClear);
  CHECK(Text(*mock) == "");
  dash::WriteDisplay("\033[1;1Hhello ");
  dash::WriteDisplay("world\r\nnext");
  CHECK(Text(*mock) == "hello world\nnext");

  // upstream's DriverStationDisplayAnsi: a screen, then one field rewritten in place
  dash::WriteDisplay(
      "\033[2J\033[H"
      "DriverStationDisplay ANSI mode\n"
      "This header and footer stay on screen.\n"
      "Seconds elapsed:      \n"
      "Only the number above is rewritten.");
  dash::WriteDisplay("\033[3;18H\033[38;5;196m    1\033[0m");
  dash::WriteDisplay("\033[3;18H\033[38;5;202m    2\033[0m");
  CHECK(Text(*mock) ==
        "DriverStationDisplay ANSI mode\n"
        "This header and footer stay on screen.\n"
        "Seconds elapsed:     2\n"
        "Only the number above is rewritten.");

  // erase to the end of the line and of the screen
  dash::WriteDisplay("\033[1;13H\033[K\033[2;1H\033[J");
  CHECK(Text(*mock) == "DriverStatio");

  // a long screen scrolls: the last rows stay
  dash::WriteDisplay(kClear);
  for (int i = 0; i < 200; ++i) {
    dash::WriteDisplay("line " + std::to_string(i) + "\n");
  }
  auto text = Text(*mock);
  REQUIRE(text);
  CHECK(text->size() <= dash::kDisplayMaxBytes);
  CHECK(text->starts_with("line 153\n"));
  CHECK(text->ends_with("line 199"));

  // a row longer than the terminal is cut
  dash::WriteDisplay(kClear);
  dash::WriteDisplay(std::string(500, 'x'));
  CHECK(Text(*mock) == std::string(dash::kDisplayMaxColumns, 'x'));

  dash::detail::ResetDisplayForTest();
  prosdk::telemetry::TelemetryRegistry::Reset();
}
