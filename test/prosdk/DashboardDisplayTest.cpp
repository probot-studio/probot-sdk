// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DriverStationDisplay on the dashboard (prosdk/platform/dashboard/DashboardDisplay.hpp): the
// screen as plain text in the telemetry table DriverStationDisplay. No core value limit in this
// process (CoreOps dashMaxValue unset): the text is capped at core's default 1024 bytes; the cap
// that follows core's PROBOT_DASH_MAX_VALUE is tested with the fake store
// (test/prosdk/dashboard/DashboardDisplayStoreTest.cpp).

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

size_t CountOf(std::string_view text, std::string_view part) {
  size_t n = 0;
  for (size_t at = text.find(part); at != std::string_view::npos; at = text.find(part, at + 1)) {
    ++n;
  }
  return n;
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

  // a long screen scrolls: the last 48 rows stay (the newline after the last one scrolls only
  // once more text follows)
  dash::WriteDisplay(kClear);
  for (int i = 0; i < 200; ++i) {
    dash::WriteDisplay("line " + std::to_string(i) + "\n");
  }
  auto text = Text(*mock);
  REQUIRE(text);
  CHECK(dash::DisplayMaxBytes() == dash::kDisplayDefaultMaxBytes);
  CHECK(text->size() <= dash::DisplayMaxBytes());
  CHECK(text->starts_with("line 152\n"));
  CHECK(text->ends_with("line 199"));

  // a row longer than the terminal is cut
  dash::WriteDisplay(kClear);
  dash::WriteDisplay(std::string(500, 'x'));
  CHECK(Text(*mock) == std::string(dash::kDisplayMaxColumns, 'x'));

  dash::detail::ResetDisplayForTest();
  prosdk::telemetry::TelemetryRegistry::Reset();
}

namespace {

std::string Repeat(std::string_view s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n; ++i) {
    out.append(s);
  }
  return out;
}

// well-formed UTF-8 (no sequence cut short, no stray continuation byte)
bool ValidUtf8(std::string_view s) {
  for (size_t i = 0; i < s.size();) {
    const auto c = static_cast<unsigned char>(s[i]);
    size_t n = c < 0x80 ? 1 : c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : c >= 0xC2 ? 2 : 0;
    if (n == 0 || i + n > s.size()) {
      return false;
    }
    for (size_t k = 1; k < n; ++k) {
      if ((static_cast<unsigned char>(s[i + k]) & 0xC0) != 0x80) {
        return false;
      }
    }
    i += n;
  }
  return true;
}

}  // namespace

TEST_CASE("DriverStationDisplay counts columns in characters, as the Driver Station does",
          "[prosdk][dashboard]") {
  prosdk::telemetry::TelemetryRegistry::Reset();
  auto mock = std::make_shared<Mock>();
  prosdk::telemetry::TelemetryRegistry::RegisterBackend("", mock);
  dash::detail::ResetDisplayForTest();

  // a row is cut at 160 characters, not 160 bytes, and never inside one
  dash::WriteDisplay(kClear);
  dash::WriteDisplay(Repeat("\xC3\xA7", 200));  // "ç" x 200
  CHECK(Text(*mock) == Repeat("\xC3\xA7", dash::kDisplayMaxColumns));
  dash::WriteDisplay(kClear);
  dash::WriteDisplay("a" + Repeat("\xE2\x82\xAC", 200));  // "a" and "€" x 200
  CHECK(Text(*mock) == "a" + Repeat("\xE2\x82\xAC", dash::kDisplayMaxColumns - 1));

  // a tab stops at the next multiple of 8 characters
  dash::WriteDisplay(kClear);
  dash::WriteDisplay("\xC5\x9F\xC4\x9F\tx");  // "şğ", tab, "x"
  CHECK(Text(*mock) == "\xC5\x9F\xC4\x9F      x");

  // cursor positioning and erase work on characters in a row with multi-byte text
  dash::WriteDisplay(kClear);
  dash::WriteDisplay("H\xC4\xB1z: 12 \xC3\xBC/s");  // "Hız: 12 ü/s"
  dash::WriteDisplay("\033[1;6H34");
  CHECK(Text(*mock) == "H\xC4\xB1z: 34 \xC3\xBC/s");
  dash::WriteDisplay("\033[1;2H\xE2\x82\xAC");  // "ı" -> "€"
  CHECK(Text(*mock) == "H\xE2\x82\xACz: 34 \xC3\xBC/s");
  dash::WriteDisplay("\033[1;9H\033[K");
  CHECK(Text(*mock) == "H\xE2\x82\xACz: 34");
  dash::WriteDisplay("\033[1;2H\033[1K");
  CHECK(Text(*mock) == "  z: 34");

  // a character split across two writes is joined; invalid bytes and C1 controls are dropped
  dash::WriteDisplay(kClear);
  dash::WriteDisplay("x\xC3");
  dash::WriteDisplay("\xA7y");
  CHECK(Text(*mock) == "x\xC3\xA7y");
  dash::WriteDisplay(kClear);
  dash::WriteDisplay("a\xFF" "b\x80" "c\xC3" "d\xC2\x85" "e\xE0\x80\x80" "f\xED\xA0\x80" "g");
  CHECK(Text(*mock) == "abcdefg");
  dash::WriteDisplay(kClear);
  dash::WriteDisplay("\xF0\x9F\x99\x82 4");  // a 4-byte character
  CHECK(Text(*mock) == "\xF0\x9F\x99\x82 4");

  // over the default 1024 bytes the top rows are left out, whole: 48 rows of 160 "ş" (320 bytes
  // each) keep the last three
  dash::WriteDisplay(kClear);
  for (size_t r = 0; r < dash::kDisplayMaxRows; ++r) {
    dash::WriteDisplay((r == 0 ? "" : "\n") + Repeat("\xC5\x9F", dash::kDisplayMaxColumns));
  }
  auto text = Text(*mock);
  REQUIRE(text);
  CHECK(text->size() == 3 * 320 + 2);
  CHECK(ValidUtf8(*text));

  dash::detail::ResetDisplayForTest();
  prosdk::telemetry::TelemetryRegistry::Reset();
}

namespace {

// a line-mode frame as DriverStationDisplay::UpdateLines() builds it: every line, the last one
// too, followed by "\033[0m\n"
std::string LineFrame(size_t lines) {
  std::string frame{kClear};
  for (size_t i = 0; i < lines; ++i) {
    frame += "L" + std::to_string(i) + "\033[0m\n";
  }
  return frame;
}

}  // namespace

TEST_CASE("DriverStationDisplay rows: all 48 in line mode, cursor positions stop at the last",
          "[prosdk][dashboard]") {
  prosdk::telemetry::TelemetryRegistry::Reset();
  auto mock = std::make_shared<Mock>();
  prosdk::telemetry::TelemetryRegistry::RegisterBackend("", mock);
  dash::detail::ResetDisplayForTest();

  // 48 lines, each with its newline: the first one (a title) stays on screen
  dash::WriteDisplay(LineFrame(dash::kDisplayMaxRows));
  auto text = Text(*mock);
  REQUIRE(text);
  CHECK(text->starts_with("L0\nL1\n"));
  CHECK(text->ends_with("\nL47"));
  CHECK(CountOf(*text, "\n") == dash::kDisplayMaxRows - 1);
  // the next frame starts from a cleared screen, with nothing scrolled into it
  dash::WriteDisplay(LineFrame(2));
  CHECK(Text(*mock) == "L0\nL1");

  // a 49th line scrolls the first one off
  dash::WriteDisplay(LineFrame(dash::kDisplayMaxRows + 1));
  text = Text(*mock);
  REQUIRE(text);
  CHECK(text->starts_with("L1\nL2\n"));
  CHECK(text->ends_with("\nL48"));

  // text after a newline on the last row lands on the new last row (raw mode, two writes)
  dash::WriteDisplay(LineFrame(dash::kDisplayMaxRows));
  dash::WriteDisplay("more");
  text = Text(*mock);
  REQUIRE(text);
  CHECK(text->starts_with("L1\n"));
  CHECK(text->ends_with("\nL47\nmore"));

  // a cursor position past the last row stops there and scrolls nothing
  dash::WriteDisplay("\033[2J\033[Htop\033[60;1Hbottom");
  text = Text(*mock);
  REQUIRE(text);
  CHECK(text->starts_with("top\n"));
  CHECK(text->ends_with("\nbottom"));
  CHECK(CountOf(*text, "\n") == dash::kDisplayMaxRows - 1);
  // so does a cursor move down
  dash::WriteDisplay("\033[2J\033[Htop\033[100Bend");
  text = Text(*mock);
  REQUIRE(text);
  CHECK(text->starts_with("top\n"));
  CHECK(text->ends_with("\n" + std::string(3, ' ') + "end"));
  CHECK(CountOf(*text, "\n") == dash::kDisplayMaxRows - 1);

  dash::detail::ResetDisplayForTest();
  prosdk::telemetry::TelemetryRegistry::Reset();
}
