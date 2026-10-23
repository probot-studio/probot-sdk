// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DriverStationDisplay through the Driver Station telemetry backend into the (fake) core store:
// the text's byte cap follows core's value limit (PROBOT_DASH_MAX_VALUE, CoreOps dashMaxValue), so
// a sketch that raises it to 8192 gets a full 48 x 160 screen, and the default 1024 bytes keep
// working: the text is cut to fit (whole rows, then on a UTF-8 boundary), so the store never
// refuses it as too large. (Core's store can still refuse a value with NoMemory when its pool,
// PROBOT_DASH_POOL_BYTES, cannot hold the grown slot; this fake store has no pool.)

#include <optional>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/platform/dashboard/DashboardDisplay.hpp"

using dashboard_test::CountOf;
using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;

namespace {

constexpr std::string_view kClear = "\033[0m\033[2J\033[H";
constexpr std::string_view kText = "/Telemetry/DriverStationDisplay/text";

std::string Repeat(std::string_view s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n; ++i) {
    out.append(s);
  }
  return out;
}

// one full screen: 48 rows of 160 columns, each row starting with its number
std::string FullScreen(std::string_view fill) {
  std::string frame{kClear};
  for (size_t r = 0; r < dash::kDisplayMaxRows; ++r) {
    std::string row = std::to_string(r % 10) + Repeat(fill, dash::kDisplayMaxColumns - 1);
    frame += row;
    if (r + 1 < dash::kDisplayMaxRows) {
      frame += '\n';
    }
  }
  return frame;
}

class DisplayFixture : public DashboardFixture {
 public:
  DisplayFixture() { dash::detail::ResetDisplayForTest(); }
  ~DisplayFixture() { dash::detail::ResetDisplayForTest(); }

  static std::optional<std::string> Text() { return fake::GetString(kText); }
  static bool Refused() {
    return !fake::Rejects().empty() || CountOf(fake::Console(), "not stored") != 0;
  }
};

}  // namespace

TEST_CASE_METHOD(DisplayFixture, "Display text follows core's PROBOT_DASH_MAX_VALUE",
                 "[prosdk][dashboard][display]") {
  SECTION("8192: a full screen of ASCII text") {
    fake::SetMaxValue(8192);
    CHECK(dash::DisplayMaxBytes() == 8192);
    dash::WriteDisplay(FullScreen("x"));
    auto text = Text();
    REQUIRE(text);
    CHECK(text->size() == dash::kDisplayFullScreenAsciiBytes);
    CHECK(dash::kDisplayFullScreenAsciiBytes == 7727);
    CHECK(text->starts_with("0" + Repeat("x", 159) + "\n1"));
    CHECK(text->ends_with("\n7" + Repeat("x", 159)));
    auto type = fake::GetString("/Telemetry/DriverStationDisplay/.type");
    REQUIRE(type);
    CHECK(*type == "DriverStationDisplay");
    CHECK_FALSE(Refused());
  }

  SECTION("8192: a full screen in line mode (a newline after every line, the last one too)") {
    fake::SetMaxValue(8192);
    std::string frame{kClear};
    for (size_t r = 0; r < dash::kDisplayMaxRows; ++r) {
      frame += std::to_string(r % 10) + Repeat("x", dash::kDisplayMaxColumns - 1) + "\033[0m\n";
    }
    dash::WriteDisplay(frame);
    auto text = Text();
    REQUIRE(text);
    CHECK(text->size() == dash::kDisplayFullScreenAsciiBytes);
    CHECK(text->starts_with("0" + Repeat("x", 159) + "\n1"));
    CHECK(text->ends_with("\n7" + Repeat("x", 159)));
    CHECK_FALSE(Refused());
  }

  SECTION("16384: a full screen of two-byte characters") {
    fake::SetMaxValue(16384);
    CHECK(dash::DisplayMaxBytes() == 16384);
    dash::WriteDisplay(FullScreen("\xC5\x9F"));  // "ş"
    auto text = Text();
    REQUIRE(text);
    CHECK(text->size() == 48 * (1 + 159 * 2) + 47);  // 15,359 B (each row starts with a digit)
    CHECK_FALSE(Refused());
    // every column two bytes: 48 x 160 x 2 + 47 = 15,407 B, also under 16384
    std::string frame{kClear};
    for (size_t r = 0; r < dash::kDisplayMaxRows; ++r) {
      frame += Repeat("\xC5\x9F", dash::kDisplayMaxColumns) + "\n";
    }
    dash::WriteDisplay(frame);
    text = Text();
    REQUIRE(text);
    CHECK(text->size() == 15407);
    CHECK(text->starts_with(Repeat("\xC5\x9F", 160) + "\n"));
    CHECK_FALSE(Refused());
  }

  SECTION("1024 (core's default): the last rows that fit, never too large") {
    CHECK(dash::DisplayMaxBytes() == 1024);
    dash::WriteDisplay(FullScreen("x"));
    auto text = Text();
    REQUIRE(text);
    // six rows of 161 bytes (160 and a newline) fit in 1024, the seventh does not
    CHECK(text->size() == 6 * 161 - 1);
    CHECK(text->starts_with("2" + Repeat("x", 159) + "\n"));
    CHECK(text->ends_with("\n7" + Repeat("x", 159)));
    CHECK_FALSE(Refused());
  }

  SECTION("32: one row cut on a character boundary") {
    // (the .type value "DriverStationDisplay", 20 bytes, needs more than core's minimum 16)
    fake::SetMaxValue(32);
    CHECK(dash::DisplayMaxBytes() == 32);
    dash::WriteDisplay(std::string{kClear} + "a" + Repeat("\xC3\xA7", 20));  // "a" + "ç" x 20
    auto text = Text();
    REQUIRE(text);
    CHECK(*text == "a" + Repeat("\xC3\xA7", 15));  // 31 bytes: the 16th "ç" would end at byte 33
    CHECK_FALSE(Refused());
  }

  SECTION("before core configured its store: core's default") {
    fake::SetReady(false);
    CHECK(dash::DisplayMaxBytes() == dash::kDisplayDefaultMaxBytes);
    fake::SetReady(true);
    fake::SetMaxValue(4096);
    CHECK(dash::DisplayMaxBytes() == 4096);
  }
}
