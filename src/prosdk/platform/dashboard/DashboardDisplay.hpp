// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Driver Station display (prosdk::DriverStationDisplay, HAL_WriteDisplayAnsi) as telemetry.
//
// Upstream's Driver Station draws the display as a panel of lines updated in place (line mode)
// or as an ANSI terminal (raw mode). The Probot Driver Station page has no such panel
// (design/core-requests.md R19). The ESP32 HAL passes every display write to WriteDisplay(), which
// keeps the screen in a small terminal and logs it as plain text in the telemetry table
// "DriverStationDisplay" (.type "DriverStationDisplay", string "text"): the Driver Station shows it
// in place in its topic tree as /Telemetry/DriverStationDisplay/text. Nothing goes to the console.
//   - The terminal handles what upstream's two modes write: text, newline (also back to column 1),
//     carriage return, backspace, tab, and the CSI sequences for cursor position (H, f), cursor
//     moves (A, B, C, D, G) and erase (J, K). Colors and every other sequence are dropped.
//   - It has at most kDisplayMaxRows rows (text below scrolls the top rows off) of at most
//     kDisplayMaxColumns columns (more is dropped). The logged text has no trailing spaces or
//     empty rows and at most kDisplayMaxBytes bytes (top rows are left out to fit).
//   - The text is logged only when it changed, so UpdateLines() resending an unchanged frame every
//     230 ms costs nothing.

#pragma once

#include <stddef.h>

#include <string_view>

namespace prosdk::platform::dashboard {

/** Rows and columns of the display terminal. */
inline constexpr size_t kDisplayMaxRows = 48;
inline constexpr size_t kDisplayMaxColumns = 160;

/** At most this many bytes of text are logged (probot-core's default PROBOT_DASH_MAX_VALUE). */
inline constexpr size_t kDisplayMaxBytes = 1024;

/** Telemetry table of the display (under the telemetry backend's "/Telemetry"). */
inline constexpr std::string_view kDisplayTable = "DriverStationDisplay";

/** Applies one display write (ANSI text) and logs the screen if it changed. Thread-safe. */
void WriteDisplay(std::string_view ansi);

namespace detail {
/** Host tests only: forgets the screen and the last logged text. */
void ResetDisplayForTest();
}  // namespace detail

}  // namespace prosdk::platform::dashboard
