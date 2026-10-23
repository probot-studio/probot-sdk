// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Driver Station display (prosdk::DriverStationDisplay, HAL_WriteDisplayAnsi) as telemetry.
//
// Upstream's Driver Station draws the display as a panel of lines updated in place (line mode)
// or as an ANSI terminal (raw mode). probot-core's Driver Station page (R19, core PR #44) shows a
// dashboard folder whose .type is "DriverStationDisplay" as a monospace text panel: its string
// child "text", lines separated by '\n', at most 48 lines of 160 columns (a column is one Unicode
// code point; spaces kept, no colors), on the "Veri" page and, for /Telemetry/DriverStationDisplay,
// as the Dashboard's "Robot Ekranı" card. The ESP32 HAL passes every display write to
// WriteDisplay(), which keeps the screen in a small terminal and logs it as plain text in the
// telemetry table "DriverStationDisplay" (.type "DriverStationDisplay", string "text"), i.e.
// /Telemetry/DriverStationDisplay/text. Nothing goes to the console.
//   - The terminal handles what upstream's two modes write: text, newline (also back to column 1),
//     carriage return, backspace, tab, and the CSI sequences for cursor position (H, f), cursor
//     moves (A, B, C, D, G) and erase (J, K). Colors and every other sequence are dropped.
//   - Text is UTF-8: one column per code point, as the Driver Station counts them; a sequence
//     split across two writes is joined; invalid bytes and C0/C1 control characters are dropped.
//   - It has at most kDisplayMaxRows rows of at most kDisplayMaxColumns columns (more is dropped).
//     As in a terminal, a newline on the last row scrolls the top row off (once more text
//     follows, so line mode's newline after its last line keeps all 48 rows), while cursor
//     positions and moves stop at the last row. The logged text has no trailing spaces or
//     empty rows and at most DisplayMaxBytes() bytes: top rows are left out to fit, and a single
//     row longer than that is cut at a UTF-8 character boundary.
//   - DisplayMaxBytes() follows core's PROBOT_DASH_MAX_VALUE (the largest value core's store
//     accepts, CoreOps::dashMaxValue): the default 1024 bytes hold about 20 columns on each of 48
//     rows; a full screen of ASCII text is kDisplayFullScreenAsciiBytes (7727) bytes, so
//     #define PROBOT_DASH_MAX_VALUE 8192 before #include <prosdk.h> shows all of it (see
//     design/API-DIFFERENCES.md for the memory this costs).
//   - The text is logged only when it changed, so UpdateLines() resending an unchanged frame every
//     230 ms costs nothing.

#pragma once

#include <stddef.h>

#include <string_view>

namespace prosdk::platform::dashboard {

/** Rows and columns of the display terminal (the Driver Station panel's size). */
inline constexpr size_t kDisplayMaxRows = 48;
inline constexpr size_t kDisplayMaxColumns = 160;

/** Bytes of a full screen of ASCII text: 48 rows of 160 columns and 47 newlines. */
inline constexpr size_t kDisplayFullScreenAsciiBytes =
    kDisplayMaxRows * kDisplayMaxColumns + kDisplayMaxRows - 1;

/** The longest text a screen can hold (every column a 4-byte UTF-8 character). */
inline constexpr size_t kDisplayMaxScreenBytes =
    kDisplayMaxRows * kDisplayMaxColumns * 4 + kDisplayMaxRows - 1;

/** The value limit assumed while core's is unknown: core's default PROBOT_DASH_MAX_VALUE. */
inline constexpr size_t kDisplayDefaultMaxBytes = 1024;

/**
 * At most this many bytes of text are logged: core's dashboard value limit as configured
 * (PROBOT_DASH_MAX_VALUE, 16-16384; kDisplayDefaultMaxBytes without core or before core's
 * runtime_setup()), never more than kDisplayMaxScreenBytes.
 */
size_t DisplayMaxBytes() noexcept;

/** Telemetry table of the display (under the telemetry backend's "/Telemetry"). */
inline constexpr std::string_view kDisplayTable = "DriverStationDisplay";

/** Applies one display write (ANSI text) and logs the screen if it changed. Thread-safe. */
void WriteDisplay(std::string_view ansi);

namespace detail {
/** Host tests only: forgets the screen and the last logged text. */
void ResetDisplayForTest();
}  // namespace detail

}  // namespace prosdk::platform::dashboard
