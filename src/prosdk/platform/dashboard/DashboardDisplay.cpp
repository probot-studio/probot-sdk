// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/dashboard/DashboardDisplay.hpp"

#include <stdint.h>

#include <algorithm>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"

namespace prosdk::platform::dashboard {
namespace {

// Length of a UTF-8 sequence from its lead byte; 0 for a byte that cannot start one.
size_t LeadLength(unsigned char c) {
  if (c >= 0xC2 && c <= 0xDF) {
    return 2;
  }
  if (c >= 0xE0 && c <= 0xEF) {
    return 3;
  }
  if (c >= 0xF0 && c <= 0xF4) {
    return 4;
  }
  return 0;
}

// True when `c` may follow `lead` as byte `index` (1-3) of a sequence: no overlong forms, no
// surrogates, nothing past U+10FFFF.
bool Continues(unsigned char lead, size_t index, unsigned char c) {
  unsigned char lo = 0x80;
  unsigned char hi = 0xBF;
  if (index == 1) {
    if (lead == 0xE0) {
      lo = 0xA0;
    } else if (lead == 0xED) {
      hi = 0x9F;
    } else if (lead == 0xF0) {
      lo = 0x90;
    } else if (lead == 0xF4) {
      hi = 0x8F;
    }
  }
  return c >= lo && c <= hi;
}

bool IsContinuation(char c) {
  return (static_cast<unsigned char>(c) & 0xC0) == 0x80;
}

// One terminal row: UTF-8 text, one column per code point.
struct Row {
  std::string text;
  size_t cells = 0;

  // byte offset of column `cell` (<= cells)
  size_t Offset(size_t cell) const {
    if (text.size() == cells) {
      return cell;  // ASCII only
    }
    size_t b = 0;
    for (size_t n = 0; n < cell; ++n) {
      ++b;
      while (b < text.size() && IsContinuation(text[b])) {
        ++b;
      }
    }
    return b;
  }

  // keeps the first `n` columns
  void Truncate(size_t n) {
    if (n < cells) {
      text.resize(Offset(n));
      cells = n;
    }
  }

  void Clear() {
    text.clear();
    cells = 0;
  }
};

// A small terminal: the cursor writes into a grid of rows. Enough for upstream's line mode
// (clear, home, lines) and raw mode (cursor positioning, erase, colors ignored).
struct Display {
  std::mutex mutex;
  std::vector<Row> rows;
  size_t row = 0;
  size_t col = 0;
  // a UTF-8 sequence not complete yet (also across writes)
  char seq[4] = {};
  size_t seqLen = 0;
  size_t seqNeed = 0;
  // a newline on the last row whose scroll has not happened yet (NewLine)
  bool pendingScroll = false;
  std::string logged;
  bool hasLogged = false;

  // one character (a complete UTF-8 sequence) at the cursor
  void Put(std::string_view ch) {
    if (col >= kDisplayMaxColumns) {
      return;
    }
    if (rows.size() <= row) {
      rows.resize(row + 1);
    }
    auto& line = rows[row];
    if (line.cells < col) {
      line.text.append(col - line.cells, ' ');
      line.cells = col;
    }
    if (col == line.cells) {
      line.text.append(ch);
      ++line.cells;
    } else {
      const size_t b = line.Offset(col);
      size_t n = 1;
      while (b + n < line.text.size() && IsContinuation(line.text[b + n])) {
        ++n;
      }
      line.text.replace(b, n, ch);
    }
    ++col;
  }

  // cursor position and moves stop at the last row, as in a terminal (they never scroll)
  void SetRow(size_t r) { row = std::min(r, kDisplayMaxRows - 1); }

  // newline: the next row; on the last row the screen scrolls one row up. That scroll waits for
  // the next byte, so a screen that ends with a newline (upstream's line mode ends every line with
  // one) keeps all kDisplayMaxRows rows.
  void NewLine() {
    if (row + 1 < kDisplayMaxRows) {
      ++row;
    } else {
      pendingScroll = true;
    }
    col = 0;
  }

  void Scroll() {
    pendingScroll = false;
    if (!rows.empty()) {
      rows.erase(rows.begin());
    }
  }

  void Erase(char final, int mode) {
    if (final == 'J') {
      if (mode == 2 || mode == 3) {
        rows.clear();
        return;
      }
      if (mode == 0) {
        if (row < rows.size()) {
          rows.resize(row + 1);
          rows[row].Truncate(col);
        }
        return;
      }
      for (size_t r = 0; r < row && r < rows.size(); ++r) {
        rows[r].Clear();
      }
    }
    if (row >= rows.size()) {
      return;
    }
    auto& line = rows[row];
    if (mode == 2) {
      line.Clear();
    } else if (mode == 0) {
      line.Truncate(col);
    } else {
      // start of the line through the cursor become spaces
      const size_t n = std::min(col + 1, line.cells);
      line.text.replace(0, line.Offset(n), n, ' ');
    }
  }

  // CSI parameters (ESC [ ...), final byte `final`
  void Csi(std::string_view params, char final) {
    if (!params.empty() && (params[0] < '0' || params[0] > ';')) {
      return;  // private sequence (ESC [ ? ...): ignored
    }
    int p[2] = {0, 0};
    size_t n = 0;
    for (char c : params) {
      if (c == ';') {
        if (++n == 2) {
          break;
        }
      } else if (c >= '0' && c <= '9' && p[n] < 10000) {
        p[n] = p[n] * 10 + (c - '0');
      }
    }
    size_t count = p[0] > 0 ? static_cast<size_t>(p[0]) : 1;
    switch (final) {
      case 'H':
      case 'f':
        SetRow(p[0] > 0 ? p[0] - 1 : 0);
        col = p[1] > 0 ? p[1] - 1 : 0;
        break;
      case 'A':
        row = row > count ? row - count : 0;
        break;
      case 'B':
        SetRow(row + count);
        break;
      case 'C':
        col += count;
        break;
      case 'D':
        col = col > count ? col - count : 0;
        break;
      case 'G':
        col = count - 1;
        break;
      case 'J':
      case 'K':
        Erase(final, p[0]);
        break;
      default:
        break;  // colors (m) and the rest: ignored
    }
  }

  void Write(std::string_view ansi) {
    for (size_t i = 0; i < ansi.size(); ++i) {
      const char c = ansi[i];
      const auto u = static_cast<unsigned char>(c);
      if (pendingScroll) {
        Scroll();
      }
      if (seqNeed != 0) {
        if (Continues(static_cast<unsigned char>(seq[0]), seqLen, u)) {
          seq[seqLen++] = c;
          if (seqLen == seqNeed) {
            // C1 control characters (U+0080-U+009F) are dropped like C0 ones
            if (!(static_cast<unsigned char>(seq[0]) == 0xC2 &&
                  static_cast<unsigned char>(seq[1]) < 0xA0)) {
              Put(std::string_view{seq, seqLen});
            }
            seqLen = 0;
            seqNeed = 0;
          }
          continue;
        }
        // invalid sequence: dropped; this byte starts over
        seqLen = 0;
        seqNeed = 0;
      }
      if (c == '\033') {
        if (i + 1 < ansi.size() && ansi[i + 1] == '[') {
          // CSI: parameters/intermediates, then one final byte in 0x40..0x7E
          size_t begin = i + 2;
          size_t end = begin;
          while (end < ansi.size() && !(ansi[end] >= 0x40 && ansi[end] <= 0x7E)) {
            ++end;
          }
          if (end < ansi.size()) {
            Csi(ansi.substr(begin, end - begin), ansi[end]);
          }
          i = end;
        } else {
          ++i;  // two-byte escape
        }
      } else if (c == '\n') {
        NewLine();
      } else if (c == '\r') {
        col = 0;
      } else if (c == '\b') {
        col = col > 0 ? col - 1 : 0;
      } else if (c == '\t') {
        do {
          Put(" ");
        } while (col % 8 != 0 && col < kDisplayMaxColumns);
      } else if (u >= 0x80) {
        seqNeed = LeadLength(u);  // 0: a byte that cannot start a sequence, dropped
        if (seqNeed != 0) {
          seq[0] = c;
          seqLen = 1;
        }
      } else if (u >= 0x20 && u != 0x7F) {
        Put(std::string_view{&c, 1});
      }
    }
  }

  // rows without trailing spaces or trailing empty rows; leading rows dropped to fit `maxBytes`,
  // and a single row longer than that cut at a character boundary
  std::string Render(size_t maxBytes) const {
    std::vector<std::string_view> lines;
    lines.reserve(rows.size());
    for (const auto& line : rows) {
      std::string_view view = line.text;
      while (!view.empty() && view.back() == ' ') {
        view.remove_suffix(1);
      }
      lines.push_back(view);
    }
    while (!lines.empty() && lines.back().empty()) {
      lines.pop_back();
    }
    size_t first = 0;
    size_t size = lines.empty() ? 0 : lines.size() - 1;
    for (auto line : lines) {
      size += line.size();
    }
    while (size > maxBytes && first + 1 < lines.size()) {
      size -= lines[first].size() + 1;
      ++first;
    }
    std::string text;
    text.reserve(std::min(size, maxBytes));
    for (size_t i = first; i < lines.size(); ++i) {
      if (i != first) {
        text.push_back('\n');
      }
      text.append(lines[i]);
    }
    if (text.size() > maxBytes) {
      size_t cut = maxBytes;
      while (cut > 0 && IsContinuation(text[cut])) {
        --cut;
      }
      text.resize(cut);
      while (!text.empty() && text.back() == ' ') {
        text.pop_back();
      }
    }
    return text;
  }
};

Display& GetDisplay() {
  static Display display;
  return display;
}

}  // namespace

size_t DisplayMaxBytes() noexcept {
  const uint32_t limit = prosdk::platform::GetCoreOps().dashMaxValue();
  return std::min(limit != 0 ? size_t{limit} : kDisplayDefaultMaxBytes, kDisplayMaxScreenBytes);
}

void WriteDisplay(std::string_view ansi) {
  const size_t maxBytes = DisplayMaxBytes();
  auto& display = GetDisplay();
  std::scoped_lock lock{display.mutex};
  display.Write(ansi);
  std::string text = display.Render(maxBytes);
  if (display.hasLogged && text == display.logged) {
    return;
  }
  display.logged = std::move(text);
  display.hasLogged = true;
  auto& table = prosdk::telemetry::GetTable(kDisplayTable);
  if (table.SetType("DriverStationDisplay")) {
    table.Log("text", std::string_view{display.logged});
  }
}

namespace detail {
void ResetDisplayForTest() {
  auto& display = GetDisplay();
  std::scoped_lock lock{display.mutex};
  display.rows.clear();
  display.row = 0;
  display.col = 0;
  display.seqLen = 0;
  display.seqNeed = 0;
  display.pendingScroll = false;
  display.logged.clear();
  display.hasLogged = false;
}
}  // namespace detail

}  // namespace prosdk::platform::dashboard
