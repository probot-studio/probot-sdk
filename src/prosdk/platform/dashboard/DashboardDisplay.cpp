// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/dashboard/DashboardDisplay.hpp"

#include <algorithm>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"

namespace prosdk::platform::dashboard {
namespace {

// A small terminal: the cursor writes into a grid of rows. Enough for upstream's line mode
// (clear, home, lines) and raw mode (cursor positioning, erase, colors ignored).
struct Display {
  std::mutex mutex;
  std::vector<std::string> rows;
  size_t row = 0;
  size_t col = 0;
  std::string logged;
  bool hasLogged = false;

  void Put(char c) {
    if (col >= kDisplayMaxColumns) {
      return;
    }
    if (rows.size() <= row) {
      rows.resize(row + 1);
    }
    auto& line = rows[row];
    if (line.size() < col) {
      line.append(col - line.size(), ' ');
    }
    if (col < line.size()) {
      line[col] = c;
    } else {
      line.push_back(c);
    }
    ++col;
  }

  void SetRow(size_t r) {
    if (r >= kDisplayMaxRows) {
      // scroll: the screen keeps its last kDisplayMaxRows rows
      size_t drop = r - kDisplayMaxRows + 1;
      rows.erase(rows.begin(), rows.begin() + std::min(drop, rows.size()));
      r = kDisplayMaxRows - 1;
    }
    row = r;
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
          if (rows[row].size() > col) {
            rows[row].resize(col);
          }
        }
        return;
      }
      for (size_t r = 0; r < row && r < rows.size(); ++r) {
        rows[r].clear();
      }
    }
    if (row >= rows.size()) {
      return;
    }
    auto& line = rows[row];
    if (mode == 2) {
      line.clear();
    } else if (mode == 0) {
      if (line.size() > col) {
        line.resize(col);
      }
    } else {
      line.replace(0, std::min(col + 1, line.size()), std::min(col + 1, line.size()), ' ');
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
        SetRow(row + 1);
        col = 0;
      } else if (c == '\r') {
        col = 0;
      } else if (c == '\b') {
        col = col > 0 ? col - 1 : 0;
      } else if (c == '\t') {
        do {
          Put(' ');
        } while (col % 8 != 0 && col < kDisplayMaxColumns);
      } else if (static_cast<unsigned char>(c) >= 0x20 && c != 0x7F) {
        Put(c);
      }
    }
  }

  // rows without trailing spaces or trailing empty rows; leading rows dropped to fit
  std::string Render() const {
    std::vector<std::string_view> lines;
    for (const auto& line : rows) {
      std::string_view view = line;
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
    while (size > kDisplayMaxBytes && first + 1 < lines.size()) {
      size -= lines[first].size() + 1;
      ++first;
    }
    std::string text;
    for (size_t i = first; i < lines.size(); ++i) {
      if (i != first) {
        text.push_back('\n');
      }
      text.append(lines[i]);
    }
    return text;
  }
};

Display& GetDisplay() {
  static Display display;
  return display;
}

}  // namespace

void WriteDisplay(std::string_view ansi) {
  auto& display = GetDisplay();
  std::scoped_lock lock{display.mutex};
  display.Write(ansi);
  std::string text = display.Render();
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
  display.logged.clear();
  display.hasLogged = false;
}
}  // namespace detail

}  // namespace prosdk::platform::dashboard
