// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DefaultTeleop.hpp"

#include <array>
#include <cstdio>

#include <prosdk/driverstation/DriverStationDisplay.hpp>

using namespace prosdk::units::literals;

namespace {
constexpr std::array<int, 20> COLORS = {196, 202, 208, 214, 220, 226, 118,
                                        46,  48,  51,  45,  39,  33,  27,
                                        21,  57,  93,  129, 165, 201};
}  // namespace

void DefaultTeleop::Start() {
  prosdk::DriverStationDisplay::SetMode(prosdk::DriverStationDisplay::Mode::RawAnsi);
  m_timer.Restart();
  m_seconds = 0;

  prosdk::DriverStationDisplay::WriteRawAnsi(
      "\033[2J\033[H"
      "DriverStationDisplay ANSI mode\n"
      "This header and footer stay on screen.\n"
      "Seconds elapsed:      \n"
      "Only the number above is rewritten.");
  UpdateSecondsDisplay();
}

void DefaultTeleop::Periodic() {
  if (m_timer.AdvanceIfElapsed(1_s)) {
    ++m_seconds;
    UpdateSecondsDisplay();
  }
}

void DefaultTeleop::UpdateSecondsDisplay() {
  int color = COLORS[m_seconds % COLORS.size()];
  std::array<char, 64> buffer;
  std::snprintf(buffer.data(), buffer.size(),
                "\033[3;18H\033[38;5;%dm%5d\033[0m", color, m_seconds);
  prosdk::DriverStationDisplay::WriteRawAnsi(buffer.data());
}
