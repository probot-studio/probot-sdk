// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// std::format front end of the platform console (prosdk/platform/Console.hpp). The wpilibc
// patches send the robot framework's status lines ("Robot program starting", opmode
// transitions, default-method warnings) here instead of stdout/stderr, so on the robot they reach
// the Driver Station console.

#pragma once

#include <format>
#include <string>
#include <utility>

#include "prosdk/platform/Console.hpp"

namespace prosdk::platform {

/** Formats and writes to the console sink. Never throws (a formatting failure drops the text). */
template <class... Args>
void ConsolePrint(std::format_string<Args...> fmt, Args&&... args) noexcept {
  try {
    ConsoleWrite(std::format(fmt, std::forward<Args>(args)...));
  } catch (...) {
  }
}

}  // namespace prosdk::platform
