// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/util/MathShared.hpp"

#include <format>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

#include "prosdk/platform/Console.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/util/mutex.hpp"
#include "prosdk/util/timestamp.hpp"

using namespace prosdk::math;

namespace {
// prosdk: errors and warnings go to the platform console (the Driver Station
// console on the robot) instead of being dropped; reporting never throws.
void ReportToConsole(std::string_view prefix, std::string_view format,
                     std::format_args args) noexcept {
  try {
    std::string text{prefix};
    text += std::vformat(format, args);
    text += '\n';
    prosdk::platform::ConsoleWrite(text);
  } catch (...) {
  }
}

class DefaultMathShared : public MathShared {
 public:
  void ReportErrorV(std::string_view format, std::format_args args) override {
    ReportToConsole("Error: ", format, args);
  }
  void ReportWarningV(std::string_view format, std::format_args args) override {
    ReportToConsole("Warning: ", format, args);
  }
  prosdk::units::second_t GetTimestamp() override {
    return prosdk::units::second_t{prosdk::util::Now() * 1.0e-9};
  }
};
}  // namespace

static std::unique_ptr<MathShared> mathShared;
static prosdk::util::mutex setLock;

MathShared& MathSharedStore::GetMathShared() {
  std::scoped_lock lock(setLock);
  if (!mathShared) {
    mathShared = std::make_unique<DefaultMathShared>();
  }
  return *mathShared;
}

void MathSharedStore::SetMathShared(std::unique_ptr<MathShared> shared) {
  std::scoped_lock lock(setLock);
  mathShared = std::move(shared);
}
