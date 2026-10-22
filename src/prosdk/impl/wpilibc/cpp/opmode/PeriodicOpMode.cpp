// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/opmode/PeriodicOpMode.hpp"

#include <utility>

#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

PeriodicOpMode::PeriodicOpMode()
    : m_startTime{
          std::chrono::nanoseconds{RobotController::GetMonotonicTime()}} {
  prosdk::util::ReportUsage("OpMode", "PeriodicOpMode");
}

void PeriodicOpMode::AddPeriodic(std::function<void()> callback,
                                 prosdk::units::second_t period,
                                 prosdk::units::second_t offset) {
  m_callbacks.emplace_back(
      std::move(callback), m_startTime,
      std::chrono::nanoseconds{static_cast<int64_t>(period.value() * 1e9)},
      std::chrono::nanoseconds{static_cast<int64_t>(offset.value() * 1e9)});
}
