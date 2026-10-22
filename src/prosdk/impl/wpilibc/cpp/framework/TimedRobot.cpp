// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/framework/TimedRobot.hpp"

#include <stdint.h>

#include <cstdio>
#include <utility>

#include "prosdk/driverstation/DriverStation.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/hal/DriverStation.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

// prosdk: hosted robot loop (design/PORT.md section 8)
RobotBase::HostedLoop TimedRobot::StartHosted() {
  if constexpr (IsSimulation()) {
    SimulationInit();
  }

  // Tell the DS that the robot is ready to be enabled
  prosdk::platform::ConsoleWrite(
      "\n********** Robot program startup complete **********\n");
  prosdk::internal::DriverStationBackend::ObserveUserProgramStarting();

  return {&m_callbacks, &m_startTime, GetPeriod()};
}

void TimedRobot::StartCompetition() {
  StartHosted();

  // Loop forever, calling the appropriate mode-dependent function
  while (true) {
    if (!m_callbacks.RunCallbacks(m_notifier)) {
      break;
    }
  }
}

void TimedRobot::EndCompetition() {
  HAL_DestroyNotifier(m_notifier);
  m_notifier = HAL_INVALID_HANDLE;
}

TimedRobot::TimedRobot(prosdk::units::second_t period)
    : IterativeRobotBase(period) {
  m_startTime = std::chrono::nanoseconds{RobotController::GetMonotonicTime()};
  AddPeriodic([=, this] { LoopFunc(); }, period);

  int32_t status = 0;
  m_notifier = HAL_CreateNotifier(&status);
  WPILIB_CheckErrorStatus(status, "InitializeNotifier");
  HAL_SetNotifierName(m_notifier, "TimedRobot", &status);

  prosdk::util::ReportUsage("Framework", "TimedRobot");
}

TimedRobot::TimedRobot(prosdk::units::hertz_t frequency)
    : TimedRobot{1 / frequency} {}

TimedRobot::~TimedRobot() {
  if (m_notifier != HAL_INVALID_HANDLE) {
    HAL_DestroyNotifier(m_notifier);
  }
}

void TimedRobot::AddPeriodic(std::function<void()> callback,
                             prosdk::units::second_t period,
                             prosdk::units::second_t offset) {
  m_callbacks.Add(std::move(callback), m_startTime, period, offset);
}
