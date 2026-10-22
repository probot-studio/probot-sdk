// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/framework/IterativeRobotBase.hpp"

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/hal/DriverStation.h"
#include "prosdk/hal/DriverStationTypes.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/util/print.hpp"

using namespace prosdk;

IterativeRobotBase::IterativeRobotBase(prosdk::units::second_t period)
    : m_period(period),
      m_watchdog(period, [this] { PrintLoopOverrunMessage(); }) {}

void IterativeRobotBase::DriverStationConnected() {}

void IterativeRobotBase::SimulationInit() {}

void IterativeRobotBase::DisabledInit() {}

void IterativeRobotBase::AutonomousInit() {}

void IterativeRobotBase::TeleopInit() {}

void IterativeRobotBase::UtilityInit() {}

void IterativeRobotBase::RobotPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    prosdk::platform::ConsolePrint("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::SimulationPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    prosdk::platform::ConsolePrint("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::DisabledPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    prosdk::platform::ConsolePrint("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::AutonomousPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    prosdk::platform::ConsolePrint("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::TeleopPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    prosdk::platform::ConsolePrint("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::UtilityPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    prosdk::platform::ConsolePrint("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::DisabledExit() {}

void IterativeRobotBase::AutonomousExit() {}

void IterativeRobotBase::TeleopExit() {}

void IterativeRobotBase::UtilityExit() {}

prosdk::units::second_t IterativeRobotBase::GetPeriod() const {
  return m_period;
}

void IterativeRobotBase::LoopFunc() {
  prosdk::internal::DriverStationBackend::RefreshData();
  m_watchdog.Reset();

  // Get current mode; treat disabled as unknown
  prosdk::hal::ControlWord word = prosdk::hal::GetControlWord();
  bool enabled = word.IsEnabled();
  RobotMode mode = enabled ? word.GetRobotMode() : RobotMode::UNKNOWN;

  if (!m_calledDsConnected && word.IsDSAttached()) {
    m_calledDsConnected = true;
    DriverStationConnected();
  }

  // If mode changed, call mode exit and entry functions
  if (m_lastMode != static_cast<int>(mode)) {
    // Call last mode's exit function
    if (m_lastMode == static_cast<int>(RobotMode::UNKNOWN)) {
      DisabledExit();
    } else if (m_lastMode == static_cast<int>(RobotMode::AUTONOMOUS)) {
      AutonomousExit();
    } else if (m_lastMode == static_cast<int>(RobotMode::TELEOPERATED)) {
      TeleopExit();
    } else if (m_lastMode == static_cast<int>(RobotMode::UTILITY)) {
      UtilityExit();
    }

    // Call current mode's entry function
    if (mode == RobotMode::UNKNOWN) {
      DisabledInit();
      m_watchdog.AddEpoch("DisabledInit()");
    } else if (mode == RobotMode::AUTONOMOUS) {
      AutonomousInit();
      m_watchdog.AddEpoch("AutonomousInit()");
    } else if (mode == RobotMode::TELEOPERATED) {
      TeleopInit();
      m_watchdog.AddEpoch("TeleopInit()");
    } else if (mode == RobotMode::UTILITY) {
      UtilityInit();
      m_watchdog.AddEpoch("UtilityInit()");
    }

    m_lastMode = static_cast<int>(mode);
  }

  // Call the appropriate function depending upon the current robot mode
  HAL_ObserveUserProgram(word.GetValue());
  if (mode == RobotMode::UNKNOWN) {
    DisabledPeriodic();
    m_watchdog.AddEpoch("DisabledPeriodic()");
  } else if (mode == RobotMode::AUTONOMOUS) {
    AutonomousPeriodic();
    m_watchdog.AddEpoch("AutonomousPeriodic()");
  } else if (mode == RobotMode::TELEOPERATED) {
    TeleopPeriodic();
    m_watchdog.AddEpoch("TeleopPeriodic()");
  } else if (mode == RobotMode::UTILITY) {
    UtilityPeriodic();
    m_watchdog.AddEpoch("UtilityPeriodic()");
  }

  RobotPeriodic();
  m_watchdog.AddEpoch("RobotPeriodic()");

  prosdk::tunables::TunableRegistry::Update();
  m_watchdog.AddEpoch("TunableRegistry::Update()");

  if constexpr (IsSimulation()) {
    HAL_SimPeriodicBefore();
    SimulationPeriodic();
    HAL_SimPeriodicAfter();
    m_watchdog.AddEpoch("SimulationPeriodic()");
  }

  m_watchdog.Disable();

  // prosdk: robot host telemetry flush (upstream: flush NetworkTables)
  prosdk::platform::FlushTelemetry();

  // Warn on loop time overruns
  if (m_watchdog.IsExpired()) {
    m_watchdog.PrintEpochs();
  }
}

void IterativeRobotBase::PrintLoopOverrunMessage() {
  WPILIB_ReportError(err::Error, "Loop time of {:.6f}s overrun",
                     m_period.value());
}

void IterativeRobotBase::PrintWatchdogEpochs() {
  m_watchdog.PrintEpochs();
}
