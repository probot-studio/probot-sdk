// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "BalanceAuto.hpp"

#include <string>

#include <prosdk/driverstation/DriverStationDisplay.hpp>

BalanceAuto::BalanceAuto() {
  prosdk::DriverStationDisplay::SetMode(prosdk::DriverStationDisplay::Mode::Line);
  m_chooser.AddOptions("Approach", {"Left", "Center", "Right"});
  m_chooser.AddIntegerOptions("Attempts", 1, 3, 1);
  m_chooser.AddDoubleOptions("Turn Scale", 0.2, 1.0, 0.2);
}

void BalanceAuto::DisabledPeriodic() {
  m_chooser.Update();
  prosdk::DriverStationDisplay::UpdateLines();
}

void BalanceAuto::Start() {
  m_approach = m_chooser.GetSelected("Approach");
  m_attempts = m_chooser.GetSelectedInteger("Attempts");
  m_turnScale = m_chooser.GetSelectedDouble("Turn Scale");
}

void BalanceAuto::Periodic() {
  prosdk::DriverStationDisplay::AddData("Selected Auto", "Balance");
  prosdk::DriverStationDisplay::AddData("Approach", m_approach);
  prosdk::DriverStationDisplay::AddData("Attempts", std::to_string(m_attempts));
  prosdk::DriverStationDisplay::AddData("Turn Scale", std::to_string(m_turnScale));
  prosdk::DriverStationDisplay::UpdateLines();
}
