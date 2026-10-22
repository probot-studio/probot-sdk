// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <prosdk/opmode/PeriodicOpMode.hpp>
#include <prosdk/system/Timer.hpp>

class DefaultTeleop : public prosdk::PeriodicOpMode {
 public:
  void Start() override;
  void Periodic() override;

 private:
  void UpdateSecondsDisplay();

  prosdk::Timer m_timer;
  int m_seconds = 0;
};
