// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/hal/SimDevice.h"
#include "prosdk/hardware/motor/PWMMotorController.hpp"

namespace prosdk {

class PWMMotorController;

namespace sim {

class PWMMotorControllerSim {
 public:
  explicit PWMMotorControllerSim(const PWMMotorController& motorctrl);

  explicit PWMMotorControllerSim(int channel);

  double GetThrottle() const;

 private:
  prosdk::hal::SimDouble m_simThrottle;
};
}  // namespace sim
}  // namespace wpi
