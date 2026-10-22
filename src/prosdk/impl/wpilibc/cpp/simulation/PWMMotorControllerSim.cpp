// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/PWMMotorControllerSim.hpp"

#include "prosdk/hal/SimDevice.h"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk;
using namespace prosdk::sim;

PWMMotorControllerSim::PWMMotorControllerSim(
    const PWMMotorController& motorctrl)
    : PWMMotorControllerSim(motorctrl.GetChannel()) {}

PWMMotorControllerSim::PWMMotorControllerSim(int channel) {
  prosdk::sim::SimDeviceSim deviceSim{"PWMMotorController", channel};
  m_simThrottle = deviceSim.GetDouble("Throttle");
}

double PWMMotorControllerSim::GetThrottle() const {
  return m_simThrottle.Get();
}
