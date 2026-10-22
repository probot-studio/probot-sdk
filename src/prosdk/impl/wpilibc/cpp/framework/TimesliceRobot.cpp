// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/framework/TimesliceRobot.hpp"

#include "prosdk/system/Errors.hpp"

using namespace prosdk;

TimesliceRobot::TimesliceRobot(prosdk::units::second_t robotPeriodicAllocation,
                               prosdk::units::second_t controllerPeriod)
    : m_nextOffset{robotPeriodicAllocation},
      m_controllerPeriod{controllerPeriod} {}

void TimesliceRobot::Schedule(std::function<void()> func,
                              prosdk::units::second_t allocation) {
  if (m_nextOffset + allocation > m_controllerPeriod) {
    throw WPILIB_MakeError(err::Error,
                           "Function scheduled at offset {} with allocation {} "
                           "exceeded controller period of {}\n",
                           m_nextOffset, allocation, m_controllerPeriod);
  }

  AddPeriodic(func, m_controllerPeriod, m_nextOffset);
  m_nextOffset += allocation;
}
