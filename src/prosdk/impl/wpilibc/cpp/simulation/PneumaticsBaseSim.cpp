// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/PneumaticsBaseSim.hpp"

#include <memory>

#include "prosdk/hardware/pneumatic/PneumaticsModuleType.hpp"
#include "prosdk/simulation/CTREPCMSim.hpp"
#include "prosdk/simulation/REVPHSim.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;
using namespace prosdk::sim;

std::shared_ptr<PneumaticsBaseSim> PneumaticsBaseSim::GetForType(
    int module, PneumaticsModuleType type) {
  switch (type) {
    case PneumaticsModuleType::REV_PH:
      return std::make_shared<REVPHSim>(module);

    case PneumaticsModuleType::CTRE_PCM:
      return std::make_shared<CTREPCMSim>(module);

    default:
      throw WPILIB_MakeError(err::InvalidParameter, "{}",
                             static_cast<int>(module));
  }
}

PneumaticsBaseSim::PneumaticsBaseSim(int module) : m_index{module} {}

PneumaticsBaseSim::PneumaticsBaseSim(const PneumaticsBase& module)
    : m_index{module.GetModuleNumber()} {}
