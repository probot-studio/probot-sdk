// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/SharpIRSim.hpp"

#include "prosdk/hal/SimDevice.h"
#include "prosdk/simulation/SimDeviceSim.hpp"
#include "prosdk/units/length.hpp"

using namespace prosdk;

SharpIRSim::SharpIRSim(const SharpIR& sharpIR)
    : SharpIRSim(sharpIR.GetChannel()) {}

SharpIRSim::SharpIRSim(int channel) {
  prosdk::sim::SimDeviceSim deviceSim{"SharpIR", channel};
  m_simRange = deviceSim.GetDouble("Range (m)");
}

void SharpIRSim::SetRange(prosdk::units::meter_t range) {
  m_simRange.Set(range.value());
}
