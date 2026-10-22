// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/AnalogEncoderSim.hpp"

#include "prosdk/hardware/rotation/AnalogEncoder.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk::sim;

AnalogEncoderSim::AnalogEncoderSim(const prosdk::AnalogEncoder& encoder) {
  prosdk::sim::SimDeviceSim deviceSim{"AnalogEncoder", encoder.GetChannel()};
  m_positionSim = deviceSim.GetDouble("Position");
}

void AnalogEncoderSim::Set(double value) {
  m_positionSim.Set(value);
}

double AnalogEncoderSim::Get() {
  return m_positionSim.Get();
}
