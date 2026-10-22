// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/DutyCycleEncoderSim.hpp"

#include "prosdk/hardware/rotation/DutyCycleEncoder.hpp"
#include "prosdk/simulation/SimDeviceSim.hpp"

using namespace prosdk::sim;

DutyCycleEncoderSim::DutyCycleEncoderSim(const prosdk::DutyCycleEncoder& encoder)
    : DutyCycleEncoderSim{encoder.GetSourceChannel()} {}

DutyCycleEncoderSim::DutyCycleEncoderSim(int channel) {
  prosdk::sim::SimDeviceSim deviceSim{"DutyCycle:DutyCycleEncoder", channel};
  m_simPosition = deviceSim.GetDouble("Position");
  m_simIsConnected = deviceSim.GetBoolean("Connected");
}

double DutyCycleEncoderSim::Get() {
  return m_simPosition.Get();
}

void DutyCycleEncoderSim::Set(double value) {
  m_simPosition.Set(value);
}

bool DutyCycleEncoderSim::IsConnected() {
  return m_simIsConnected.Get();
}

void DutyCycleEncoderSim::SetConnected(bool isConnected) {
  m_simIsConnected.Set(isConnected);
}
