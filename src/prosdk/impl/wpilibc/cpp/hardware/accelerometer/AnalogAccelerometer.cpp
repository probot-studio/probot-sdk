// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/accelerometer/AnalogAccelerometer.hpp"

#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/NullDeleter.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

AnalogAccelerometer::AnalogAccelerometer(int channel)
    : AnalogAccelerometer(std::make_shared<AnalogInput>(channel)) {}

AnalogAccelerometer::AnalogAccelerometer(AnalogInput* channel)
    : m_analogInput(channel, prosdk::util::NullDeleter<AnalogInput>()) {
  if (!channel) {
    throw WPILIB_MakeError(err::NullParameter, "channel");
  }
  InitAccelerometer();
}

AnalogAccelerometer::AnalogAccelerometer(std::shared_ptr<AnalogInput> channel)
    : m_analogInput(channel) {
  if (!channel) {
    throw WPILIB_MakeError(err::NullParameter, "channel");
  }
  InitAccelerometer();
}

double AnalogAccelerometer::GetAcceleration() const {
  return (m_analogInput->GetVoltage() - m_zeroGVoltage) / m_voltsPerG;
}

void AnalogAccelerometer::SetSensitivity(double sensitivity) {
  m_voltsPerG = sensitivity;
}

void AnalogAccelerometer::SetZero(double zero) {
  m_zeroGVoltage = zero;
}

void AnalogAccelerometer::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("Value", GetAcceleration());
}

std::string_view AnalogAccelerometer::GetTelemetryType() const {
  return "Accelerometer";
}

void AnalogAccelerometer::InitAccelerometer() {
  prosdk::util::ReportUsage("IO", m_analogInput->GetChannel(), "Accelerometer");
}
