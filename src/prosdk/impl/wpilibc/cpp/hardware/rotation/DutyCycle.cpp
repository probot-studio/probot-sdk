// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/rotation/DutyCycle.hpp"

#include <string>

#include "prosdk/hal/DutyCycle.h"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/SensorUtil.hpp"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

DutyCycle::DutyCycle(int channel) : m_channel{channel} {
  InitDutyCycle();
}

void DutyCycle::InitDutyCycle() {
  int32_t status = 0;
  std::string stackTrace = prosdk::util::GetStackTrace(1);
  m_handle = HAL_InitializeDutyCycle(m_channel, stackTrace.c_str(), &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", GetSourceChannel());
  prosdk::util::ReportUsage("IO", m_channel, "DutyCycle");
}

prosdk::units::hertz_t DutyCycle::GetFrequency() const {
  int32_t status = 0;
  auto retVal = HAL_GetDutyCycleFrequency(m_handle, &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", GetSourceChannel());
  return prosdk::units::hertz_t{retVal};
}

double DutyCycle::GetOutput() const {
  int32_t status = 0;
  auto retVal = HAL_GetDutyCycleOutput(m_handle, &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", GetSourceChannel());
  return retVal;
}

prosdk::units::second_t DutyCycle::GetHighTime() const {
  int32_t status = 0;
  auto retVal = HAL_GetDutyCycleHighTime(m_handle, &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", GetSourceChannel());
  return prosdk::units::nanosecond_t{static_cast<double>(retVal)};
}

int DutyCycle::GetSourceChannel() const {
  return m_channel;
}

void DutyCycle::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("Frequency", GetFrequency());
  table.Log("Output", GetOutput());
}

std::string_view DutyCycle::GetTelemetryType() const {
  return "Duty Cycle";
}
