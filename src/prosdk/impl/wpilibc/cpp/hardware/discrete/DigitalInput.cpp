// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/discrete/DigitalInput.hpp"

#include <string>

#include "prosdk/hal/DIO.h"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/SensorUtil.hpp"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

DigitalInput::DigitalInput(int channel) {
  m_channel = channel;

  int32_t status = 0;
  std::string stackTrace = prosdk::util::GetStackTrace(1);
  m_handle = HAL_InitializeDIOPort(channel, true, stackTrace.c_str(), &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", channel);

  prosdk::util::ReportUsage("IO", channel, "DigitalInput");
}

bool DigitalInput::Get() const {
  int32_t status = 0;
  bool value = HAL_GetDIO(m_handle, &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", m_channel);
  return value;
}

void DigitalInput::SetSimDevice(HAL_SimDeviceHandle device) {
  HAL_SetDIOSimDevice(m_handle, device);
}

int DigitalInput::GetChannel() const {
  return m_channel;
}

void DigitalInput::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("Value", Get());
}

std::string_view DigitalInput::GetTelemetryType() const {
  return "Digital Input";
}
