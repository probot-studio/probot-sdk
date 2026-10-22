// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/counter/Tachometer.hpp"

#include <string>

#include "prosdk/hal/Counter.h"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

Tachometer::Tachometer(int channel, EdgeConfiguration configuration)
    : m_channel{channel} {
  int32_t status = 0;
  std::string stackTrace = prosdk::util::GetStackTrace(1);
  m_handle = HAL_InitializeCounter(
      channel, configuration == EdgeConfiguration::RISING_EDGE,
      stackTrace.c_str(), &status);
  WPILIB_CheckErrorStatus(status, "{}", channel);

  prosdk::util::ReportUsage("IO", channel, "Tachometer");
}

void Tachometer::SetEdgeConfiguration(EdgeConfiguration configuration) {
  int32_t status = 0;
  bool rising = configuration == EdgeConfiguration::RISING_EDGE;
  HAL_SetCounterEdgeConfiguration(m_handle, rising, &status);
  WPILIB_CheckErrorStatus(status, "{}", m_channel);
}

void Tachometer::SetRateWindow(prosdk::units::millisecond_t window) {
  int32_t status = 0;
  HAL_SetCounterRateWindow(m_handle, static_cast<int32_t>(window.value()),
                           &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", m_channel);
}

prosdk::units::hertz_t Tachometer::GetFrequency() const {
  int32_t status = 0;
  double rate = HAL_GetCounterRate(m_handle, &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", m_channel);
  return prosdk::units::hertz_t{rate};
}

int Tachometer::GetEdgesPerRevolution() const {
  return m_edgesPerRevolution;
}
void Tachometer::SetEdgesPerRevolution(int edges) {
  m_edgesPerRevolution = edges;
}

prosdk::units::turns_per_second_t Tachometer::GetRevolutionsPerSecond() const {
  int edgesPerRevolution = GetEdgesPerRevolution();
  if (edgesPerRevolution == 0) {
    return 0_tps;
  }
  auto rotationHz = GetFrequency() / edgesPerRevolution;
  return prosdk::units::turns_per_second_t{rotationHz.value()};
}

prosdk::units::revolutions_per_minute_t Tachometer::GetRevolutionsPerMinute()
    const {
  return prosdk::units::revolutions_per_minute_t{GetRevolutionsPerSecond()};
}

bool Tachometer::GetStopped() const {
  int32_t status = 0;
  bool stopped = HAL_GetCounterStopped(m_handle, &status);
  WPILIB_CheckErrorStatus(status, "Channel {}", m_channel);
  return stopped;
}

void Tachometer::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("RPM", GetRevolutionsPerMinute());
}

std::string_view Tachometer::GetTelemetryType() const {
  return "Tachometer";
}
