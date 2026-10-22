// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/range/SharpIR.hpp"

#include <algorithm>

#include "prosdk/hardware/discrete/AnalogInput.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

SharpIR SharpIR::GP2Y0A02YK0F(int channel) {
  return SharpIR(channel, 62.28, -1.092, 20_cm, 150_cm);
}

SharpIR SharpIR::GP2Y0A21YK0F(int channel) {
  return SharpIR(channel, 26.449, -1.226, 10_cm, 80_cm);
}

SharpIR SharpIR::GP2Y0A41SK0F(int channel) {
  return SharpIR(channel, 12.354, -1.07, 4_cm, 30_cm);
}

SharpIR SharpIR::GP2Y0A51SK0F(int channel) {
  return SharpIR(channel, 5.2819, -1.161, 2_cm, 15_cm);
}

SharpIR::SharpIR(int channel, double a, double b, prosdk::units::meter_t min,
                 prosdk::units::meter_t max)
    : m_sensor(channel), m_A(a), m_B(b), m_min(min), m_max(max) {
  prosdk::util::ReportUsage("IO", channel, "SharpIR");

  m_simDevice = prosdk::hal::SimDevice("SharpIR", m_sensor.GetChannel());
  if (m_simDevice) {
    m_simRange = m_simDevice.CreateDouble(
        "Range (m)", prosdk::hal::SimDevice::Direction::kInput, 0.0);
    m_sensor.SetSimDevice(m_simDevice);
  }
}

int SharpIR::GetChannel() const {
  return m_sensor.GetChannel();
}

prosdk::units::meter_t SharpIR::GetRange() const {
  if (m_simRange) {
    return std::clamp(prosdk::units::meter_t{m_simRange.Get()}, m_min, m_max);
  } else {
    // Don't allow zero/negative values
    auto v = std::max(m_sensor.GetVoltage(), 0.00001);

    return std::clamp(prosdk::units::meter_t{m_A * std::pow(v, m_B) * 1e-2}, m_min,
                      m_max);
  }
}

void SharpIR::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("Value", GetRange());
}

std::string_view SharpIR::GetTelemetryType() const {
  return "Ultrasonic";
}
