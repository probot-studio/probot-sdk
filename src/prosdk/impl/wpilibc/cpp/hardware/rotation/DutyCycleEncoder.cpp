// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/rotation/DutyCycleEncoder.hpp"

#include <memory>
#include <utility>

#include "prosdk/hardware/rotation/DutyCycle.hpp"
#include "prosdk/math/util/MathUtil.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/NullDeleter.hpp"

using namespace prosdk;

DutyCycleEncoder::DutyCycleEncoder(int channel)
    : m_dutyCycle{std::make_shared<DutyCycle>(channel)} {
  Init(1.0, 0.0);
}

DutyCycleEncoder::DutyCycleEncoder(DutyCycle& dutyCycle)
    : m_dutyCycle{&dutyCycle, prosdk::util::NullDeleter<DutyCycle>{}} {
  Init(1.0, 0.0);
}

DutyCycleEncoder::DutyCycleEncoder(DutyCycle* dutyCycle)
    : m_dutyCycle{dutyCycle, prosdk::util::NullDeleter<DutyCycle>{}} {
  Init(1.0, 0.0);
}

DutyCycleEncoder::DutyCycleEncoder(std::shared_ptr<DutyCycle> dutyCycle)
    : m_dutyCycle{std::move(dutyCycle)} {
  Init(1.0, 0.0);
}

DutyCycleEncoder::DutyCycleEncoder(int channel, double fullRange,
                                   double expectedZero)
    : m_dutyCycle{std::make_shared<DutyCycle>(channel)} {
  Init(fullRange, expectedZero);
}

DutyCycleEncoder::DutyCycleEncoder(DutyCycle& dutyCycle, double fullRange,
                                   double expectedZero)
    : m_dutyCycle{&dutyCycle, prosdk::util::NullDeleter<DutyCycle>{}} {
  Init(fullRange, expectedZero);
}

DutyCycleEncoder::DutyCycleEncoder(DutyCycle* dutyCycle, double fullRange,
                                   double expectedZero)
    : m_dutyCycle{dutyCycle, prosdk::util::NullDeleter<DutyCycle>{}} {
  Init(fullRange, expectedZero);
}

DutyCycleEncoder::DutyCycleEncoder(std::shared_ptr<DutyCycle> dutyCycle,
                                   double fullRange, double expectedZero)
    : m_dutyCycle{std::move(dutyCycle)} {
  Init(fullRange, expectedZero);
}

void DutyCycleEncoder::Init(double fullRange, double expectedZero) {
  m_simDevice = prosdk::hal::SimDevice{"DutyCycle:DutyCycleEncoder",
                                    m_dutyCycle->GetSourceChannel()};

  if (m_simDevice) {
    m_simPosition = m_simDevice.CreateDouble(
        "Position", prosdk::hal::SimDevice::Direction::kInput, 0.0);
    m_simIsConnected = m_simDevice.CreateBoolean(
        "Connected", prosdk::hal::SimDevice::Direction::kInput, true);
  }

  m_fullRange = fullRange;
  m_expectedZero = expectedZero;
}

double DutyCycleEncoder::Get() const {
  if (m_simPosition) {
    return m_simPosition.Get();
  }

  double pos;
  // Compute output percentage (0-1)
  if (m_period.value() == 0.0) {
    pos = m_dutyCycle->GetOutput();
  } else {
    auto highTime = m_dutyCycle->GetHighTime();
    pos = highTime / m_period;
  }

  // Map sensor range if range isn't full
  pos = MapSensorRange(pos);

  // Compute full range and offset
  pos = pos * m_fullRange - m_expectedZero;

  // Map from 0 - Full Range
  double result = prosdk::math::InputModulus(pos, 0.0, m_fullRange);
  // Invert if necessary
  if (m_isInverted) {
    return m_fullRange - result;
  }
  return result;
}

double DutyCycleEncoder::MapSensorRange(double pos) const {
  if (pos < m_sensorMin) {
    pos = m_sensorMin;
  }
  if (pos > m_sensorMax) {
    pos = m_sensorMax;
  }
  pos = (pos - m_sensorMin) / (m_sensorMax - m_sensorMin);
  return pos;
}

void DutyCycleEncoder::SetDutyCycleRange(double min, double max) {
  m_sensorMin = std::clamp(min, 0.0, 1.0);
  m_sensorMax = std::clamp(max, 0.0, 1.0);
}

prosdk::units::hertz_t DutyCycleEncoder::GetFrequency() const {
  return m_dutyCycle->GetFrequency();
}

bool DutyCycleEncoder::IsConnected() const {
  if (m_simIsConnected) {
    return m_simIsConnected.Get();
  }
  return GetFrequency() > m_frequencyThreshold;
}

void DutyCycleEncoder::SetConnectedFrequencyThreshold(
    prosdk::units::hertz_t frequency) {
  if (frequency < 0_Hz) {
    frequency = 0_Hz;
  }
  m_frequencyThreshold = frequency;
}

void DutyCycleEncoder::SetInverted(bool inverted) {
  m_isInverted = inverted;
}

void DutyCycleEncoder::SetAssumedFrequency(prosdk::units::hertz_t frequency) {
  if (frequency.value() == 0) {
    m_period = 0_s;
  } else {
    m_period = 1.0 / frequency;
  }
}

int DutyCycleEncoder::GetSourceChannel() const {
  return m_dutyCycle->GetSourceChannel();
}

void DutyCycleEncoder::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("Position", Get());
  table.Log("Is Connected", IsConnected());
}

std::string_view DutyCycleEncoder::GetTelemetryType() const {
  return "AbsoluteEncoder";
}
