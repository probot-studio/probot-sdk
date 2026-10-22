// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// DRV8317H driver (DRV8317H.hpp). Facts from TI datasheet SLVSGT3 (December 2022): 3x PWM truth
// table (section 8.3.2.2, Table 8-4), MODE pin decode (Table 8-5), up to 200 kHz PWM (section 1),
// nSLEEP and tWAKE 1-3 ms (section 7.5, Table 6-1), H-variant faults on nFAULT with a fixed 5 ms
// automatic retry (sections 8.3.11.1, 8.3.11.3, 8.3.11.4, 8.3.11.7). Not checked on hardware.

#include "prosdk/drivers/motor/DRV8317H.hpp"

#include <cmath>
#include <cstdlib>
#include <format>

#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
constexpr int64_t kStoppedUs = 500000;  // no Hall step for this long: speed 0

double CleanThrottle(double throttle) {
  if (!std::isfinite(throttle)) {
    return 0.0;
  }
  return throttle < -1.0 ? -1.0 : (throttle > 1.0 ? 1.0 : throttle);
}
}  // namespace

DRV8317H::DRV8317H(const Pins& pins) : m_pins{pins} {
  platform::hal::SixStepPins halPins;
  halPins.inh = {pins.inhA, pins.inhB, pins.inhC};
  halPins.inl = {pins.inlA, pins.inlB, pins.inlC};
  halPins.hall = {pins.hallA, pins.hallB, pins.hallC};
  int32_t status = 0;
  auto stack = util::GetStackTrace(1);
  m_handle = platform::hal::OpenSixStep(halPins, static_cast<uint32_t>(kDefaultPwmRate),
                                        stack.c_str(), &status);
  WPILIB_CheckErrorStatus(status, "DRV8317H on INH GPIO {}/{}/{}", pins.inhA, pins.inhB,
                          pins.inhC);
  if (pins.nSleep >= 0) {
    m_nSleep.emplace(pins.nSleep);
    m_nSleep->Set(true);  // awake; the outputs follow the inputs after tWAKE (1-3 ms)
  }
  if (pins.nFault >= 0) {
    m_nFault.emplace(pins.nFault);
  }
  m_simDevice = hal::SimDevice{"DRV8317H", pins.inhA};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("IO", pins.inhA, "DRV8317H");
}

void DRV8317H::Output(double throttle) {
  if (m_handle != platform::hal::kInvalidSixStepHandle) {
    int32_t status = 0;
    platform::hal::SetSixStepOutput(m_handle, throttle, m_brake, &status);
    WPILIB_ReportError(status, "DRV8317H on INH GPIO {}", m_pins.inhA);
  }
  if (m_simThrottle) {
    m_simThrottle.Set(throttle);
  }
}

void DRV8317H::SetThrottle(double throttle) {
  m_throttle = CleanThrottle(throttle);
  Output(m_inverted ? -m_throttle : m_throttle);
  Feed();
}

double DRV8317H::GetThrottle() const {
  return m_throttle;
}

void DRV8317H::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool DRV8317H::GetInverted() const {
  return m_inverted;
}

void DRV8317H::Disable() {
  m_throttle = 0.0;
  bool brake = m_brake;
  m_brake = false;  // coast
  Output(0.0);
  m_brake = brake;
}

void DRV8317H::SetBrakeMode(bool brake) {
  m_brake = brake;
}

bool DRV8317H::GetBrakeMode() const {
  return m_brake;
}

bool DRV8317H::SetCommutationTable(const CommutationTable& table) {
  int32_t status = 0;
  platform::hal::SetSixStepTable(m_handle, table, &status);
  WPILIB_ReportError(status, "DRV8317H on INH GPIO {}: commutation table", m_pins.inhA);
  return status == 0;
}

bool DRV8317H::IsFault() const {
  return m_nFault && !m_nFault->Get();
}

void DRV8317H::SetPolePairs(int polePairs) {
  if (polePairs >= 1) {
    m_polePairs = polePairs;
  }
}

int DRV8317H::GetPolePairs() const {
  return m_polePairs;
}

platform::hal::SixStepFeedback DRV8317H::Feedback() const {
  platform::hal::SixStepFeedback feedback;
  int32_t status = 0;
  if (m_handle != platform::hal::kInvalidSixStepHandle) {
    platform::hal::GetSixStepFeedback(m_handle, &feedback, &status);
  }
  return feedback;
}

int DRV8317H::GetHallState() const {
  return Feedback().hallState;
}

units::turn_t DRV8317H::GetPosition() const {
  double turns = static_cast<double>(Feedback().steps) / (6.0 * m_polePairs);
  return units::turn_t{m_inverted ? -turns : turns};
}

units::revolutions_per_minute_t DRV8317H::GetRpm() const {
  platform::hal::SixStepFeedback f = Feedback();
  int64_t elapsed = f.nowUs - f.lastStepUs;
  if (f.periodUs == 0 || elapsed > kStoppedUs) {
    return units::revolutions_per_minute_t{0.0};
  }
  int64_t period = std::llabs(f.periodUs);
  double seconds = static_cast<double>(elapsed > period ? elapsed : period) * 1e-6;
  double rpm = 60.0 / (6.0 * m_polePairs * seconds);
  if ((f.periodUs < 0) != m_inverted) {
    rpm = -rpm;
  }
  return units::revolutions_per_minute_t{rpm};
}

void DRV8317H::StopMotor() {
  Disable();
}

std::string DRV8317H::GetDescription() const {
  return std::format("DRV8317H {}/{}/{}", m_pins.inhA, m_pins.inhB, m_pins.inhC);
}

void DRV8317H::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
  table.Log("RPM", GetRpm().value());
}

std::string_view DRV8317H::GetTelemetryType() const {
  return "Motor Controller";
}
