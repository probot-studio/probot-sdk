// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// VNH7070AS driver (VNH7070AS.hpp). Facts from ST datasheet DocID028088 Rev 5: truth tables 11
// (operation and CS addressing), 12 (on-state faults) and 13 (off-state diagnostic); standby
// entry and exit in Table 6 and its note 1; fPWM in Table 8; K and VSENSEH in Table 10. Brake
// convention as in prosdk::VNH5019 (INA = INB = low, PWM = brake strength: LSA and LSB on).

#include "prosdk/drivers/motor/VNH7070AS.hpp"

#include <chrono>
#include <cmath>
#include <format>

#include "prosdk/hal/DriverStation.h"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
double Clamp(double value, double lo, double hi) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return value < lo ? lo : (value > hi ? hi : value);
}

// PWM rises 20 us after INA/INB (Table 6 note 1). Wall-clock busy wait: short, and independent
// of the simulated robot time on the host.
void WaitInputToPwm() {
  auto end = std::chrono::steady_clock::now() + std::chrono::microseconds{20};
  while (std::chrono::steady_clock::now() < end) {
  }
}
}  // namespace

VNH7070AS::VNH7070AS(int ina, int inb, int pwm, int sel0, int csAnalog)
    : m_ina{ina}, m_inb{inb}, m_pwm{pwm} {
  if (sel0 >= 0) {
    m_sel0.emplace(sel0);
  }
  if (csAnalog >= 0) {
    m_cs.emplace(csAnalog);
  }
  m_ina.Set(false);
  m_inb.Set(false);
  if (m_sel0) {
    m_sel0->Set(false);
  }
  m_pwm.SetPWMRate(kDefaultPwmRate);
  m_pwm.EnablePWM(0.0);
  m_simDevice = hal::SimDevice{"VNH7070AS", ina};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("IO", ina, "VNH7070AS");
}

void VNH7070AS::SetDirection(bool ina, bool inb) {
  m_ina.Set(ina);
  m_inb.Set(inb);
  if (m_sel0) {
    m_sel0->Set(ina);  // CS reports HSDA (forward) or HSDB (reverse), Table 11
  }
}

void VNH7070AS::Output(double throttle) {
  bool ina = throttle > 0.0;
  bool inb = throttle < 0.0;
  double duty = std::abs(throttle);
  if (throttle == 0.0) {
    duty = m_brake ? m_brakeStrength : 0.0;
  }
  if (m_ina.Get() != ina || m_inb.Get() != inb) {
    m_pwm.UpdateDutyCycle(0.0);  // no drive while the direction pins switch
    SetDirection(ina, inb);
    if (duty > 0.0) {
      WaitInputToPwm();
    }
  }
  m_pwm.UpdateDutyCycle(duty);
  m_bridgeOn = duty > 0.0;
  if (m_simThrottle) {
    m_simThrottle.Set(throttle);
  }
}

void VNH7070AS::SetThrottle(double throttle) {
  m_throttle = Clamp(throttle, -1.0, 1.0);
  Output(m_inverted ? -m_throttle : m_throttle);
  Feed();
}

double VNH7070AS::GetThrottle() const {
  return m_throttle;
}

void VNH7070AS::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool VNH7070AS::GetInverted() const {
  return m_inverted;
}

void VNH7070AS::Disable() {
  m_throttle = 0.0;
  m_pwm.UpdateDutyCycle(0.0);
  SetDirection(false, false);
  m_bridgeOn = false;
  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }
}

void VNH7070AS::SetBrakeMode(bool brake) {
  m_brake = brake;
}

bool VNH7070AS::GetBrakeMode() const {
  return m_brake;
}

void VNH7070AS::SetBrakeStrength(double strength) {
  m_brakeStrength = Clamp(strength, 0.0, 1.0);
}

double VNH7070AS::GetBrakeStrength() const {
  return m_brakeStrength;
}

void VNH7070AS::SetCurrentSense(units::ohm_t senseResistance, double dividerRatio) {
  if (senseResistance.value() > 0.0) {
    m_senseResistance = senseResistance;
  }
  if (dividerRatio > 0.0 && dividerRatio <= 1.0) {
    m_dividerRatio = dividerRatio;
  }
}

double VNH7070AS::GetSenseVoltage() const {
  return m_cs ? m_cs->GetVoltage() / m_dividerRatio : 0.0;
}

units::ampere_t VNH7070AS::GetCurrent() const {
  if (!m_cs) {
    return units::ampere_t{0.0};
  }
  return units::ampere_t{GetSenseVoltage() / m_senseResistance.value() * kSenseRatio};
}

bool VNH7070AS::IsFault() const {
  if (!m_cs || !m_bridgeOn) {
    return false;
  }
  // braking needs PWM high; with the output gate closed the PWM is 0 and the chip is in the
  // off state, whose CS diagnostic is not a fault (Table 13)
  if (!m_ina.Get() && !m_inb.Get() && !HAL_GetOutputsEnabled()) {
    return false;
  }
  return GetSenseVoltage() >= kFaultSenseVoltage;
}

void VNH7070AS::StopMotor() {
  Disable();
}

std::string VNH7070AS::GetDescription() const {
  return std::format("VNH7070AS {}/{}/{}", m_ina.GetChannel(), m_inb.GetChannel(),
                     m_pwm.GetChannel());
}

void VNH7070AS::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
  if (m_cs) {
    table.Log("Current", GetCurrent().value());
  }
}

std::string_view VNH7070AS::GetTelemetryType() const {
  return "Motor Controller";
}
