// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Brake convention as in Pololu's VNH5019 libraries: INA = INB = low with PWM = brake strength
// (both low-side switches on while PWM is high).

#include "prosdk/drivers/motor/VNH5019.hpp"

#include <cmath>
#include <format>

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
}  // namespace

VNH5019::VNH5019(int ina, int inb, int pwm) : m_ina{ina}, m_inb{inb}, m_pwm{pwm} {
  Init(ina);
}

VNH5019::VNH5019(int ina, int inb, int pwm, int ena, int enb)
    : m_ina{ina}, m_inb{inb}, m_pwm{pwm} {
  if (ena >= 0) {
    m_ena.emplace(ena);
  }
  if (enb >= 0 && enb != ena) {
    m_enb.emplace(enb);
  }
  Init(ina);
}

void VNH5019::Init(int ina) {
  m_ina.Set(false);
  m_inb.Set(false);
  SetEnables(false);
  m_pwm.SetPWMRate(kDefaultPwmRate);
  m_pwm.EnablePWM(0.0);
  m_simDevice = hal::SimDevice{"VNH5019", ina};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("IO", ina, "VNH5019");
}

void VNH5019::SetEnables(bool on) {
  if (m_ena) {
    m_ena->Set(on);
  }
  if (m_enb) {
    m_enb->Set(on);
  }
}

void VNH5019::Output(double throttle) {
  SetEnables(true);
  if (throttle == 0.0) {
    m_pwm.UpdateDutyCycle(0.0);
    m_ina.Set(false);
    m_inb.Set(false);
    m_pwm.UpdateDutyCycle(m_brake ? m_brakeStrength : 0.0);
  } else {
    bool forward = throttle > 0.0;
    if (m_ina.Get() != forward || m_inb.Get() != !forward) {
      m_pwm.UpdateDutyCycle(0.0);  // no drive while the direction pins switch
      m_ina.Set(forward);
      m_inb.Set(!forward);
    }
    m_pwm.UpdateDutyCycle(std::abs(throttle));
  }
  if (m_simThrottle) {
    m_simThrottle.Set(throttle);
  }
}

void VNH5019::SetThrottle(double throttle) {
  m_throttle = Clamp(throttle, -1.0, 1.0);
  Output(m_inverted ? -m_throttle : m_throttle);
  Feed();
}

double VNH5019::GetThrottle() const {
  return m_throttle;
}

void VNH5019::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool VNH5019::GetInverted() const {
  return m_inverted;
}

void VNH5019::Disable() {
  m_throttle = 0.0;
  m_pwm.UpdateDutyCycle(0.0);
  m_ina.Set(false);
  m_inb.Set(false);
  SetEnables(false);
  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }
}

void VNH5019::SetBrakeMode(bool brake) {
  m_brake = brake;
}

bool VNH5019::GetBrakeMode() const {
  return m_brake;
}

void VNH5019::SetBrakeStrength(double strength) {
  m_brakeStrength = Clamp(strength, 0.0, 1.0);
}

double VNH5019::GetBrakeStrength() const {
  return m_brakeStrength;
}

void VNH5019::StopMotor() {
  Disable();
}

std::string VNH5019::GetDescription() const {
  return std::format("VNH5019 {}/{}/{}", m_ina.GetChannel(), m_inb.GetChannel(),
                     m_pwm.GetChannel());
}

void VNH5019::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
}

std::string_view VNH5019::GetTelemetryType() const {
  return "Motor Controller";
}
