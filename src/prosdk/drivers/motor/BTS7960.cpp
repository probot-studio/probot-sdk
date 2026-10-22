// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/drivers/motor/BTS7960.hpp"

#include <cmath>
#include <format>

#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
double CleanThrottle(double throttle) {
  if (!std::isfinite(throttle)) {
    return 0.0;
  }
  return throttle < -1.0 ? -1.0 : (throttle > 1.0 ? 1.0 : throttle);
}
}  // namespace

BTS7960::BTS7960(int rpwm, int lpwm, int enable, Wiring wiring)
    : m_wiring{wiring}, m_rpwm{rpwm}, m_lpwm{lpwm}, m_enable{enable} {
  m_enable.SetPWMRate(kDefaultPwmRate);
  if (m_wiring == Wiring::PWM_ON_INPUTS) {
    m_enable.Set(false);
    m_rpwm.EnablePWM(0.0);
    m_lpwm.EnablePWM(0.0);
  } else {
    m_rpwm.Set(false);
    m_lpwm.Set(false);
    m_enable.EnablePWM(0.0);
  }
  m_simDevice = hal::SimDevice{"BTS7960", rpwm};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("IO", rpwm, "BTS7960");
}

void BTS7960::Output(double throttle) {
  double magnitude = std::abs(throttle);
  if (m_wiring == Wiring::PWM_ON_INPUTS) {
    // the inactive side first, so both PWMs are never high together
    if (throttle >= 0.0) {
      m_lpwm.UpdateDutyCycle(0.0);
      m_rpwm.UpdateDutyCycle(magnitude);
    } else {
      m_rpwm.UpdateDutyCycle(0.0);
      m_lpwm.UpdateDutyCycle(magnitude);
    }
    m_enable.Set(true);
  } else {
    if (throttle == 0.0) {
      m_enable.UpdateDutyCycle(0.0);
      m_rpwm.Set(false);
      m_lpwm.Set(false);
      return;
    }
    if ((throttle > 0.0) != m_rpwm.Get()) {
      m_enable.UpdateDutyCycle(0.0);  // no drive while the direction pins switch
    }
    m_rpwm.Set(throttle > 0.0);
    m_lpwm.Set(throttle < 0.0);
    m_enable.UpdateDutyCycle(magnitude);
  }
}

void BTS7960::SetThrottle(double throttle) {
  m_throttle = CleanThrottle(throttle);
  Output(m_inverted ? -m_throttle : m_throttle);
  if (m_simThrottle) {
    m_simThrottle.Set(m_inverted ? -m_throttle : m_throttle);
  }
  Feed();
}

double BTS7960::GetThrottle() const {
  return m_throttle;
}

void BTS7960::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool BTS7960::GetInverted() const {
  return m_inverted;
}

void BTS7960::Disable() {
  m_throttle = 0.0;
  if (m_wiring == Wiring::PWM_ON_INPUTS) {
    m_enable.Set(false);
    m_rpwm.UpdateDutyCycle(0.0);
    m_lpwm.UpdateDutyCycle(0.0);
  } else {
    m_enable.UpdateDutyCycle(0.0);
    m_rpwm.Set(false);
    m_lpwm.Set(false);
  }
  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }
}

void BTS7960::StopMotor() {
  Disable();
}

std::string BTS7960::GetDescription() const {
  return std::format("BTS7960 {}/{}", m_rpwm.GetChannel(), m_lpwm.GetChannel());
}

BTS7960::Wiring BTS7960::GetWiring() const {
  return m_wiring;
}

void BTS7960::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
}

std::string_view BTS7960::GetTelemetryType() const {
  return "Motor Controller";
}
