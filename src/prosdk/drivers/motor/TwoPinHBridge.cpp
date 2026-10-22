// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Truth tables: UTC BA6208 datasheet QW-R109-004.G, "Input truth table" (p.3); L9110 datasheet
// shipped in the Boardoza L9110S repo (assets/L9110S Datasheet.pdf), "Logical relationship".

#include "prosdk/drivers/motor/TwoPinHBridge.hpp"

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

TwoPinHBridge::TwoPinHBridge(int inA, int inB, ZeroBehavior zeroBehavior)
    : m_inA{inA}, m_inB{inB}, m_zeroBehavior{zeroBehavior} {
  m_inA.SetPWMRate(kDefaultPwmRate);
  m_inA.EnablePWM(0.0);
  m_inB.EnablePWM(0.0);
  m_simDevice = hal::SimDevice{"TwoPinHBridge", inA};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("IO", inA, "TwoPinHBridge");
}

void TwoPinHBridge::Output(double throttle) {
  // the inactive side first: from H/H (brake) this passes through the target direction only,
  // never through the opposite one
  if (throttle > 0.0) {
    m_inB.UpdateDutyCycle(0.0);
    m_inA.UpdateDutyCycle(throttle);
  } else if (throttle < 0.0) {
    m_inA.UpdateDutyCycle(0.0);
    m_inB.UpdateDutyCycle(-throttle);
  } else {
    double level = m_zeroBehavior == ZeroBehavior::kBrake ? 1.0 : 0.0;
    m_inA.UpdateDutyCycle(level);
    m_inB.UpdateDutyCycle(level);
  }
  if (m_simThrottle) {
    m_simThrottle.Set(throttle);
  }
}

void TwoPinHBridge::SetThrottle(double throttle) {
  m_throttle = CleanThrottle(throttle);
  Output(m_inverted ? -m_throttle : m_throttle);
  Feed();
}

double TwoPinHBridge::GetThrottle() const {
  return m_throttle;
}

void TwoPinHBridge::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool TwoPinHBridge::GetInverted() const {
  return m_inverted;
}

void TwoPinHBridge::Disable() {
  m_throttle = 0.0;
  m_inA.UpdateDutyCycle(0.0);
  m_inB.UpdateDutyCycle(0.0);
  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }
}

void TwoPinHBridge::SetZeroBehavior(ZeroBehavior zeroBehavior) {
  m_zeroBehavior = zeroBehavior;
}

TwoPinHBridge::ZeroBehavior TwoPinHBridge::GetZeroBehavior() const {
  return m_zeroBehavior;
}

void TwoPinHBridge::StopMotor() {
  Disable();
}

std::string TwoPinHBridge::GetDescription() const {
  return std::format("TwoPinHBridge {}/{}", m_inA.GetChannel(), m_inB.GetChannel());
}

void TwoPinHBridge::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
}

std::string_view TwoPinHBridge::GetTelemetryType() const {
  return "Motor Controller";
}
