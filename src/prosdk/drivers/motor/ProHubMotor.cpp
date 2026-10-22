// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/drivers/motor/ProHubMotor.hpp"

#include <cmath>
#include <format>

#include "prosdk/drivers/ProHub.hpp"
#include "prosdk/drivers/pwm/PCA9685.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
int CheckPort(int port) {
  if (port < 1 || port > prohub::kNumMotorPorts) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "ProHub motor port {} (valid: 1 to {})",
                           port, prohub::kNumMotorPorts);
  }
  return port;
}
}  // namespace

ProHubMotor::ProHubMotor(PCA9685& pca, int port)
    : m_pca{pca},
      m_port{CheckPort(port)},
      m_forward{prohub::GetMotorChannels(port).forward},
      m_reverse{prohub::GetMotorChannels(port).reverse} {
  m_pca.SetOff(m_forward);
  m_pca.SetOff(m_reverse);
  m_simDevice = hal::SimDevice{"ProHubMotor", port};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("ProHubMotor", port, "");
}

ProHubMotor::~ProHubMotor() {
  m_pca.SetOff(m_forward);
  m_pca.SetOff(m_reverse);
}

void ProHubMotor::SetThrottle(double throttle) {
  if (!std::isfinite(throttle)) {
    throttle = 0.0;
  }
  m_throttle = throttle < -1.0 ? -1.0 : (throttle > 1.0 ? 1.0 : throttle);
  double out = m_inverted ? -m_throttle : m_throttle;
  if (out >= 0.0) {
    m_pca.SetOff(m_reverse);
    m_pca.SetDutyCycle(m_forward, out);
  } else {
    m_pca.SetOff(m_forward);
    m_pca.SetDutyCycle(m_reverse, -out);
  }
  if (m_simThrottle) {
    m_simThrottle.Set(out);
  }
  Feed();
}

double ProHubMotor::GetThrottle() const {
  return m_throttle;
}

void ProHubMotor::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool ProHubMotor::GetInverted() const {
  return m_inverted;
}

void ProHubMotor::Disable() {
  m_throttle = 0.0;
  m_pca.SetOff(m_forward);
  m_pca.SetOff(m_reverse);
  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }
}

void ProHubMotor::StopMotor() {
  Disable();
}

std::string ProHubMotor::GetDescription() const {
  return std::format("ProHub motor {}", m_port);
}

int ProHubMotor::GetPort() const {
  return m_port;
}

void ProHubMotor::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
}

std::string_view ProHubMotor::GetTelemetryType() const {
  return "Motor Controller";
}
