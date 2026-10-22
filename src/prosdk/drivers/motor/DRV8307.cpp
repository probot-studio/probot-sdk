// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// DRV8307 driver (DRV8307.hpp). Facts from TI datasheet SLVSCK2A: pin functions (section 5),
// fPWM 16-50 kHz (section 6.3), HALLOUT = Hall U (section 7.3.2, Figure 4), ENABLEn and standby
// (sections 7.3.3, 8.1.2), BRAKE = 1 brakes (Table 2), stop with ENABLEn instead of 0 % duty
// (section 7.4.1), ENABLEn and BRAKE together for a faster stop (section 8.1.3).

#include "prosdk/drivers/motor/DRV8307.hpp"

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

DRV8307::DRV8307(int pwm, int enableN, int dir, int brake, int lockN, int hallOut, int faultN)
    : m_pwm{pwm}, m_enableN{enableN} {
  m_enableN.Set(true);  // disabled until the first non-zero throttle
  if (dir >= 0) {
    m_dir.emplace(dir);
    m_dir->Set(false);
  }
  if (brake >= 0) {
    m_brakePin.emplace(brake);
    m_brakePin->Set(false);  // coast, as after Disable()
  }
  if (lockN >= 0) {
    m_lockN.emplace(lockN);
  }
  if (hallOut >= 0) {
    m_hallOut.emplace(hallOut, EdgeConfiguration::RISING_EDGE);
    m_hallOut->SetEdgesPerRevolution(m_polePairs);
  }
  if (faultN >= 0) {
    m_faultN.emplace(faultN);
  }
  m_pwm.SetPWMRate(kDefaultPwmRate);
  m_pwm.EnablePWM(0.0);
  m_simDevice = hal::SimDevice{"DRV8307", pwm};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble("Throttle", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("IO", pwm, "DRV8307");
}

void DRV8307::Output(double throttle) {
  if (throttle < 0.0 && !m_dir) {
    throttle = 0.0;  // the direction is set on the board
  }
  if (throttle == 0.0) {
    m_enableN.Set(true);
    if (m_brakePin) {
      m_brakePin->Set(m_brake);
    }
    m_pwm.UpdateDutyCycle(0.0);
  } else {
    if (m_brakePin) {
      m_brakePin->Set(false);
    }
    if (m_dir) {
      m_dir->Set(throttle > 0.0);
    }
    m_pwm.UpdateDutyCycle(std::abs(throttle));
    m_enableN.Set(false);
  }
  if (m_simThrottle) {
    m_simThrottle.Set(throttle);
  }
}

void DRV8307::SetThrottle(double throttle) {
  m_throttle = CleanThrottle(throttle);
  Output(m_inverted ? -m_throttle : m_throttle);
  Feed();
}

double DRV8307::GetThrottle() const {
  return m_throttle;
}

void DRV8307::SetInverted(bool isInverted) {
  m_inverted = isInverted;
}

bool DRV8307::GetInverted() const {
  return m_inverted;
}

void DRV8307::Disable() {
  m_throttle = 0.0;
  m_enableN.Set(true);
  if (m_brakePin) {
    m_brakePin->Set(false);
  }
  m_pwm.UpdateDutyCycle(0.0);
  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }
}

void DRV8307::SetBrakeMode(bool brake) {
  m_brake = brake;
}

bool DRV8307::GetBrakeMode() const {
  return m_brake;
}

bool DRV8307::IsLocked() const {
  return m_lockN && !m_lockN->Get();
}

bool DRV8307::IsFault() const {
  return m_faultN && !m_faultN->Get();
}

void DRV8307::SetPolePairs(int polePairs) {
  if (polePairs < 1) {
    return;
  }
  m_polePairs = polePairs;
  if (m_hallOut) {
    m_hallOut->SetEdgesPerRevolution(polePairs);
  }
}

int DRV8307::GetPolePairs() const {
  return m_polePairs;
}

units::revolutions_per_minute_t DRV8307::GetRpm() const {
  if (!m_hallOut) {
    return units::revolutions_per_minute_t{0.0};
  }
  return m_hallOut->GetRevolutionsPerMinute();
}

void DRV8307::StopMotor() {
  Disable();
}

std::string DRV8307::GetDescription() const {
  return std::format("DRV8307 {}/{}", m_pwm.GetChannel(), m_enableN.GetChannel());
}

void DRV8307::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
  if (m_hallOut) {
    table.Log("RPM", GetRpm().value());
  }
}

std::string_view DRV8307::GetTelemetryType() const {
  return "Motor Controller";
}
