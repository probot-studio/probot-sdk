// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/motor/PWMMotorController.hpp"

#include <format>
#include <string>

#include "prosdk/system/RobotController.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"

using namespace prosdk;

void PWMMotorController::SetThrottle(double throttle) {
  if (m_isInverted) {
    throttle = -throttle;
  }
  SetDutyCycleInternal(throttle);

  for (auto& follower : m_nonowningFollowers) {
    follower->SetThrottle(throttle);
  }
  for (auto& follower : m_owningFollowers) {
    follower->SetThrottle(throttle);
  }

  Feed();
}

double PWMMotorController::GetThrottle() const {
  return GetDutyCycleInternal() * (m_isInverted ? -1.0 : 1.0);
}

prosdk::units::volt_t PWMMotorController::GetVoltage() const {
  return GetThrottle() * RobotController::GetBatteryVoltage();
}

void PWMMotorController::SetInverted(bool isInverted) {
  m_isInverted = isInverted;
}

bool PWMMotorController::GetInverted() const {
  return m_isInverted;
}

void PWMMotorController::Disable() {
  m_pwm.SetDisabled();

  if (m_simThrottle) {
    m_simThrottle.Set(0.0);
  }

  for (auto& follower : m_nonowningFollowers) {
    follower->Disable();
  }
  for (auto& follower : m_owningFollowers) {
    follower->Disable();
  }
}

void PWMMotorController::StopMotor() {
  Disable();
}

std::string PWMMotorController::GetDescription() const {
  return std::format("PWM {}", GetChannel());
}

int PWMMotorController::GetChannel() const {
  return m_pwm.GetChannel();
}

void PWMMotorController::EnableDeadbandElimination(bool eliminateDeadband) {
  m_eliminateDeadband = eliminateDeadband;
}

void PWMMotorController::AddFollower(PWMMotorController& follower) {
  m_nonowningFollowers.emplace_back(&follower);
}

PWMMotorController::PWMMotorController(int channel) : m_pwm{channel} {
  m_simDevice = prosdk::hal::SimDevice{"PWMMotorController", channel};
  if (m_simDevice) {
    m_simThrottle = m_simDevice.CreateDouble(
        "Throttle", prosdk::hal::SimDevice::Direction::kOutput, 0.0);
    m_pwm.SetSimDevice(m_simDevice);
  }
}

void PWMMotorController::LogTo(prosdk::telemetry::TelemetryTable& table) const {
  table.Log("Value", GetThrottle());
}

std::string_view PWMMotorController::GetTelemetryType() const {
  return "Motor Controller";
}

prosdk::units::microsecond_t PWMMotorController::GetMinPositivePwm() const {
  if (m_eliminateDeadband) {
    return m_deadbandMaxPwm;
  } else {
    return m_centerPwm + 1_us;
  }
}

prosdk::units::microsecond_t PWMMotorController::GetMaxNegativePwm() const {
  if (m_eliminateDeadband) {
    return m_deadbandMinPwm;
  } else {
    return m_centerPwm - 1_us;
  }
}

prosdk::units::microsecond_t PWMMotorController::GetPositiveScaleFactor() const {
  return m_maxPwm - GetMinPositivePwm();
}

prosdk::units::microsecond_t PWMMotorController::GetNegativeScaleFactor() const {
  return GetMaxNegativePwm() - m_minPwm;
}

void PWMMotorController::SetDutyCycleInternal(double dutyCycle) {
  if (std::isfinite(dutyCycle)) {
    dutyCycle = std::clamp(dutyCycle, -1.0, 1.0);
  } else {
    dutyCycle = 0.0;
  }

  if (m_simThrottle) {
    m_simThrottle.Set(dutyCycle);
  }

  prosdk::units::microsecond_t rawValue;
  if (dutyCycle == 0.0) {
    rawValue = m_centerPwm;
  } else if (dutyCycle > 0.0) {
    rawValue = prosdk::units::microsecond_t{static_cast<double>(std::lround(
                   (dutyCycle * GetPositiveScaleFactor()).value()))} +
               GetMinPositivePwm();
  } else {
    rawValue = prosdk::units::microsecond_t{static_cast<double>(std::lround(
                   (dutyCycle * GetNegativeScaleFactor()).value()))} +
               GetMaxNegativePwm();
  }

  m_pwm.SetPulseTime(rawValue);
}

double PWMMotorController::GetDutyCycleInternal() const {
  prosdk::units::microsecond_t rawValue = m_pwm.GetPulseTime();

  if (rawValue == 0_us) {
    return 0.0;
  } else if (rawValue > m_maxPwm) {
    return 1.0;
  } else if (rawValue < m_minPwm) {
    return -1.0;
  } else if (rawValue > GetMinPositivePwm()) {
    return ((rawValue - GetMinPositivePwm()) / GetPositiveScaleFactor())
        .value();
  } else if (rawValue < GetMaxNegativePwm()) {
    return ((rawValue - GetMaxNegativePwm()) / GetNegativeScaleFactor())
        .value();
  } else {
    return 0.0;
  }
}

void PWMMotorController::SetBounds(prosdk::units::microsecond_t maxPwm,
                                   prosdk::units::microsecond_t deadbandMaxPwm,
                                   prosdk::units::microsecond_t centerPwm,
                                   prosdk::units::microsecond_t deadbandMinPwm,
                                   prosdk::units::microsecond_t minPwm) {
  m_maxPwm = maxPwm;
  m_deadbandMaxPwm = deadbandMaxPwm;
  m_centerPwm = centerPwm;
  m_deadbandMinPwm = deadbandMinPwm;
  m_minPwm = minPwm;
}
