// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// Behavior of the WPILib 2026 frc::Servo (WPILib BSD license), re-ported on the 2027 PWM.

#include "prosdk/drivers/servo/Servo.hpp"

#include <cmath>

#include "prosdk/drivers/pwm/PCA9685.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

Servo::Servo(int channel) : m_pwm{std::in_place, channel}, m_channel{channel} {
  m_pwm->SetOutputPeriod(20_ms);
  m_pwm->SetDisabled();
  m_simDevice = hal::SimDevice{"Servo", channel};
  if (m_simDevice) {
    m_simPosition = m_simDevice.CreateDouble("Position", hal::SimDevice::Direction::kOutput, 0.0);
    m_pwm->SetSimDevice(m_simDevice);
  }
  util::ReportUsage("IO", channel, "Servo");
}

Servo::Servo(PCA9685& pca, int channel) : m_pca{&pca}, m_channel{channel} {
  if (channel < 0 || channel >= PCA9685::kNumChannels) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "PCA9685 servo channel {} (valid: 0 to {})",
                           channel, PCA9685::kNumChannels - 1);
  }
  if (pca.GetPeriod() <= m_maxPwm) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "PCA9685 0x{:02X} runs at {:.0f} Hz: its period ({:.0f} us) is shorter "
                           "than the servo pulse ({:.0f} us); use at most 400 Hz",
                           pca.GetAddress(), pca.GetFrequency().value(), pca.GetPeriod().value(),
                           m_maxPwm.value());
  }
  pca.SetOff(channel);
  m_simDevice = hal::SimDevice{"Servo", 1000 * pca.GetAddress() + channel};
  if (m_simDevice) {
    m_simPosition = m_simDevice.CreateDouble("Position", hal::SimDevice::Direction::kOutput, 0.0);
  }
  util::ReportUsage("PCA9685 Servo", 1000 * pca.GetAddress() + channel, "");
}

Servo::~Servo() {
  if (m_pca.pca != nullptr) {
    m_pca.pca->SetOff(m_channel);
  }
}

void Servo::Apply() {
  if (m_simPosition) {
    m_simPosition.Set(m_offline ? 0.0 : m_position);
  }
  units::microsecond_t pulse =
      m_minPwm + units::microsecond_t{std::round((m_position * (m_maxPwm - m_minPwm)).value())};
  if (m_pwm) {
    if (m_offline) {
      m_pwm->SetDisabled();
    } else {
      m_pwm->SetPulseTime(pulse);
    }
  } else if (m_pca.pca != nullptr) {
    if (m_offline) {
      m_pca.pca->SetOff(m_channel);
    } else {
      m_pca.pca->SetPulseWidth(m_channel, pulse);
    }
  }
}

void Servo::Set(double value) {
  if (std::isnan(value)) {
    SetOffline();
    return;
  }
  m_position = value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
  m_offline = false;
  Apply();
}

void Servo::SetOffline() {
  m_offline = true;
  Apply();
}

double Servo::Get() const {
  return m_offline ? 0.0 : m_position;
}

void Servo::SetAngle(double angle) {
  if (std::isnan(angle)) {
    SetOffline();
    return;
  }
  if (angle < kMinServoAngle) {
    angle = kMinServoAngle;
  } else if (angle > kMaxServoAngle) {
    angle = kMaxServoAngle;
  }
  Set((angle - kMinServoAngle) / (kMaxServoAngle - kMinServoAngle));
}

double Servo::GetAngle() const {
  return Get() * (kMaxServoAngle - kMinServoAngle) + kMinServoAngle;
}

double Servo::GetMaxAngle() const {
  return kMaxServoAngle;
}

double Servo::GetMinAngle() const {
  return kMinServoAngle;
}

int Servo::GetChannel() const {
  return m_channel;
}

// A servo has no deadband: deadbandMax, center and deadbandMin are accepted for 2026 API
// compatibility and ignored.
void Servo::SetBounds(units::microsecond_t max, units::microsecond_t /*deadbandMax*/,
                      units::microsecond_t /*center*/, units::microsecond_t /*deadbandMin*/,
                      units::microsecond_t min) {
  if (!(min > 0_us && max > min && max < 4096_us)) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "Servo bounds min {:.0f} us, max {:.0f} us (need 0 < min < max < 4096)",
                           min.value(), max.value());
  }
  if (m_pca.pca != nullptr && m_pca.pca->GetPeriod() <= max) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "Servo max pulse {:.0f} us does not fit the PCA9685 period ({:.0f} us)",
                           max.value(), m_pca.pca->GetPeriod().value());
  }
  m_maxPwm = max;
  m_minPwm = min;
  if (!m_offline) {
    Apply();
  }
}

units::microsecond_t Servo::GetMaxPulse() const {
  return m_maxPwm;
}

units::microsecond_t Servo::GetMinPulse() const {
  return m_minPwm;
}

void Servo::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", Get());
}

std::string_view Servo::GetTelemetryType() const {
  return "Servo";
}
