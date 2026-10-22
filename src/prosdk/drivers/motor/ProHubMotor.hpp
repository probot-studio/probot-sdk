// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ProHub motor port: an H-bridge driven by two PCA9685 channels (design/drivers.md, "ProHub").

#pragma once

#include <string>
#include <string_view>

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/motor/MotorController.hpp"
#include "prosdk/hardware/motor/MotorSafety.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

class PCA9685;

/**
 * Motor on a ProHub motor port. A positive throttle drives the port's forward channel with
 * duty |throttle| (the reverse channel fully off), a negative throttle the reverse channel. A
 * throttle of 0 and Disable() turn both channels fully off.
 *
 * The output reaches the chip with the PCA9685's update period (10 ms by default) and only
 * while the robot is enabled (PCA9685 output gate).
 */
class ProHubMotor : public MotorController,
                    public MotorSafety,
                    public prosdk::telemetry::TelemetryLoggable {
 public:
  /**
   * Constructs a motor on a ProHub port.
   *
   * @param pca  The PCA9685 serving the port (address 0x40 for ports 1-4, 0x41 for 5-8); must
   *             outlive the motor.
   * @param port Motor port, 1 to 8 (prosdk::prohub::GetMotorChannels).
   * @throws prosdk::RuntimeError (ChannelIndexOutOfRange) for an invalid port.
   */
  ProHubMotor(PCA9685& pca, int port);
  ~ProHubMotor() override;

  ProHubMotor(const ProHubMotor&) = delete;
  ProHubMotor& operator=(const ProHubMotor&) = delete;

  /** Sets the throttle, -1 to 1 (clamped; NaN is 0). Feeds the motor safety watchdog. */
  void SetThrottle(double throttle) override;
  /** The last throttle set (0 after Disable). */
  double GetThrottle() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  void Disable() override;

  // MotorSafety
  void StopMotor() override;
  std::string GetDescription() const override;

  /** The motor port, 1 to 8. */
  int GetPort() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  PCA9685& m_pca;
  int m_port;
  int m_forward;
  int m_reverse;
  double m_throttle = 0.0;
  bool m_inverted = false;
  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimDouble m_simThrottle;
};

}  // namespace prosdk
