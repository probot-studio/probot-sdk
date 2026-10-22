// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include "Constants.hpp"
#include <prosdk/commands2/SubsystemBase.hpp>
#include <prosdk/commands2/sysid/SysIdRoutine.hpp>
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/system/RobotController.hpp>

class Shooter : public prosdk::cmd::SubsystemBase {
 public:
  Shooter();

  prosdk::cmd::CommandPtr RunShooterCommand(
      std::function<double()> shooterVelocity);
  prosdk::cmd::CommandPtr SysIdQuasistatic(prosdk::cmd::sysid::Direction direction);
  prosdk::cmd::CommandPtr SysIdDynamic(prosdk::cmd::sysid::Direction direction);

 private:
  prosdk::PWMSparkMax shooterMotor{constants::shooter::SHOOTER_MOTOR_PORT};
  prosdk::PWMSparkMax feederMotor{constants::shooter::FEEDER_MOTOR_PORT};

  prosdk::Encoder shooterEncoder{constants::shooter::ENCODER_PORTS[0],
                              constants::shooter::ENCODER_PORTS[1],
                              constants::shooter::ENCODER_REVERSED};

  prosdk::cmd::sysid::SysIdRoutine sysIdRoutine{
      prosdk::cmd::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                              nullptr},
      prosdk::cmd::sysid::Mechanism{
          [this](prosdk::units::volt_t driveVoltage) {
            shooterMotor.SetVoltage(driveVoltage);
          },
          [this](prosdk::sysid::SysIdRoutineLog* log) {
            log->Motor("shooter-wheel")
                .voltage(shooterMotor.GetThrottle() *
                         prosdk::RobotController::GetBatteryVoltage())
                .position(prosdk::units::turn_t{shooterEncoder.GetDistance()})
                .velocity(
                    prosdk::units::turns_per_second_t{shooterEncoder.GetRate()});
          },
          this}};
  prosdk::math::PIDController shooterFeedback{constants::shooter::kP, 0, 0};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::radians> shooterFeedforward{
      constants::shooter::kS, constants::shooter::kV, constants::shooter::kA};
};
