// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include "Constants.hpp"
#include <prosdk/commands2/SubsystemBase.hpp>
#include <prosdk/commands2/sysid/SysIdRoutine.hpp>
#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/system/RobotController.hpp>

class Drive : public prosdk::cmd::SubsystemBase {
 public:
  Drive();

  prosdk::cmd::CommandPtr ArcadeDriveCommand(std::function<double()> fwd,
                                          std::function<double()> rot);
  prosdk::cmd::CommandPtr SysIdQuasistatic(prosdk::cmd::sysid::Direction direction);
  prosdk::cmd::CommandPtr SysIdDynamic(prosdk::cmd::sysid::Direction direction);

 private:
  prosdk::PWMSparkMax leftMotor{constants::drive::LEFT_MOTOR1_PORT};
  prosdk::PWMSparkMax rightMotor{constants::drive::RIGHT_MOTOR1_PORT};
  prosdk::DifferentialDrive drive{
      [this](auto val) { leftMotor.SetThrottle(val); },
      [this](auto val) { rightMotor.SetThrottle(val); }};

  prosdk::Encoder leftEncoder{constants::drive::LEFT_ENCODER_PORTS[0],
                           constants::drive::LEFT_ENCODER_PORTS[1],
                           constants::drive::LEFT_ENCODER_REVERSED};

  prosdk::Encoder rightEncoder{constants::drive::RIGHT_ENCODER_PORTS[0],
                            constants::drive::RIGHT_ENCODER_PORTS[1],
                            constants::drive::RIGHT_ENCODER_REVERSED};

  prosdk::cmd::sysid::SysIdRoutine sysIdRoutine{
      prosdk::cmd::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                              nullptr},
      prosdk::cmd::sysid::Mechanism{
          [this](prosdk::units::volt_t driveVoltage) {
            leftMotor.SetVoltage(driveVoltage);
            rightMotor.SetVoltage(driveVoltage);
          },
          [this](prosdk::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-left")
                .voltage(leftMotor.GetThrottle() *
                         prosdk::RobotController::GetBatteryVoltage())
                .position(prosdk::units::meter_t{leftEncoder.GetDistance()})
                .velocity(
                    prosdk::units::meters_per_second_t{leftEncoder.GetRate()});
            log->Motor("drive-right")
                .voltage(rightMotor.GetThrottle() *
                         prosdk::RobotController::GetBatteryVoltage())
                .position(prosdk::units::meter_t{rightEncoder.GetDistance()})
                .velocity(
                    prosdk::units::meters_per_second_t{rightEncoder.GetRate()});
          },
          this}};
};
