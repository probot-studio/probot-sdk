// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.hpp"

#include <prosdk/commands2/Commands.hpp>

Drive::Drive() {
  leftMotor.AddFollower(prosdk::PWMSparkMax{constants::drive::LEFT_MOTOR2_PORT});
  rightMotor.AddFollower(prosdk::PWMSparkMax{constants::drive::RIGHT_MOTOR2_PORT});

  rightMotor.SetInverted(true);

  leftEncoder.SetDistancePerPulse(
      constants::drive::ENCODER_DISTANCE_PER_PULSE.value());
  rightEncoder.SetDistancePerPulse(
      constants::drive::ENCODER_DISTANCE_PER_PULSE.value());

  drive.SetSafetyEnabled(false);
}

prosdk::cmd::CommandPtr Drive::ArcadeDriveCommand(std::function<double()> fwd,
                                               std::function<double()> rot) {
  return prosdk::cmd::Run([this, fwd, rot] { drive.ArcadeDrive(fwd(), rot()); },
                       {this})
      .WithName("Arcade Drive");
}

prosdk::cmd::CommandPtr Drive::SysIdQuasistatic(
    prosdk::cmd::sysid::Direction direction) {
  return sysIdRoutine.Quasistatic(direction);
}

prosdk::cmd::CommandPtr Drive::SysIdDynamic(prosdk::cmd::sysid::Direction direction) {
  return sysIdRoutine.Dynamic(direction);
}
