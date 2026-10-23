// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.hpp"

#include <prosdk/commands2/Commands.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/voltage.hpp>

Shooter::Shooter() : SubsystemBase{"Shooter"} {
  shooterEncoder.SetDistancePerPulse(
      constants::shooter::ENCODER_DISTANCE_PER_PULSE.value());
}

prosdk::cmd::CommandPtr Shooter::RunShooterCommand(
    std::function<double()> shooterVelocity) {
  return prosdk::cmd::Run(
             [this, shooterVelocity] {
               shooterMotor.SetVoltage(
                   prosdk::units::volt_t{shooterFeedback.Calculate(
                       shooterEncoder.GetRate(), shooterVelocity())} +
                   shooterFeedforward.Calculate(
                       prosdk::units::turns_per_second_t{shooterVelocity()}));
               feederMotor.SetThrottle(constants::shooter::FEEDER_VELOCITY);
             },
             {this})
      .WithName("Set Shooter Velocity");
}

prosdk::cmd::CommandPtr Shooter::SysIdQuasistatic(
    prosdk::cmd::sysid::Direction direction) {
  return sysIdRoutine.Quasistatic(direction);
}

prosdk::cmd::CommandPtr Shooter::SysIdDynamic(
    prosdk::cmd::sysid::Direction direction) {
  return sysIdRoutine.Dynamic(direction);
}
