// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.hpp"

void Drivetrain::Drive(prosdk::units::meters_per_second_t xVelocity,
                       prosdk::units::meters_per_second_t yVelocity,
                       prosdk::units::radians_per_second_t rot, bool fieldRelative,
                       prosdk::units::second_t period) {
  prosdk::math::ChassisVelocities chassisVelocities{xVelocity, yVelocity, rot};
  if (fieldRelative) {
    chassisVelocities = chassisVelocities.ToRobotRelative(imu.GetRotation2d());
  }
  chassisVelocities = chassisVelocities.Discretize(period);

  auto [fl, fr, bl, br] = kinematics.DesaturateWheelVelocities(
      kinematics.ToSwerveModuleVelocities(chassisVelocities), MAX_VELOCITY);
  frontLeft.SetDesiredVelocity(fl);
  frontRight.SetDesiredVelocity(fr);
  backLeft.SetDesiredVelocity(bl);
  backRight.SetDesiredVelocity(br);
}

void Drivetrain::UpdateOdometry() {
  odometry.Update(imu.GetRotation2d(),
                  {frontLeft.GetPosition(), frontRight.GetPosition(),
                   backLeft.GetPosition(), backRight.GetPosition()});
}
