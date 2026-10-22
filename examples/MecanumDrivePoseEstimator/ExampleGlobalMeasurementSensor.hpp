// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <prosdk/math/geometry/Pose2d.hpp>
#include <prosdk/math/random/Normal.hpp>

/**
 * This dummy class represents a global measurement sensor, such as a computer
 * vision solution.
 */
class ExampleGlobalMeasurementSensor {
 public:
  static prosdk::math::Pose2d GetEstimatedGlobalPose(
      const prosdk::math::Pose2d& estimatedRobotPose) {
    auto randVec = prosdk::math::Normal(0.1, 0.1, 0.1);
    return prosdk::math::Pose2d{
        estimatedRobotPose.X() + prosdk::units::meter_t{randVec(0)},
        estimatedRobotPose.Y() + prosdk::units::meter_t{randVec(1)},
        estimatedRobotPose.Rotation() +
            prosdk::math::Rotation2d{prosdk::units::radian_t{randVec(2)}}};
  }
};
