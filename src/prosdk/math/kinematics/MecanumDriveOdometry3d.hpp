// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelAccelerations.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelVelocities.hpp"
#include "prosdk/math/kinematics/Odometry3d.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * Class for mecanum drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * mecanum wheel encoders.
 *
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
class WPILIB_DLLEXPORT MecanumDriveOdometry3d
    : public Odometry3d<MecanumDriveKinematics, MecanumDriveWheelPositions,
                        MecanumDriveWheelVelocities,
                        MecanumDriveWheelAccelerations> {
 public:
  /**
   * Constructs a MecanumDriveOdometry3d object.
   *
   * @param kinematics The mecanum drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope. This does not need to
   * be offset to match the robot's orientation on the field.
   * @param wheelPositions The current distances measured by each wheel.
   * @param initialPose The starting position of the robot on the field.
   */
  explicit MecanumDriveOdometry3d(
      MecanumDriveKinematics kinematics, const Rotation3d& gyroAngle,
      const MecanumDriveWheelPositions& wheelPositions,
      const Pose3d& initialPose = Pose3d{});
};

}  // namespace prosdk::math
