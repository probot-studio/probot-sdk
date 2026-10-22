// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/MecanumDriveOdometry.hpp"

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/math/kinematics/Odometry.hpp"
#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::math;

MecanumDriveOdometry::MecanumDriveOdometry(
    MecanumDriveKinematics kinematics, const Rotation2d& gyroAngle,
    const MecanumDriveWheelPositions& wheelPositions, const Pose2d& initialPose)
    : Odometry(kinematics, gyroAngle, wheelPositions, initialPose) {
  prosdk::util::ReportUsage("MecanumDriveOdometry", "");
}
