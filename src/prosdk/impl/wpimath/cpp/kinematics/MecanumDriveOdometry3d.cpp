// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/MecanumDriveOdometry3d.hpp"

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/math/kinematics/Odometry3d.hpp"
#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::math;

MecanumDriveOdometry3d::MecanumDriveOdometry3d(
    MecanumDriveKinematics kinematics, const Rotation3d& gyroAngle,
    const MecanumDriveWheelPositions& wheelPositions, const Pose3d& initialPose)
    : Odometry3d(kinematics, gyroAngle, wheelPositions, initialPose) {
  prosdk::util::ReportUsage("MecanumDriveOdometry3d", "");
}
