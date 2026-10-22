// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/DifferentialDriveOdometry.hpp"

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/kinematics/Odometry.hpp"
#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::math;

DifferentialDriveOdometry::DifferentialDriveOdometry(
    const Rotation2d& gyroAngle, prosdk::units::meter_t leftDistance,
    prosdk::units::meter_t rightDistance, const Pose2d& initialPose)
    : Odometry(DifferentialDriveKinematics{1_m}, gyroAngle,
               {leftDistance, rightDistance}, initialPose) {
  prosdk::util::ReportUsage("DifferentialDriveOdometry", "");
}
