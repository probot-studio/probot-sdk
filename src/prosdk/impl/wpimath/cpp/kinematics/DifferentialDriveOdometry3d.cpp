// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/DifferentialDriveOdometry3d.hpp"

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/kinematics/Odometry3d.hpp"
#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::math;

DifferentialDriveOdometry3d::DifferentialDriveOdometry3d(
    const Rotation3d& gyroAngle, prosdk::units::meter_t leftDistance,
    prosdk::units::meter_t rightDistance, const Pose3d& initialPose)
    : Odometry3d(DifferentialDriveKinematics{1_m}, gyroAngle,
                 {leftDistance, rightDistance}, initialPose) {
  prosdk::util::ReportUsage("DifferentialDriveOdometry3d", "");
}
