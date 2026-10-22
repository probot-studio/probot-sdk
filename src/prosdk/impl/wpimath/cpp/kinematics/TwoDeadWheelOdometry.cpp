// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/TwoDeadWheelOdometry.hpp"

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::math;

TwoDeadWheelOdometry::TwoDeadWheelOdometry(prosdk::units::meter_t xWheelYPos,
                                           prosdk::units::meter_t yWheelXPos,
                                           prosdk::units::meter_t xWheelPos,
                                           prosdk::units::meter_t yWheelPos,
                                           const Rotation2d& gyroAngle,
                                           const Pose2d& initialPose)
    : m_xWheelYPos(xWheelYPos),
      m_yWheelXPos(yWheelXPos),
      m_pose(initialPose),
      m_previousXWheelPos(xWheelPos),
      m_previousYWheelPos(yWheelPos),
      m_previousGyroAngle(gyroAngle) {
  prosdk::util::ReportUsage("TwoDeadWheelOdometry", "");
}
