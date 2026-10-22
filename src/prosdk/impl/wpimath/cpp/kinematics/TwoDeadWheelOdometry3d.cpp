// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/TwoDeadWheelOdometry3d.hpp"

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk::math;

TwoDeadWheelOdometry3d::TwoDeadWheelOdometry3d(prosdk::units::meter_t xWheelYPos,
                                               prosdk::units::meter_t yWheelXPos,
                                               prosdk::units::meter_t xWheelPos,
                                               prosdk::units::meter_t yWheelPos,
                                               const Rotation3d& gyroAngle,
                                               const Pose3d& initialPose)
    : m_xWheelYPos(xWheelYPos),
      m_yWheelXPos(yWheelXPos),
      m_pose(initialPose),
      m_previousXWheelPos(xWheelPos),
      m_previousYWheelPos(yWheelPos),
      m_previousGyroAngle(gyroAngle) {
  prosdk::util::ReportUsage("TwoDeadWheelOdometry3d", "");
}
