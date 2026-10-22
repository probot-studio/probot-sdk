// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/geometry/Twist2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelAccelerations.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelVelocities.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/velocity.hpp"

using namespace prosdk::math;

MecanumDriveWheelVelocities MecanumDriveKinematics::ToWheelVelocities(
    const ChassisVelocities& chassisVelocities,
    const Translation2d& centerOfRotation) const {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation != m_previousCoR) {
    auto fl = m_frontLeftWheel - centerOfRotation;
    auto fr = m_frontRightWheel - centerOfRotation;
    auto rl = m_rearLeftWheel - centerOfRotation;
    auto rr = m_rearRightWheel - centerOfRotation;

    SetInverseKinematics(fl, fr, rl, rr);

    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisVelocitiesVector{chassisVelocities.vx.value(),
                                          chassisVelocities.vy.value(),
                                          chassisVelocities.omega.value()};

  Eigen::Vector4d wheelsVector = m_inverseKinematics * chassisVelocitiesVector;

  MecanumDriveWheelVelocities wheelVelocities;
  wheelVelocities.frontLeft = prosdk::units::meters_per_second_t{wheelsVector(0)};
  wheelVelocities.frontRight = prosdk::units::meters_per_second_t{wheelsVector(1)};
  wheelVelocities.rearLeft = prosdk::units::meters_per_second_t{wheelsVector(2)};
  wheelVelocities.rearRight = prosdk::units::meters_per_second_t{wheelsVector(3)};
  return wheelVelocities;
}

ChassisVelocities MecanumDriveKinematics::ToChassisVelocities(
    const MecanumDriveWheelVelocities& wheelVelocities) const {
  Eigen::Vector4d wheelVelocitiesVector{
      wheelVelocities.frontLeft.value(), wheelVelocities.frontRight.value(),
      wheelVelocities.rearLeft.value(), wheelVelocities.rearRight.value()};

  Eigen::Vector3d chassisVelocitiesVector =
      m_forwardKinematics.solve(wheelVelocitiesVector);

  return {
      prosdk::units::meters_per_second_t{chassisVelocitiesVector(0)},  // NOLINT
      prosdk::units::meters_per_second_t{chassisVelocitiesVector(1)},
      prosdk::units::radians_per_second_t{chassisVelocitiesVector(2)}};
}

Twist2d MecanumDriveKinematics::ToTwist2d(
    const MecanumDriveWheelPositions& start,
    const MecanumDriveWheelPositions& end) const {
  Eigen::Vector4d wheelDeltasVector{
      end.frontLeft.value() - start.frontLeft.value(),
      end.frontRight.value() - start.frontRight.value(),
      end.rearLeft.value() - start.rearLeft.value(),
      end.rearRight.value() - start.rearRight.value()};

  Eigen::Vector3d twistVector = m_forwardKinematics.solve(wheelDeltasVector);

  return {prosdk::units::meter_t{twistVector(0)},
          prosdk::units::meter_t{twistVector(1)},
          prosdk::units::radian_t{twistVector(2)}};
}

Twist2d MecanumDriveKinematics::ToTwist2d(
    const MecanumDriveWheelPositions& wheelDeltas) const {
  Eigen::Vector4d wheelDeltasVector{
      wheelDeltas.frontLeft.value(), wheelDeltas.frontRight.value(),
      wheelDeltas.rearLeft.value(), wheelDeltas.rearRight.value()};

  Eigen::Vector3d twistVector = m_forwardKinematics.solve(wheelDeltasVector);

  return {prosdk::units::meter_t{twistVector(0)},
          prosdk::units::meter_t{twistVector(1)},
          prosdk::units::radian_t{twistVector(2)}};
}

void MecanumDriveKinematics::SetInverseKinematics(Translation2d fl,
                                                  Translation2d fr,
                                                  Translation2d rl,
                                                  Translation2d rr) const {
  m_inverseKinematics = Matrixd<4, 3>{{1, -1, (-(fl.X() + fl.Y())).value()},
                                      {1, 1, (fr.X() - fr.Y()).value()},
                                      {1, 1, (rl.X() - rl.Y()).value()},
                                      {1, -1, (-(rr.X() + rr.Y())).value()}};
}

ChassisAccelerations MecanumDriveKinematics::ToChassisAccelerations(
    const MecanumDriveWheelAccelerations& wheelAccelerations) const {
  Eigen::Vector4d wheelAccelerationsVector{
      wheelAccelerations.frontLeft.value(),
      wheelAccelerations.frontRight.value(),
      wheelAccelerations.rearLeft.value(),
      wheelAccelerations.rearRight.value()};

  Eigen::Vector3d chassisAccelerationsVector =
      m_forwardKinematics.solve(wheelAccelerationsVector);

  return {
      prosdk::units::meters_per_second_squared_t{chassisAccelerationsVector(0)},
      prosdk::units::meters_per_second_squared_t{chassisAccelerationsVector(1)},
      prosdk::units::radians_per_second_squared_t{chassisAccelerationsVector(2)}};
}

MecanumDriveWheelAccelerations MecanumDriveKinematics::ToWheelAccelerations(
    const ChassisAccelerations& chassisAccelerations,
    const Translation2d& centerOfRotation) const {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation != m_previousCoR) {
    auto fl = m_frontLeftWheel - centerOfRotation;
    auto fr = m_frontRightWheel - centerOfRotation;
    auto rl = m_rearLeftWheel - centerOfRotation;
    auto rr = m_rearRightWheel - centerOfRotation;

    SetInverseKinematics(fl, fr, rl, rr);

    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisAccelerationsVector{
      chassisAccelerations.ax.value(), chassisAccelerations.ay.value(),
      chassisAccelerations.alpha.value()};

  Eigen::Vector4d wheelsVector =
      m_inverseKinematics * chassisAccelerationsVector;

  MecanumDriveWheelAccelerations wheelAccelerations;
  wheelAccelerations.frontLeft =
      prosdk::units::meters_per_second_squared_t{wheelsVector(0)};
  wheelAccelerations.frontRight =
      prosdk::units::meters_per_second_squared_t{wheelsVector(1)};
  wheelAccelerations.rearLeft =
      prosdk::units::meters_per_second_squared_t{wheelsVector(2)};
  wheelAccelerations.rearRight =
      prosdk::units::meters_per_second_squared_t{wheelsVector(3)};
  return wheelAccelerations;
}
