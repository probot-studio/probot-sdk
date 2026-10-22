// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/geometry/Translation3d.hpp"
#include "prosdk/math/geometry/Twist3d.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * Class for double follower wheel odometry. Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from
 * 2 encoders and a gyroscope.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
class WPILIB_DLLEXPORT TwoDeadWheelOdometry3d {
 public:
  /**
   * Constructs a TwoDeadWheelOdometry3d object.
   *
   * @param xWheelYPos The y-position of the forward-facing wheel relative to
   * the center of the robot in meters.
   * @param yWheelXPos The x-position of the left-facing wheel relative to the
   * center of the robot in meters.
   * @param xWheelPos The distance traveled by the forward-facing wheel, in
   * meters.
   * @param yWheelPos The distance traveled by the left-facing wheel, in meters.
   * @param gyroAngle The angle reported by the gyroscope. This does not need to
   * be offset to match the robot's orientation on the field.
   * @param initialPose The starting position of the robot on the field.
   */
  explicit TwoDeadWheelOdometry3d(prosdk::units::meter_t xWheelYPos,
                                  prosdk::units::meter_t yWheelXPos,
                                  prosdk::units::meter_t xWheelPos,
                                  prosdk::units::meter_t yWheelPos,
                                  const Rotation3d& gyroAngle,
                                  const Pose3d& initialPose = Pose3d{});

  /**
   * Resets the robot's position on the field.
   *
   * @param xWheelPos The distance traveled by the forward-facing wheel, in
   * meters.
   * @param yWheelPos The distance traveled by the left-facing wheel, in meters.
   * @param gyroAngle The angle reported by the gyroscope. This does not need to
   * be offset to match the robot's orientation on the field.
   * @param pose The new position of the robot on the field.
   */
  void ResetPosition(prosdk::units::meter_t xWheelPos,
                     prosdk::units::meter_t yWheelPos, const Rotation3d& gyroAngle,
                     const Pose3d& pose) {
    m_previousGyroAngle = gyroAngle;
    m_previousXWheelPos = xWheelPos;
    m_previousYWheelPos = yWheelPos;
    m_pose = pose;
  }

  /**
   * Resets the pose.
   *
   * @param pose The pose to reset to.
   */
  void ResetPose(const Pose3d& pose) { m_pose = pose; }

  /**
   * Resets the translation of the pose.
   *
   * @param translation The translation to reset to.
   */
  void ResetTranslation(const Translation3d& translation) {
    m_pose = Pose3d{translation, m_pose.Rotation()};
  }

  /**
   * Resets the rotation of the pose.
   *
   * @param rotation The rotation to reset to.
   */
  void ResetRotation(const Rotation3d& rotation) {
    m_pose = Pose3d{m_pose.Translation(), rotation};
  }

  /**
   * Returns the position of the robot on the field.
   * @return The pose of the robot.
   */
  const Pose3d& GetPose() const { return m_pose; }

  /**
   * Updates the robot's position.
   *
   * @param xWheelPos The distance traveled by the forward-facing wheel, in
   * meters.
   * @param yWheelPos The distance traveled by the left-facing wheel, in meters.
   * @param gyroAngle The angle reported by the gyroscope. This does not need to
   * be offset to match the robot's orientation on the field.
   * @return The updated pose.
   */
  const Pose3d& Update(prosdk::units::meter_t xWheelPos,
                       prosdk::units::meter_t yWheelPos,
                       const Rotation3d& gyroAngle) {
    const auto deltaAngle = gyroAngle.RelativeTo(m_previousGyroAngle);
    const auto angleDifference = deltaAngle.ToVector();

    const auto deltaTheta = deltaAngle.ToRotation2d().Radians();
    const auto deltaX =
        xWheelPos - m_previousXWheelPos + m_xWheelYPos * deltaTheta / 1_rad;
    const auto deltaY =
        yWheelPos - m_previousYWheelPos - m_yWheelXPos * deltaTheta / 1_rad;
    Twist3d twist{deltaX,
                  deltaY,
                  0_m,
                  prosdk::units::radian_t{angleDifference(0)},
                  prosdk::units::radian_t{angleDifference(1)},
                  prosdk::units::radian_t{angleDifference(2)}};

    m_pose = m_pose + twist.Exp();

    m_previousXWheelPos = xWheelPos;
    m_previousYWheelPos = yWheelPos;
    m_previousGyroAngle = gyroAngle;

    return m_pose;
  }

  /**
   * Converts measured wheel velocities to chassis velocities using inverse
   * kinematics.
   *
   * @param vx The velocity of the forward-facing wheel.
   * @param vy The velocity of the sideways-facing wheel.
   * @param omega The angular velocity of the robot as reported by the gyro.
   * @return The velocity of the chassis.
   */
  ChassisVelocities ToChassisVelocities(
      prosdk::units::meters_per_second_t vx, prosdk::units::meters_per_second_t vy,
      prosdk::units::radians_per_second_t omega) const {
    return {vx + m_xWheelYPos * omega / 1_rad,
            vy - m_yWheelXPos * omega / 1_rad, omega};
  }

 private:
  const prosdk::units::meter_t m_xWheelYPos;
  const prosdk::units::meter_t m_yWheelXPos;
  Pose3d m_pose;

  prosdk::units::meter_t m_previousXWheelPos;
  prosdk::units::meter_t m_previousYWheelPos;

  Rotation3d m_previousGyroAngle;
};

}  // namespace prosdk::math
