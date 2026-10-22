// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Transform2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/kinematics/DifferentialDriveWheelVelocities.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * Represents a single sample in a differential drive trajectory.
 */
class DifferentialSample : public HolonomicSample {
 public:
  /**
   * The left wheel velocity at this sample.
   */
  prosdk::units::meters_per_second_t leftVelocity{0_mps};

  /**
   * The right wheel velocity at this sample.
   */
  prosdk::units::meters_per_second_t rightVelocity{0_mps};

  /** Constructs a default DifferentialSample with all zero values. */
  constexpr DifferentialSample() = default;

  /**
   * Constructs a DifferentialSample.
   *
   * @param time The time of the sample relative to the trajectory start.
   * @param pose The robot pose at this sample (in the field reference frame).
   * @param velocity The robot velocity at this sample (in the field reference
   *                 frame).
   * @param acceleration The robot acceleration at this sample (in the field
   *                     reference frame).
   * @param leftVelocity The left wheel velocity at this sample.
   * @param rightVelocity The right wheel velocity at this sample.
   */
  constexpr DifferentialSample(prosdk::units::second_t time, const Pose2d& pose,
                               const ChassisVelocities& velocity,
                               const ChassisAccelerations& acceleration,
                               prosdk::units::meters_per_second_t leftVelocity,
                               prosdk::units::meters_per_second_t rightVelocity)
      : HolonomicSample{time, pose, velocity, acceleration},
        leftVelocity{leftVelocity},
        rightVelocity{rightVelocity} {}

  /**
   * Constructs a DifferentialSample.
   *
   * @param time The time of the sample relative to the trajectory start.
   * @param pose The robot pose at this sample (in the field reference frame).
   * @param velocity The robot velocity at this sample (in the field reference
   *                 frame).
   * @param acceleration The robot acceleration at this sample (in the field
   *                     reference frame).
   * @param kinematics The kinematics of the drivetrain.
   */
  constexpr DifferentialSample(prosdk::units::second_t time, const Pose2d& pose,
                               const ChassisVelocities& velocity,
                               const ChassisAccelerations& acceleration,
                               const DifferentialDriveKinematics& kinematics)
      : HolonomicSample{time, pose, velocity, acceleration} {
    // Wheel velocitys are derived from the robot-relative velocity.
    auto wheelVelocitys =
        kinematics.ToWheelVelocities(velocity.ToRobotRelative(pose.Rotation()));
    leftVelocity = wheelVelocitys.left;
    rightVelocity = wheelVelocitys.right;
  }

  /**
   * Constructs a DifferentialSample from a HolonomicSample.
   *
   * @param sample The HolonomicSample to copy.
   * @param leftVelocity The left wheel velocity at this sample.
   * @param rightVelocity The right wheel velocity at this sample.
   */
  constexpr DifferentialSample(const HolonomicSample& sample,
                               prosdk::units::meters_per_second_t leftVelocity,
                               prosdk::units::meters_per_second_t rightVelocity)
      : DifferentialSample{sample.time,         sample.pose,  sample.velocity,
                           sample.acceleration, leftVelocity, rightVelocity} {}

  /**
   * Constructs a DifferentialSample from a HolonomicSample.
   *
   * @param sample The HolonomicSample to copy.
   * @param kinematics The kinematics of the drivetrain.
   */
  constexpr DifferentialSample(const HolonomicSample& sample,
                               const DifferentialDriveKinematics& kinematics)
      : DifferentialSample{sample.time, sample.pose, sample.velocity,
                           sample.acceleration, kinematics} {}

  /**
   * Constructs a DifferentialSample from a SplineSample.
   *
   * @param sample The SplineSample to copy.
   * @param kinematics The kinematics of the drivetrain.
   */
  constexpr DifferentialSample(const DrivetrainSplineSample& sample,
                               const DifferentialDriveKinematics& kinematics)
      : HolonomicSample{sample.time, sample.pose, sample.velocity,
                        sample.acceleration},
        leftVelocity{kinematics
                         .ToWheelVelocities(sample.velocity.ToRobotRelative(
                             sample.pose.Rotation()))
                         .left},
        rightVelocity{kinematics
                          .ToWheelVelocities(sample.velocity.ToRobotRelative(
                              sample.pose.Rotation()))
                          .right} {}

  /**
   * Transforms the pose of this sample by the given transform.
   *
   * @param transform The transform to apply to the pose.
   * @return A new sample with the transformed pose.
   */
  constexpr DifferentialSample Transform(const Transform2d& transform) const {
    return DifferentialSample{
        time,
        pose.TransformBy(transform),
        velocity.ToFieldRelative(transform.Rotation()),
        acceleration.ToFieldRelative(transform.Rotation()),
        leftVelocity,
        rightVelocity};
  }

  /**
   * Transforms this sample to be relative to the given pose.
   *
   * @param other The pose to make this sample relative to.
   * @return A new sample with the relative pose.
   */
  constexpr DifferentialSample RelativeTo(const Pose2d& other) const {
    return DifferentialSample{time,
                              pose.RelativeTo(other),
                              velocity.ToRobotRelative(other.Rotation()),
                              acceleration.ToRobotRelative(other.Rotation()),
                              leftVelocity,
                              rightVelocity};
  }

  /**
   * Checks equality between this DifferentialSample and another object.
   *
   * @return True if the samples are equal.
   */
  constexpr bool operator==(const DifferentialSample& other) const = default;
};

WPILIB_DLLEXPORT
void to_json(prosdk::util::json& json, const DifferentialSample& sample);

WPILIB_DLLEXPORT
void from_json(const prosdk::util::json& json, DifferentialSample& sample);

WPILIB_DLLEXPORT
void to_json(prosdk::util::json& json,
             const std::vector<DifferentialSample>& samples);

WPILIB_DLLEXPORT
void from_json(const prosdk::util::json& json,
               std::vector<DifferentialSample>& samples);

}  // namespace prosdk::math

#include "prosdk/math/trajectory/proto/DifferentialSampleProto.hpp"
#include "prosdk/math/trajectory/struct/DifferentialSampleStruct.hpp"
