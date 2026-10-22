// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/DifferentialDrivePoseEstimator.hpp"

#include "prosdk/math/estimator/PoseEstimator.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/array.hpp"

using namespace prosdk::math;

DifferentialDrivePoseEstimator::DifferentialDrivePoseEstimator(
    const Rotation2d& gyroAngle, prosdk::units::meter_t leftDistance,
    prosdk::units::meter_t rightDistance, const Pose2d& initialPose)
    : DifferentialDrivePoseEstimator{gyroAngle,          leftDistance,
                                     rightDistance,      initialPose,
                                     {0.02, 0.02, 0.01}, {0.1, 0.1, 0.1}} {}

DifferentialDrivePoseEstimator::DifferentialDrivePoseEstimator(
    const Rotation2d& gyroAngle, prosdk::units::meter_t leftDistance,
    prosdk::units::meter_t rightDistance, const Pose2d& initialPose,
    const prosdk::util::array<double, 3>& stateStdDevs,
    const prosdk::util::array<double, 3>& visionMeasurementStdDevs)
    : PoseEstimator(m_odometryImpl, stateStdDevs, visionMeasurementStdDevs),
      m_odometryImpl{gyroAngle, leftDistance, rightDistance, initialPose} {
  ResetPose(initialPose);
}
