// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/MecanumDrivePoseEstimator.hpp"

#include "prosdk/math/estimator/PoseEstimator.hpp"
#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/util/array.hpp"

using namespace prosdk::math;

prosdk::math::MecanumDrivePoseEstimator::MecanumDrivePoseEstimator(
    MecanumDriveKinematics& kinematics, const Rotation2d& gyroAngle,
    const MecanumDriveWheelPositions& wheelPositions, const Pose2d& initialPose)
    : MecanumDrivePoseEstimator{kinematics,      gyroAngle,
                                wheelPositions,  initialPose,
                                {0.1, 0.1, 0.1}, {0.45, 0.45, 0.45}} {}

prosdk::math::MecanumDrivePoseEstimator::MecanumDrivePoseEstimator(
    MecanumDriveKinematics& kinematics, const Rotation2d& gyroAngle,
    const MecanumDriveWheelPositions& wheelPositions, const Pose2d& initialPose,
    const prosdk::util::array<double, 3>& stateStdDevs,
    const prosdk::util::array<double, 3>& visionMeasurementStdDevs)
    : PoseEstimator(m_odometryImpl, stateStdDevs, visionMeasurementStdDevs),
      m_odometryImpl(kinematics, gyroAngle, wheelPositions, initialPose) {
  ResetPose(initialPose);
}
