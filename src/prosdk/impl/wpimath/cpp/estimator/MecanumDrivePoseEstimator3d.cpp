// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/MecanumDrivePoseEstimator3d.hpp"

#include "prosdk/math/estimator/PoseEstimator3d.hpp"
#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/kinematics/MecanumDriveKinematics.hpp"
#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/util/array.hpp"

using namespace prosdk::math;

prosdk::math::MecanumDrivePoseEstimator3d::MecanumDrivePoseEstimator3d(
    MecanumDriveKinematics& kinematics, const Rotation3d& gyroAngle,
    const MecanumDriveWheelPositions& wheelPositions, const Pose3d& initialPose)
    : MecanumDrivePoseEstimator3d{
          kinematics,           gyroAngle,
          wheelPositions,       initialPose,
          {0.1, 0.1, 0.1, 0.1}, {0.45, 0.45, 0.45, 0.45}} {}

prosdk::math::MecanumDrivePoseEstimator3d::MecanumDrivePoseEstimator3d(
    MecanumDriveKinematics& kinematics, const Rotation3d& gyroAngle,
    const MecanumDriveWheelPositions& wheelPositions, const Pose3d& initialPose,
    const prosdk::util::array<double, 4>& stateStdDevs,
    const prosdk::util::array<double, 4>& visionMeasurementStdDevs)
    : PoseEstimator3d(m_odometryImpl, stateStdDevs, visionMeasurementStdDevs),
      m_odometryImpl(kinematics, gyroAngle, wheelPositions, initialPose) {
  ResetPose(initialPose);
}
