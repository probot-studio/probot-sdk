// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/estimator/DifferentialDrivePoseEstimator3d.hpp"

#include "prosdk/math/estimator/PoseEstimator3d.hpp"
#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/array.hpp"

using namespace prosdk::math;

DifferentialDrivePoseEstimator3d::DifferentialDrivePoseEstimator3d(
    const Rotation3d& gyroAngle, prosdk::units::meter_t leftDistance,
    prosdk::units::meter_t rightDistance, const Pose3d& initialPose)
    : DifferentialDrivePoseEstimator3d{gyroAngle,
                                       leftDistance,
                                       rightDistance,
                                       initialPose,
                                       {0.02, 0.02, 0.02, 0.01},
                                       {0.1, 0.1, 0.1, 0.1}} {}

DifferentialDrivePoseEstimator3d::DifferentialDrivePoseEstimator3d(
    const Rotation3d& gyroAngle, prosdk::units::meter_t leftDistance,
    prosdk::units::meter_t rightDistance, const Pose3d& initialPose,
    const prosdk::util::array<double, 4>& stateStdDevs,
    const prosdk::util::array<double, 4>& visionMeasurementStdDevs)
    : PoseEstimator3d(m_odometryImpl, stateStdDevs, visionMeasurementStdDevs),
      m_odometryImpl{gyroAngle, leftDistance, rightDistance, initialPose} {
  ResetPose(initialPose);
}
