// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/DifferentialTrajectory.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include <prosdk/thirdparty/eigen/Eigen/Core>
#include <prosdk/thirdparty/gcem/gcem.hpp>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/system/NumericalIntegration.hpp"
#include "prosdk/math/trajectory/DifferentialSample.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/MathExtras.hpp"
#include "prosdk/util/json.hpp"

using namespace prosdk::math;

DifferentialSample DifferentialTrajectory::Interpolate(
    const DifferentialSample& start, const DifferentialSample& end,
    double t) const {
  prosdk::units::second_t interpTime = prosdk::util::Lerp(start.time, end.time, t);
  auto interpDt = interpTime - start.time;

  // The integration state holds wheel velocities (vₗ, vᵣ), which are
  // frame-invariant; the field-relative chassis velocity is reconstructed from
  // the integrated wheel velocities and heading below.
  Eigen::Vector<double, 6> initialState{
      {start.pose.X().value()},
      {start.pose.Y().value()},
      {start.pose.Rotation().Radians().value()},
      {start.leftVelocity.value()},
      {start.rightVelocity.value(), start.velocity.omega.value()}};

  // Wheel and angular accelerations are computed by finite difference between
  // the two samples (frame-independent, so no kinematics/trackwidth is needed).
  double segmentDt = (end.time - start.time).value();
  double leftAccel =
      segmentDt == 0
          ? 0
          : (end.leftVelocity - start.leftVelocity).value() / segmentDt;
  double rightAccel =
      segmentDt == 0
          ? 0
          : (end.rightVelocity - start.rightVelocity).value() / segmentDt;
  double angularAccel =
      segmentDt == 0
          ? 0
          : (end.velocity.omega - start.velocity.omega).value() / segmentDt;

  Eigen::Vector<double, 3> initialInput{
      {leftAccel}, {rightAccel}, {angularAccel}};

  // Integrate state derivatives [vₗ, vᵣ, ω, aₗ, aᵣ, α] to new states [x, y, θ,
  // vₗ, vᵣ, ω]
  auto dynamics = [&](const Eigen::Vector<double, 6>& state,
                      const Eigen::Vector<double, 3>& input) {
    double theta = state(2);
    double vl = state(3);
    double vr = state(4);
    double omega = state(5);
    double leftAccel = input(0);
    double rightAccel = input(1);
    double alpha = input(2);

    double v = (vl + vr) / 2.0;

    return Eigen::Vector<double, 6>{{v * gcem::cos(theta)},
                                    {v * gcem::sin(theta)},
                                    {omega},
                                    {leftAccel},
                                    {rightAccel},
                                    {alpha}};
  };

  Eigen::Vector<double, 6> endState =
      prosdk::math::RKDP(dynamics, initialState, initialInput, interpDt);

  double x = endState(0);
  double y = endState(1);
  double theta = endState(2);
  double vl = endState(3);
  double vr = endState(4);
  double vx = (vl + vr) / 2.0;
  double omega = endState(5);

  auto ax = prosdk::util::Lerp(start.acceleration.ax, end.acceleration.ax, t);
  auto ay = prosdk::util::Lerp(start.acceleration.ay, end.acceleration.ay, t);
  auto alpha =
      prosdk::util::Lerp(start.acceleration.alpha, end.acceleration.alpha, t);

  Rotation2d heading{prosdk::units::radian_t{theta}};

  // Reconstruct the field-relative velocity from robot-relative forward
  // velocity.
  ChassisVelocities fieldVelocity = ChassisVelocities{
      prosdk::units::meters_per_second_t{vx}, 0_mps,
      prosdk::units::radians_per_second_t{
          omega}}.ToFieldRelative(heading);

  return {interpTime,
          Pose2d{prosdk::units::meter_t{x}, prosdk::units::meter_t{y}, heading},
          fieldVelocity,
          ChassisAccelerations{prosdk::units::meters_per_second_squared_t{ax},
                               prosdk::units::meters_per_second_squared_t{ay},
                               prosdk::units::radians_per_second_squared_t{alpha}},
          prosdk::units::meters_per_second_t{vl},
          prosdk::units::meters_per_second_t{vr}};
}

DifferentialTrajectory DifferentialTrajectory::TransformBy(
    const Transform2d& transform) const {
  const Pose2d& firstPose = Start().pose;
  Pose2d transformedFirstPose = firstPose.TransformBy(transform);

  // The whole trajectory is rigidly rotated by the transform's rotation, so the
  // field-relative velocities and accelerations rotate by the same amount.
  const Rotation2d& rotation = transform.Rotation();

  std::vector<DifferentialSample> transformedSamples;
  transformedSamples.reserve(Samples().size());

  // Transform first sample
  transformedSamples.push_back(
      DifferentialSample(Start().time, transformedFirstPose,
                         Start().velocity.ToFieldRelative(rotation),
                         Start().acceleration.ToFieldRelative(rotation),
                         Start().leftVelocity, Start().rightVelocity));

  // Transform remaining samples
  for (size_t i = 1; i < Samples().size(); ++i) {
    const auto& sample = Samples()[i];
    transformedSamples.push_back(DifferentialSample(
        sample.time, transformedFirstPose + (sample.pose - firstPose),
        sample.velocity.ToFieldRelative(rotation),
        sample.acceleration.ToFieldRelative(rotation), sample.leftVelocity,
        sample.rightVelocity));
  }

  return DifferentialTrajectory(std::move(transformedSamples));
}

DifferentialTrajectory DifferentialTrajectory::RelativeTo(
    const Pose2d& pose) const {
  std::vector<DifferentialSample> samples;
  samples.reserve(m_samples.size());
  for (const auto& s : m_samples) {
    samples.emplace_back(s.RelativeTo(pose));
  }
  return DifferentialTrajectory{std::move(samples)};
}

DifferentialTrajectory DifferentialTrajectory::Concatenate(
    const DifferentialTrajectory& other) const {
  std::vector<DifferentialSample> samples;
  samples.reserve(m_samples.size() + other.Samples().size());

  // Copy existing
  samples.insert(samples.end(), m_samples.begin(), m_samples.end());

  // Append other with time offset
  for (auto s : other.Samples()) {
    s.time += Duration();
    samples.emplace_back(std::move(s));
  }

  return DifferentialTrajectory{std::move(samples)};
}

void prosdk::math::to_json(prosdk::util::json& json,
                        const DifferentialTrajectory& trajectory) {
  json = prosdk::util::json::object("samples", trajectory.Samples());
}

DifferentialTrajectory prosdk::util::json_serializer<DifferentialTrajectory>::from(
    const json& json) {
  std::vector<DifferentialSample> samples;
  from_json(json.at("samples"), samples);
  return DifferentialTrajectory{std::move(samples)};
}
