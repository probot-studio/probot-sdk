// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"

#include <format>
#include <functional>
#include <utility>
#include <vector>

#include "prosdk/math/geometry/Transform2d.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/spline/Spline.hpp"
#include "prosdk/math/spline/SplineHelper.hpp"
#include "prosdk/math/spline/SplineParameterizer.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryParameterizer.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/curvature.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/units/time.hpp"

using namespace prosdk::math;

const DrivetrainSplineTrajectory
    DrivetrainSplineTrajectoryGenerator::DO_NOTHING_TRAJECTORY(
        std::vector<DrivetrainSplineSample>{DrivetrainSplineSample{
            0_s, Pose2d{}, ChassisVelocities{}, ChassisAccelerations{},
            prosdk::units::curvature_t{0.0}}});
std::function<void(const char*)>
    DrivetrainSplineTrajectoryGenerator::s_errorFunc;

void DrivetrainSplineTrajectoryGenerator::ReportError(const char* error) {
  if (s_errorFunc) {
    s_errorFunc(error);
  } else {
    // prosdk: platform console (DS console on the robot), never throws
    prosdk::platform::ConsoleWrite(
        std::format("DrivetrainSplineTrajectoryGenerator error: {}\n", error));
  }
}

DrivetrainSplineTrajectory DrivetrainSplineTrajectoryGenerator::Generate(
    Spline<3>::ControlVector initial,
    const std::vector<Translation2d>& interiorWaypoints,
    Spline<3>::ControlVector end, const TrajectoryConfig& config) {
  const Transform2d flip{Translation2d{}, 180_deg};

  // Make theta normal for trajectory generation if path is reversed.
  // Flip the headings.
  if (config.IsReversed()) {
    initial.x[1] *= -1;
    initial.y[1] *= -1;
    end.x[1] *= -1;
    end.y[1] *= -1;
  }

  std::vector<prosdk::math::SplineParameterizer::PoseWithCurvature> points;
  try {
    points =
        SplinePointsFromSplines(SplineHelper::CubicSplinesFromControlVectors(
            initial, interiorWaypoints, end));
  } catch (SplineParameterizer::MalformedSplineException& e) {
    ReportError(e.what());
    return DO_NOTHING_TRAJECTORY;
  }

  // After trajectory generation, flip theta back so it's relative to the
  // field. Also fix curvature.
  if (config.IsReversed()) {
    for (auto& point : points) {
      point = {point.first + flip, -point.second};
    }
  }

  return DrivetrainSplineTrajectoryParameterizer::Parameterize(
      points, config.Constraints(), config.StartVelocity(),
      config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
      config.IsReversed());
}

DrivetrainSplineTrajectory DrivetrainSplineTrajectoryGenerator::Generate(
    const Pose2d& start, const std::vector<Translation2d>& interiorWaypoints,
    const Pose2d& end, const TrajectoryConfig& config) {
  auto [startCV, endCV] = SplineHelper::CubicControlVectorsFromWaypoints(
      start, interiorWaypoints, end);
  return Generate(startCV, interiorWaypoints, endCV, config);
}

DrivetrainSplineTrajectory DrivetrainSplineTrajectoryGenerator::Generate(
    std::vector<Spline<5>::ControlVector> controlVectors,
    const TrajectoryConfig& config) {
  const Transform2d flip{Translation2d{}, 180_deg};
  // Make theta normal for trajectory generation if path is reversed.
  if (config.IsReversed()) {
    for (auto& vector : controlVectors) {
      // Flip the headings.
      vector.x[1] *= -1;
      vector.y[1] *= -1;
    }
  }

  std::vector<prosdk::math::SplineParameterizer::PoseWithCurvature> points;
  try {
    points = SplinePointsFromSplines(
        SplineHelper::QuinticSplinesFromControlVectors(controlVectors));
  } catch (SplineParameterizer::MalformedSplineException& e) {
    ReportError(e.what());
    return DO_NOTHING_TRAJECTORY;
  }

  // After trajectory generation, flip theta back so it's relative to the
  // field. Also fix curvature.
  if (config.IsReversed()) {
    for (auto& point : points) {
      point = {point.first + flip, -point.second};
    }
  }

  return DrivetrainSplineTrajectoryParameterizer::Parameterize(
      points, config.Constraints(), config.StartVelocity(),
      config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
      config.IsReversed());
}

DrivetrainSplineTrajectory DrivetrainSplineTrajectoryGenerator::Generate(
    const std::vector<Pose2d>& waypoints, const TrajectoryConfig& config) {
  auto newWaypoints = waypoints;
  const Transform2d flip{Translation2d{}, 180_deg};
  if (config.IsReversed()) {
    for (auto& waypoint : newWaypoints) {
      waypoint = waypoint + flip;
    }
  }

  std::vector<SplineParameterizer::PoseWithCurvature> points;
  try {
    points = SplinePointsFromSplines(SplineHelper::OptimizeCurvature(
        SplineHelper::QuinticSplinesFromWaypoints(newWaypoints)));
  } catch (SplineParameterizer::MalformedSplineException& e) {
    ReportError(e.what());
    return DO_NOTHING_TRAJECTORY;
  }

  // After trajectory generation, flip theta back so it's relative to the
  // field. Also fix curvature.
  if (config.IsReversed()) {
    for (auto& point : points) {
      point = {point.first + flip, -point.second};
    }
  }

  return DrivetrainSplineTrajectoryParameterizer::Parameterize(
      points, config.Constraints(), config.StartVelocity(),
      config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
      config.IsReversed());
}

void DrivetrainSplineTrajectoryGenerator::SetErrorHandler(
    std::function<void(const char*)> func) {
  s_errorFunc = std::move(func);
}
