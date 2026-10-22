// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineSample.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/math/trajectory/HolonomicTrajectory.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/curvature.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"

using namespace prosdk::math;

namespace {

DrivetrainSplineSample SplineSample(
    prosdk::units::second_t time, prosdk::units::meter_t x,
    prosdk::units::meters_per_second_t velocity,
    prosdk::units::meters_per_second_squared_t acceleration) {
  return DrivetrainSplineSample{time, Pose2d{x, 0_m, 0_deg}, velocity,
                                acceleration, prosdk::units::curvature_t{0.0}};
}

HolonomicSample HolonomicSampleWithAccel(
    prosdk::units::second_t time, prosdk::units::meters_per_second_squared_t ax) {
  return HolonomicSample{time, Pose2d{}, ChassisVelocities{},
                         ChassisAccelerations{ax, 0_mps_sq, 0_rad_per_s_sq}};
}

}  // namespace

TEST_CASE("TrajectorySampleAtTest ExactTimestampReturnsStoredSample",
          "[wpimath]") {
  // Waits at x = 0 from 0 s to 1 s, then accelerates to x = 1 at 2 s. The first
  // segment has zero length, so interpolating to its far end divides by zero.
  DrivetrainSplineTrajectory trajectory{std::vector<DrivetrainSplineSample>{
      SplineSample(0_s, 0_m, 0_mps, 0_mps_sq),
      SplineSample(1_s, 0_m, 0_mps, 2_mps_sq),
      SplineSample(2_s, 1_m, 2_mps, 0_mps_sq)}};

  auto sample = trajectory.SampleAt(1_s);

  CHECK_FALSE(std::isnan(sample.pose.X().value()));
  CHECK(sample == trajectory.Samples()[1]);
}

TEST_CASE("TrajectorySampleAtTest ExactTimestampOfEverySample", "[wpimath]") {
  HolonomicTrajectory trajectory{
      std::vector<HolonomicSample>{HolonomicSampleWithAccel(0_s, 0_mps_sq),
                                   HolonomicSampleWithAccel(1_s, 1_mps_sq),
                                   HolonomicSampleWithAccel(2_s, 2_mps_sq),
                                   HolonomicSampleWithAccel(3_s, 3_mps_sq)}};

  for (const auto& expected : trajectory.Samples()) {
    CHECK(trajectory.SampleAt(expected.time) == expected);
  }
}

TEST_CASE("TrajectorySampleAtTest JoinReturnsSecondTrajectoryStart",
          "[wpimath]") {
  HolonomicTrajectory first{
      std::vector<HolonomicSample>{HolonomicSampleWithAccel(0_s, 0_mps_sq),
                                   HolonomicSampleWithAccel(1_s, 0_mps_sq)}};
  HolonomicTrajectory second{
      std::vector<HolonomicSample>{HolonomicSampleWithAccel(0_s, 2_mps_sq),
                                   HolonomicSampleWithAccel(1_s, 0_mps_sq)}};

  auto joined = first.Concatenate(second);

  // Both trajectories contribute a sample at t = 1 s. The last one wins, so the
  // feedforward at the join comes from the start of the second trajectory.
  REQUIRE(joined.Samples().size() == 4);
  CHECK(joined.SampleAt(1_s).acceleration.ax == 2_mps_sq);
}

TEST_CASE("TrajectorySampleAtTest OutOfRangeAndEdgeTimes", "[wpimath]") {
  HolonomicTrajectory trajectory{
      std::vector<HolonomicSample>{HolonomicSampleWithAccel(0_s, 1_mps_sq),
                                   HolonomicSampleWithAccel(1_s, 2_mps_sq)}};

  CHECK(trajectory.SampleAt(-1_s) == trajectory.Start());
  CHECK(trajectory.SampleAt(-0.0_s) == trajectory.Start());
  CHECK(trajectory.SampleAt(0_s) == trajectory.Start());
  CHECK(trajectory.SampleAt(1_s) == trajectory.End());
  CHECK(trajectory.SampleAt(5_s) == trajectory.End());
}

TEST_CASE("TrajectorySampleAtTest NaNTimeReturnsASample", "[wpimath]") {
  HolonomicTrajectory trajectory{
      std::vector<HolonomicSample>{HolonomicSampleWithAccel(0_s, 1_mps_sq),
                                   HolonomicSampleWithAccel(1_s, 2_mps_sq)}};

  // Garbage in, but it must not read past the end of the samples.
  auto sample = trajectory.SampleAt(prosdk::units::second_t{std::nan("")});
  CHECK((sample == trajectory.Start() || sample == trajectory.End()));
}
