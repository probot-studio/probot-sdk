// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstddef>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/trajectory/DifferentialSample.hpp"
#include "prosdk/math/trajectory/DifferentialTrajectory.hpp"
#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/math/trajectory/HolonomicTrajectory.hpp"
#include "prosdk/math/trajectory/TestDrivetrainSplineTrajectory.hpp"
#include "prosdk/math/trajectory/TrajectoryConfig.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/json.hpp"

using namespace prosdk::math;

TEST_CASE("TrajectorySerializationTest TestJsonSerialization", "[wpimath]") {
  TrajectoryConfig config{12_fps, 12_fps_sq};
  auto splineTrajectory = TestDrivetrainSplineTrajectory::GetTrajectory(config);

  // Convert DrivetrainSplineTrajectory to HolonomicTrajectory
  std::vector<HolonomicSample> samples;
  for (const auto& splineSample : splineTrajectory.Samples()) {
    samples.emplace_back(splineSample.time, splineSample.pose,
                         splineSample.velocity, splineSample.acceleration);
  }
  HolonomicTrajectory trajectory{std::move(samples)};

  // Serialize to JSON
  prosdk::util::json json;
  to_json(json, trajectory);

  // Deserialize from JSON
  auto deserializedTrajectory = json.get<HolonomicTrajectory>();

  // Verify they are equal
  CHECK(trajectory.Duration() == deserializedTrajectory.Duration());
  CHECK(trajectory.Samples().size() == deserializedTrajectory.Samples().size());

  for (size_t i = 0; i < trajectory.Samples().size(); ++i) {
    const auto& original = trajectory.Samples()[i];
    const auto& deserialized = deserializedTrajectory.Samples()[i];

    CHECK(original.time == deserialized.time);
    CHECK(original.pose == deserialized.pose);
    CHECK(original.velocity == deserialized.velocity);
    CHECK(original.acceleration == deserialized.acceleration);
  }
}

TEST_CASE("TrajectorySerializationTest TestDifferentialSerialization",
          "[wpimath]") {
  TrajectoryConfig config{12_fps, 12_fps_sq};
  auto splineTrajectory = TestDrivetrainSplineTrajectory::GetTrajectory(config);

  DifferentialDriveKinematics kinematics{0.5_m};
  std::vector<DifferentialSample> samples;
  for (const auto& splineSample : splineTrajectory.Samples()) {
    HolonomicSample holonomicSample{splineSample.time, splineSample.pose,
                                    splineSample.velocity,
                                    splineSample.acceleration};
    samples.emplace_back(holonomicSample, kinematics);
  }

  DifferentialTrajectory trajectory{std::move(samples)};

  // Serialize to JSON
  prosdk::util::json json;
  to_json(json, trajectory);

  // Deserialize from JSON
  auto deserializedTrajectory = json.get<DifferentialTrajectory>();

  // Verify they are equal
  CHECK(trajectory.Duration() == deserializedTrajectory.Duration());
  CHECK(trajectory.Samples().size() == deserializedTrajectory.Samples().size());

  for (size_t i = 0; i < trajectory.Samples().size(); ++i) {
    const auto& original = trajectory.Samples()[i];
    const auto& deserialized = deserializedTrajectory.Samples()[i];

    CHECK(original.time == deserialized.time);
    CHECK(original.pose == deserialized.pose);
    CHECK(original.velocity == deserialized.velocity);
    CHECK(original.acceleration == deserialized.acceleration);
    CHECK(original.leftVelocity == deserialized.leftVelocity);
    CHECK(original.rightVelocity == deserialized.rightVelocity);
  }
}
