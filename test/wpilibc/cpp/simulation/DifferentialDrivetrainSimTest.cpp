// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/DifferentialDrivetrainSim.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "prosdk/math/controller/LTVUnicycleController.hpp"
#include "prosdk/math/controller/LinearPlantInversionFeedforward.hpp"
#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/math/system/NumericalIntegration.hpp"
#include "prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp"
#include "prosdk/math/trajectory/constraint/DifferentialDriveKinematicsConstraint.hpp"
#include "prosdk/simulation/RoboRioSim.hpp"
#include "prosdk/units/current.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/moment_of_inertia.hpp"

TEST_CASE("DifferentialDrivetrainSimTest Convergence",
          "[wpilibc][simulation]") {
  using State = prosdk::sim::DifferentialDrivetrainSim::State;

  prosdk::sim::RoboRioSim::ResetData();

  auto motor = prosdk::math::DCMotor::NEO(2);
  auto plant = prosdk::math::Models::DifferentialDriveFromPhysicalConstants(
      motor, 50_kg, 2_in, 12_in, 0.5_kg_sq_m, 1.0);

  prosdk::math::DifferentialDriveKinematics kinematics{24_in};
  prosdk::sim::DifferentialDrivetrainSim sim{
      plant, 24_in, motor,
      1.0,   2_in,  {0.001, 0.001, 0.0001, 0.1, 0.1, 0.005, 0.005}};

  prosdk::math::LinearPlantInversionFeedforward feedforward{plant, 20_ms};
  prosdk::math::LTVUnicycleController feedback{20_ms};

  feedforward.Reset(prosdk::math::Vectord<2>{0.0, 0.0});

  // Ground truth.
  prosdk::math::Vectord<7> groundTruthX = prosdk::math::Vectord<7>::Zero();

  prosdk::math::TrajectoryConfig config{1_mps, 1_mps_sq};
  config.AddConstraint(
      prosdk::math::DifferentialDriveKinematicsConstraint(kinematics, 1_mps));

  auto trajectory = prosdk::math::DrivetrainSplineTrajectoryGenerator::Generate(
      prosdk::math::Pose2d{}, {}, prosdk::math::Pose2d{2_m, 2_m, 0_rad}, config);

  for (auto t = 0_s; t < trajectory.Duration(); t += 20_ms) {
    auto state = trajectory.SampleAt(t);
    auto feedbackOut = feedback.Calculate(sim.GetPose(), state);

    auto [l, r] = kinematics.ToWheelVelocities(feedbackOut);
    auto voltages =
        feedforward.Calculate(prosdk::math::Vectord<2>{l.value(), r.value()});
    auto clampedVoltages = sim.ClampInput(voltages);

    // Sim periodic code.
    sim.SetInputs(prosdk::units::volt_t{clampedVoltages(0, 0)},
                  prosdk::units::volt_t{clampedVoltages(1, 0)});
    sim.Update(20_ms);

    // Update ground truth.
    groundTruthX = prosdk::math::RKDP(
        [&sim](const auto& x, const auto& u) -> prosdk::math::Vectord<7> {
          return sim.Dynamics(x, u);
        },
        groundTruthX, clampedVoltages, 20_ms);
  }

  // 2 inch tolerance is OK since our ground truth is an approximation of the
  // ODE solution using prosdk::math::RKDP anyway
  CHECK_THAT(groundTruthX(0, 0),
             Catch::Matchers::WithinAbs(sim.GetState(State::X), 0.05));
  CHECK_THAT(groundTruthX(1, 0),
             Catch::Matchers::WithinAbs(sim.GetState(State::Y), 0.05));
  CHECK_THAT(groundTruthX(2, 0),
             Catch::Matchers::WithinAbs(sim.GetState(State::HEADING), 0.01));
}

TEST_CASE("DifferentialDrivetrainSimTest Current", "[wpilibc][simulation]") {
  auto motor = prosdk::math::DCMotor::NEO(2);
  auto plant = prosdk::math::Models::DifferentialDriveFromPhysicalConstants(
      motor, 50_kg, 2_in, 12_in, 0.5_kg_sq_m, 1.0);

  prosdk::math::DifferentialDriveKinematics kinematics{24_in};
  prosdk::sim::DifferentialDrivetrainSim sim{plant, 24_in, motor, 1.0, 2_in};

  sim.SetInputs(-12_V, 12_V);
  for (int i = 0; i < 10; ++i) {
    sim.Update(20_ms);
  }
  CHECK(sim.GetCurrentDraw() > 0_A);

  sim.SetInputs(12_V, 12_V);
  for (int i = 0; i < 20; ++i) {
    sim.Update(20_ms);
  }
  CHECK(sim.GetCurrentDraw() > 0_A);

  sim.SetInputs(-12_V, 12_V);
  for (int i = 0; i < 30; ++i) {
    sim.Update(20_ms);
  }
  CHECK(sim.GetCurrentDraw() > 0_A);
}

TEST_CASE("DifferentialDrivetrainSimTest ModelStability",
          "[wpilibc][simulation]") {
  auto motor = prosdk::math::DCMotor::NEO(2);
  auto plant = prosdk::math::Models::DifferentialDriveFromPhysicalConstants(
      motor, 50_kg, 2_in, 12_in, 2_kg_sq_m, 5.0);

  prosdk::math::DifferentialDriveKinematics kinematics{24_in};
  prosdk::sim::DifferentialDrivetrainSim sim{plant, 24_in, motor, 1.0, 2_in};

  sim.SetInputs(2_V, 4_V);

  // 10 seconds should be enough time to verify stability
  for (int i = 0; i < 500; ++i) {
    sim.Update(20_ms);
  }

  CHECK(prosdk::units::math::abs(sim.GetPose().Translation().Norm()) < 100_m);
}
