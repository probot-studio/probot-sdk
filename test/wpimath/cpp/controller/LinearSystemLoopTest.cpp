// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/LinearSystemLoop.hpp"

#include <numbers>
#include <random>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/controller/LinearPlantInversionFeedforward.hpp"
#include "prosdk/math/controller/LinearQuadraticRegulator.hpp"
#include "prosdk/math/estimator/KalmanFilter.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/math/trajectory/TrapezoidProfile.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/mass.hpp"
#include "prosdk/units/moment_of_inertia.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

inline constexpr prosdk::units::second_t DT = 5_ms;
inline constexpr double POSITION_STDDEV = 0.0001;

namespace {
std::default_random_engine generator;
std::normal_distribution<double> distribution{0.0, 1.0};
}  // namespace

TEST_CASE("LinearSystemLoopTest StateSpaceEnabled", "[wpimath]") {
  prosdk::math::LinearSystem<2, 1, 2> plant{
      prosdk::math::Models::ElevatorFromPhysicalConstants(
          prosdk::math::DCMotor::Vex775Pro(2), 5_kg, 0.0181864_m, 1.0)};

  prosdk::math::LinearSystem<2, 1, 1> slicedPlant{plant.Slice(0)};

  prosdk::math::KalmanFilter<2, 1, 1> observer{
      slicedPlant, {0.05, 1.0}, {POSITION_STDDEV}, DT};

  prosdk::math::LinearQuadraticRegulator<2, 1> controller{
      slicedPlant, {0.02, 0.4}, {12.0}, DT};

  prosdk::math::LinearSystemLoop<2, 1, 1> loop{slicedPlant, controller, observer,
                                            12_V, DT};
  loop.Reset({0, 0});

  prosdk::math::Vectord<2> references{2.0, 0.0};
  loop.SetNextR(references);

  prosdk::math::TrapezoidProfile<prosdk::units::meters>::Constraints constraints{
      4_mps, 3_mps_sq};
  prosdk::math::TrapezoidProfile<prosdk::units::meters> profile{constraints};

  prosdk::math::TrapezoidProfile<prosdk::units::meters>::State state{
      prosdk::units::meter_t{loop.Xhat(0)},
      prosdk::units::meters_per_second_t{loop.Xhat(1)}};
  for (int i = 0; i < 1000; ++i) {
    state = profile.Calculate(DT, state,
                              {prosdk::units::meter_t{references(0)},
                               prosdk::units::meters_per_second_t{references(1)}});
    loop.SetNextR({state.position.value(), state.velocity.value()});

    prosdk::math::Matrixd<1, 1> y{
        slicedPlant.CalculateY(loop.Xhat(), loop.U()) +
        prosdk::math::Vectord<1>{distribution(generator) * POSITION_STDDEV}};

    loop.Correct(y);
    loop.Predict(DT);

    double u = loop.U(0);

    CHECK(u > -12.1);
    CHECK(u <= 12.1);
  }

  CHECK_NEAR(2.0, loop.Xhat(0), 0.05);
  CHECK_NEAR(0.0, loop.Xhat(1), 0.5);
}

TEST_CASE("LinearSystemLoopTest FlywheelEnabled", "[wpimath]") {
  prosdk::math::LinearSystem<1, 1, 1> plant{
      prosdk::math::Models::FlywheelFromPhysicalConstants(
          prosdk::math::DCMotor::NEO(2), 0.00289_kg_sq_m, 1.0)};

  prosdk::math::KalmanFilter<1, 1, 1> observer{
      plant, {1.0}, {POSITION_STDDEV}, DT};

  prosdk::math::LinearQuadraticRegulator<1, 1> controller{
      plant, {9.0}, {12.0}, DT};

  prosdk::math::LinearPlantInversionFeedforward feedforward{plant, DT};

  prosdk::math::LinearSystemLoop<1, 1, 1> loop{controller, feedforward, observer,
                                            12_V};
  loop.Reset(prosdk::math::Vectord<1>{0.0});

  prosdk::math::Vectord<1> references{3000.0 / 60.0 * 2.0 * std::numbers::pi};
  loop.SetNextR(references);

  prosdk::units::second_t time = 0_s;
  while (time < 10_s) {
    loop.SetNextR(references);
    prosdk::math::Matrixd<1, 1> y{
        plant.CalculateY(loop.Xhat(), loop.U()) +
        prosdk::math::Vectord<1>{distribution(generator) * POSITION_STDDEV}};

    loop.Correct(y);
    loop.Predict(DT);

    double u = loop.U(0);

    CHECK(u > -12.1);
    CHECK(u <= 12.1);

    time += DT;
  }

  CHECK_NEAR(0.0, loop.Error().value(), 0.1);
}

TEST_CASE("LinearSystemLoopTest AtReference", "[wpimath]") {
  prosdk::math::LinearSystem<2, 1, 2> plant{
      prosdk::math::Models::ElevatorFromPhysicalConstants(
          prosdk::math::DCMotor::Vex775Pro(2), 5_kg, 0.0181864_m, 1.0)};

  prosdk::math::LinearSystem<2, 1, 1> slicedPlant{plant.Slice(0)};

  prosdk::math::KalmanFilter<2, 1, 1> observer{
      slicedPlant, {0.05, 1.0}, {POSITION_STDDEV}, DT};

  prosdk::math::LinearQuadraticRegulator<2, 1> controller{
      slicedPlant, {0.02, 0.4}, {12.0}, DT};

  prosdk::math::LinearSystemLoop<2, 1, 1> loop{slicedPlant, controller, observer,
                                            12_V, DT};
  loop.Reset({0, 0});

  // Default tolerance is zero and the error is zero, so the loop is at
  // reference.
  CHECK(loop.AtReference());

  loop.SetTolerance({0.1, 0.2});
  loop.SetNextR({0, 0});

  // AtReference() delegates to the controller, whose error is snapshotted
  // during Predict() as nextR - xHat.
  loop.SetXhat({0.05, 0.1});
  loop.Predict(DT);
  CHECK(loop.AtReference());

  loop.SetXhat({0.2, 0.1});
  loop.Predict(DT);
  CHECK_FALSE(loop.AtReference());

  // Error exactly at the tolerance boundary is considered at reference.
  loop.SetXhat({0.1, 0.2});
  loop.Predict(DT);
  CHECK(loop.AtReference());
}
