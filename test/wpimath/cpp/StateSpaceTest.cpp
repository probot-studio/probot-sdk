// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <random>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/controller/LinearQuadraticRegulator.hpp"
#include "prosdk/math/estimator/KalmanFilter.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/math/system/LinearSystemLoop.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/mass.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/voltage.hpp"

namespace prosdk::math {

constexpr double POSITION_STDDEV = 0.0001;
constexpr auto DT = 0.00505_s;

class StateSpaceTest {
 public:
  LinearSystem<2, 1, 1> plant = [] {
    auto motors = DCMotor::Vex775Pro(2);

    // Carriage mass
    constexpr auto m = 5_kg;

    // Radius of pulley
    constexpr auto r = 0.0181864_m;

    // Gear ratio
    constexpr double G = 40.0 / 40.0;

    return prosdk::math::Models::ElevatorFromPhysicalConstants(motors, m, r, G)
        .Slice(0);
  }();
  LinearQuadraticRegulator<2, 1> controller{plant, {0.02, 0.4}, {12.0}, DT};
  KalmanFilter<2, 1, 1> observer{plant, {0.05, 1.0}, {0.0001}, DT};
  LinearSystemLoop<2, 1, 1> loop{plant, controller, observer, 12_V, DT};
};

void Update(const LinearSystem<2, 1, 1>& plant, LinearSystemLoop<2, 1, 1>& loop,
            double noise) {
  Vectord<1> y = plant.CalculateY(loop.Xhat(), loop.U()) + Vectord<1>{noise};
  loop.Correct(y);
  loop.Predict(DT);
}

TEST_CASE_METHOD(StateSpaceTest, "StateSpaceTest CorrectPredictLoop",
                 "[wpimath]") {
  std::default_random_engine generator;
  std::normal_distribution<double> dist{0.0, POSITION_STDDEV};

  Vectord<2> references{2.0, 0.0};
  loop.SetNextR(references);

  for (int i = 0; i < 1000; i++) {
    Update(plant, loop, dist(generator));
    CHECK(-12.0 - 1e-12 <= loop.U(0));
    CHECK(loop.U(0) <= 12.0 + 1e-12);
  }

  CHECK_NEAR(loop.Xhat(0), 2.0, 0.05);
  CHECK_NEAR(loop.Xhat(1), 0.0, 0.5);
}

}  // namespace prosdk::math
