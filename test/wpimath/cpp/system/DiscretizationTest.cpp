// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/Discretization.hpp"

#include <functional>

#include <prosdk/thirdparty/eigen/Eigen/Eigenvalues>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/math/system/NumericalIntegration.hpp"
#include "prosdk/units/time.hpp"

// Check that for a simple second-order system that we can easily analyze
// analytically,
TEST_CASE("DiscretizationTest DiscretizeA", "[wpimath]") {
  prosdk::math::Matrixd<2, 2> contA{{0, 1}, {0, 0}};

  prosdk::math::Vectord<2> x0{1, 1};
  prosdk::math::Matrixd<2, 2> discA;

  prosdk::math::DiscretizeA<2>(contA, 1_s, &discA);
  prosdk::math::Vectord<2> x1Discrete = discA * x0;

  // We now have pos = vel = 1 and accel = 0, which should give us:
  prosdk::math::Vectord<2> x1Truth{1.0 * x0(0) + 1.0 * x0(1),
                                0.0 * x0(0) + 1.0 * x0(1)};

  CHECK(x1Truth == x1Discrete);
}

// Check that for a simple second-order system that we can easily analyze
// analytically,
TEST_CASE("DiscretizationTest DiscretizeAB", "[wpimath]") {
  prosdk::math::Matrixd<2, 2> contA{{0, 1}, {0, 0}};
  prosdk::math::Matrixd<2, 1> contB{0, 1};

  prosdk::math::Vectord<2> x0{1, 1};
  prosdk::math::Vectord<1> u{1};
  prosdk::math::Matrixd<2, 2> discA;
  prosdk::math::Matrixd<2, 1> discB;

  prosdk::math::DiscretizeAB<2, 1>(contA, contB, 1_s, &discA, &discB);
  prosdk::math::Vectord<2> x1Discrete = discA * x0 + discB * u;

  // We now have pos = vel = accel = 1, which should give us:
  prosdk::math::Vectord<2> x1Truth{1.0 * x0(0) + 1.0 * x0(1) + 0.5 * u(0),
                                0.0 * x0(0) + 1.0 * x0(1) + 1.0 * u(0)};

  CHECK(x1Truth == x1Discrete);
}

//                                               T
// Test that the discrete approximation of Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
//                                               0
TEST_CASE("DiscretizationTest DiscretizeSlowModelAQ", "[wpimath]") {
  prosdk::math::Matrixd<2, 2> contA{{0, 1}, {0, 0}};
  prosdk::math::Matrixd<2, 2> contQ{{1, 0}, {0, 1}};

  constexpr prosdk::units::second_t dt = 1_s;

  //       T
  // Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //       0
  prosdk::math::Matrixd<2, 2> discQIntegrated = prosdk::math::RKDP<
      std::function<prosdk::math::Matrixd<2, 2>(prosdk::units::second_t,
                                             const prosdk::math::Matrixd<2, 2>&)>,
      prosdk::math::Matrixd<2, 2>>(
      [&](prosdk::units::second_t t, const prosdk::math::Matrixd<2, 2>&) {
        return prosdk::math::Matrixd<2, 2>((contA * t.value()).exp() * contQ *
                                        (contA.transpose() * t.value()).exp());
      },
      0_s, prosdk::math::Matrixd<2, 2>::Zero(), dt);

  prosdk::math::Matrixd<2, 2> discA;
  prosdk::math::Matrixd<2, 2> discQ;
  prosdk::math::DiscretizeAQ<2>(contA, contQ, dt, &discA, &discQ);

  UNSCOPED_INFO("Expected these to be nearly equal:\ndiscQ:\n"
                << discQ << "\ndiscQIntegrated:\n"
                << discQIntegrated);
  CHECK((discQIntegrated - discQ).norm() < 1e-10);
}

//                                               T
// Test that the discrete approximation of Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
//                                               0
TEST_CASE("DiscretizationTest DiscretizeFastModelAQ", "[wpimath]") {
  prosdk::math::Matrixd<2, 2> contA{{0, 1}, {0, -1406.29}};
  prosdk::math::Matrixd<2, 2> contQ{{0.0025, 0}, {0, 1}};

  constexpr prosdk::units::second_t dt = 5_ms;

  //       T
  // Q_d = ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //       0
  prosdk::math::Matrixd<2, 2> discQIntegrated = prosdk::math::RKDP<
      std::function<prosdk::math::Matrixd<2, 2>(prosdk::units::second_t,
                                             const prosdk::math::Matrixd<2, 2>&)>,
      prosdk::math::Matrixd<2, 2>>(
      [&](prosdk::units::second_t t, const prosdk::math::Matrixd<2, 2>&) {
        return prosdk::math::Matrixd<2, 2>((contA * t.value()).exp() * contQ *
                                        (contA.transpose() * t.value()).exp());
      },
      0_s, prosdk::math::Matrixd<2, 2>::Zero(), dt);

  prosdk::math::Matrixd<2, 2> discA;
  prosdk::math::Matrixd<2, 2> discQ;
  prosdk::math::DiscretizeAQ<2>(contA, contQ, dt, &discA, &discQ);

  UNSCOPED_INFO("Expected these to be nearly equal:\ndiscQ:\n"
                << discQ << "\ndiscQIntegrated:\n"
                << discQIntegrated);
  CHECK((discQIntegrated - discQ).norm() < 1e-3);
}

// Test that DiscretizeR() works
TEST_CASE("DiscretizationTest DiscretizeR", "[wpimath]") {
  prosdk::math::Matrixd<2, 2> contR{{2.0, 0.0}, {0.0, 1.0}};
  prosdk::math::Matrixd<2, 2> discRTruth{{4.0, 0.0}, {0.0, 2.0}};

  prosdk::math::Matrixd<2, 2> discR = prosdk::math::DiscretizeR<2>(contR, 500_ms);

  UNSCOPED_INFO("Expected these to be nearly equal:\ndiscR:\n"
                << discR << "\ndiscRTruth:\n"
                << discRTruth);
  CHECK((discRTruth - discR).norm() < 1e-10);
}
