// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/NumericalJacobian.hpp"

#include <prosdk/thirdparty/eigen/Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/linalg/EigenCore.hpp"

prosdk::math::Matrixd<4, 4> A{
    {1, 2, 4, 1}, {5, 2, 3, 4}, {5, 1, 3, 2}, {1, 1, 3, 7}};
prosdk::math::Matrixd<4, 2> B{{1, 1}, {2, 1}, {3, 2}, {3, 7}};

// Function from which to recover A and B
prosdk::math::Vectord<4> AxBuFn(const prosdk::math::Vectord<4>& x,
                             const prosdk::math::Vectord<2>& u) {
  return A * x + B * u;
}

// Test that we can recover A from AxBuFn() pretty accurately
TEST_CASE("NumericalJacobianTest Ax", "[wpimath]") {
  prosdk::math::Matrixd<4, 4> newA = prosdk::math::NumericalJacobianX<4, 4, 2>(
      AxBuFn, prosdk::math::Vectord<4>::Zero(), prosdk::math::Vectord<2>::Zero());
  CHECK(newA.isApprox(A));
}

// Test that we can recover B from AxBuFn() pretty accurately
TEST_CASE("NumericalJacobianTest Bu", "[wpimath]") {
  prosdk::math::Matrixd<4, 2> newB = prosdk::math::NumericalJacobianU<4, 4, 2>(
      AxBuFn, prosdk::math::Vectord<4>::Zero(), prosdk::math::Vectord<2>::Zero());
  CHECK(newB.isApprox(B));
}

Eigen::VectorXd AxBuFn_DynamicSize(const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& u) {
  return A * x + B * u;
}

// Test that we can recover A from AxBuFn() pretty accurately
TEST_CASE("NumericalJacobianTest Ax_DynamicSize", "[wpimath]") {
  Eigen::MatrixXd newA = prosdk::math::NumericalJacobianX(
      AxBuFn_DynamicSize, prosdk::math::Vectord<4>::Zero(),
      prosdk::math::Vectord<2>::Zero());
  CHECK(newA.isApprox(A));
}

// Test that we can recover B from AxBuFn() pretty accurately
TEST_CASE("NumericalJacobianTest Bu_DynamicSize", "[wpimath]") {
  Eigen::MatrixXd newB = prosdk::math::NumericalJacobianU(
      AxBuFn_DynamicSize, prosdk::math::Vectord<4>::Zero(),
      prosdk::math::Vectord<2>::Zero());
  CHECK(newB.isApprox(B));
}

prosdk::math::Matrixd<3, 4> C{{1, 2, 4, 1}, {5, 2, 3, 4}, {5, 1, 3, 2}};
prosdk::math::Matrixd<3, 2> D{{1, 1}, {2, 1}, {3, 2}};

// Function from which to recover C and D
prosdk::math::Vectord<3> CxDuFn(const prosdk::math::Vectord<4>& x,
                             const prosdk::math::Vectord<2>& u) {
  return C * x + D * u;
}

// Test that we can recover C from CxDuFn() pretty accurately
TEST_CASE("NumericalJacobianTest Cx", "[wpimath]") {
  prosdk::math::Matrixd<3, 4> newC = prosdk::math::NumericalJacobianX<3, 4, 2>(
      CxDuFn, prosdk::math::Vectord<4>::Zero(), prosdk::math::Vectord<2>::Zero());
  CHECK(newC.isApprox(C));
}

// Test that we can recover D from CxDuFn() pretty accurately
TEST_CASE("NumericalJacobianTest Du", "[wpimath]") {
  prosdk::math::Matrixd<3, 2> newD = prosdk::math::NumericalJacobianU<3, 4, 2>(
      CxDuFn, prosdk::math::Vectord<4>::Zero(), prosdk::math::Vectord<2>::Zero());
  CHECK(newD.isApprox(D));
}

Eigen::VectorXd CxDuFn_DynamicSize(const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& u) {
  return C * x + D * u;
}

TEST_CASE("NumericalJacobianTest Cx_DynamicSize", "[wpimath]") {
  Eigen::MatrixXd newC = prosdk::math::NumericalJacobianX(
      CxDuFn_DynamicSize, Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(2));
  CHECK(newC.isApprox(C));
}

TEST_CASE("NumericalJacobianTest Du_DynamicSize", "[wpimath]") {
  Eigen::MatrixXd newD = prosdk::math::NumericalJacobianU(
      CxDuFn_DynamicSize, Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(2));
  CHECK(newD.isApprox(D));
}
