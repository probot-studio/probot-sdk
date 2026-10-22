// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/util/StateSpaceUtil.hpp"

#include <vector>

#include <prosdk/thirdparty/eigen/Eigen/Core>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/TestAssertions.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"

TEST_CASE("StateSpaceUtilTest CostParameterPack", "[wpimath]") {
  constexpr prosdk::math::Matrixd<3, 3> mat = prosdk::math::CostMatrix(1.0, 2.0, 3.0);
  CHECK_NEAR(mat(0, 0), 1.0, 1e-3);
  CHECK_NEAR(mat(0, 1), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 0), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 1), 1.0 / 4.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(2, 2), 1.0 / 9.0, 1e-3);
}

TEST_CASE("StateSpaceUtilTest CostArray", "[wpimath]") {
  constexpr prosdk::math::Matrixd<3, 3> mat =
      prosdk::math::CostMatrix<3>({1.0, 2.0, 3.0});
  CHECK_NEAR(mat(0, 0), 1.0, 1e-3);
  CHECK_NEAR(mat(0, 1), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 0), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 1), 1.0 / 4.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(2, 2), 1.0 / 9.0, 1e-3);
}

TEST_CASE("StateSpaceUtilTest CostDynamic", "[wpimath]") {
  Eigen::MatrixXd mat = prosdk::math::CostMatrix(std::vector{1.0, 2.0, 3.0});
  CHECK_NEAR(mat(0, 0), 1.0, 1e-3);
  CHECK_NEAR(mat(0, 1), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 0), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 1), 1.0 / 4.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(2, 2), 1.0 / 9.0, 1e-3);
}

TEST_CASE("StateSpaceUtilTest CovParameterPack", "[wpimath]") {
  constexpr prosdk::math::Matrixd<3, 3> mat =
      prosdk::math::CovarianceMatrix(1.0, 2.0, 3.0);
  CHECK_NEAR(mat(0, 0), 1.0, 1e-3);
  CHECK_NEAR(mat(0, 1), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 0), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 1), 4.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(2, 2), 9.0, 1e-3);
}

TEST_CASE("StateSpaceUtilTest CovArray", "[wpimath]") {
  constexpr prosdk::math::Matrixd<3, 3> mat =
      prosdk::math::CovarianceMatrix<3>({1.0, 2.0, 3.0});
  CHECK_NEAR(mat(0, 0), 1.0, 1e-3);
  CHECK_NEAR(mat(0, 1), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 0), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 1), 4.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(2, 2), 9.0, 1e-3);
}

TEST_CASE("StateSpaceUtilTest CovDynamic", "[wpimath]") {
  Eigen::MatrixXd mat = prosdk::math::CovarianceMatrix(std::vector{1.0, 2.0, 3.0});
  CHECK_NEAR(mat(0, 0), 1.0, 1e-3);
  CHECK_NEAR(mat(0, 1), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 0), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 1), 4.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(0, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(1, 2), 0.0, 1e-3);
  CHECK_NEAR(mat(2, 2), 9.0, 1e-3);
}

TEST_CASE("StateSpaceUtilTest DesaturateInputVector", "[wpimath]") {
  constexpr Eigen::Vector2d vec1{{10.0, 12.0}};
  CHECK(prosdk::math::DesaturateInputVector<2>(vec1, 12.0) == vec1);
  CHECK(prosdk::math::DesaturateInputVector<2>(vec1, 10.0) ==
        (Eigen::Vector2d{{25.0 / 3.0}, {10.0}}));

  constexpr Eigen::Vector2d vec2{{10.0, -12.0}};
  CHECK(prosdk::math::DesaturateInputVector<2>(vec2, 12.0) == vec2);
  CHECK(prosdk::math::DesaturateInputVector<2>(vec2, 10.0) ==
        (Eigen::Vector2d{{25.0 / 3.0}, {-10.0}}));

  constexpr Eigen::Vector2d vec3{{0.0, 0.0}};
  CHECK(prosdk::math::DesaturateInputVector<2>(vec3, 12.0) == vec3);
}
