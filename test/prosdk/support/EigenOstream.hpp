// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Test-only stand-in for Eigen's operator<<, which prosdk builds without (EIGEN_NO_IO in
// prosdk/platform/config.hpp). The upstream wpimath tests rely on it when Catch2 prints the
// operands of CHECK(a == b) for Eigen types; without it Catch2 treats a matrix as a range and fails
// to compile. Force-included into wpimath_tests by tools/build/modules/wpimath.cmake. The text
// comes from the std::formatter in prosdk/math/fmt/Eigen.hpp.

#pragma once

#include <format>
#include <ostream>

#include <prosdk/math/fmt/Eigen.hpp>

namespace Eigen {

template <typename Derived>
std::ostream& operator<<(std::ostream& os, const DenseBase<Derived>& m) {
  return os << std::format("{}", m.derived());
}

}  // namespace Eigen
