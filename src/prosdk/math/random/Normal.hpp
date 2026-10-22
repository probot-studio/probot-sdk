// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <random>
#include <span>

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/platform/Random.hpp"
#include "prosdk/util/Algorithm.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

/**
 * Creates a vector of normally distributed random values with the given
 * standard deviations for each element.
 *
 * @param stdDevs An array whose elements are the standard deviations of each
 *     element of the random vector.
 * @return Vector of normally distributed values.
 */
template <std::same_as<double>... Ts>
Eigen::Vector<double, sizeof...(Ts)> Normal(Ts... stdDevs) {
  // prosdk: one shared engine seeded once (no per-call random_device + mt19937 on the stack)
  prosdk::platform::RandomEngine gen;

  Eigen::Vector<double, sizeof...(Ts)> result;
  prosdk::util::for_each(
      [&](int i, double stdDev) {
        // Passing a standard deviation of 0.0 to std::normal_distribution is
        // undefined behavior
        if (stdDev == 0.0) {
          result(i) = 0.0;
        } else {
          std::normal_distribution distr{0.0, stdDev};
          result(i) = distr(gen);
        }
      },
      stdDevs...);
  return result;
}

/**
 * Creates a vector of normally distributed random values with the given
 * standard deviations for each element.
 *
 * @param stdDevs An array whose elements are the standard deviations of each
 *     element of the random vector.
 * @return Vector of normally distributed values.
 */
template <int N>
Eigen::Vector<double, N> Normal(const std::array<double, N>& stdDevs) {
  // prosdk: one shared engine seeded once (no per-call random_device + mt19937 on the stack)
  prosdk::platform::RandomEngine gen;

  Eigen::Vector<double, N> result;
  for (size_t i = 0; i < stdDevs.size(); ++i) {
    // Passing a standard deviation of 0.0 to std::normal_distribution is
    // undefined behavior
    if (stdDevs[i] == 0.0) {
      result(i) = 0.0;
    } else {
      std::normal_distribution distr{0.0, stdDevs[i]};
      result(i) = distr(gen);
    }
  }
  return result;
}

/**
 * Creates a vector of normally distributed random values with the given
 * standard deviations for each element.
 *
 * @param stdDevs A possibly variable length container whose elements are the
 *     standard deviations of each element of the random vector.
 * @return Vector of normally distributed values.
 */
WPILIB_DLLEXPORT Eigen::VectorXd Normal(const std::span<const double> stdDevs);

}  // namespace prosdk::math
