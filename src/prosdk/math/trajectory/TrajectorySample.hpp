// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <concepts>

#include "prosdk/units/time.hpp"

namespace prosdk::math {

/**
 * Requirements for a type usable as a Trajectory sample: copyable, comparable,
 * and carrying a `time` member relative to the trajectory start.
 */
template <typename SampleType>
concept TrajectorySample =
    std::copyable<SampleType> && std::equality_comparable<SampleType> &&
    requires(const SampleType& sample) {
      { sample.time } -> std::convertible_to<prosdk::units::second_t>;
    };

}  // namespace prosdk::math
