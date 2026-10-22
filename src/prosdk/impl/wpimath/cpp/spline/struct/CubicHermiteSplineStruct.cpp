// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/spline/struct/CubicHermiteSplineStruct.hpp"

namespace {
constexpr size_t X_INITIAL_OFF = 0;
constexpr size_t X_FINAL_OFF = X_INITIAL_OFF + 2 * 8;
constexpr size_t Y_INITIAL_OFF = X_FINAL_OFF + 2 * 8;
constexpr size_t Y_FINAL_OFF = Y_INITIAL_OFF + 2 * 8;
}  // namespace

prosdk::math::CubicHermiteSpline prosdk::util::Struct<
    prosdk::math::CubicHermiteSpline>::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::CubicHermiteSpline{
      prosdk::util::UnpackStructArray<double, X_INITIAL_OFF, 2>(data),
      prosdk::util::UnpackStructArray<double, X_FINAL_OFF, 2>(data),
      prosdk::util::UnpackStructArray<double, Y_INITIAL_OFF, 2>(data),
      prosdk::util::UnpackStructArray<double, Y_FINAL_OFF, 2>(data)};
}

void prosdk::util::Struct<prosdk::math::CubicHermiteSpline>::Pack(
    std::span<uint8_t> data, const prosdk::math::CubicHermiteSpline& value) {
  prosdk::util::PackStructArray<X_INITIAL_OFF, 2>(
      data, value.GetInitialControlVector().x);
  prosdk::util::PackStructArray<X_FINAL_OFF, 2>(data,
                                             value.GetFinalControlVector().x);
  prosdk::util::PackStructArray<Y_INITIAL_OFF, 2>(
      data, value.GetInitialControlVector().y);
  prosdk::util::PackStructArray<Y_FINAL_OFF, 2>(data,
                                             value.GetFinalControlVector().y);
}
