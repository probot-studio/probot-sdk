// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/shape/struct/Ellipse2dStruct.hpp"

namespace {
constexpr size_t CENTER_OFF = 0;
constexpr size_t X_SEMI_AXIS_OFF =
    CENTER_OFF + prosdk::util::GetStructSize<prosdk::math::Pose2d>();
constexpr size_t Y_SEMI_AXIS_OFF = X_SEMI_AXIS_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Ellipse2d>;

prosdk::math::Ellipse2d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Ellipse2d{
      prosdk::util::UnpackStruct<prosdk::math::Pose2d, CENTER_OFF>(data),
      prosdk::units::meter_t{
          prosdk::util::UnpackStruct<double, X_SEMI_AXIS_OFF>(data)},
      prosdk::units::meter_t{
          prosdk::util::UnpackStruct<double, Y_SEMI_AXIS_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Ellipse2d& value) {
  prosdk::util::PackStruct<CENTER_OFF>(data, value.Center());
  prosdk::util::PackStruct<X_SEMI_AXIS_OFF>(data, value.XSemiAxis().value());
  prosdk::util::PackStruct<Y_SEMI_AXIS_OFF>(data, value.YSemiAxis().value());
}
