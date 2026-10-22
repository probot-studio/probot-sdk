// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/Translation2dStruct.hpp"

namespace {
constexpr size_t X_OFF = 0;
constexpr size_t Y_OFF = X_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Translation2d>;

prosdk::math::Translation2d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Translation2d{
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, X_OFF>(data)},
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, Y_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Translation2d& value) {
  prosdk::util::PackStruct<X_OFF>(data, value.X().value());
  prosdk::util::PackStruct<Y_OFF>(data, value.Y().value());
}
