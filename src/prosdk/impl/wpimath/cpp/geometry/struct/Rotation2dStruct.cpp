// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/Rotation2dStruct.hpp"

namespace {
constexpr size_t VALUE_OFF = 0;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Rotation2d>;

prosdk::math::Rotation2d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Rotation2d{
      prosdk::units::radian_t{prosdk::util::UnpackStruct<double, VALUE_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Rotation2d& value) {
  prosdk::util::PackStruct<VALUE_OFF>(data, value.Radians().value());
}
