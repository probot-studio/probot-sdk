// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/DifferentialDriveWheelPositionsStruct.hpp"

namespace {
constexpr size_t LEFT_OFF = 0;
constexpr size_t RIGHT_OFF = LEFT_OFF + 8;
}  // namespace

using StructType =
    prosdk::util::Struct<prosdk::math::DifferentialDriveWheelPositions>;

prosdk::math::DifferentialDriveWheelPositions StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::DifferentialDriveWheelPositions{
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, LEFT_OFF>(data)},
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, RIGHT_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::DifferentialDriveWheelPositions& value) {
  prosdk::util::PackStruct<LEFT_OFF>(data, value.left.value());
  prosdk::util::PackStruct<RIGHT_OFF>(data, value.right.value());
}
