// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/SwerveModulePositionStruct.hpp"

namespace {
constexpr size_t DISTANCE_OFF = 0;
constexpr size_t ANGLE_OFF = DISTANCE_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::SwerveModulePosition>;

prosdk::math::SwerveModulePosition StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::SwerveModulePosition{
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, DISTANCE_OFF>(data)},
      prosdk::util::UnpackStruct<prosdk::math::Rotation2d, ANGLE_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::SwerveModulePosition& value) {
  prosdk::util::PackStruct<DISTANCE_OFF>(data, value.distance.value());
  prosdk::util::PackStruct<ANGLE_OFF>(data, value.angle);
}
