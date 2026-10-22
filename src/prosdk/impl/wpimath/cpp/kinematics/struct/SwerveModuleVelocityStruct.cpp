// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/SwerveModuleVelocityStruct.hpp"

namespace {
constexpr size_t VELOCITY_OFF = 0;
constexpr size_t ANGLE_OFF = VELOCITY_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::SwerveModuleVelocity>;

prosdk::math::SwerveModuleVelocity StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::SwerveModuleVelocity{
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, VELOCITY_OFF>(data)},
      prosdk::util::UnpackStruct<prosdk::math::Rotation2d, ANGLE_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::SwerveModuleVelocity& value) {
  prosdk::util::PackStruct<VELOCITY_OFF>(data, value.velocity.value());
  prosdk::util::PackStruct<ANGLE_OFF>(data, value.angle);
}
