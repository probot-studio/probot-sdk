// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/SwerveModuleAccelerationStruct.hpp"

#include "prosdk/math/kinematics/SwerveModuleAcceleration.hpp"
#include "prosdk/util/struct/Struct.hpp"

prosdk::math::SwerveModuleAcceleration
prosdk::util::Struct<prosdk::math::SwerveModuleAcceleration>::Unpack(
    std::span<const uint8_t> data) {
  constexpr size_t ACCELERATION_OFF = 0;
  constexpr size_t ANGLE_OFF = ACCELERATION_OFF + 8;
  return prosdk::math::SwerveModuleAcceleration{
      units::meters_per_second_squared_t{
          prosdk::util::UnpackStruct<double, ACCELERATION_OFF>(data)},
      prosdk::util::UnpackStruct<prosdk::math::Rotation2d, ANGLE_OFF>(data)};
}

void prosdk::util::Struct<prosdk::math::SwerveModuleAcceleration>::Pack(
    std::span<uint8_t> data, const prosdk::math::SwerveModuleAcceleration& value) {
  constexpr size_t ACCELERATION_OFF = 0;
  constexpr size_t ANGLE_OFF = ACCELERATION_OFF + 8;
  prosdk::util::PackStruct<ACCELERATION_OFF>(data, value.acceleration.value());
  prosdk::util::PackStruct<ANGLE_OFF>(data, value.angle);
}
