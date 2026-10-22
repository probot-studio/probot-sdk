// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/MecanumDriveWheelVelocitiesStruct.hpp"

namespace {
constexpr size_t FRONT_LEFT_OFF = 0;
constexpr size_t FRONT_RIGHT_OFF = FRONT_LEFT_OFF + 8;
constexpr size_t REAR_LEFT_OFF = FRONT_RIGHT_OFF + 8;
constexpr size_t REAR_RIGHT_OFF = REAR_LEFT_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::MecanumDriveWheelVelocities>;

prosdk::math::MecanumDriveWheelVelocities StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::MecanumDriveWheelVelocities{
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, FRONT_LEFT_OFF>(data)},
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, FRONT_RIGHT_OFF>(data)},
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, REAR_LEFT_OFF>(data)},
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, REAR_RIGHT_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::MecanumDriveWheelVelocities& value) {
  prosdk::util::PackStruct<FRONT_LEFT_OFF>(data, value.frontLeft.value());
  prosdk::util::PackStruct<FRONT_RIGHT_OFF>(data, value.frontRight.value());
  prosdk::util::PackStruct<REAR_LEFT_OFF>(data, value.rearLeft.value());
  prosdk::util::PackStruct<REAR_RIGHT_OFF>(data, value.rearRight.value());
}
