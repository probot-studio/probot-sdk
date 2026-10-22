// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/DifferentialDriveWheelAccelerationsStruct.hpp"

#include "prosdk/math/kinematics/DifferentialDriveWheelAccelerations.hpp"
#include "prosdk/util/struct/Struct.hpp"

prosdk::math::DifferentialDriveWheelAccelerations
prosdk::util::Struct<prosdk::math::DifferentialDriveWheelAccelerations>::Unpack(
    std::span<const uint8_t> data) {
  constexpr size_t LEFT_OFF = 0;
  constexpr size_t RIGHT_OFF = LEFT_OFF + 8;
  return prosdk::math::DifferentialDriveWheelAccelerations{
      units::meters_per_second_squared_t{
          prosdk::util::UnpackStruct<double, LEFT_OFF>(data)},
      units::meters_per_second_squared_t{
          prosdk::util::UnpackStruct<double, RIGHT_OFF>(data)},
  };
}

void prosdk::util::Struct<prosdk::math::DifferentialDriveWheelAccelerations>::Pack(
    std::span<uint8_t> data,
    const prosdk::math::DifferentialDriveWheelAccelerations& value) {
  constexpr size_t LEFT_OFF = 0;
  constexpr size_t RIGHT_OFF = LEFT_OFF + 8;
  prosdk::util::PackStruct<LEFT_OFF>(data, value.left.value());
  prosdk::util::PackStruct<RIGHT_OFF>(data, value.right.value());
}
