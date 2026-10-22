// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/ChassisAccelerationsStruct.hpp"

#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/util/struct/Struct.hpp"

prosdk::math::ChassisAccelerations prosdk::util::Struct<
    prosdk::math::ChassisAccelerations>::Unpack(std::span<const uint8_t> data) {
  constexpr size_t AX_OFF = 0;
  constexpr size_t AY_OFF = AX_OFF + 8;
  constexpr size_t ALPHA_OFF = AY_OFF + 8;
  return prosdk::math::ChassisAccelerations{
      units::meters_per_second_squared_t{
          prosdk::util::UnpackStruct<double, AX_OFF>(data)},
      units::meters_per_second_squared_t{
          prosdk::util::UnpackStruct<double, AY_OFF>(data)},
      units::radians_per_second_squared_t{
          prosdk::util::UnpackStruct<double, ALPHA_OFF>(data)},
  };
}

void prosdk::util::Struct<prosdk::math::ChassisAccelerations>::Pack(
    std::span<uint8_t> data, const prosdk::math::ChassisAccelerations& value) {
  constexpr size_t AX_OFF = 0;
  constexpr size_t AY_OFF = AX_OFF + 8;
  constexpr size_t ALPHA_OFF = AY_OFF + 8;
  prosdk::util::PackStruct<AX_OFF>(data, value.ax.value());
  prosdk::util::PackStruct<AY_OFF>(data, value.ay.value());
  prosdk::util::PackStruct<ALPHA_OFF>(data, value.alpha.value());
}
