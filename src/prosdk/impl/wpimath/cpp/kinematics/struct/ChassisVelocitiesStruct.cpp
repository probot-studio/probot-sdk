// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/ChassisVelocitiesStruct.hpp"

namespace {
constexpr size_t VX_OFF = 0;
constexpr size_t VY_OFF = VX_OFF + 8;
constexpr size_t OMEGA_OFF = VY_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::ChassisVelocities>;

prosdk::math::ChassisVelocities StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::ChassisVelocities{
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, VX_OFF>(data)},
      prosdk::units::meters_per_second_t{
          prosdk::util::UnpackStruct<double, VY_OFF>(data)},
      prosdk::units::radians_per_second_t{
          prosdk::util::UnpackStruct<double, OMEGA_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::ChassisVelocities& value) {
  prosdk::util::PackStruct<VX_OFF>(data, value.vx.value());
  prosdk::util::PackStruct<VY_OFF>(data, value.vy.value());
  prosdk::util::PackStruct<OMEGA_OFF>(data, value.omega.value());
}
