// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/Twist2dStruct.hpp"

namespace {
constexpr size_t DX_OFF = 0;
constexpr size_t DY_OFF = DX_OFF + 8;
constexpr size_t DTHETA_OFF = DY_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Twist2d>;

prosdk::math::Twist2d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Twist2d{
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, DX_OFF>(data)},
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, DY_OFF>(data)},
      prosdk::units::radian_t{prosdk::util::UnpackStruct<double, DTHETA_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Twist2d& value) {
  prosdk::util::PackStruct<DX_OFF>(data, value.dx.value());
  prosdk::util::PackStruct<DY_OFF>(data, value.dy.value());
  prosdk::util::PackStruct<DTHETA_OFF>(data, value.dtheta.value());
}
