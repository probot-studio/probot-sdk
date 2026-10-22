// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/Twist3dStruct.hpp"

namespace {
constexpr size_t DX_OFF = 0;
constexpr size_t DY_OFF = DX_OFF + 8;
constexpr size_t DZ_OFF = DY_OFF + 8;
constexpr size_t RX_OFF = DZ_OFF + 8;
constexpr size_t RY_OFF = RX_OFF + 8;
constexpr size_t RZ_OFF = RY_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Twist3d>;

prosdk::math::Twist3d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Twist3d{
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, DX_OFF>(data)},
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, DY_OFF>(data)},
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, DZ_OFF>(data)},
      prosdk::units::radian_t{prosdk::util::UnpackStruct<double, RX_OFF>(data)},
      prosdk::units::radian_t{prosdk::util::UnpackStruct<double, RY_OFF>(data)},
      prosdk::units::radian_t{prosdk::util::UnpackStruct<double, RZ_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Twist3d& value) {
  prosdk::util::PackStruct<DX_OFF>(data, value.dx.value());
  prosdk::util::PackStruct<DY_OFF>(data, value.dy.value());
  prosdk::util::PackStruct<DZ_OFF>(data, value.dz.value());
  prosdk::util::PackStruct<RX_OFF>(data, value.rx.value());
  prosdk::util::PackStruct<RY_OFF>(data, value.ry.value());
  prosdk::util::PackStruct<RZ_OFF>(data, value.rz.value());
}
