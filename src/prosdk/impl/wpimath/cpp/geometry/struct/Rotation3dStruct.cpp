// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/Rotation3dStruct.hpp"

namespace {
constexpr size_t Q_OFF = 0;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Rotation3d>;

prosdk::math::Rotation3d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Rotation3d{
      prosdk::util::UnpackStruct<prosdk::math::Quaternion, Q_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Rotation3d& value) {
  prosdk::util::PackStruct<Q_OFF>(data, value.GetQuaternion());
}
