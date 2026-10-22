// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/Transform2dStruct.hpp"

namespace {
constexpr size_t TRANSLATION_OFF = 0;
constexpr size_t ROTATION_OFF =
    TRANSLATION_OFF + prosdk::util::GetStructSize<prosdk::math::Translation2d>();
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Transform2d>;

prosdk::math::Transform2d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Transform2d{
      prosdk::util::UnpackStruct<prosdk::math::Translation2d, TRANSLATION_OFF>(data),
      prosdk::util::UnpackStruct<prosdk::math::Rotation2d, ROTATION_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Transform2d& value) {
  prosdk::util::PackStruct<TRANSLATION_OFF>(data, value.Translation());
  prosdk::util::PackStruct<ROTATION_OFF>(data, value.Rotation());
}
