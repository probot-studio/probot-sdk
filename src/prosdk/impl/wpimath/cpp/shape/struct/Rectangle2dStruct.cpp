// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/shape/struct/Rectangle2dStruct.hpp"

namespace {
constexpr size_t CENTER_OFF = 0;
constexpr size_t X_WIDTH_OFF =
    CENTER_OFF + prosdk::util::GetStructSize<prosdk::math::Pose2d>();
constexpr size_t Y_WIDTH_OFF = X_WIDTH_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Rectangle2d>;

prosdk::math::Rectangle2d StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Rectangle2d{
      prosdk::util::UnpackStruct<prosdk::math::Pose2d, CENTER_OFF>(data),
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, X_WIDTH_OFF>(data)},
      prosdk::units::meter_t{prosdk::util::UnpackStruct<double, Y_WIDTH_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Rectangle2d& value) {
  prosdk::util::PackStruct<CENTER_OFF>(data, value.Center());
  prosdk::util::PackStruct<X_WIDTH_OFF>(data, value.XWidth().value());
  prosdk::util::PackStruct<Y_WIDTH_OFF>(data, value.YWidth().value());
}
