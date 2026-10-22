// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/struct/QuaternionStruct.hpp"

namespace {
constexpr size_t W_OFF = 0;
constexpr size_t X_OFF = W_OFF + 8;
constexpr size_t Y_OFF = X_OFF + 8;
constexpr size_t Z_OFF = Y_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::Quaternion>;

prosdk::math::Quaternion StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::Quaternion{
      prosdk::util::UnpackStruct<double, W_OFF>(data),
      prosdk::util::UnpackStruct<double, X_OFF>(data),
      prosdk::util::UnpackStruct<double, Y_OFF>(data),
      prosdk::util::UnpackStruct<double, Z_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::Quaternion& value) {
  prosdk::util::PackStruct<W_OFF>(data, value.W());
  prosdk::util::PackStruct<X_OFF>(data, value.X());
  prosdk::util::PackStruct<Y_OFF>(data, value.Y());
  prosdk::util::PackStruct<Z_OFF>(data, value.Z());
}
