// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/MecanumDriveKinematicsStruct.hpp"

namespace {
constexpr size_t FRONT_LEFT_OFF = 0;
constexpr size_t FRONT_RIGHT_OFF =
    FRONT_LEFT_OFF + prosdk::util::GetStructSize<prosdk::math::Translation2d>();
constexpr size_t REAR_LEFT_OFF =
    FRONT_RIGHT_OFF + prosdk::util::GetStructSize<prosdk::math::Translation2d>();
constexpr size_t REAR_RIGHT_OFF =
    REAR_LEFT_OFF + prosdk::util::GetStructSize<prosdk::math::Translation2d>();
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::MecanumDriveKinematics>;

prosdk::math::MecanumDriveKinematics StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::MecanumDriveKinematics{
      prosdk::util::UnpackStruct<prosdk::math::Translation2d, FRONT_LEFT_OFF>(data),
      prosdk::util::UnpackStruct<prosdk::math::Translation2d, FRONT_RIGHT_OFF>(data),
      prosdk::util::UnpackStruct<prosdk::math::Translation2d, REAR_LEFT_OFF>(data),
      prosdk::util::UnpackStruct<prosdk::math::Translation2d, REAR_RIGHT_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::MecanumDriveKinematics& value) {
  prosdk::util::PackStruct<FRONT_LEFT_OFF>(data, value.GetFrontLeft());
  prosdk::util::PackStruct<FRONT_RIGHT_OFF>(data, value.GetFrontRight());
  prosdk::util::PackStruct<REAR_LEFT_OFF>(data, value.GetRearLeft());
  prosdk::util::PackStruct<REAR_RIGHT_OFF>(data, value.GetRearRight());
}
