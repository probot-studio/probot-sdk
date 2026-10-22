// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/struct/DifferentialDriveKinematicsStruct.hpp"

namespace {
constexpr size_t TRACKWIDTH_OFF = 0;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::DifferentialDriveKinematics>;

prosdk::math::DifferentialDriveKinematics StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::DifferentialDriveKinematics{
      prosdk::units::meter_t{
          prosdk::util::UnpackStruct<double, TRACKWIDTH_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::DifferentialDriveKinematics& value) {
  prosdk::util::PackStruct<TRACKWIDTH_OFF>(data, value.trackwidth.value());
}
