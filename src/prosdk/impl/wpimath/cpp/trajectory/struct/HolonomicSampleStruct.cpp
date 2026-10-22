// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/struct/HolonomicSampleStruct.hpp"

namespace {
constexpr size_t TIMESTAMP_OFF = 0;
constexpr size_t POSE_OFF = TIMESTAMP_OFF + 8;
constexpr size_t VELOCITY_OFF = POSE_OFF + 24;
constexpr size_t ACCELERATION_OFF = VELOCITY_OFF + 24;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::HolonomicSample>;

prosdk::math::HolonomicSample StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::HolonomicSample{
      prosdk::units::second_t{
          prosdk::util::UnpackStruct<double, TIMESTAMP_OFF>(data)},
      prosdk::util::UnpackStruct<prosdk::math::Pose2d, POSE_OFF>(data),
      prosdk::util::UnpackStruct<prosdk::math::ChassisVelocities, VELOCITY_OFF>(data),
      prosdk::util::UnpackStruct<prosdk::math::ChassisAccelerations,
                              ACCELERATION_OFF>(data),
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::HolonomicSample& value) {
  prosdk::util::PackStruct<TIMESTAMP_OFF>(data, value.time.value());
  prosdk::util::PackStruct<POSE_OFF>(data, value.pose);
  prosdk::util::PackStruct<VELOCITY_OFF>(data, value.velocity);
  prosdk::util::PackStruct<ACCELERATION_OFF>(data, value.acceleration);
}
