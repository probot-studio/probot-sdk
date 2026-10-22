// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"

using namespace prosdk::math;

namespace {

using StructType = prosdk::util::Struct<prosdk::math::DifferentialDriveKinematics>;
const DifferentialDriveKinematics EXPECTED_DATA{
    DifferentialDriveKinematics{1.74_m}};
}  // namespace

TEST_CASE("DifferentialDriveKinematicsStructTest Roundtrip", "[wpimath]") {
  uint8_t buffer[StructType::GetSize()];
  std::memset(buffer, 0, StructType::GetSize());
  StructType::Pack(buffer, EXPECTED_DATA);

  DifferentialDriveKinematics unpacked_data = StructType::Unpack(buffer);

  CHECK(EXPECTED_DATA.trackwidth.value() == unpacked_data.trackwidth.value());
}
