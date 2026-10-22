// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/kinematics/DifferentialDriveKinematics.hpp"
#include "prosdk/util/SmallVector.hpp"

using namespace prosdk::math;

namespace {

const DifferentialDriveKinematics EXPECTED_DATA =
    DifferentialDriveKinematics{1.74_m};
}  // namespace

TEST_CASE("DifferentialDriveKinematicsProtoTest Roundtrip", "[wpimath]") {
  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());

  CHECK(EXPECTED_DATA.trackwidth.value() == unpacked_data->trackwidth.value());
}
