// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/kinematics/SwerveModulePosition.hpp"
#include "prosdk/util/SmallVector.hpp"

using namespace prosdk::math;

namespace {

const SwerveModulePosition EXPECTED_DATA =
    SwerveModulePosition{3.504_m, Rotation2d{17.4_rad}};
}  // namespace

TEST_CASE("SwerveModulePositionProtoTest Roundtrip", "[wpimath]") {
  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());

  CHECK(EXPECTED_DATA.distance.value() == unpacked_data->distance.value());
  CHECK(EXPECTED_DATA.angle == unpacked_data->angle);
}
