// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/kinematics/MecanumDriveWheelPositions.hpp"
#include "prosdk/util/SmallVector.hpp"

using namespace prosdk::math;

namespace {

const MecanumDriveWheelPositions EXPECTED_DATA =
    MecanumDriveWheelPositions{17.4_m, 2.29_m, 22.9_m, 1.74_m};
}  // namespace

TEST_CASE("MecanumDriveWheelPositionsProtoTest Roundtrip", "[wpimath]") {
  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());

  CHECK(EXPECTED_DATA.frontLeft.value() == unpacked_data->frontLeft.value());
  CHECK(EXPECTED_DATA.frontRight.value() == unpacked_data->frontRight.value());
  CHECK(EXPECTED_DATA.rearLeft.value() == unpacked_data->rearLeft.value());
  CHECK(EXPECTED_DATA.rearRight.value() == unpacked_data->rearRight.value());
}
