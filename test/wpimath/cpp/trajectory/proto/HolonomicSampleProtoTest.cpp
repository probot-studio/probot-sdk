// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/util/SmallVector.hpp"

using namespace prosdk::math;

namespace {
const HolonomicSample EXPECTED_DATA =
    HolonomicSample{1.5_s, Pose2d{1.1_m, 2.2_m, 30_deg},
                    ChassisVelocities{1_mps, 2_mps, 3_rad_per_s},
                    ChassisAccelerations{4_mps_sq, 5_mps_sq, 6_rad_per_s_sq}};
}  // namespace

TEST_CASE("HolonomicSampleProtoTest Roundtrip", "[wpimath]") {
  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());

  CHECK(EXPECTED_DATA == unpacked_data.value());
}
