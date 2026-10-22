// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/util/SmallVector.hpp"

using namespace prosdk::math;

namespace {

const Translation2d EXPECTED_DATA = Translation2d{3.504_m, 22.9_m};
}  // namespace

TEST_CASE("Translation2dProtoTest Roundtrip", "[wpimath]") {
  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());
  CHECK(EXPECTED_DATA.X().value() == unpacked_data->X().value());
  CHECK(EXPECTED_DATA.Y().value() == unpacked_data->Y().value());
}
