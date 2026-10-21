// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TestProtoInner.hpp"

#include <string>
#include <utility>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "wpiutil_test.npb.h"

std::optional<TestProtoInner> prosdk::util::Protobuf<TestProtoInner>::Unpack(
    prosdk::util::ProtoInputStream<TestProtoInner>& stream) {
  prosdk::util::UnpackCallback<std::string> str;
  wpi_proto_TestProtoInner msg{
      .msg = str.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto istr = str.Items();

  if (istr.empty()) {
    return {};
  }

  return TestProtoInner{std::move(istr[0])};
}

bool prosdk::util::Protobuf<TestProtoInner>::Pack(
    prosdk::util::ProtoOutputStream<TestProtoInner>& stream,
    const TestProtoInner& value) {
  prosdk::util::PackCallback str{&value.msg};
  wpi_proto_TestProtoInner msg{
      .msg = str.Callback(),
  };
  return stream.Encode(msg);
}

namespace {
using ProtoType = prosdk::util::Protobuf<TestProtoInner>;
}  // namespace

TEST_CASE("TestProtoInnerTest RoundtripNanopb", "[wpiutil][proto]") {
  const TestProtoInner EXPECTED_DATA = TestProtoInner{"Hello!"};

  prosdk::util::ProtobufMessage<TestProtoInner> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  std::optional<TestProtoInner> unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());
  CHECK(EXPECTED_DATA.msg == unpacked_data->msg);
}

TEST_CASE("TestProtoInnerTest RoundtripNanopbEmpty", "[wpiutil][proto]") {
  const TestProtoInner EXPECTED_DATA = TestProtoInner{"Hello!"};

  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());
  CHECK(EXPECTED_DATA.msg == unpacked_data->msg);
}
