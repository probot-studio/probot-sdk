// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_range_equals.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>

#include "TestProtoInner.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "wpiutil_test.npb.h"

struct RepeatedTestProto {
  std::vector<double> double_msg;
  std::vector<float> float_msg;
  std::vector<int32_t> int32_msg;
  std::vector<int64_t> int64_msg;
  std::vector<uint32_t> uint32_msg;
  std::vector<uint64_t> uint64_msg;
  std::vector<int32_t> sint32_msg;
  std::vector<int64_t> sint64_msg;
  std::vector<uint32_t> fixed32_msg;
  std::vector<uint64_t> fixed64_msg;
  std::vector<int32_t> sfixed32_msg;
  std::vector<int64_t> sfixed64_msg;
  prosdk::util::SmallVector<bool, 128> bool_msg;
  std::vector<std::string> string_msg;
  std::vector<std::vector<uint8_t>> bytes_msg;
  std::vector<TestProtoInner> TestProtoInner_msg;
};

template <>
struct prosdk::util::Protobuf<RepeatedTestProto> {
  using MessageStruct = wpi_proto_RepeatedTestProto;
  using InputStream = prosdk::util::ProtoInputStream<RepeatedTestProto>;
  using OutputStream = prosdk::util::ProtoOutputStream<RepeatedTestProto>;
  static std::optional<RepeatedTestProto> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const RepeatedTestProto& value);
};

std::optional<RepeatedTestProto> prosdk::util::Protobuf<RepeatedTestProto>::Unpack(
    InputStream& stream) {
  RepeatedTestProto toRet;

  prosdk::util::DirectUnpackCallback<double, std::vector<double>> double_msg{
      toRet.double_msg};
  prosdk::util::DirectUnpackCallback<float, std::vector<float>> float_msg{
      toRet.float_msg};
  prosdk::util::DirectUnpackCallback<int32_t, std::vector<int32_t>> int32_msg{
      toRet.int32_msg};
  prosdk::util::DirectUnpackCallback<int64_t, std::vector<int64_t>> int64_msg{
      toRet.int64_msg};
  prosdk::util::DirectUnpackCallback<uint32_t, std::vector<uint32_t>> uint32_msg{
      toRet.uint32_msg};
  prosdk::util::DirectUnpackCallback<uint64_t, std::vector<uint64_t>> uint64_msg{
      toRet.uint64_msg};
  prosdk::util::DirectUnpackCallback<int32_t, std::vector<int32_t>> sint32_msg{
      toRet.sint32_msg};
  prosdk::util::DirectUnpackCallback<int64_t, std::vector<int64_t>> sint64_msg{
      toRet.sint64_msg};
  prosdk::util::DirectUnpackCallback<uint32_t, std::vector<uint32_t>> fixed32_msg{
      toRet.fixed32_msg};
  prosdk::util::DirectUnpackCallback<uint64_t, std::vector<uint64_t>> fixed64_msg{
      toRet.fixed64_msg};
  prosdk::util::DirectUnpackCallback<int32_t, std::vector<int32_t>> sfixed32_msg{
      toRet.sfixed32_msg};
  prosdk::util::DirectUnpackCallback<int64_t, std::vector<int64_t>> sfixed64_msg{
      toRet.sfixed64_msg};
  prosdk::util::DirectUnpackCallback<bool, prosdk::util::SmallVector<bool, 128>>
      bool_msg{toRet.bool_msg};
  prosdk::util::DirectUnpackCallback<std::string, std::vector<std::string>>
      string_msg{toRet.string_msg};
  prosdk::util::DirectUnpackCallback<std::vector<uint8_t>,
                                  std::vector<std::vector<uint8_t>>>
      bytes_msg{toRet.bytes_msg};
  prosdk::util::DirectUnpackCallback<TestProtoInner, std::vector<TestProtoInner>>
      TestProtoInner_msg{toRet.TestProtoInner_msg};

  wpi_proto_RepeatedTestProto msg{
      .double_msg = double_msg.Callback(),
      .float_msg = float_msg.Callback(),
      .int32_msg = int32_msg.Callback(),
      .int64_msg = int64_msg.Callback(),
      .uint32_msg = uint32_msg.Callback(),
      .uint64_msg = uint64_msg.Callback(),
      .sint32_msg = sint32_msg.Callback(),
      .sint64_msg = sint64_msg.Callback(),
      .fixed32_msg = fixed32_msg.Callback(),
      .fixed64_msg = fixed64_msg.Callback(),
      .sfixed32_msg = sfixed32_msg.Callback(),
      .sfixed64_msg = sfixed64_msg.Callback(),
      .bool_msg = bool_msg.Callback(),
      .string_msg = string_msg.Callback(),
      .bytes_msg = bytes_msg.Callback(),
      .TestProtoInner_msg = TestProtoInner_msg.Callback(),
  };

  if (!stream.Decode(msg)) {
    return {};
  }

  return toRet;
}

bool prosdk::util::Protobuf<RepeatedTestProto>::Pack(
    OutputStream& stream, const RepeatedTestProto& value) {
  prosdk::util::PackCallback<double> double_msg{value.double_msg};
  prosdk::util::PackCallback<float> float_msg{value.float_msg};
  prosdk::util::PackCallback<int32_t> int32_msg{value.int32_msg};
  prosdk::util::PackCallback<int64_t> int64_msg{value.int64_msg};
  prosdk::util::PackCallback<uint32_t> uint32_msg{value.uint32_msg};
  prosdk::util::PackCallback<uint64_t> uint64_msg{value.uint64_msg};
  prosdk::util::PackCallback<int32_t> sint32_msg{value.sint32_msg};
  prosdk::util::PackCallback<int64_t> sint64_msg{value.sint64_msg};
  prosdk::util::PackCallback<uint32_t> fixed32_msg{value.fixed32_msg};
  prosdk::util::PackCallback<uint64_t> fixed64_msg{value.fixed64_msg};
  prosdk::util::PackCallback<int32_t> sfixed32_msg{value.sfixed32_msg};
  prosdk::util::PackCallback<int64_t> sfixed64_msg{value.sfixed64_msg};
  prosdk::util::PackCallback<bool> bool_msg{value.bool_msg};
  prosdk::util::PackCallback<std::string> string_msg{value.string_msg};
  prosdk::util::PackCallback<std::vector<uint8_t>> bytes_msg{value.bytes_msg};
  prosdk::util::PackCallback<TestProtoInner> TestProtoInner_msg{
      value.TestProtoInner_msg};
  wpi_proto_RepeatedTestProto msg{
      .double_msg = double_msg.Callback(),
      .float_msg = float_msg.Callback(),
      .int32_msg = int32_msg.Callback(),
      .int64_msg = int64_msg.Callback(),
      .uint32_msg = uint32_msg.Callback(),
      .uint64_msg = uint64_msg.Callback(),
      .sint32_msg = sint32_msg.Callback(),
      .sint64_msg = sint64_msg.Callback(),
      .fixed32_msg = fixed32_msg.Callback(),
      .fixed64_msg = fixed64_msg.Callback(),
      .sfixed32_msg = sfixed32_msg.Callback(),
      .sfixed64_msg = sfixed64_msg.Callback(),
      .bool_msg = bool_msg.Callback(),
      .string_msg = string_msg.Callback(),
      .bytes_msg = bytes_msg.Callback(),
      .TestProtoInner_msg = TestProtoInner_msg.Callback(),
  };
  return stream.Encode(msg);
}

namespace {
using ProtoType = prosdk::util::Protobuf<RepeatedTestProto>;
}  // namespace

TEST_CASE("RepeatedTestProtoTest RoundtripNanopb", "[wpiutil][proto]") {
  RepeatedTestProto EXPECTED_DATA = RepeatedTestProto{};
  EXPECTED_DATA.bool_msg.emplace_back(true);
  EXPECTED_DATA.bool_msg.emplace_back(false);

  EXPECTED_DATA.double_msg.emplace_back(5.05);

  prosdk::util::ProtobufMessage<decltype(EXPECTED_DATA)> message;
  prosdk::util::SmallVector<uint8_t, 64> buf;

  REQUIRE(message.Pack(buf, EXPECTED_DATA));
  auto unpacked_data = message.Unpack(buf);
  REQUIRE(unpacked_data.has_value());

  REQUIRE(EXPECTED_DATA.double_msg.size() == unpacked_data->double_msg.size());
  REQUIRE(EXPECTED_DATA.float_msg.size() == unpacked_data->float_msg.size());
  REQUIRE(EXPECTED_DATA.int32_msg.size() == unpacked_data->int32_msg.size());
  REQUIRE(EXPECTED_DATA.int64_msg.size() == unpacked_data->int64_msg.size());
  REQUIRE(EXPECTED_DATA.uint32_msg.size() == unpacked_data->uint32_msg.size());
  REQUIRE(EXPECTED_DATA.uint64_msg.size() == unpacked_data->uint64_msg.size());
  REQUIRE(EXPECTED_DATA.sint32_msg.size() == unpacked_data->sint32_msg.size());
  REQUIRE(EXPECTED_DATA.sint64_msg.size() == unpacked_data->sint64_msg.size());
  REQUIRE(EXPECTED_DATA.fixed32_msg.size() ==
          unpacked_data->fixed32_msg.size());
  REQUIRE(EXPECTED_DATA.fixed64_msg.size() ==
          unpacked_data->fixed64_msg.size());
  REQUIRE(EXPECTED_DATA.sfixed32_msg.size() ==
          unpacked_data->sfixed32_msg.size());
  REQUIRE(EXPECTED_DATA.sfixed64_msg.size() ==
          unpacked_data->sfixed64_msg.size());
  REQUIRE(EXPECTED_DATA.bool_msg.size() == unpacked_data->bool_msg.size());
  REQUIRE(EXPECTED_DATA.string_msg.size() == unpacked_data->string_msg.size());
  REQUIRE(EXPECTED_DATA.bytes_msg.size() == unpacked_data->bytes_msg.size());
  REQUIRE(EXPECTED_DATA.TestProtoInner_msg.size() ==
          unpacked_data->TestProtoInner_msg.size());
}
