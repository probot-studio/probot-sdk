// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "prosdk/util/protobuf/Protobuf.hpp"
#include "wpiutil_test.npb.h"

struct TestProtoInner {
  std::string msg;
};

template <>
struct prosdk::util::Protobuf<TestProtoInner> {
  using MessageStruct = wpi_proto_TestProtoInner;
  using InputStream = prosdk::util::ProtoInputStream<TestProtoInner>;
  using OutputStream = prosdk::util::ProtoOutputStream<TestProtoInner>;
  static std::optional<TestProtoInner> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const TestProtoInner& value);
};
