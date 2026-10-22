// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/thirdparty/nanopb/pb.h"
#include "prosdk/math/controller/ArmFeedforward.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::ArmFeedforward> {
  using MessageStruct = wpi_proto_ProtobufArmFeedforward;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::ArmFeedforward>;
  using OutputStream = prosdk::util::ProtoOutputStream<prosdk::math::ArmFeedforward>;
  static std::optional<prosdk::math::ArmFeedforward> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::ArmFeedforward& value);
};
