// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/thirdparty/nanopb/pb.h"
#include "prosdk/math/controller/DifferentialDriveFeedforward.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

template <>
struct WPILIB_DLLEXPORT
    prosdk::util::Protobuf<prosdk::math::DifferentialDriveFeedforward> {
  using MessageStruct = wpi_proto_ProtobufDifferentialDriveFeedforward;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::DifferentialDriveFeedforward>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::DifferentialDriveFeedforward>;
  static std::optional<prosdk::math::DifferentialDriveFeedforward> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::DifferentialDriveFeedforward& value);
};
