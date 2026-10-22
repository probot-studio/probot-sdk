// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/thirdparty/nanopb/pb.h"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/geometry2d.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::Translation2d> final {
  using MessageStruct = wpi_proto_ProtobufTranslation2d;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::Translation2d>;
  using OutputStream = prosdk::util::ProtoOutputStream<prosdk::math::Translation2d>;
  static std::optional<prosdk::math::Translation2d> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const prosdk::math::Translation2d& value);
};
