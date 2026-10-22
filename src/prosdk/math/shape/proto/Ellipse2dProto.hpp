// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/shape/Ellipse2d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/shape.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::Ellipse2d> final {
  using MessageStruct = wpi_proto_ProtobufEllipse2d;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::Ellipse2d>;
  using OutputStream = prosdk::util::ProtoOutputStream<prosdk::math::Ellipse2d>;
  static std::optional<prosdk::math::Ellipse2d> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const prosdk::math::Ellipse2d& value);
};
