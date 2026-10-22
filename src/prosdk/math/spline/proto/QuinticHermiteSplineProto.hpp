// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/spline/QuinticHermiteSpline.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/spline.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::QuinticHermiteSpline> {
  using MessageStruct = wpi_proto_ProtobufQuinticHermiteSpline;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::QuinticHermiteSpline>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::QuinticHermiteSpline>;
  static std::optional<prosdk::math::QuinticHermiteSpline> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::QuinticHermiteSpline& value);
};
