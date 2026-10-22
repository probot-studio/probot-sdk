// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/spline/CubicHermiteSpline.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/spline.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::CubicHermiteSpline> {
  using MessageStruct = wpi_proto_ProtobufCubicHermiteSpline;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::CubicHermiteSpline>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::CubicHermiteSpline>;
  static std::optional<prosdk::math::CubicHermiteSpline> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::CubicHermiteSpline& value);
};
