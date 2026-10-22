// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/trajectory.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::HolonomicSample> {
  using MessageStruct = wpi_proto_ProtobufHolonomicSample;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::HolonomicSample>;
  using OutputStream = prosdk::util::ProtoOutputStream<prosdk::math::HolonomicSample>;
  static std::optional<prosdk::math::HolonomicSample> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::HolonomicSample& value);
};
