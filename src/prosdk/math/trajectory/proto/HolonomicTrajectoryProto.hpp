// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/thirdparty/nanopb/pb.h"
#include "prosdk/math/trajectory/HolonomicTrajectory.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/trajectory.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::HolonomicTrajectory> {
  using MessageStruct = wpi_proto_ProtobufHolonomicTrajectory;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::HolonomicTrajectory>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::HolonomicTrajectory>;
  static std::optional<prosdk::math::HolonomicTrajectory> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::HolonomicTrajectory& value);
};
