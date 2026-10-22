// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/SwerveModulePosition.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::SwerveModulePosition> {
  using MessageStruct = wpi_proto_ProtobufSwerveModulePosition;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::SwerveModulePosition>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::SwerveModulePosition>;
  static std::optional<prosdk::math::SwerveModulePosition> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::SwerveModulePosition& value);
};
