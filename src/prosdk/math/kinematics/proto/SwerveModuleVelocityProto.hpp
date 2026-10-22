// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/SwerveModuleVelocity.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::SwerveModuleVelocity> {
  using MessageStruct = wpi_proto_ProtobufSwerveModuleVelocity;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::SwerveModuleVelocity>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::SwerveModuleVelocity>;
  static std::optional<prosdk::math::SwerveModuleVelocity> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::SwerveModuleVelocity& value);
};
