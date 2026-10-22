// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::ChassisVelocities> {
  using MessageStruct = wpi_proto_ProtobufChassisVelocities;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::ChassisVelocities>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::ChassisVelocities>;
  static std::optional<prosdk::math::ChassisVelocities> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::ChassisVelocities& value);
};
