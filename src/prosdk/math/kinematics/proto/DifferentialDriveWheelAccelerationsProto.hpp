// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/DifferentialDriveWheelAccelerations.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

template <>
struct WPILIB_DLLEXPORT
    prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelAccelerations> {
  using MessageStruct = wpi_proto_ProtobufDifferentialDriveWheelAccelerations;
  using InputStream = prosdk::util::ProtoInputStream<
      prosdk::math::DifferentialDriveWheelAccelerations>;
  using OutputStream = prosdk::util::ProtoOutputStream<
      prosdk::math::DifferentialDriveWheelAccelerations>;
  static std::optional<prosdk::math::DifferentialDriveWheelAccelerations> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::DifferentialDriveWheelAccelerations& value);
};
