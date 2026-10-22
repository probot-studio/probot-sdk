// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/thirdparty/nanopb/pb.h"
#include "prosdk/math/controller/DifferentialDriveWheelVoltages.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

template <>
struct WPILIB_DLLEXPORT
    prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelVoltages> {
  using MessageStruct = wpi_proto_ProtobufDifferentialDriveWheelVoltages;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::DifferentialDriveWheelVoltages>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::DifferentialDriveWheelVoltages>;
  static std::optional<prosdk::math::DifferentialDriveWheelVoltages> Unpack(
      InputStream& stream);
  static bool Pack(OutputStream& stream,
                   const prosdk::math::DifferentialDriveWheelVoltages& value);
};
