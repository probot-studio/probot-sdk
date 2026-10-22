// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/system.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::DCMotor> {
  using MessageStruct = wpi_proto_ProtobufDCMotor;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::DCMotor>;
  using OutputStream = prosdk::util::ProtoOutputStream<prosdk::math::DCMotor>;
  static std::optional<prosdk::math::DCMotor> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const prosdk::math::DCMotor& value);
};
