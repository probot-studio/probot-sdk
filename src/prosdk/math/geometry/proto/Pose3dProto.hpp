// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/geometry3d.npb.h"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Protobuf<prosdk::math::Pose3d> final {
  using MessageStruct = wpi_proto_ProtobufPose3d;
  using InputStream = prosdk::util::ProtoInputStream<prosdk::math::Pose3d>;
  using OutputStream = prosdk::util::ProtoOutputStream<prosdk::math::Pose3d>;
  static std::optional<prosdk::math::Pose3d> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const prosdk::math::Pose3d& value);
};
