// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include "prosdk/math/kinematics/SwerveDriveKinematics.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

template <size_t NumModules>
struct prosdk::util::Protobuf<prosdk::math::SwerveDriveKinematics<NumModules>> {
  using MessageStruct = wpi_proto_ProtobufSwerveDriveKinematics;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::SwerveDriveKinematics<NumModules>>;
  using OutputStream = prosdk::util::ProtoOutputStream<
      prosdk::math::SwerveDriveKinematics<NumModules>>;

  static std::optional<prosdk::math::SwerveDriveKinematics<NumModules>> Unpack(
      InputStream& stream) {
    prosdk::util::WpiArrayUnpackCallback<prosdk::math::Translation2d, NumModules>
        modules;
    wpi_proto_ProtobufSwerveDriveKinematics msg{
        .modules = modules.Callback(),
    };
    modules.SetLimits(prosdk::util::DecodeLimits::Fail);
    if (!stream.Decode(msg)) {
      return {};
    }

    return prosdk::math::SwerveDriveKinematics<NumModules>{modules.Array()};
  }

  static bool Pack(OutputStream& stream,
                   const prosdk::math::SwerveDriveKinematics<NumModules>& value) {
    prosdk::util::PackCallback<prosdk::math::Translation2d> modules{
        value.GetModules()};
    wpi_proto_ProtobufSwerveDriveKinematics msg{
        .modules = modules.Callback(),
    };
    return stream.Encode(msg);
  }
};
