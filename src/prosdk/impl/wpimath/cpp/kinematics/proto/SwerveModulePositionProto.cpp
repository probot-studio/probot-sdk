// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/SwerveModulePositionProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::SwerveModulePosition> prosdk::util::Protobuf<
    prosdk::math::SwerveModulePosition>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Rotation2d> angle;
  wpi_proto_ProtobufSwerveModulePosition msg{
      .distance = 0,
      .angle = angle.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto iangle = angle.Items();

  if (iangle.empty()) {
    return {};
  }

  return prosdk::math::SwerveModulePosition{
      prosdk::units::meter_t{msg.distance},
      iangle[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::SwerveModulePosition>::Pack(
    OutputStream& stream, const prosdk::math::SwerveModulePosition& value) {
  prosdk::util::PackCallback angle{&value.angle};
  wpi_proto_ProtobufSwerveModulePosition msg{
      .distance = value.distance.value(),
      .angle = angle.Callback(),
  };
  return stream.Encode(msg);
}
