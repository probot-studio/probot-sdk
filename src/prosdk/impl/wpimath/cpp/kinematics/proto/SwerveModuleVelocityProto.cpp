// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/SwerveModuleVelocityProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::SwerveModuleVelocity> prosdk::util::Protobuf<
    prosdk::math::SwerveModuleVelocity>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Rotation2d> angle;
  wpi_proto_ProtobufSwerveModuleVelocity msg{
      .velocity = 0,
      .angle = angle.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto iangle = angle.Items();

  if (iangle.empty()) {
    return {};
  }

  return prosdk::math::SwerveModuleVelocity{
      prosdk::units::meters_per_second_t{msg.velocity},
      iangle[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::SwerveModuleVelocity>::Pack(
    OutputStream& stream, const prosdk::math::SwerveModuleVelocity& value) {
  prosdk::util::PackCallback angle{&value.angle};
  wpi_proto_ProtobufSwerveModuleVelocity msg{
      .velocity = value.velocity.value(),
      .angle = angle.Callback(),
  };
  return stream.Encode(msg);
}
