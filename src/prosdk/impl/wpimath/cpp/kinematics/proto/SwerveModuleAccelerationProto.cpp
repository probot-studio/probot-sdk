// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/SwerveModuleAccelerationProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::SwerveModuleAcceleration> prosdk::util::Protobuf<
    prosdk::math::SwerveModuleAcceleration>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Rotation2d> angle;
  wpi_proto_ProtobufSwerveModuleAcceleration msg{
      .acceleration = 0,
      .angle = angle.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto iangle = angle.Items();

  if (iangle.empty()) {
    return {};
  }

  return prosdk::math::SwerveModuleAcceleration{
      units::meters_per_second_squared_t{msg.acceleration},
      iangle[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::SwerveModuleAcceleration>::Pack(
    OutputStream& stream, const prosdk::math::SwerveModuleAcceleration& value) {
  prosdk::util::PackCallback angle{&value.angle};
  wpi_proto_ProtobufSwerveModuleAcceleration msg{
      .acceleration = value.acceleration.value(),
      .angle = angle.Callback(),
  };
  return stream.Encode(msg);
}
