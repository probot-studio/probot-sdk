// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/DifferentialDriveWheelVelocitiesProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::DifferentialDriveWheelVelocities> prosdk::util::Protobuf<
    prosdk::math::DifferentialDriveWheelVelocities>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelVelocities msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialDriveWheelVelocities{
      prosdk::units::meters_per_second_t{msg.left},
      prosdk::units::meters_per_second_t{msg.right},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelVelocities>::Pack(
    OutputStream& stream,
    const prosdk::math::DifferentialDriveWheelVelocities& value) {
  wpi_proto_ProtobufDifferentialDriveWheelVelocities msg{
      .left = value.left.value(),
      .right = value.right.value(),
  };
  return stream.Encode(msg);
}
