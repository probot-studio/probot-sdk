// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/DifferentialDriveWheelAccelerationsProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::DifferentialDriveWheelAccelerations>
prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelAccelerations>::Unpack(
    InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelAccelerations msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialDriveWheelAccelerations{
      units::meters_per_second_squared_t{msg.left},
      units::meters_per_second_squared_t{msg.right},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelAccelerations>::Pack(
    OutputStream& stream,
    const prosdk::math::DifferentialDriveWheelAccelerations& value) {
  wpi_proto_ProtobufDifferentialDriveWheelAccelerations msg{
      .left = value.left.value(),
      .right = value.right.value(),
  };
  return stream.Encode(msg);
}
