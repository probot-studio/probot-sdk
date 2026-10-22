// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/MecanumDriveWheelAccelerationsProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::MecanumDriveWheelAccelerations> prosdk::util::Protobuf<
    prosdk::math::MecanumDriveWheelAccelerations>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufMecanumDriveWheelAccelerations msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::MecanumDriveWheelAccelerations{
      units::meters_per_second_squared_t{msg.front_left},
      units::meters_per_second_squared_t{msg.front_right},
      units::meters_per_second_squared_t{msg.rear_left},
      units::meters_per_second_squared_t{msg.rear_right},
  };
}

bool prosdk::util::Protobuf<prosdk::math::MecanumDriveWheelAccelerations>::Pack(
    OutputStream& stream,
    const prosdk::math::MecanumDriveWheelAccelerations& value) {
  wpi_proto_ProtobufMecanumDriveWheelAccelerations msg{
      .front_left = value.frontLeft.value(),
      .front_right = value.frontRight.value(),
      .rear_left = value.rearLeft.value(),
      .rear_right = value.rearRight.value(),
  };
  return stream.Encode(msg);
}
