// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/MecanumDriveWheelPositionsProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::MecanumDriveWheelPositions> prosdk::util::Protobuf<
    prosdk::math::MecanumDriveWheelPositions>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufMecanumDriveWheelPositions msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::MecanumDriveWheelPositions{
      prosdk::units::meter_t{msg.front_left},
      prosdk::units::meter_t{msg.front_right},
      prosdk::units::meter_t{msg.rear_left},
      prosdk::units::meter_t{msg.rear_right},
  };
}

bool prosdk::util::Protobuf<prosdk::math::MecanumDriveWheelPositions>::Pack(
    OutputStream& stream, const prosdk::math::MecanumDriveWheelPositions& value) {
  wpi_proto_ProtobufMecanumDriveWheelPositions msg{
      .front_left = value.frontLeft.value(),
      .front_right = value.frontRight.value(),
      .rear_left = value.rearLeft.value(),
      .rear_right = value.rearRight.value(),
  };
  return stream.Encode(msg);
}
