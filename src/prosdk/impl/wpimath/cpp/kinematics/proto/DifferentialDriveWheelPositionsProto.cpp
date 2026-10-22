// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/DifferentialDriveWheelPositionsProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::DifferentialDriveWheelPositions> prosdk::util::Protobuf<
    prosdk::math::DifferentialDriveWheelPositions>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelPositions msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialDriveWheelPositions{
      prosdk::units::meter_t{msg.left},
      prosdk::units::meter_t{msg.right},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelPositions>::Pack(
    OutputStream& stream,
    const prosdk::math::DifferentialDriveWheelPositions& value) {
  wpi_proto_ProtobufDifferentialDriveWheelPositions msg{
      .left = value.left.value(),
      .right = value.right.value(),
  };
  return stream.Encode(msg);
}
