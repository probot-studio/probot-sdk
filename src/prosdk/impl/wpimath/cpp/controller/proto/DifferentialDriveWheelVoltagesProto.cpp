// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/proto/DifferentialDriveWheelVoltagesProto.hpp"

#include <optional>

#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

std::optional<prosdk::math::DifferentialDriveWheelVoltages> prosdk::util::Protobuf<
    prosdk::math::DifferentialDriveWheelVoltages>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveWheelVoltages msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialDriveWheelVoltages{
      prosdk::units::volt_t{msg.left},
      prosdk::units::volt_t{msg.right},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialDriveWheelVoltages>::Pack(
    OutputStream& stream,
    const prosdk::math::DifferentialDriveWheelVoltages& value) {
  wpi_proto_ProtobufDifferentialDriveWheelVoltages msg{
      .left = value.left.value(),
      .right = value.right.value(),
  };
  return stream.Encode(msg);
}
