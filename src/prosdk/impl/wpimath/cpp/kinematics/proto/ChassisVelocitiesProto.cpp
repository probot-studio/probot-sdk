// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/ChassisVelocitiesProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::ChassisVelocities>
prosdk::util::Protobuf<prosdk::math::ChassisVelocities>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufChassisVelocities msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::ChassisVelocities{
      prosdk::units::meters_per_second_t{msg.vx},
      prosdk::units::meters_per_second_t{msg.vy},
      prosdk::units::radians_per_second_t{msg.omega},
  };
}

bool prosdk::util::Protobuf<prosdk::math::ChassisVelocities>::Pack(
    OutputStream& stream, const prosdk::math::ChassisVelocities& value) {
  wpi_proto_ProtobufChassisVelocities msg{
      .vx = value.vx.value(),
      .vy = value.vy.value(),
      .omega = value.omega.value(),
  };
  return stream.Encode(msg);
}
