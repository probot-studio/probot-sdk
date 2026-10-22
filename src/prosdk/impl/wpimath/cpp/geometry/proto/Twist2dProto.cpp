// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/Twist2dProto.hpp"

#include "prosdk/generated/wpimath/protobuf/geometry2d.npb.h"

std::optional<prosdk::math::Twist2d>
prosdk::util::Protobuf<prosdk::math::Twist2d>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufTwist2d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::Twist2d{
      prosdk::units::meter_t{msg.dx},
      prosdk::units::meter_t{msg.dy},
      prosdk::units::radian_t{msg.dtheta},
  };
}

bool prosdk::util::Protobuf<prosdk::math::Twist2d>::Pack(
    OutputStream& stream, const prosdk::math::Twist2d& value) {
  wpi_proto_ProtobufTwist2d msg{
      .dx = value.dx.value(),
      .dy = value.dy.value(),
      .dtheta = value.dtheta.value(),
  };
  return stream.Encode(msg);
}
