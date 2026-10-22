// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/Twist3dProto.hpp"

#include "prosdk/generated/wpimath/protobuf/geometry3d.npb.h"

std::optional<prosdk::math::Twist3d>
prosdk::util::Protobuf<prosdk::math::Twist3d>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufTwist3d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::Twist3d{
      prosdk::units::meter_t{msg.dx},  prosdk::units::meter_t{msg.dy},
      prosdk::units::meter_t{msg.dz},  prosdk::units::radian_t{msg.rx},
      prosdk::units::radian_t{msg.ry}, prosdk::units::radian_t{msg.rz},
  };
}

bool prosdk::util::Protobuf<prosdk::math::Twist3d>::Pack(
    OutputStream& stream, const prosdk::math::Twist3d& value) {
  wpi_proto_ProtobufTwist3d msg{
      .dx = value.dx.value(),
      .dy = value.dy.value(),
      .dz = value.dz.value(),
      .rx = value.rx.value(),
      .ry = value.ry.value(),
      .rz = value.rz.value(),
  };
  return stream.Encode(msg);
}
