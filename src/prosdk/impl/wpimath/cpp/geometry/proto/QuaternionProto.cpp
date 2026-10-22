// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/QuaternionProto.hpp"

#include "prosdk/generated/wpimath/protobuf/geometry3d.npb.h"

std::optional<prosdk::math::Quaternion>
prosdk::util::Protobuf<prosdk::math::Quaternion>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufQuaternion msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::Quaternion{
      msg.w,
      msg.x,
      msg.y,
      msg.z,
  };
}

bool prosdk::util::Protobuf<prosdk::math::Quaternion>::Pack(
    OutputStream& stream, const prosdk::math::Quaternion& value) {
  wpi_proto_ProtobufQuaternion msg{
      .w = value.W(),
      .x = value.X(),
      .y = value.Y(),
      .z = value.Z(),
  };
  return stream.Encode(msg);
}
