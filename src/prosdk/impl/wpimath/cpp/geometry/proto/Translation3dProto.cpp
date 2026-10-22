// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/Translation3dProto.hpp"

#include "prosdk/generated/wpimath/protobuf/geometry3d.npb.h"

std::optional<prosdk::math::Translation3d>
prosdk::util::Protobuf<prosdk::math::Translation3d>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufTranslation3d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::Translation3d{
      prosdk::units::meter_t{msg.x},
      prosdk::units::meter_t{msg.y},
      prosdk::units::meter_t{msg.z},
  };
}

bool prosdk::util::Protobuf<prosdk::math::Translation3d>::Pack(
    OutputStream& stream, const prosdk::math::Translation3d& value) {
  wpi_proto_ProtobufTranslation3d msg{
      .x = value.X().value(),
      .y = value.Y().value(),
      .z = value.Z().value(),
  };
  return stream.Encode(msg);
}
