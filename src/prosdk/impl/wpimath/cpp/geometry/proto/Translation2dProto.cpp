// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/Translation2dProto.hpp"

#include "prosdk/generated/wpimath/protobuf/geometry2d.npb.h"

std::optional<prosdk::math::Translation2d>
prosdk::util::Protobuf<prosdk::math::Translation2d>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufTranslation2d msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::Translation2d{
      prosdk::units::meter_t{msg.x},
      prosdk::units::meter_t{msg.y},
  };
}

bool prosdk::util::Protobuf<prosdk::math::Translation2d>::Pack(
    OutputStream& stream, const prosdk::math::Translation2d& value) {
  wpi_proto_ProtobufTranslation2d msg{
      .x = value.X().value(),
      .y = value.Y().value(),
  };
  return stream.Encode(msg);
}
