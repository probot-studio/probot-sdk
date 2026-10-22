// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/shape/proto/Rectangle2dProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/shape.npb.h"

std::optional<prosdk::math::Rectangle2d>
prosdk::util::Protobuf<prosdk::math::Rectangle2d>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Pose2d> pose;
  wpi_proto_ProtobufRectangle2d msg{
      .center = pose.Callback(),
      .xWidth = 0,
      .yWidth = 0,
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto ipose = pose.Items();

  if (ipose.empty()) {
    return {};
  }

  return prosdk::math::Rectangle2d{
      ipose[0],
      prosdk::units::meter_t{msg.xWidth},
      prosdk::units::meter_t{msg.yWidth},
  };
}

bool prosdk::util::Protobuf<prosdk::math::Rectangle2d>::Pack(
    OutputStream& stream, const prosdk::math::Rectangle2d& value) {
  prosdk::util::PackCallback pose{&value.Center()};
  wpi_proto_ProtobufRectangle2d msg{
      .center = pose.Callback(),
      .xWidth = value.XWidth().value(),
      .yWidth = value.YWidth().value(),
  };
  return stream.Encode(msg);
}
