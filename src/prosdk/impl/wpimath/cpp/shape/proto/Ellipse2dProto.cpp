// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/shape/proto/Ellipse2dProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/shape.npb.h"

std::optional<prosdk::math::Ellipse2d>
prosdk::util::Protobuf<prosdk::math::Ellipse2d>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Pose2d> pose;
  wpi_proto_ProtobufEllipse2d msg{
      .center = pose.Callback(),
      .xSemiAxis = 0,
      .ySemiAxis = 0,
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto ipose = pose.Items();

  if (ipose.empty()) {
    return {};
  }

  return prosdk::math::Ellipse2d{
      ipose[0],
      prosdk::units::meter_t{msg.xSemiAxis},
      prosdk::units::meter_t{msg.ySemiAxis},
  };
}

bool prosdk::util::Protobuf<prosdk::math::Ellipse2d>::Pack(
    OutputStream& stream, const prosdk::math::Ellipse2d& value) {
  prosdk::util::PackCallback pose{&value.Center()};
  wpi_proto_ProtobufEllipse2d msg{
      .center = pose.Callback(),
      .xSemiAxis = value.XSemiAxis().value(),
      .ySemiAxis = value.YSemiAxis().value(),
  };
  return stream.Encode(msg);
}
