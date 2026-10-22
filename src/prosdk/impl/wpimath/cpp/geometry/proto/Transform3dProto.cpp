// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/Transform3dProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/geometry3d.npb.h"

std::optional<prosdk::math::Transform3d>
prosdk::util::Protobuf<prosdk::math::Transform3d>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Translation3d> tsln;
  prosdk::util::UnpackCallback<prosdk::math::Rotation3d> rot;
  wpi_proto_ProtobufTransform3d msg{
      .translation = tsln.Callback(),
      .rotation = rot.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto itsln = tsln.Items();
  auto irot = rot.Items();

  if (itsln.empty() || irot.empty()) {
    return {};
  }

  return prosdk::math::Transform3d{
      itsln[0],
      irot[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::Transform3d>::Pack(
    OutputStream& stream, const prosdk::math::Transform3d& value) {
  prosdk::util::PackCallback tsln{&value.Translation()};
  prosdk::util::PackCallback rot{&value.Rotation()};
  wpi_proto_ProtobufTransform3d msg{
      .translation = tsln.Callback(),
      .rotation = rot.Callback(),
  };
  return stream.Encode(msg);
}
