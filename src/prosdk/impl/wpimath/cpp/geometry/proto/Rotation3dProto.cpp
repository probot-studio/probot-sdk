// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/proto/Rotation3dProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/geometry3d.npb.h"

std::optional<prosdk::math::Rotation3d>
prosdk::util::Protobuf<prosdk::math::Rotation3d>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Quaternion> quat;
  wpi_proto_ProtobufRotation3d msg{
      .q = quat.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto iquat = quat.Items();

  if (iquat.empty()) {
    return {};
  }

  return prosdk::math::Rotation3d{
      iquat[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::Rotation3d>::Pack(
    OutputStream& stream, const prosdk::math::Rotation3d& value) {
  prosdk::util::PackCallback quat{&value.GetQuaternion()};
  wpi_proto_ProtobufRotation3d msg{
      .q = quat.Callback(),
  };
  return stream.Encode(msg);
}
