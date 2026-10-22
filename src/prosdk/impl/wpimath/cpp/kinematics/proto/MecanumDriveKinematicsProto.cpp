// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/MecanumDriveKinematicsProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::MecanumDriveKinematics> prosdk::util::Protobuf<
    prosdk::math::MecanumDriveKinematics>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Translation2d> frontLeft;
  prosdk::util::UnpackCallback<prosdk::math::Translation2d> frontRight;
  prosdk::util::UnpackCallback<prosdk::math::Translation2d> rearLeft;
  prosdk::util::UnpackCallback<prosdk::math::Translation2d> rearRight;
  wpi_proto_ProtobufMecanumDriveKinematics msg{
      .front_left = frontLeft.Callback(),
      .front_right = frontRight.Callback(),
      .rear_left = rearLeft.Callback(),
      .rear_right = rearRight.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto ifrontLeft = frontLeft.Items();
  auto ifrontRight = frontRight.Items();
  auto irearLeft = rearLeft.Items();
  auto irearRight = rearRight.Items();

  if (ifrontLeft.empty() || ifrontRight.empty() || irearLeft.empty() ||
      irearRight.empty()) {
    return {};
  }

  return prosdk::math::MecanumDriveKinematics{
      ifrontLeft[0],
      ifrontRight[0],
      irearLeft[0],
      irearRight[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::MecanumDriveKinematics>::Pack(
    OutputStream& stream, const prosdk::math::MecanumDriveKinematics& value) {
  prosdk::util::PackCallback frontLeft{&value.GetFrontLeft()};
  prosdk::util::PackCallback frontRight{&value.GetFrontRight()};
  prosdk::util::PackCallback rearLeft{&value.GetRearLeft()};
  prosdk::util::PackCallback rearRight{&value.GetRearRight()};
  wpi_proto_ProtobufMecanumDriveKinematics msg{
      .front_left = frontLeft.Callback(),
      .front_right = frontRight.Callback(),
      .rear_left = rearLeft.Callback(),
      .rear_right = rearRight.Callback(),
  };
  return stream.Encode(msg);
}
