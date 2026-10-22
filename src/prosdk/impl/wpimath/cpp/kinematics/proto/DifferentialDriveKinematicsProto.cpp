// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/DifferentialDriveKinematicsProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::DifferentialDriveKinematics> prosdk::util::Protobuf<
    prosdk::math::DifferentialDriveKinematics>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveKinematics msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialDriveKinematics{
      prosdk::units::meter_t{msg.trackwidth},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialDriveKinematics>::Pack(
    OutputStream& stream, const prosdk::math::DifferentialDriveKinematics& value) {
  wpi_proto_ProtobufDifferentialDriveKinematics msg{
      .trackwidth = value.trackwidth.value(),
  };
  return stream.Encode(msg);
}
