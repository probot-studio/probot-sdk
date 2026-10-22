// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/ChassisAccelerationsProto.hpp"

#include "prosdk/generated/wpimath/protobuf/kinematics.npb.h"

std::optional<prosdk::math::ChassisAccelerations> prosdk::util::Protobuf<
    prosdk::math::ChassisAccelerations>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufChassisAccelerations msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::ChassisAccelerations{
      units::meters_per_second_squared_t{msg.ax},
      units::meters_per_second_squared_t{msg.ay},
      units::radians_per_second_squared_t{msg.alpha},
  };
}

bool prosdk::util::Protobuf<prosdk::math::ChassisAccelerations>::Pack(
    OutputStream& stream, const prosdk::math::ChassisAccelerations& value) {
  wpi_proto_ProtobufChassisAccelerations msg{
      .ax = value.ax.value(),
      .ay = value.ay.value(),
      .alpha = value.alpha.value(),
  };
  return stream.Encode(msg);
}
