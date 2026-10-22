// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/proto/DifferentialDriveFeedforwardProto.hpp"

#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

std::optional<prosdk::math::DifferentialDriveFeedforward> prosdk::util::Protobuf<
    prosdk::math::DifferentialDriveFeedforward>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDifferentialDriveFeedforward msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialDriveFeedforward{
      decltype(1_V / 1_mps){msg.kvLinear},
      decltype(1_V / 1_mps_sq){msg.kaLinear},
      decltype(1_V / 1_mps){msg.kvAngular},
      decltype(1_V / 1_mps_sq){msg.kaAngular},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialDriveFeedforward>::Pack(
    OutputStream& stream,
    const prosdk::math::DifferentialDriveFeedforward& value) {
  wpi_proto_ProtobufDifferentialDriveFeedforward msg{
      .kvLinear = value.kvLinear.value(),
      .kaLinear = value.kaLinear.value(),
      .kvAngular = value.kvAngular.value(),
      .kaAngular = value.kaAngular.value(),
  };
  return stream.Encode(msg);
}
