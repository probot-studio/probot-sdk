// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/proto/ElevatorFeedforwardProto.hpp"

#include <optional>

#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

std::optional<prosdk::math::ElevatorFeedforward> prosdk::util::Protobuf<
    prosdk::math::ElevatorFeedforward>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufElevatorFeedforward msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::ElevatorFeedforward{
      prosdk::units::volt_t{msg.ks},
      prosdk::units::volt_t{msg.kg},
      prosdk::units::unit_t<prosdk::math::ElevatorFeedforward::kv_unit>{msg.kv},
      prosdk::units::unit_t<prosdk::math::ElevatorFeedforward::ka_unit>{msg.ka},
  };
}

bool prosdk::util::Protobuf<prosdk::math::ElevatorFeedforward>::Pack(
    OutputStream& stream, const prosdk::math::ElevatorFeedforward& value) {
  wpi_proto_ProtobufElevatorFeedforward msg{
      .ks = value.GetKs().value(),
      .kg = value.GetKg().value(),
      .kv = value.GetKv().value(),
      .ka = value.GetKa().value(),
      .dt = 0,
  };
  return stream.Encode(msg);
}
