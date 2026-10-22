// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/proto/DCMotorProto.hpp"

#include <optional>

#include "prosdk/generated/wpimath/protobuf/system.npb.h"

std::optional<prosdk::math::DCMotor>
prosdk::util::Protobuf<prosdk::math::DCMotor>::Unpack(InputStream& stream) {
  wpi_proto_ProtobufDCMotor msg;
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DCMotor{
      prosdk::units::volt_t{msg.nominal_voltage},
      prosdk::units::newton_meter_t{msg.stall_torque},
      prosdk::units::ampere_t{msg.stall_current},
      prosdk::units::ampere_t{msg.free_current},
      prosdk::units::radians_per_second_t{msg.free_speed},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DCMotor>::Pack(
    OutputStream& stream, const prosdk::math::DCMotor& value) {
  wpi_proto_ProtobufDCMotor msg{
      .nominal_voltage = value.nominalVoltage.value(),
      .stall_torque = value.stallTorque.value(),
      .stall_current = value.stallCurrent.value(),
      .free_current = value.freeCurrent.value(),
      .free_speed = value.freeSpeed.value(),
  };
  return stream.Encode(msg);
}
