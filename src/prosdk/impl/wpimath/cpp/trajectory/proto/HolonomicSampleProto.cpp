// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/proto/HolonomicSampleProto.hpp"

#include "prosdk/math/geometry/proto/Pose2dProto.hpp"
#include "prosdk/math/kinematics/proto/ChassisAccelerationsProto.hpp"
#include "prosdk/math/kinematics/proto/ChassisVelocitiesProto.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/trajectory.npb.h"

std::optional<prosdk::math::HolonomicSample>
prosdk::util::Protobuf<prosdk::math::HolonomicSample>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Pose2d> pose;
  prosdk::util::UnpackCallback<prosdk::math::ChassisVelocities> velocity;
  prosdk::util::UnpackCallback<prosdk::math::ChassisAccelerations> acceleration;
  wpi_proto_ProtobufHolonomicSample msg{
      .time = 0,
      .pose = pose.Callback(),
      .velocities = velocity.Callback(),
      .accelerations = acceleration.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  auto iPose = pose.Items();
  auto iVel = velocity.Items();
  auto iAccel = acceleration.Items();

  if (iPose.empty() || iVel.empty() || iAccel.empty()) {
    return {};
  }

  return prosdk::math::HolonomicSample{
      prosdk::units::second_t{msg.time},
      iPose[0],
      iVel[0],
      iAccel[0],
  };
}

bool prosdk::util::Protobuf<prosdk::math::HolonomicSample>::Pack(
    OutputStream& stream, const prosdk::math::HolonomicSample& value) {
  prosdk::util::PackCallback pose{&value.pose};
  prosdk::util::PackCallback velocity{&value.velocity};
  prosdk::util::PackCallback acceleration{&value.acceleration};
  wpi_proto_ProtobufHolonomicSample msg{
      .time = value.time.value(),
      .pose = pose.Callback(),
      .velocities = velocity.Callback(),
      .accelerations = acceleration.Callback(),
  };
  return stream.Encode(msg);
}
