// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/proto/DifferentialSampleProto.hpp"

#include "prosdk/math/geometry/proto/Pose2dProto.hpp"
#include "prosdk/math/kinematics/proto/ChassisAccelerationsProto.hpp"
#include "prosdk/math/kinematics/proto/ChassisVelocitiesProto.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/trajectory.npb.h"

std::optional<prosdk::math::DifferentialSample> prosdk::util::Protobuf<
    prosdk::math::DifferentialSample>::Unpack(InputStream& stream) {
  prosdk::util::UnpackCallback<prosdk::math::Pose2d> pose;
  prosdk::util::UnpackCallback<prosdk::math::ChassisVelocities> velocity;
  prosdk::util::UnpackCallback<prosdk::math::ChassisAccelerations> acceleration;
  wpi_proto_ProtobufDifferentialSample msg{
      .time = 0,
      .pose = pose.Callback(),
      .velocities = velocity.Callback(),
      .accelerations = acceleration.Callback(),
      .left_velocity = 0,
      .right_velocity = 0,
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

  return prosdk::math::DifferentialSample{
      prosdk::units::second_t{msg.time},
      iPose[0],
      iVel[0],
      iAccel[0],
      prosdk::units::meters_per_second_t{msg.left_velocity},
      prosdk::units::meters_per_second_t{msg.right_velocity},
  };
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialSample>::Pack(
    OutputStream& stream, const prosdk::math::DifferentialSample& value) {
  prosdk::util::PackCallback pose{&value.pose};
  prosdk::util::PackCallback velocity{&value.velocity};
  prosdk::util::PackCallback acceleration{&value.acceleration};
  wpi_proto_ProtobufDifferentialSample msg{
      .time = value.time.value(),
      .pose = pose.Callback(),
      .velocities = velocity.Callback(),
      .accelerations = acceleration.Callback(),
      .left_velocity = value.leftVelocity.value(),
      .right_velocity = value.rightVelocity.value(),
  };
  return stream.Encode(msg);
}
