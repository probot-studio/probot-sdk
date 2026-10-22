// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/proto/HolonomicTrajectoryProto.hpp"

#include <utility>

#include "prosdk/math/trajectory/proto/HolonomicSampleProto.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/trajectory.npb.h"

std::optional<prosdk::math::HolonomicTrajectory> prosdk::util::Protobuf<
    prosdk::math::HolonomicTrajectory>::Unpack(InputStream& stream) {
  prosdk::util::StdVectorUnpackCallback<prosdk::math::HolonomicSample, SIZE_MAX>
      samples;
  wpi_proto_ProtobufHolonomicTrajectory msg{
      .samples = samples.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::HolonomicTrajectory{std::move(samples.Vec())};
}

bool prosdk::util::Protobuf<prosdk::math::HolonomicTrajectory>::Pack(
    OutputStream& stream, const prosdk::math::HolonomicTrajectory& value) {
  prosdk::util::PackCallback<prosdk::math::HolonomicSample> samples{value.Samples()};
  wpi_proto_ProtobufHolonomicTrajectory msg{
      .samples = samples.Callback(),
  };
  return stream.Encode(msg);
}
