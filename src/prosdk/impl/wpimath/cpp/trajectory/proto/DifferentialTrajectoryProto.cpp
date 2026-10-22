// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/proto/DifferentialTrajectoryProto.hpp"

#include <utility>

#include "prosdk/math/trajectory/proto/DifferentialSampleProto.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/trajectory.npb.h"

std::optional<prosdk::math::DifferentialTrajectory> prosdk::util::Protobuf<
    prosdk::math::DifferentialTrajectory>::Unpack(InputStream& stream) {
  prosdk::util::StdVectorUnpackCallback<prosdk::math::DifferentialSample, SIZE_MAX>
      samples;
  wpi_proto_ProtobufDifferentialTrajectory msg{
      .samples = samples.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  return prosdk::math::DifferentialTrajectory{std::move(samples.Vec())};
}

bool prosdk::util::Protobuf<prosdk::math::DifferentialTrajectory>::Pack(
    OutputStream& stream, const prosdk::math::DifferentialTrajectory& value) {
  prosdk::util::PackCallback<prosdk::math::DifferentialSample> samples{
      value.Samples()};
  wpi_proto_ProtobufDifferentialTrajectory msg{
      .samples = samples.Callback(),
  };
  return stream.Encode(msg);
}
