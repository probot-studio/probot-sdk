// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/spline/proto/CubicHermiteSplineProto.hpp"

#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/spline.npb.h"

std::optional<prosdk::math::CubicHermiteSpline> prosdk::util::Protobuf<
    prosdk::math::CubicHermiteSpline>::Unpack(InputStream& stream) {
  prosdk::util::WpiArrayUnpackCallback<double, 2> xInitial;
  prosdk::util::WpiArrayUnpackCallback<double, 2> xFinal;
  prosdk::util::WpiArrayUnpackCallback<double, 2> yInitial;
  prosdk::util::WpiArrayUnpackCallback<double, 2> yFinal;
  wpi_proto_ProtobufCubicHermiteSpline msg{
      .x_initial = xInitial.Callback(),
      .x_final = xFinal.Callback(),
      .y_initial = yInitial.Callback(),
      .y_final = yFinal.Callback(),
  };
  if (!stream.Decode(msg)) {
    return {};
  }

  if (!xInitial.IsFull() || !yInitial.IsFull() || !xFinal.IsFull() ||
      !yFinal.IsFull()) {
    return {};
  }

  return prosdk::math::CubicHermiteSpline{
      xInitial.Array(),
      xFinal.Array(),
      yInitial.Array(),
      yFinal.Array(),
  };
}

bool prosdk::util::Protobuf<prosdk::math::CubicHermiteSpline>::Pack(
    OutputStream& stream, const prosdk::math::CubicHermiteSpline& value) {
  prosdk::util::PackCallback<double> xInitial{value.GetInitialControlVector().x};
  prosdk::util::PackCallback<double> xFinal{value.GetFinalControlVector().x};
  prosdk::util::PackCallback<double> yInitial{value.GetInitialControlVector().y};
  prosdk::util::PackCallback<double> yFinal{value.GetFinalControlVector().y};
  wpi_proto_ProtobufCubicHermiteSpline msg{
      .x_initial = xInitial.Callback(),
      .x_final = xFinal.Callback(),
      .y_initial = yInitial.Callback(),
      .y_final = yFinal.Callback(),
  };
  return stream.Encode(msg);
}
