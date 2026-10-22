// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/wpimath.npb.h"

template <int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  requires(Cols != 1)
struct prosdk::util::Protobuf<
    prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>> {
  using MessageStruct = wpi_proto_ProtobufMatrix;
  using InputStream = prosdk::util::ProtoInputStream<
      prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>>;
  using OutputStream = prosdk::util::ProtoOutputStream<
      prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>>;

  static std::optional<
      prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>>
  Unpack(InputStream& stream) {
    constexpr bool isSmall = Rows * Cols * sizeof(double) < 256;
    using UnpackType = std::conditional_t<
        isSmall, prosdk::util::UnpackCallback<double, Rows * Cols>,
        prosdk::util::StdVectorUnpackCallback<double, Rows * Cols>>;
    UnpackType data;
    data.Vec().reserve(Rows * Cols);
    data.SetLimits(prosdk::util::DecodeLimits::Fail);
    wpi_proto_ProtobufMatrix msg;
    msg.data = data.Callback();
    if (!stream.Decode(msg)) {
      return {};
    }

    if (msg.num_rows != Rows || msg.num_cols != Cols) {
      return {};
    }

    auto items = data.Items();

    if (items.size() != Rows * Cols) {
      return {};
    }

    prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols> mat;
    for (int i = 0; i < Rows * Cols; i++) {
      mat(i) = items[i];
    }

    return mat;
  }

  static bool Pack(
      OutputStream& stream,
      const prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>& value) {
    std::span<const double> dataSpan{value.data(),
                                     static_cast<size_t>(Rows * Cols)};
    prosdk::util::PackCallback<double> data{dataSpan};
    wpi_proto_ProtobufMatrix msg{
        .num_rows = Rows,
        .num_cols = Cols,
        .data = data.Callback(),
    };
    return stream.Encode(msg);
  }
};
