// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/wpimath.npb.h"

template <int Size, int Options, int MaxRows, int MaxCols>
struct prosdk::util::Protobuf<
    prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>> {
  using MessageStruct = wpi_proto_ProtobufVector;
  using InputStream = prosdk::util::ProtoInputStream<
      prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>>;
  using OutputStream = prosdk::util::ProtoOutputStream<
      prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>>;

  static std::optional<prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>>
  Unpack(InputStream& stream) {
    constexpr bool isSmall = Size * sizeof(double) < 256;
    using UnpackType =
        std::conditional_t<isSmall, prosdk::util::UnpackCallback<double, Size>,
                           prosdk::util::StdVectorUnpackCallback<double, Size>>;
    UnpackType rows;
    rows.Vec().reserve(Size);
    rows.SetLimits(prosdk::util::DecodeLimits::Fail);
    wpi_proto_ProtobufVector msg{
        .rows = rows.Callback(),
    };
    if (!stream.Decode(msg)) {
      return {};
    }

    auto items = rows.Items();

    if (items.size() != Size) {
      return {};
    }

    prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols> mat;
    for (int i = 0; i < Size; i++) {
      mat(i) = items[i];
    }

    return mat;
  }

  static bool Pack(
      OutputStream& stream,
      const prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>& value) {
    std::span<const double> rowsSpan{value.data(), static_cast<size_t>(Size)};
    prosdk::util::PackCallback<double> rows{rowsSpan};
    wpi_proto_ProtobufVector msg{
        .rows = rows.Callback(),
    };
    return stream.Encode(msg);
  }
};
