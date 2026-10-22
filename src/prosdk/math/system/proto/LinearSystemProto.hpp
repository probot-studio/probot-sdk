// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include "prosdk/math/linalg/proto/MatrixProto.hpp"
#include "prosdk/math/system/LinearSystem.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/protobuf/ProtobufCallbacks.hpp"
#include "prosdk/generated/wpimath/protobuf/system.npb.h"

template <int States, int Inputs, int Outputs>
struct prosdk::util::Protobuf<prosdk::math::LinearSystem<States, Inputs, Outputs>> {
  using MessageStruct = wpi_proto_ProtobufLinearSystem;
  using InputStream = prosdk::util::ProtoInputStream<
      prosdk::math::LinearSystem<States, Inputs, Outputs>>;
  using OutputStream = prosdk::util::ProtoOutputStream<
      prosdk::math::LinearSystem<States, Inputs, Outputs>>;

  static std::optional<prosdk::math::LinearSystem<States, Inputs, Outputs>> Unpack(
      InputStream& stream) {
    prosdk::util::UnpackCallback<prosdk::math::Matrixd<States, States>> a;
    a.SetLimits(prosdk::util::DecodeLimits::Fail);
    prosdk::util::UnpackCallback<prosdk::math::Matrixd<States, Inputs>> b;
    a.SetLimits(prosdk::util::DecodeLimits::Fail);
    prosdk::util::UnpackCallback<prosdk::math::Matrixd<Outputs, States>> c;
    a.SetLimits(prosdk::util::DecodeLimits::Fail);
    prosdk::util::UnpackCallback<prosdk::math::Matrixd<Outputs, Inputs>> d;
    a.SetLimits(prosdk::util::DecodeLimits::Fail);

    wpi_proto_ProtobufLinearSystem msg;
    msg.a = a.Callback();
    msg.b = b.Callback();
    msg.c = c.Callback();
    msg.d = d.Callback();

    if (!stream.Decode(msg)) {
      return {};
    }

    if (msg.num_inputs != Inputs || msg.num_outputs != Outputs ||
        msg.num_states != States) {
      return {};
    }

    auto ai = a.Items();
    auto bi = b.Items();
    auto ci = c.Items();
    auto di = d.Items();

    if (ai.empty() || bi.empty() || ci.empty() || di.empty()) {
      return {};
    }

    return prosdk::math::LinearSystem<States, Inputs, Outputs>{
        std::move(ai[0]),
        std::move(bi[0]),
        std::move(ci[0]),
        std::move(di[0]),
    };
  }

  static bool Pack(
      OutputStream& stream,
      const prosdk::math::LinearSystem<States, Inputs, Outputs>& value) {
    prosdk::util::PackCallback a{&value.A()};
    prosdk::util::PackCallback b{&value.B()};
    prosdk::util::PackCallback c{&value.C()};
    prosdk::util::PackCallback d{&value.D()};

    wpi_proto_ProtobufLinearSystem msg{
        .num_states = States,
        .num_inputs = Inputs,
        .num_outputs = Outputs,
        .a = a.Callback(),
        .b = b.Callback(),
        .c = c.Callback(),
        .d = d.Callback(),
    };

    return stream.Encode(msg);
  }
};
