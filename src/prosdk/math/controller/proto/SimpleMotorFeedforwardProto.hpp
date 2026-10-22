// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/thirdparty/nanopb/pb.h"
#include "prosdk/math/controller/SimpleMotorFeedforward.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/generated/wpimath/protobuf/controller.npb.h"

// Everything is converted into units for
// prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> or
// prosdk::math::SimpleMotorFeedforward<prosdk::units::radians>

template <class Distance>
  requires prosdk::units::length_unit<Distance> ||
           prosdk::units::angle_unit<Distance> ||
           prosdk::units::dimensionless_unit<Distance>
struct prosdk::util::Protobuf<prosdk::math::SimpleMotorFeedforward<Distance>> {
  using MessageStruct = wpi_proto_ProtobufSimpleMotorFeedforward;
  using InputStream =
      prosdk::util::ProtoInputStream<prosdk::math::SimpleMotorFeedforward<Distance>>;
  using OutputStream =
      prosdk::util::ProtoOutputStream<prosdk::math::SimpleMotorFeedforward<Distance>>;

  static std::optional<prosdk::math::SimpleMotorFeedforward<Distance>> Unpack(
      InputStream& stream) {
    using BaseUnit =
        prosdk::units::unit<std::ratio<1>,
                         prosdk::units::traits::base_unit_of<Distance>>;
    using BaseFeedforward = prosdk::math::SimpleMotorFeedforward<BaseUnit>;
    wpi_proto_ProtobufSimpleMotorFeedforward msg;
    if (!stream.Decode(msg)) {
      return {};
    }

    return prosdk::math::SimpleMotorFeedforward<Distance>{
        prosdk::units::volt_t{msg.ks},
        prosdk::units::unit_t<typename BaseFeedforward::kv_unit>{msg.kv},
        prosdk::units::unit_t<typename BaseFeedforward::ka_unit>{msg.ka},
        prosdk::units::second_t{msg.dt},
    };
  }

  static bool Pack(OutputStream& stream,
                   const prosdk::math::SimpleMotorFeedforward<Distance>& value) {
    using BaseUnit =
        prosdk::units::unit<std::ratio<1>,
                         prosdk::units::traits::base_unit_of<Distance>>;
    using BaseFeedforward = prosdk::math::SimpleMotorFeedforward<BaseUnit>;
    wpi_proto_ProtobufSimpleMotorFeedforward msg{
        .ks = value.GetKs().value(),
        .kv =
            prosdk::units::unit_t<typename BaseFeedforward::kv_unit>{value.GetKv()}
                .value(),
        .ka =
            prosdk::units::unit_t<typename BaseFeedforward::ka_unit>{value.GetKa()}
                .value(),
        .dt = prosdk::units::second_t{value.GetDt()}.value(),
    };
    return stream.Encode(msg);
  }
};
