// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/struct/DifferentialDriveFeedforwardStruct.hpp"

namespace {
constexpr size_t KV_LINEAR_OFF = 0;
constexpr size_t KA_LINEAR_OFF = KV_LINEAR_OFF + 8;
constexpr size_t KV_ANGULAR_OFF = KA_LINEAR_OFF + 8;
constexpr size_t KA_ANGULAR_OFF = KV_ANGULAR_OFF + 8;
}  // namespace

prosdk::math::DifferentialDriveFeedforward
prosdk::util::Struct<prosdk::math::DifferentialDriveFeedforward>::Unpack(
    std::span<const uint8_t> data) {
  return {decltype(1_V /
                   1_mps){prosdk::util::UnpackStruct<double, KV_LINEAR_OFF>(data)},
          decltype(1_V / 1_mps_sq){
              prosdk::util::UnpackStruct<double, KA_LINEAR_OFF>(data)},
          decltype(1_V / 1_mps){
              prosdk::util::UnpackStruct<double, KV_ANGULAR_OFF>(data)},
          decltype(1_V / 1_mps_sq){
              prosdk::util::UnpackStruct<double, KA_ANGULAR_OFF>(data)}};
}

void prosdk::util::Struct<prosdk::math::DifferentialDriveFeedforward>::Pack(
    std::span<uint8_t> data,
    const prosdk::math::DifferentialDriveFeedforward& value) {
  prosdk::util::PackStruct<KV_LINEAR_OFF>(data, value.kvLinear.value());
  prosdk::util::PackStruct<KA_LINEAR_OFF>(data, value.kaLinear.value());
  prosdk::util::PackStruct<KV_ANGULAR_OFF>(data, value.kvAngular.value());
  prosdk::util::PackStruct<KA_ANGULAR_OFF>(data, value.kaAngular.value());
}
