// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/struct/ElevatorFeedforwardStruct.hpp"

namespace {
constexpr size_t KS_OFF = 0;
constexpr size_t KG_OFF = KS_OFF + 8;
constexpr size_t KV_OFF = KG_OFF + 8;
constexpr size_t KA_OFF = KV_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::ElevatorFeedforward>;

prosdk::math::ElevatorFeedforward StructType::Unpack(
    std::span<const uint8_t> data) {
  return prosdk::math::ElevatorFeedforward{
      prosdk::units::volt_t{prosdk::util::UnpackStruct<double, KS_OFF>(data)},
      prosdk::units::volt_t{prosdk::util::UnpackStruct<double, KG_OFF>(data)},
      prosdk::units::unit_t<prosdk::math::ElevatorFeedforward::kv_unit>{
          prosdk::util::UnpackStruct<double, KV_OFF>(data)},
      prosdk::units::unit_t<prosdk::math::ElevatorFeedforward::ka_unit>{
          prosdk::util::UnpackStruct<double, KA_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::ElevatorFeedforward& value) {
  prosdk::util::PackStruct<KS_OFF>(data, value.GetKs().value());
  prosdk::util::PackStruct<KG_OFF>(data, value.GetKg().value());
  prosdk::util::PackStruct<KV_OFF>(data, value.GetKv().value());
  prosdk::util::PackStruct<KA_OFF>(data, value.GetKa().value());
}
