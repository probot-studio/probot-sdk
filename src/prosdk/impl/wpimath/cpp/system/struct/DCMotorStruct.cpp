// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/system/struct/DCMotorStruct.hpp"

namespace {
constexpr size_t NOMINAL_VOLTAGE_OFF = 0;
constexpr size_t STALL_TORQUE_OFF = NOMINAL_VOLTAGE_OFF + 8;
constexpr size_t STALL_CURRENT_OFF = STALL_TORQUE_OFF + 8;
constexpr size_t FREE_CURRENT_OFF = STALL_CURRENT_OFF + 8;
constexpr size_t FREE_SPEED_OFF = FREE_CURRENT_OFF + 8;
}  // namespace

using StructType = prosdk::util::Struct<prosdk::math::DCMotor>;

prosdk::math::DCMotor StructType::Unpack(std::span<const uint8_t> data) {
  return prosdk::math::DCMotor{
      prosdk::units::volt_t{
          prosdk::util::UnpackStruct<double, NOMINAL_VOLTAGE_OFF>(data)},
      prosdk::units::newton_meter_t{
          prosdk::util::UnpackStruct<double, STALL_TORQUE_OFF>(data)},
      prosdk::units::ampere_t{
          prosdk::util::UnpackStruct<double, STALL_CURRENT_OFF>(data)},
      prosdk::units::ampere_t{
          prosdk::util::UnpackStruct<double, FREE_CURRENT_OFF>(data)},
      prosdk::units::radians_per_second_t{
          prosdk::util::UnpackStruct<double, FREE_SPEED_OFF>(data)},
  };
}

void StructType::Pack(std::span<uint8_t> data,
                      const prosdk::math::DCMotor& value) {
  prosdk::util::PackStruct<NOMINAL_VOLTAGE_OFF>(data,
                                             value.nominalVoltage.value());
  prosdk::util::PackStruct<STALL_TORQUE_OFF>(data, value.stallTorque.value());
  prosdk::util::PackStruct<STALL_CURRENT_OFF>(data, value.stallCurrent.value());
  prosdk::util::PackStruct<FREE_CURRENT_OFF>(data, value.freeCurrent.value());
  prosdk::util::PackStruct<FREE_SPEED_OFF>(data, value.freeSpeed.value());
}
