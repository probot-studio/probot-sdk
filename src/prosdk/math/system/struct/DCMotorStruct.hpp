// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/system/DCMotor.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::DCMotor> {
  static constexpr std::string_view GetTypeName() { return "DCMotor"; }
  static constexpr size_t GetSize() { return 40; }
  static constexpr std::string_view GetSchema() {
    return "double nominal_voltage;double stall_torque;double "
           "stall_current;double "
           "free_current;double free_speed";
  }

  static prosdk::math::DCMotor Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const prosdk::math::DCMotor& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::math::DCMotor>);
