// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/kinematics/MecanumDriveWheelVelocities.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT
    prosdk::util::Struct<prosdk::math::MecanumDriveWheelVelocities> {
  static constexpr std::string_view GetTypeName() {
    return "MecanumDriveWheelVelocities";
  }
  static constexpr size_t GetSize() { return 32; }
  static constexpr std::string_view GetSchema() {
    return "double front_left;double front_right;double rear_left;"
           "double rear_right";
  }

  static prosdk::math::MecanumDriveWheelVelocities Unpack(
      std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::MecanumDriveWheelVelocities& value);
};

static_assert(
    prosdk::util::StructSerializable<prosdk::math::MecanumDriveWheelVelocities>);
