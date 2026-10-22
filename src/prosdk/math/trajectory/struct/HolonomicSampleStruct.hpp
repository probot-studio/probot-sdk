// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::HolonomicSample> {
  static constexpr std::string_view GetTypeName() { return "HolonomicSample"; }
  static constexpr size_t GetSize() {
    return sizeof(double) + prosdk::util::GetStructSize<prosdk::math::Pose2d>() +
           prosdk::util::GetStructSize<prosdk::math::ChassisVelocities>() +
           prosdk::util::GetStructSize<prosdk::math::ChassisAccelerations>();
  }
  static constexpr std::string_view GetSchema() {
    return "double time;Pose2d pose;ChassisVelocities velocity;"
           "ChassisAccelerations acceleration";
  }

  static prosdk::math::HolonomicSample Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::HolonomicSample& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Pose2d>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::ChassisVelocities>(fn);
    prosdk::util::ForEachStructSchema<prosdk::math::ChassisAccelerations>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::HolonomicSample>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::HolonomicSample>);
