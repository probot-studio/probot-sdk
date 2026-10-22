// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/trajectory/DifferentialSample.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::DifferentialSample> {
  static constexpr std::string_view GetTypeName() {
    return "DifferentialSample";
  }
  static constexpr size_t GetSize() {
    return prosdk::util::GetStructSize<prosdk::math::HolonomicSample>() + 8 + 8;
  }
  static constexpr std::string_view GetSchema() {
    return "double time;Pose2d pose;ChassisVelocities velocity;"
           "ChassisAccelerations acceleration;double leftVelocity;"
           "double rightVelocity";
  }

  static prosdk::math::DifferentialSample Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::DifferentialSample& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::HolonomicSample>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::DifferentialSample>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::DifferentialSample>);
