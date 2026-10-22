// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/shape/Ellipse2d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::Ellipse2d> final {
  static constexpr std::string_view GetTypeName() { return "Ellipse2d"; }
  static constexpr size_t GetSize() {
    return prosdk::util::GetStructSize<prosdk::math::Pose2d>() + 16;
  }
  static constexpr std::string_view GetSchema() {
    return "Pose2d center;double xSemiAxis;double ySemiAxis";
  }

  static prosdk::math::Ellipse2d Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const prosdk::math::Ellipse2d& value);
  static void ForEachNested(
      std::invocable<std::string_view, std::string_view> auto fn) {
    prosdk::util::ForEachStructSchema<prosdk::math::Pose2d>(fn);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Ellipse2d>);
static_assert(prosdk::util::HasNestedStruct<prosdk::math::Ellipse2d>);
