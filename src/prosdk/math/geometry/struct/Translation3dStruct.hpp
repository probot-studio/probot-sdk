// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/geometry/Translation3d.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::Translation3d> final {
  static constexpr std::string_view GetTypeName() { return "Translation3d"; }
  static constexpr size_t GetSize() { return 24; }
  static constexpr std::string_view GetSchema() {
    return "double x;double y;double z";
  }

  static prosdk::math::Translation3d Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::Translation3d& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Translation3d>);
