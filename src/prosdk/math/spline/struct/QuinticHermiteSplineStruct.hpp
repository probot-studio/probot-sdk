// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/spline/QuinticHermiteSpline.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <>
struct WPILIB_DLLEXPORT prosdk::util::Struct<prosdk::math::QuinticHermiteSpline> {
  static constexpr std::string_view GetTypeName() {
    return "QuinticHermiteSpline";
  }
  static constexpr size_t GetSize() { return 4 * 3 * 8; }
  static constexpr std::string_view GetSchema() {
    return "double xInitial[3];double xFinal[3];double yInitial[3];double "
           "yFinal[3]";
  }

  static prosdk::math::QuinticHermiteSpline Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::math::QuinticHermiteSpline& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::math::QuinticHermiteSpline>);
