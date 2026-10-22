// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/util/ct_string.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  requires(Cols != 1)
struct prosdk::util::Struct<
    prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>> {
  static constexpr ct_string TYPE_NAME =
      prosdk::util::Concat("Matrix__"_ct_string, prosdk::util::NumToCtString<Rows>(),
                        "_"_ct_string, prosdk::util::NumToCtString<Cols>());
  static constexpr std::string_view GetTypeName() { return TYPE_NAME; }
  static constexpr size_t GetSize() { return Rows * Cols * 8; }
  static constexpr ct_string SCHEMA =
      prosdk::util::Concat("double data["_ct_string,
                        prosdk::util::NumToCtString<Rows * Cols>(), "]"_ct_string);
  static constexpr std::string_view GetSchema() { return SCHEMA; }

  static prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols> Unpack(
      std::span<const uint8_t> data) {
    constexpr size_t DATA_OFF = 0;
    prosdk::util::array<double, Rows * Cols> mat_data =
        prosdk::util::UnpackStructArray<double, DATA_OFF, Rows * Cols>(data);
    prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols> mat;
    for (int i = 0; i < Rows * Cols; i++) {
      mat(i) = mat_data[i];
    }
    return mat;
  }

  static void Pack(
      std::span<uint8_t> data,
      const prosdk::math::Matrixd<Rows, Cols, Options, MaxRows, MaxCols>& value) {
    constexpr size_t DATA_OFF = 0;
    prosdk::util::array<double, Rows * Cols> mat_data(prosdk::util::empty_array);
    for (int i = 0; i < Rows * Cols; i++) {
      mat_data[i] = value(i);
    }
    prosdk::util::PackStructArray<DATA_OFF, Rows * Cols>(data, mat_data);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Matrixd<1, 2>>);
static_assert(prosdk::util::StructSerializable<prosdk::math::Matrixd<3, 3>>);
