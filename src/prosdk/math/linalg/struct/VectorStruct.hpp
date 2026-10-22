// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/util/ct_string.hpp"
#include "prosdk/util/struct/Struct.hpp"

template <int Size, int Options, int MaxRows, int MaxCols>
struct prosdk::util::Struct<
    prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>> {
  static constexpr ct_string TYPE_NAME =
      prosdk::util::Concat("Vector__"_ct_string, prosdk::util::NumToCtString<Size>());
  static constexpr std::string_view GetTypeName() { return TYPE_NAME; }
  static constexpr size_t GetSize() { return Size * 8; }
  static constexpr ct_string SCHEMA =
      prosdk::util::Concat("double data["_ct_string,
                        prosdk::util::NumToCtString<Size>(), "]"_ct_string);
  static constexpr std::string_view GetSchema() { return SCHEMA; }

  static prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols> Unpack(
      std::span<const uint8_t> data) {
    constexpr size_t DATA_OFF = 0;
    prosdk::util::array<double, Size> vec_data =
        prosdk::util::UnpackStructArray<double, DATA_OFF, Size>(data);
    prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols> vec;
    for (int i = 0; i < Size; i++) {
      vec(i) = vec_data[i];
    }
    return vec;
  }

  static void Pack(
      std::span<uint8_t> data,
      const prosdk::math::Matrixd<Size, 1, Options, MaxRows, MaxCols>& value) {
    constexpr size_t DATA_OFF = 0;
    prosdk::util::array<double, Size> vec_data(prosdk::util::empty_array);
    for (int i = 0; i < Size; i++) {
      vec_data[i] = value(i);
    }
    prosdk::util::PackStructArray<DATA_OFF, Size>(data, vec_data);
  }
};

static_assert(prosdk::util::StructSerializable<prosdk::math::Vectord<1>>);
static_assert(prosdk::util::StructSerializable<prosdk::math::Vectord<3>>);
