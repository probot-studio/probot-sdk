// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::cxx::byteswap: C++23 std::byteswap ([bit.byteswap]) for C++20.

#pragma once

#include <bit>
#include <concepts>
#include <cstdint>

namespace prosdk::cxx {

template <std::integral T>
constexpr T byteswap(T value) noexcept {
  if constexpr (sizeof(T) == 1) {
    return value;
  } else if constexpr (sizeof(T) == 2) {
    return static_cast<T>(__builtin_bswap16(static_cast<std::uint16_t>(value)));
  } else if constexpr (sizeof(T) == 4) {
    return static_cast<T>(__builtin_bswap32(static_cast<std::uint32_t>(value)));
  } else if constexpr (sizeof(T) == 8) {
    return static_cast<T>(__builtin_bswap64(static_cast<std::uint64_t>(value)));
  } else {
    static_assert(sizeof(T) <= 8, "byteswap: unsupported integer size");
  }
}

}  // namespace prosdk::cxx
