// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdio>
#include <prosdk/cxx/print.hpp>
#include <system_error>
#include <utility>

namespace prosdk::util {

/**
 * Wrapper around std::print() that squelches write failure exceptions.
 */
template <typename... T>
inline void print(std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::print(fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

/**
 * Wrapper around std::print() that squelches write failure exceptions.
 */
template <typename... T>
inline void print(std::FILE* f, std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::print(f, fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

/**
 * Wrapper around std::println() that squelches write failure exceptions.
 */
template <typename... T>
inline void println(std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::println(fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

/**
 * Wrapper around std::println() that squelches write failure exceptions.
 */
template <typename... T>
inline void println(std::FILE* f, std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::println(f, fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

}  // namespace prosdk::util
