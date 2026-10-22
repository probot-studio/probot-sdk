// Copyright (c) Sleipnir contributors

#pragma once

#include <prosdk/platform/config.hpp>  // prosdk: configuration

#ifndef SLEIPNIR_DISABLE_DIAGNOSTICS
#include <cstdio>
#include <format>
#include <prosdk/cxx/print.hpp>
#include <system_error>
#include <utility>
#endif

namespace slp {

#ifndef SLEIPNIR_DISABLE_DIAGNOSTICS

/// Wrapper around std::print() that squelches write failure exceptions.
template <typename... T>
void print(std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::print(fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

/// Wrapper around std::print() that squelches write failure exceptions.
template <typename... T>
void print(std::FILE* f, std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::print(f, fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

/// Wrapper around std::println() that squelches write failure exceptions.
template <typename... T>
void println(std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::println(fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

/// Wrapper around std::println() that squelches write failure exceptions.
template <typename... T>
void println(std::FILE* f, std::format_string<T...> fmt, T&&... args) {
  try {
    prosdk::cxx::println(f, fmt, std::forward<T>(args)...);
  } catch (const std::system_error&) {
  }
}

#else

template <typename... Args>
void print([[maybe_unused]] Args&&... args) {}

template <typename... Args>
void println([[maybe_unused]] Args&&... args) {}

#endif

}  // namespace slp
