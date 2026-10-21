// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// prosdk::cxx::print / println: C++23 std::print / std::println ([print.fun]) on top of
// C++20 std::format and fwrite. Like the standard, a failed write throws std::system_error.
// Vendored code is rewritten from std::print/std::println to these names by
// tools/upstream/wpilib.py.

#pragma once

#include <cerrno>
#include <cstdio>
#include <format>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace prosdk::cxx {

namespace detail {
inline void print_write(std::FILE* stream, std::string_view text) {
  if (!text.empty() && std::fwrite(text.data(), 1, text.size(), stream) != text.size()) {
    throw std::system_error(errno != 0 ? errno : EIO, std::generic_category(), "print");
  }
}
}  // namespace detail

inline void vprint_unicode(std::FILE* stream, std::string_view fmt, std::format_args args) {
  detail::print_write(stream, std::vformat(fmt, args));
}

inline void vprint_nonunicode(std::FILE* stream, std::string_view fmt, std::format_args args) {
  detail::print_write(stream, std::vformat(fmt, args));
}

inline void vprint_unicode(std::string_view fmt, std::format_args args) {
  vprint_unicode(stdout, fmt, args);
}

inline void vprint_nonunicode(std::string_view fmt, std::format_args args) {
  vprint_nonunicode(stdout, fmt, args);
}

template <class... Args>
void print(std::FILE* stream, std::format_string<Args...> fmt, Args&&... args) {
  detail::print_write(stream, std::format(fmt, std::forward<Args>(args)...));
}

template <class... Args>
void print(std::format_string<Args...> fmt, Args&&... args) {
  print(stdout, fmt, std::forward<Args>(args)...);
}

template <class... Args>
void println(std::FILE* stream, std::format_string<Args...> fmt, Args&&... args) {
  std::string text = std::format(fmt, std::forward<Args>(args)...);
  text.push_back('\n');
  detail::print_write(stream, text);
}

template <class... Args>
void println(std::format_string<Args...> fmt, Args&&... args) {
  println(stdout, fmt, std::forward<Args>(args)...);
}

inline void println(std::FILE* stream) {
  detail::print_write(stream, "\n");
}

inline void println() {
  println(stdout);
}

}  // namespace prosdk::cxx
