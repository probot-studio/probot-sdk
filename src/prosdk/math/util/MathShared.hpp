// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <format>
#include <memory>
#include <string_view>

#include "prosdk/units/time.hpp"
#include "prosdk/util/SymbolExports.hpp"

namespace prosdk::math {

class WPILIB_DLLEXPORT MathShared {
 public:
  virtual ~MathShared() = default;
  virtual void ReportErrorV(std::string_view format, std::format_args args) = 0;
  virtual void ReportWarningV(std::string_view format,
                              std::format_args args) = 0;
  virtual prosdk::units::second_t GetTimestamp() = 0;

  template <typename S, typename... Args>
  inline void ReportError(const S& format, Args&&... args) {
    ReportErrorV(format, std::make_format_args(args...));
  }

  template <typename S, typename... Args>
  inline void ReportWarning(const S& format, Args&&... args) {
    ReportWarningV(format, std::make_format_args(args...));
  }
};

class WPILIB_DLLEXPORT MathSharedStore {
 public:
  static MathShared& GetMathShared();

  static void SetMathShared(std::unique_ptr<MathShared> shared);

  static void ReportErrorV(std::string_view format, std::format_args args) {
    GetMathShared().ReportErrorV(format, args);
  }

  template <typename S, typename... Args>
  static inline void ReportError(const S& format, Args&&... args) {
    ReportErrorV(format, std::make_format_args(args...));
  }

  static void ReportWarningV(std::string_view format, std::format_args args) {
    GetMathShared().ReportWarningV(format, args);
  }

  template <typename S, typename... Args>
  static inline void ReportWarning(const S& format, Args&&... args) {
    ReportWarningV(format, std::make_format_args(args...));
  }

  static prosdk::units::second_t GetTimestamp() {
    return GetMathShared().GetTimestamp();
  }
};

}  // namespace prosdk::math
