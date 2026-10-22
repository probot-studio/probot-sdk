// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/system/Errors.hpp"

#include <format>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "prosdk/hal/DriverStation.hpp"
#include "prosdk/hal/HAL.h"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/fs.hpp"

using namespace prosdk;

RuntimeError::RuntimeError(int32_t code, std::string&& loc, std::string&& stack,
                           std::string&& message)
    : runtime_error{std::move(message)}, m_data{std::make_shared<Data>()} {
  m_data->code = code;
  m_data->loc = std::move(loc);
  m_data->stack = stack;
}

RuntimeError::RuntimeError(int32_t code, const char* fileName, int lineNumber,
                           const char* funcName, std::string&& stack,
                           std::string&& message)
    : RuntimeError{
          code,
          std::format("{} [{}:{}]", funcName,
                      prosdk::fs::path{fileName}.filename().string(), lineNumber),
          std::move(stack), std::move(message)} {}

void RuntimeError::Report() const {
  prosdk::hal::SendError(m_data->code < 0, m_data->code, what(),
                      m_data->loc.c_str(), m_data->stack.c_str(), 1);
}

const char* prosdk::GetErrorMessage(int32_t* code) {
  switch (*code) {
#define S(label, offset, message) \
  case err::label:                \
    return message;
#include "prosdk/system/WPIErrors.mac"
#undef S
#define S(label, offset, message) \
  case warn::label:               \
    return message;
#include "prosdk/system/WPIWarnings.mac"
#undef S
    default:
      return HAL_GetLastError(code);
  }
}

void prosdk::ReportErrorV(int32_t status, const char* fileName, int lineNumber,
                       const char* funcName, std::string_view format,
                       std::format_args args) {
  if (status == 0) {
    return;
  }
  std::vector<char> out;
  std::format_to(std::back_inserter(out), "{}: ", GetErrorMessage(&status));
  std::vformat_to(std::back_inserter(out), format, args);
  out.push_back('\0');
  prosdk::hal::SendError(status < 0, status, out.data(), funcName,
                      prosdk::util::GetStackTrace(2).c_str(), 1);
}

RuntimeError prosdk::MakeErrorV(int32_t status, const char* fileName,
                             int lineNumber, const char* funcName,
                             std::string_view format, std::format_args args) {
  std::vector<char> out;
  std::format_to(std::back_inserter(out), "{}: ", GetErrorMessage(&status));
  std::vformat_to(std::back_inserter(out), format, args);
  return RuntimeError{status,
                      fileName,
                      lineNumber,
                      funcName,
                      prosdk::util::GetStackTrace(2),
                      std::string(out.begin(), out.end())};
}
