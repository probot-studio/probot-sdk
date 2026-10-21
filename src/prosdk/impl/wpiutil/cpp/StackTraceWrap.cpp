// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <atomic>
#include <string>

#include "prosdk/util/StackTrace.hpp"

static std::atomic<std::string (*)(int offset)> gStackTraceImpl{
    prosdk::util::GetStackTraceDefault};

std::string prosdk::util::GetStackTrace(int offset) {
  return (gStackTraceImpl.load())(offset);
}

void prosdk::util::SetGetStackTraceImpl(std::string (*func)(int offset)) {
  gStackTraceImpl = func ? func : prosdk::util::GetStackTraceDefault;
}
