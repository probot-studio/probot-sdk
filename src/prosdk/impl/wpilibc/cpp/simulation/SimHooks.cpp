// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/SimHooks.hpp"

#include "prosdk/hal/simulation/MockHooks.hpp"

namespace prosdk::sim {

void SetRuntimeType(HAL_RuntimeType type) {
  HALSIM_SetRuntimeType(type);
}

void WaitForProgramStart(bool waitForFirstNotifier) {
  HALSIM_WaitForProgramStart(waitForFirstNotifier);
}

void SetProgramStarted(bool started) {
  HALSIM_SetProgramStarted(started);
}

bool GetProgramStarted() {
  return HALSIM_GetProgramStarted();
}

void SetProgramState(prosdk::hal::ControlWord controlWord) {
  prosdk::hal::sim::SetProgramState(controlWord);
}

prosdk::hal::ControlWord GetProgramState() {
  return prosdk::hal::sim::GetProgramState();
}

void RestartTiming() {
  HALSIM_RestartTiming();
}

void PauseTiming() {
  HALSIM_PauseTiming();
}

void ResumeTiming() {
  HALSIM_ResumeTiming();
}

bool IsTimingPaused() {
  return HALSIM_IsTimingPaused();
}

void StepTiming(prosdk::units::second_t delta) {
  HALSIM_StepTiming(static_cast<int64_t>(delta.value() * 1e9));
}

void StepTimingAsync(prosdk::units::second_t delta) {
  HALSIM_StepTimingAsync(static_cast<int64_t>(delta.value() * 1e9));
}

}  // namespace prosdk::sim
