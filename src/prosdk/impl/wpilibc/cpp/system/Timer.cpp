// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/system/Timer.hpp"

#include <stdint.h>

#include <chrono>
#include <thread>

#include "prosdk/driverstation/MatchState.hpp"
#include "prosdk/system/RobotController.hpp"

namespace prosdk {

void Wait(prosdk::units::second_t seconds) {
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds.value()));
}

prosdk::units::second_t GetSystemTime() {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::system_clock;

  return prosdk::units::second_t{
      duration_cast<duration<double>>(system_clock::now().time_since_epoch())
          .count()};
}

}  // namespace wpi

using namespace prosdk;

namespace {

std::chrono::nanoseconds GetTimestampNanoseconds() {
  return std::chrono::nanoseconds{
      static_cast<int64_t>(prosdk::RobotController::GetTime())};
}

}  // namespace

Timer::Timer() {
  Reset();
}

prosdk::units::second_t Timer::Get() const {
  return prosdk::units::nanosecond_t{GetNanoseconds()};
}

double Timer::GetNanoseconds() const {
  if (m_running) {
    return static_cast<double>(
               (GetTimestampNanoseconds() - m_startTime).count()) -
           m_startTimeRemainderNs + m_accumulatedTimeNs;
  } else {
    return m_accumulatedTimeNs;
  }
}

void Timer::Reset() {
  m_accumulatedTimeNs = 0.0;
  m_startTime = GetTimestampNanoseconds();
  m_startTimeRemainderNs = 0.0;
}

void Timer::Start() {
  if (!m_running) {
    m_startTime = GetTimestampNanoseconds();
    m_startTimeRemainderNs = 0.0;
    m_running = true;
  }
}

void Timer::Restart() {
  if (m_running) {
    Stop();
  }
  Reset();
  Start();
}

void Timer::Stop() {
  if (m_running) {
    m_accumulatedTimeNs = GetNanoseconds();
    m_running = false;
  }
}

bool Timer::HasElapsed(prosdk::units::second_t period) const {
  return GetNanoseconds() >= prosdk::units::nanosecond_t{period}.value();
}

bool Timer::AdvanceIfElapsed(prosdk::units::second_t period) {
  double periodNs = prosdk::units::nanosecond_t{period}.value();

  if (GetNanoseconds() >= periodNs) {
    // Advance the start time by the period.
    double advanceNs = m_startTimeRemainderNs + periodNs;
    auto wholeNs = static_cast<int64_t>(advanceNs);
    m_startTime += std::chrono::nanoseconds{wholeNs};
    m_startTimeRemainderNs = advanceNs - wholeNs;
    // Don't set it to the current time... we want to avoid drift.
    return true;
  } else {
    return false;
  }
}

bool Timer::IsRunning() const {
  return m_running;
}

Timer Timer::CreateStarted() {
  Timer timer;
  timer.Start();
  return timer;
}

prosdk::units::second_t Timer::GetTimestamp() {
  return GetTimestampNanoseconds();
}

prosdk::units::second_t Timer::GetMonotonicTimestamp() {
  return std::chrono::nanoseconds{
      static_cast<int64_t>(prosdk::RobotController::GetMonotonicTime())};
}

prosdk::units::second_t Timer::GetMatchTime() {
  return prosdk::MatchState::GetMatchTime();
}
