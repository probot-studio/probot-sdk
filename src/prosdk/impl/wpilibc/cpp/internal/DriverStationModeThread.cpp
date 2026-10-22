// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/internal/DriverStationModeThread.hpp"

#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/hal/DriverStation.h"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/platform/WpilibcConfig.hpp"
#include "prosdk/util/Synchronization.hpp"

using namespace prosdk::internal;

DriverStationModeThread::DriverStationModeThread(prosdk::hal::ControlWord word)
    : m_userControlWord{word.GetValue().value} {
  HAL_ProvideNewDataEventHandle(m_event.GetHandle());
  m_keepAlive = true;
  // prosdk: never the ESP-IDF pthread defaults (design/PORT.md section 7)
  prosdk::platform::ScopedThreadConfig threadConfig{
      {.name = "dsmode", .stackSize = PROSDK_DS_MODE_THREAD_STACK_SIZE}};
  m_thread = std::thread{[this] { Run(); }};
}

DriverStationModeThread::~DriverStationModeThread() {
  HAL_RemoveNewDataEventHandle(m_event.GetHandle());
  m_keepAlive = false;
  m_event.Set();
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void DriverStationModeThread::Run() {
  for (;;) {
    bool timedOut = false;
    prosdk::util::WaitForObject(m_event.GetHandle(), 0.1, &timedOut);
    if (!m_keepAlive) {
      break;
    }
    prosdk::internal::DriverStationBackend::RefreshData();
    HAL_ObserveUserProgram({.value = m_userControlWord});
  }
}
