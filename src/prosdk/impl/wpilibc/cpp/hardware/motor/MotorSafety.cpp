// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hardware/motor/MotorSafety.hpp"

#include <utility>

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/hal/DriverStation.h"
#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/platform/config.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/util/SafeThread.hpp"
#include "prosdk/util/SmallPtrSet.hpp"

using namespace prosdk;

namespace {
class Thread : public prosdk::util::SafeThread {
 public:
  Thread() {}
  void Main() override;
};

void Thread::Main() {
  prosdk::util::Event event{false, false};
  HAL_ProvideNewDataEventHandle(event.GetHandle());

  int safetyCounter = 0;
  while (m_active) {
    bool timedOut = false;
    bool signaled = prosdk::util::WaitForObject(event.GetHandle(), 0.1, &timedOut);
    if (signaled) {
      HAL_ControlWord controlWord;
      HAL_GetControlWord(&controlWord);
      if (!HAL_ControlWord_IsEnabled(controlWord) ||
          !HAL_ControlWord_IsDSAttached(controlWord)) {
        safetyCounter = 0;
      }
      if (++safetyCounter >= 4) {
        MotorSafety::CheckMotors();
        safetyCounter = 0;
      }
    } else {
      safetyCounter = 0;
    }
  }

  HAL_RemoveNewDataEventHandle(event.GetHandle());
}
}  // namespace

static std::atomic_bool gShutdown{false};

namespace {
struct MotorSafetyManager {
  ~MotorSafetyManager() { gShutdown = true; }

  prosdk::util::SafeThreadOwner<Thread> thread;
  prosdk::util::SmallPtrSet<MotorSafety*, 32> instanceList;
  prosdk::util::mutex listMutex;
  bool threadStarted = false;
};
}  // namespace

static MotorSafetyManager& GetManager() {
  static MotorSafetyManager manager;
  return manager;
}

#if !PROSDK_TARGET_HW
namespace prosdk::impl {
void ResetMotorSafety() {
  auto& manager = GetManager();
  std::scoped_lock lock(manager.listMutex);
  manager.instanceList.clear();
  manager.thread.Stop();
  manager.thread.Join();
  manager.thread = prosdk::util::SafeThreadOwner<Thread>{};
  manager.threadStarted = false;
}
}  // namespace prosdk::impl
#endif

MotorSafety::MotorSafety() {
  auto& manager = GetManager();
  std::scoped_lock lock(manager.listMutex);
  manager.instanceList.insert(this);
  if (!manager.threadStarted) {
    manager.threadStarted = true;
    manager.thread.Start();
  }
}

MotorSafety::~MotorSafety() {
  auto& manager = GetManager();
  std::scoped_lock lock(manager.listMutex);
  manager.instanceList.erase(this);
}

MotorSafety::MotorSafety(MotorSafety&& rhs)
    : m_expiration(std::move(rhs.m_expiration)),
      m_enabled(std::move(rhs.m_enabled)),
      m_stopTime(std::move(rhs.m_stopTime)) {}

MotorSafety& MotorSafety::operator=(MotorSafety&& rhs) {
  std::scoped_lock lock(m_thisMutex, rhs.m_thisMutex);

  m_expiration = std::move(rhs.m_expiration);
  m_enabled = std::move(rhs.m_enabled);
  m_stopTime = std::move(rhs.m_stopTime);

  return *this;
}

void MotorSafety::Feed() {
  std::scoped_lock lock(m_thisMutex);
  m_stopTime = Timer::GetMonotonicTimestamp() + m_expiration;
}

void MotorSafety::SetExpiration(prosdk::units::second_t expirationTime) {
  std::scoped_lock lock(m_thisMutex);
  m_expiration = expirationTime;
}

prosdk::units::second_t MotorSafety::GetExpiration() const {
  std::scoped_lock lock(m_thisMutex);
  return m_expiration;
}

bool MotorSafety::IsAlive() const {
  std::scoped_lock lock(m_thisMutex);
  return !m_enabled || m_stopTime > Timer::GetMonotonicTimestamp();
}

void MotorSafety::SetSafetyEnabled(bool enabled) {
  std::scoped_lock lock(m_thisMutex);
  m_enabled = enabled;
}

bool MotorSafety::IsSafetyEnabled() const {
  std::scoped_lock lock(m_thisMutex);
  return m_enabled;
}

void MotorSafety::Check() {
  bool enabled;
  prosdk::units::second_t stopTime;

  {
    std::scoped_lock lock(m_thisMutex);
    enabled = m_enabled;
    stopTime = m_stopTime;
  }

  if (!enabled || RobotState::IsDisabled() || RobotState::IsUtility()) {
    return;
  }

  if (stopTime < Timer::GetMonotonicTimestamp()) {
    WPILIB_ReportError(
        err::Timeout,
        "{}... Output not updated often enough. See "
        "https://docs.wpilib.org/motorsafety for more information.",
        GetDescription());

    try {
      StopMotor();
    } catch (prosdk::RuntimeError& e) {
      e.Report();
    } catch (std::exception& e) {
      WPILIB_ReportError(err::Error,
                         "{} StopMotor threw unexpected exception: {}",
                         GetDescription(), e.what());
    }
  }
}

void MotorSafety::CheckMotors() {
  auto& manager = GetManager();
  std::scoped_lock lock(manager.listMutex);
  for (auto elem : manager.instanceList) {
    elem->Check();
  }
}
