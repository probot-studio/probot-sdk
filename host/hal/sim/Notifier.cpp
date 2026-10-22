// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/hal/Notifier.h"

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "HALInitializer.hpp"
#include "NotifierInternal.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/HAL.h"
#include "prosdk/hal/Types.h"
#include "prosdk/hal/handles/UnlimitedHandleResource.hpp"
#include "prosdk/hal/simulation/NotifierData.h"
#include "prosdk/util/SafeThread.hpp"
#include "prosdk/util/SmallVector.hpp"
#include "prosdk/util/StringExtras.hpp"
#include "prosdk/util/Synchronization.hpp"
#include "prosdk/util/priority_queue.hpp"
#include "prosdk/util/string.hpp"

namespace {
static constexpr int64_t NO_ALARM = std::numeric_limits<int64_t>::max();

struct Notifier {
  std::string name;
  std::atomic<int64_t> alarmTime = NO_ALARM;
  int64_t intervalTime = 0;
  std::atomic<int32_t> userOverrunCount = 0;
  int32_t overrunCount = 0;
  std::atomic_flag handlerSignaled{};
};
}  // namespace

using namespace prosdk::hal;

class NotifierThread : public prosdk::util::SafeThread {
 public:
  void Main() override;

  void ProcessAlarms(prosdk::util::SmallVectorImpl<HAL_NotifierHandle>* signaled);

  bool m_paused = false;

  UnlimitedHandleResource<HAL_NotifierHandle, Notifier,
                          HAL_HandleEnum::NOTIFIER>
      m_handles;

  struct Alarm {
    HAL_NotifierHandle handle;
    std::shared_ptr<Notifier> notifier;
    bool operator==(const Alarm& rhs) const { return handle == rhs.handle; }
    bool operator>(const Alarm& rhs) const {
      return notifier->alarmTime > rhs.notifier->alarmTime;
    }
  };
  prosdk::util::priority_queue<Alarm, std::vector<Alarm>, std::greater<Alarm>>
      m_alarmQueue;
};

class NotifierInstance {
 public:
  NotifierInstance() { owner.Start(); }
  prosdk::util::SafeThreadOwner<NotifierThread> owner;
};

static NotifierInstance* notifierInstance;
static std::atomic<uint64_t> notifierAlarmSetCount{0};

namespace prosdk::hal::init {
void InitializeNotifier() {
  static NotifierInstance n;
  notifierInstance = &n;
}
}  // namespace prosdk::hal::init

void NotifierThread::Main() {
  std::unique_lock lock(m_mutex);
  while (m_active) {
    if (m_paused || m_alarmQueue.empty()) {
      // No alarms, wait indefinitely
      m_cond.wait(lock);
      continue;
    }

    // Wait until next alarm
    const Alarm& alarm = m_alarmQueue.top();
    int64_t curTime = HAL_GetMonotonicTime();
    if (alarm.notifier->alarmTime > curTime) {
      m_cond.wait_for(
          lock, std::chrono::nanoseconds{alarm.notifier->alarmTime - curTime});
    }
    if (!m_active) {
      break;
    }

    // Check paused again as we may have been paused while waiting
    if (m_paused) {
      continue;
    }

    ProcessAlarms(nullptr);
  }
}

void NotifierThread::ProcessAlarms(
    prosdk::util::SmallVectorImpl<HAL_NotifierHandle>* signaled) {
  int64_t curTime = HAL_GetMonotonicTime();

  // Process alarms
  while (!m_alarmQueue.empty() &&
         m_alarmQueue.top().notifier->alarmTime <= curTime) {
    Alarm alarm = m_alarmQueue.pop();
    HAL_NotifierHandle handle = alarm.handle;
    Notifier& notifier = *alarm.notifier;

    if (notifier.intervalTime > 0) {
      // Schedule next alarm
      notifier.alarmTime += notifier.intervalTime;
      if (curTime >= notifier.alarmTime) {
        // We missed at least one interval
        int32_t missed = static_cast<int32_t>((curTime - notifier.alarmTime) /
                                              notifier.intervalTime) +
                         1;
        notifier.overrunCount += missed;
        notifier.alarmTime +=
            missed * notifier.intervalTime;  // Skip missed intervals
      }
      // Reinsert into queue
      m_alarmQueue.push(std::move(alarm));
    } else {
      // Disable one-shot alarm
      notifier.alarmTime = NO_ALARM;
    }

    // If the last call was acknowledged, signal the handler
    if (!notifier.handlerSignaled.test_and_set()) {
      if (signaled) {
        signaled->emplace_back(handle);
      }
      // copy the overrun count for the handler to read, reset the local count
      notifier.userOverrunCount = notifier.overrunCount;
      notifier.overrunCount = 0;
      prosdk::util::SetSignalObject(handle);
    }
  }
}

void prosdk::hal::PauseNotifiers() {
  auto thr = notifierInstance->owner.GetThread();
  thr->m_paused = true;
}

void prosdk::hal::ResumeNotifiers() {
  auto thr = notifierInstance->owner.GetThread();
  thr->m_paused = false;
  thr->m_cond.notify_all();
}

void prosdk::hal::WakeupNotifiers() {
  auto thr = notifierInstance->owner.GetThread();
  thr->ProcessAlarms(nullptr);
}

static void DoWaitNotifiers(
    prosdk::util::detail::SafeThreadProxy<NotifierThread>& thr,
    prosdk::util::SmallVectorImpl<HAL_NotifierHandle>& signaled) {
  // Wait for signaled notifiers to acknowledge their last alarm
  for (;;) {
    signaled.erase(std::remove_if(signaled.begin(), signaled.end(),
                                  [&](HAL_NotifierHandle handle) {
                                    auto notifier = thr->m_handles.Get(handle);
                                    return !notifier ||
                                           !notifier->handlerSignaled.test();
                                  }),
                   signaled.end());
    if (signaled.empty()) {
      break;
    }
    thr->m_cond.wait_for(thr.GetLock(), std::chrono::milliseconds{1});
  }
}

void prosdk::hal::WaitNotifiers() {
  auto thr = notifierInstance->owner.GetThread();

  prosdk::util::SmallVector<HAL_NotifierHandle, 8> signaled;
  thr->m_handles.ForEach([&](HAL_NotifierHandle handle, Notifier* notifier) {
    if (notifier->handlerSignaled.test()) {
      signaled.emplace_back(handle);
    }
  });
  DoWaitNotifiers(thr, signaled);
}

void prosdk::hal::WakeupWaitNotifiers() {
  auto thr = notifierInstance->owner.GetThread();

  prosdk::util::SmallVector<HAL_NotifierHandle, 8> signaled;
  thr->ProcessAlarms(&signaled);
  DoWaitNotifiers(thr, signaled);
}

uint64_t prosdk::hal::GetNotifierAlarmSetCount() {
  return notifierAlarmSetCount;
}

extern "C" {

HAL_NotifierHandle HAL_CreateNotifier(int32_t* status) {
  prosdk::hal::init::CheckInit();
  std::shared_ptr<Notifier> notifier = std::make_shared<Notifier>();
  HAL_NotifierHandle handle =
      notifierInstance->owner.GetThread()->m_handles.Allocate(notifier);
  if (handle == HAL_INVALID_HANDLE) {
    *status = HAL_HANDLE_ERROR;
    return HAL_INVALID_HANDLE;
  }
  prosdk::util::CreateSignalObject(handle);
  return handle;
}

void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle,
                         const WPI_String* name, int32_t* status) {
  auto thr = notifierInstance->owner.GetThread();
  auto notifier = thr->m_handles.Get(notifierHandle);
  if (!notifier) {
    return;
  }
  notifier->name = prosdk::util::to_string_view(name);
}

void HAL_DestroyNotifier(HAL_NotifierHandle notifierHandle) {
  prosdk::util::DestroySignalObject(notifierHandle);
  auto thr = notifierInstance->owner.GetThread();
  auto notifier = thr->m_handles.Free(notifierHandle);
  thr->m_alarmQueue.remove({notifierHandle, notifier});
}

void HAL_SetNotifierAlarm(HAL_NotifierHandle notifierHandle, int64_t alarmTime,
                          int64_t intervalTime, HAL_Bool absolute, HAL_Bool ack,
                          int32_t* status) {
  auto thr = notifierInstance->owner.GetThread();
  auto notifier = thr->m_handles.Get(notifierHandle);
  if (!notifier) {
    return;
  }

  if (ack) {
    notifier->handlerSignaled.clear();
    prosdk::util::ResetSignalObject(notifierHandle);
  }

  if (!absolute) {
    alarmTime += HAL_GetMonotonicTime();
  }

  int64_t prevWakeup = NO_ALARM;
  if (!thr->m_alarmQueue.empty()) {
    prevWakeup = thr->m_alarmQueue.top().notifier->alarmTime;
    thr->m_alarmQueue.remove({notifierHandle, notifier});
  }
  notifier->alarmTime = alarmTime;
  notifier->intervalTime = intervalTime;
  notifier->overrunCount = 0;
  thr->m_alarmQueue.push({notifierHandle, notifier});
  ++notifierAlarmSetCount;

  // wake up notifier thread if needed
  if (alarmTime < prevWakeup) {
    thr->m_cond.notify_all();
  }
}

void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle, HAL_Bool ack,
                             int32_t* status) {
  auto thr = notifierInstance->owner.GetThread();
  auto notifier = thr->m_handles.Get(notifierHandle);
  if (!notifier) {
    return;
  }

  if (ack) {
    notifier->handlerSignaled.clear();
    prosdk::util::ResetSignalObject(notifierHandle);
  }

  thr->m_alarmQueue.remove({notifierHandle, notifier});
  notifier->alarmTime = NO_ALARM;
}

void HAL_AcknowledgeNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto thr = notifierInstance->owner.GetThread();
  auto notifier = thr->m_handles.Get(notifierHandle);
  if (!notifier) {
    return;
  }
  notifier->handlerSignaled.clear();
  prosdk::util::ResetSignalObject(notifierHandle);
}

int32_t HAL_GetNotifierOverrun(HAL_NotifierHandle notifierHandle,
                               int32_t* status) {
  auto notifier =
      notifierInstance->owner.GetThread()->m_handles.Get(notifierHandle);
  if (!notifier) {
    return -1;
  }
  return notifier->userOverrunCount;
}

int64_t HALSIM_GetNextNotifierTimeout(void) {
  auto thr = notifierInstance->owner.GetThread();
  if (thr->m_alarmQueue.empty()) {
    return NO_ALARM;
  }
  return thr->m_alarmQueue.top().notifier->alarmTime;
}

int32_t HALSIM_GetNumNotifiers(void) {
  auto thr = notifierInstance->owner.GetThread();
  int32_t count = 0;
  thr->m_handles.ForEach([&](auto, auto) { ++count; });
  return count;
}

int32_t HALSIM_GetNotifierInfo(struct HALSIM_NotifierInfo* arr, int32_t size) {
  auto thr = notifierInstance->owner.GetThread();
  int32_t num = 0;
  thr->m_handles.ForEach([&](HAL_NotifierHandle handle, Notifier* notifier) {
    if (num < size) {
      arr[num].handle = handle;
      if (notifier->name.empty()) {
        prosdk::util::format_to_n_c_str(arr[num].name, sizeof(arr[num].name),
                                     "Notifier{}",
                                     static_cast<int>(getHandleIndex(handle)));
      } else {
        std::strncpy(arr[num].name, notifier->name.c_str(),
                     sizeof(arr[num].name) - 1);
        arr[num].name[sizeof(arr[num].name) - 1] = '\0';
      }
      arr[num].alarmTime = notifier->alarmTime;
      arr[num].intervalTime = notifier->intervalTime;
      arr[num].overrunCount = notifier->overrunCount;
    }
    ++num;
  });
  return num;
}

}  // extern "C"
