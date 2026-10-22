// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <thread>
#include <utility>

#include "prosdk/util/Synchronization.hpp"
#include "prosdk/util/condition_variable.hpp"
#include "prosdk/util/mutex.hpp"

namespace prosdk::util {

/**
 * Base class for SafeThreadOwner threads.
 */
class SafeThreadBase {
 public:
  virtual ~SafeThreadBase() = default;
  virtual void Main() = 0;
  virtual void Stop() = 0;

  mutable prosdk::util::mutex m_mutex;
  std::atomic_bool m_active{true};
  std::thread::id m_threadId;

  // prosdk: creation settings of the thread, read by SafeThreadOwner::Start()
  // (set them in the derived class's constructor). On the ESP32 the thread
  // gets this stack size in bytes and this FreeRTOS task name; 0 / nullptr
  // select the prosdk defaults (PROSDK_THREAD_STACK_SIZE, "prosdk";
  // prosdk/platform/Thread.hpp). Ignored on the host.
  std::size_t m_stackSize = 0;
  const char* m_threadName = nullptr;
};

class SafeThread : public SafeThreadBase {
 public:
  void Stop() override;

  prosdk::util::condition_variable m_cond;
};

class SafeThreadEvent : public SafeThreadBase {
 public:
  SafeThreadEvent() : m_stopEvent{true} {}

  void Stop() override;

  Event m_stopEvent;
};

namespace detail {

/**
 * Non-template proxy base class for common proxy code.
 */
class SafeThreadProxyBase {
 public:
  explicit SafeThreadProxyBase(std::shared_ptr<SafeThreadBase> thr);
  explicit operator bool() const { return m_thread != nullptr; }
  std::unique_lock<prosdk::util::mutex>& GetLock() { return m_lock; }

 protected:
  std::shared_ptr<SafeThreadBase> m_thread;
  std::unique_lock<prosdk::util::mutex> m_lock;
};

/**
 * A proxy for SafeThread.
 *
 * Also serves as a scoped lock on SafeThread::m_mutex.
 */
template <typename T>
class SafeThreadProxy : public SafeThreadProxyBase {
 public:
  explicit SafeThreadProxy(std::shared_ptr<SafeThreadBase> thr)
      : SafeThreadProxyBase(std::move(thr)) {}
  T& operator*() const { return *static_cast<T*>(m_thread.get()); }
  T* operator->() const { return static_cast<T*>(m_thread.get()); }
};

/**
 * Non-template owner base class for common owner code.
 */
class SafeThreadOwnerBase {
 public:
  void Stop();
  void Join();

  SafeThreadOwnerBase() noexcept = default;
  SafeThreadOwnerBase(const SafeThreadOwnerBase&) = delete;
  SafeThreadOwnerBase& operator=(const SafeThreadOwnerBase&) = delete;
  SafeThreadOwnerBase(SafeThreadOwnerBase&& other) noexcept
      : SafeThreadOwnerBase() {
    swap(*this, other);
  }
  SafeThreadOwnerBase& operator=(SafeThreadOwnerBase&& other) noexcept {
    swap(*this, other);
    return *this;
  }
  ~SafeThreadOwnerBase();

  friend void swap(SafeThreadOwnerBase& lhs, SafeThreadOwnerBase& rhs) noexcept;

  explicit operator bool() const;

  std::thread::native_handle_type GetNativeThreadHandle();

  void SetJoinAtExit(bool joinAtExit) { m_joinAtExit = joinAtExit; }

 protected:
  void Start(std::shared_ptr<SafeThreadBase> thr);
  std::shared_ptr<SafeThreadBase> GetThreadSharedPtr() const;

 private:
  mutable prosdk::util::mutex m_mutex;
  std::thread m_stdThread;
  std::weak_ptr<SafeThreadBase> m_thread;
  std::atomic_bool m_joinAtExit{true};
};

void swap(SafeThreadOwnerBase& lhs, SafeThreadOwnerBase& rhs) noexcept;

}  // namespace detail

template <typename T>
class SafeThreadOwner : public detail::SafeThreadOwnerBase {
 public:
  template <typename... Args>
  void Start(Args&&... args) {
    detail::SafeThreadOwnerBase::Start(
        std::make_shared<T>(std::forward<Args>(args)...));
  }

  using Proxy = typename detail::SafeThreadProxy<T>;
  Proxy GetThread() const {
    return Proxy(detail::SafeThreadOwnerBase::GetThreadSharedPtr());
  }

  std::shared_ptr<T> GetThreadSharedPtr() const {
    return std::static_pointer_cast<T>(
        detail::SafeThreadOwnerBase::GetThreadSharedPtr());
  }
};

}  // namespace prosdk::util
