// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/RobotHost.hpp"

#include <atomic>

namespace prosdk::platform {
namespace {
using Hook = void (*)();

constinit std::atomic<Hook> g_robotBaseConstructed{nullptr};
constinit std::atomic<Hook> g_flushTelemetry{nullptr};

void Call(const std::atomic<Hook>& hook) {
  if (Hook fn = hook.load(std::memory_order_acquire)) {
    fn();
  }
}
}  // namespace

void SetRobotHostHooks(const RobotHostHooks& hooks) noexcept {
  g_robotBaseConstructed.store(hooks.robotBaseConstructed, std::memory_order_release);
  g_flushTelemetry.store(hooks.flushTelemetry, std::memory_order_release);
}

void OnRobotBaseConstructed() {
  Call(g_robotBaseConstructed);
}

void FlushTelemetry() {
  Call(g_flushTelemetry);
}

}  // namespace prosdk::platform
