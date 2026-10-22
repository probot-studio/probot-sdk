// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Hook points between the vendored wpilibc robot framework and the robot host (the probot-core
// integration, design/PORT.md section 8).
//
// Upstream wpilibc starts a NetworkTables server and registers the NetworkTables telemetry and
// tunable backends in RobotBase's constructor, and flushes NetworkTables at the end of every
// robot loop iteration. ntcore is not ported; the patches in tools/upstream/patches/wpilibc call
// these hooks at those places instead. Until hooks are installed both calls are no-ops (host
// build, unit tests).

#pragma once

namespace prosdk::platform {

/** Robot host callbacks. A null entry is a no-op. */
struct RobotHostHooks {
  /**
   * Called at the end of prosdk::RobotBase::RobotBase() on the thread that constructs the robot,
   * after MathShared and the telemetry/tunable warning reporters are set up (where upstream
   * registered the NetworkTables backends). The derived robot class is not constructed yet.
   */
  void (*robotBaseConstructed)();
  /**
   * Called once per robot loop iteration by IterativeRobotBase::LoopFunc() and
   * OpModeRobotBase::LoopFunc(), after the user code and before the loop-overrun report (where
   * upstream called NetworkTableInstance::FlushLocal()). Runs on the robot thread; keep it short.
   */
  void (*flushTelemetry)();
};

/**
 * Installs the hooks (copied). Replaces any previous hooks. Thread-safe and safe during static
 * initialization; normally called once by the robot host before the robot is constructed.
 */
void SetRobotHostHooks(const RobotHostHooks& hooks) noexcept;

/** Calls RobotHostHooks::robotBaseConstructed if installed. */
void OnRobotBaseConstructed();

/** Calls RobotHostHooks::flushTelemetry if installed. */
void FlushTelemetry();

}  // namespace prosdk::platform
