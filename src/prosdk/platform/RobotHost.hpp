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

#include <stdint.h>

namespace prosdk {
class RobotBase;
}  // namespace prosdk

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

// ---- hosted robot program (the robot side of StartRobot; design/PORT.md section 8) ----
//
// On the robot there is no main(): PROSDK_ROBOT_MAIN(MyRobot) (prosdk/platform/RobotMain.hpp)
// registers a factory at static initialization, and the robot host runs the program on
// probot-core's user task:
//   - boot (once, before core's first loop pass): installs the dashboard hooks above, runs
//     RunHALInitialization(), constructs the robot (exceptions are reported like upstream
//     StartRobot and halt the program), calls RobotBase::StartHosted() and sets core's loop
//     period from the robot's periodic callbacks;
//   - every core loop pass (a pre-cycle callback registered after the HAL's and the dashboard's):
//     runs the callbacks that are due (PeriodicPriorityQueue::RunDue), keeps the schedule aligned
//     with core's pass grid and follows AddPeriodic() changes of the pass period;
//   - after a probot-core emergency stop it never touches the robot again (core deleted the user
//     task; outputs are off through the HAL).
// A halted program (constructor or loop exception, invalid period, robot class without a hosted
// loop) keeps its outputs off (the HAL output gate) and reports to the console. With probot-core
// R12 it also stops the Driver Station (probot::requestStop()) at the halt and again whenever the
// DS inits or starts an opmode, so the DS does not show a running robot; without R12 the DS can
// still start it (nothing runs, outputs stay off).

/** Creates the user's robot (on core's user task). */
using RobotFactory = prosdk::RobotBase* (*)();

/** The state of the hosted robot program. */
enum class RobotProgramState : uint8_t {
  kNone = 0,     ///< no PROSDK_ROBOT_MAIN in the program
  kRegistered,   ///< registered, waiting for core's boot
  kRunning,      ///< constructed; its loop runs from core's cycle callbacks
  kHalted,       ///< failed or crashed; outputs stay off
  kEStopped,     ///< stopped by a probot-core emergency stop
};

/**
 * Registers the robot program (PROSDK_ROBOT_MAIN). Safe during static initialization: it only
 * stores the factory and registers core boot callback; the robot is constructed at core's boot,
 * never here. Only the first registration is taken; a second one returns false.
 *
 * @param factory      Creates the robot.
 * @param name         The robot class name (for messages); a string literal.
 * @param offerUtility After the robot is constructed, add a UTILITY opmode option "Utility" and
 *                     publish the opmode options, so that probot-core offers Utility mode (it
 *                     does only when one is listed). PROSDK_ROBOT_MAIN sets it for an
 *                     IterativeRobotBase (TimedRobot, TimesliceRobot): upstream's Driver Station
 *                     always offers Utility to those; an OpModeRobot publishes its own options.
 */
bool SetRobotMain(RobotFactory factory, const char* name, bool offerUtility = false) noexcept;

/** The state of the hosted robot program. */
RobotProgramState GetRobotProgramState() noexcept;

/** True once the hosted robot program halted (its outputs stay disabled). */
bool RobotProgramHalted() noexcept;

namespace detail {
/**
 * Host tests only: deletes the hosted robot, unregisters its cycle callback and forgets the
 * registration, so that the next SetRobotMain() starts over (HAL initialization is kept).
 */
void ResetRobotMainForTest();
}  // namespace detail

}  // namespace prosdk::platform
