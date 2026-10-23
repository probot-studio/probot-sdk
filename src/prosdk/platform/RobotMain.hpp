// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// PROSDK_ROBOT_MAIN(RobotClass): the robot program's entry point (WPILib's
// `int main() { return wpi::StartRobot<Robot>(); }`), for a sketch (included by <prosdk.h>).
//
// Write it once, at namespace scope, in one .cpp/.ino file of the sketch:
//
//   #include <prosdk.h>
//   #include <prosdk/framework/TimedRobot.hpp>
//
//   class Robot : public prosdk::TimedRobot { ... };
//
//   PROSDK_ROBOT_MAIN(Robot)
//
// On the robot it registers the robot class with the robot host (prosdk/platform/RobotHost.hpp)
// during static initialization; the robot itself is constructed later, on probot-core's user task
// at boot, and its loop runs from core's loop passes (no thread, no main()). A second
// PROSDK_ROBOT_MAIN in the same program is a link error (duplicate
// prosdk_robot_main_defined_twice). On the host (simulation, tests) it defines main() with
// prosdk::StartRobot<RobotClass>(), as upstream.
//
// RobotClass must derive from TimedRobot, TimesliceRobot or OpModeRobot<RobotClass> on the robot
// (a class that derives from RobotBase directly needs a blocking StartCompetition(), which cannot
// run inside core's loop; the robot host reports it and halts).
//
// For a TimedRobot or TimesliceRobot the robot host publishes one UTILITY opmode option, "Utility",
// after the constructor (with any options the constructor added), so that the Driver Station
// offers Utility mode and UtilityInit/UtilityPeriodic/UtilityExit run, as upstream.

#pragma once

#include <type_traits>

#include "prosdk/framework/RobotBase.hpp"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/platform/config.hpp"

namespace prosdk {
class IterativeRobotBase;
}  // namespace prosdk

namespace prosdk::platform::detail {

/** Robot factory for PROSDK_ROBOT_MAIN (on core's user task; the robot is never deleted). */
template <class Robot>
prosdk::RobotBase* NewRobot() {
  return new Robot();
}

}  // namespace prosdk::platform::detail

#if PROSDK_TARGET_HW
#define PROSDK_ROBOT_MAIN(RobotClass)                                                     \
  static_assert(std::is_base_of_v<::prosdk::RobotBase, RobotClass>,                      \
                "PROSDK_ROBOT_MAIN needs a robot class derived from prosdk::RobotBase "   \
                "(TimedRobot, TimesliceRobot or OpModeRobot)");                          \
  extern "C" {                                                                            \
  [[maybe_unused]] char prosdk_robot_main_defined_twice = 0;                              \
  }                                                                                       \
  [[maybe_unused]] static const bool prosdk_robot_main_registered =                       \
      ::prosdk::platform::SetRobotMain(                                                  \
          &::prosdk::platform::detail::NewRobot<RobotClass>, #RobotClass,                 \
          std::is_base_of_v<::prosdk::IterativeRobotBase, RobotClass>);
#else
#define PROSDK_ROBOT_MAIN(RobotClass)                                                   \
  static_assert(std::is_base_of_v<::prosdk::RobotBase, RobotClass>,                    \
                "PROSDK_ROBOT_MAIN needs a robot class derived from prosdk::RobotBase " \
                "(TimedRobot, TimesliceRobot or OpModeRobot)");                        \
  int main() {                                                                          \
    return ::prosdk::StartRobot<RobotClass>();                                          \
  }
#endif
