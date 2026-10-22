// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include "prosdk/framework/IterativeRobotBase.hpp"
#include "prosdk/hal/Notifier.hpp"
#include "prosdk/internal/PeriodicPriorityQueue.hpp"
#include "prosdk/units/frequency.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/util/Handle.hpp"

namespace prosdk {

/**
 * TimedRobot implements the IterativeRobotBase robot program framework.
 *
 * The TimedRobot class is intended to be subclassed by a user creating a
 * robot program.
 *
 * Periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
class TimedRobot : public IterativeRobotBase {
 public:
  /// Default loop period.
  static constexpr auto DEFAULT_PERIOD = 20_ms;

  /**
   * Provide an alternate "main loop" via StartCompetition().
   */
  void StartCompetition() override;

  /**
   * Ends the main loop in StartCompetition().
   */
  void EndCompetition() override;

  /**
   * prosdk: StartCompetition() without its loop (see RobotBase::StartHosted()).
   */
  HostedLoop StartHosted() override;

  /**
   * Constructor for TimedRobot.
   *
   * @param period The period of the robot loop function.
   */
  explicit TimedRobot(prosdk::units::second_t period = DEFAULT_PERIOD);

  /**
   * Constructor for TimedRobot.
   *
   * @param frequency The frequency of the robot loop function.
   */
  explicit TimedRobot(prosdk::units::hertz_t frequency);

  TimedRobot(TimedRobot&&) = default;
  TimedRobot& operator=(TimedRobot&&) = default;

  ~TimedRobot() override;

  /**
   * Return the system clock time in nanoseconds for the start of the current
   *  periodic loop. This is in the same time base as
   * Timer.GetMonotonicTimestamp(), but is stable through a loop. It is updated
   * at the beginning of every periodic callback (including the normal periodic
   * loop).
   *
   * @return Robot running time in nanoseconds, as of the start of the current
   * periodic function.
   */
  prosdk::units::nanosecond_t GetLoopStartTime() const {
    return m_callbacks.GetLoopStartTime();
  }

  /**
   * Add a callback to run at a specific period with a starting time offset.
   *
   * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
   * run synchronously. Interactions between them are thread-safe.
   *
   * @param callback The callback to run.
   * @param period   The period at which to run the callback.
   * @param offset   The offset from the common starting time. This is useful
   *                 for scheduling a callback in a different timeslot relative
   *                 to TimedRobot.
   */
  void AddPeriodic(std::function<void()> callback, prosdk::units::second_t period,
                   prosdk::units::second_t offset = 0_s);

 protected:
  prosdk::util::Handle<HAL_NotifierHandle, HAL_DestroyNotifier> m_notifier;
  std::chrono::nanoseconds m_startTime;

 private:
  prosdk::internal::PeriodicPriorityQueue m_callbacks;
};

}  // namespace wpi
