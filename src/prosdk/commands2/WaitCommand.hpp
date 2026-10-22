// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/commands2/Command.hpp"
#include "prosdk/commands2/CommandHelper.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk::cmd {
/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * This class is provided by the Commands v2 VendorDep
 */
class WaitCommand : public CommandHelper<Command, WaitCommand> {
 public:
  /**
   * Creates a new WaitCommand.  This command will do nothing, and end after the
   * specified duration.
   *
   * @param duration the time to wait
   */
  explicit WaitCommand(prosdk::units::second_t duration);

  WaitCommand(WaitCommand&& other) = default;

  WaitCommand(const WaitCommand& other) = default;

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  bool RunsWhenDisabled() const override;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;

 protected:
  /// The timer used for waiting.
  prosdk::Timer m_timer;

 private:
  prosdk::units::second_t m_duration;
};
}  // namespace prosdk::cmd
