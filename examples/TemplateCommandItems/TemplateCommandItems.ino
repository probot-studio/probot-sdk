// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TemplateCommandItems: the eight command-based class templates of upstream
// wpilibcExamples/src/main/cpp/commands (empty class, Command, InstantCommand,
// ParallelCommandGroup, ParallelDeadlineGroup, ParallelRaceGroup, SequentialCommandGroup,
// Subsystem), ported to prosdk. Each class is one tab; copy the tab you need into your
// sketch and rename ReplaceMe... Upstream has no robot program for them; the small TimedRobot
// below (prosdk addition) constructs one of each so the sketch compiles and links, and runs
// the CommandScheduler. No hardware.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/commands2/CommandScheduler.hpp>
#include <prosdk/framework/TimedRobot.hpp>

#include "ReplaceMeCommand2.hpp"
#include "ReplaceMeEmptyClass.hpp"
#include "ReplaceMeInstantCommand2.hpp"
#include "ReplaceMeParallelCommandGroup.hpp"
#include "ReplaceMeParallelDeadlineGroup.hpp"
#include "ReplaceMeParallelRaceGroup.hpp"
#include "ReplaceMeSequentialCommandGroup.hpp"
#include "ReplaceMeSubsystem2.hpp"

class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    prosdk::cmd::CommandScheduler::GetInstance().Run();
  }

  void TeleopInit() override {
    prosdk::cmd::CommandScheduler::GetInstance().Schedule(
        {&command, &instantCommand, &sequentialGroup, &parallelGroup, &deadlineGroup,
         &raceGroup});
  }

 private:
  [[maybe_unused]] ReplaceMeEmptyClass emptyClass;
  ReplaceMeSubsystem2 subsystem;
  ReplaceMeCommand2 command;
  ReplaceMeInstantCommand2 instantCommand;
  ReplaceMeSequentialCommandGroup sequentialGroup;
  ReplaceMeParallelCommandGroup parallelGroup;
  ReplaceMeParallelDeadlineGroup deadlineGroup;
  ReplaceMeParallelRaceGroup raceGroup;
};

PROSDK_ROBOT_MAIN(Robot)
