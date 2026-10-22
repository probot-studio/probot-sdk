// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/PrintCommand.hpp"

#include <string>

#include "prosdk/platform/Console.hpp"

using namespace prosdk::cmd;

PrintCommand::PrintCommand(std::string_view message)
    : CommandHelper{
          // prosdk: to the platform console (Driver Station console on the
          // robot) instead of stdout
          [str = std::string(message).append("\n")] {
            prosdk::platform::ConsoleWrite(str);
          },
          {}} {
}

bool PrintCommand::RunsWhenDisabled() const {
  return true;
}
