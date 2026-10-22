// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos.hpp"

#include "ExampleCommand.hpp"
#include <prosdk/commands2/Commands.hpp>

prosdk::cmd::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return prosdk::cmd::Sequence(subsystem->ExampleMethodCommand(),
                            ExampleCommand(subsystem).ToPtr());
}
