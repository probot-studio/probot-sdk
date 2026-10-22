// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/system/Filesystem.hpp"

#include <string>

#include <system_error>

#include "prosdk/platform/Storage.hpp"
#include "prosdk/util/fs.hpp"

// prosdk: the SystemCore paths are replaced by the platform storage locations
// (prosdk/platform/Storage.hpp: LittleFS on the robot, the upstream simulation paths on the host).

std::string prosdk::filesystem::GetLaunchDirectory() {
  std::error_code ec;
  auto path = fs::current_path(ec);
  return ec ? std::string{"/"} : path.string();
}

std::string prosdk::filesystem::GetOperatingDirectory() {
  return prosdk::platform::GetOperatingDirectory();
}

std::string prosdk::filesystem::GetDeployDirectory() {
  return prosdk::platform::GetDeployDirectory();
}
