// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

/** WPILib FileSystem namespace */
namespace prosdk::filesystem {

/**
 * Obtains the current working path that the program was launched with.
 * This is analogous to the `pwd` command on unix.
 *
 * @return The result of the current working path lookup.
 */
std::string GetLaunchDirectory();

/**
 * Obtains the operating directory of the program. On the robot (prosdk), this
 * is the LittleFS mount point /littlefs. In simulation, it is where the
 * simulation was launched from (`pwd`). Configurable with
 * prosdk::platform::SetOperatingDirectory() (prosdk/platform/Storage.hpp).
 *
 * @return The result of the operating directory lookup.
 */
std::string GetOperatingDirectory();

/**
 * Obtains the deploy directory of the program, which is the remote location
 * src/main/deploy is deployed to by default. On the robot (prosdk), this is
 * /littlefs/deploy (the sketch's data/deploy folder after a LittleFS upload).
 * In simulation, it is where the simulation was launched from, in the
 * subdirectory "src/main/deploy" (`pwd`/src/main/deploy). Configurable with
 * prosdk::platform::SetDeployDirectory() (prosdk/platform/Storage.hpp).
 *
 * @return The result of the operating directory lookup
 */
std::string GetDeployDirectory();

}  // namespace prosdk::filesystem
