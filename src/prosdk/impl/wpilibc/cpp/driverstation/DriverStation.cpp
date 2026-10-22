// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/driverstation/DriverStation.hpp"

#include <array>
#include <memory>
#include <mutex>

#include "prosdk/driverstation/Gamepad.hpp"
#include "prosdk/driverstation/GenericHID.hpp"
#include "prosdk/system/Errors.hpp"

using namespace prosdk;

namespace {
std::mutex dsMutex;
std::array<std::unique_ptr<GenericHID>,
           prosdk::internal::DriverStationBackend::JOYSTICK_PORTS>
    hids;
std::array<std::unique_ptr<Gamepad>,
           prosdk::internal::DriverStationBackend::JOYSTICK_PORTS>
    gamepads;

void ValidatePort(int port) {
  if (port < 0 || port >= prosdk::internal::DriverStationBackend::JOYSTICK_PORTS) {
    throw WPILIB_MakeError(warn::BadJoystickIndex, "port {} out of range",
                           port);
  }
}
}  // namespace

GenericHID& DriverStation::GetGenericHIDUnderLock(int port) {
  ValidatePort(port);

  if (!hids[port]) {
    hids[port].reset(new GenericHID(port));
  }

  return *hids[port];
}

GenericHID& DriverStation::GetGenericHID(int port) {
  std::scoped_lock lock{dsMutex};
  return GetGenericHIDUnderLock(port);
}

Gamepad& DriverStation::GetGamepad(int port) {
  std::scoped_lock lock{dsMutex};
  ValidatePort(port);

  if (!gamepads[port]) {
    gamepads[port] = std::make_unique<Gamepad>(GetGenericHIDUnderLock(port));
  }

  return *gamepads[port];
}

void prosdk::internal::DriverStationBackend::ResetCachedHIDData() {
  std::scoped_lock lock{dsMutex};

  for (auto& gamepad : gamepads) {
    gamepad.reset();
  }
  for (auto& hid : hids) {
    hid.reset();
  }
}
