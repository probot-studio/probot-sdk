// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// probot-core gamepad -> WPILib HAL joystick (design/hal-esp32.md, "Driver Station"). Used by the
// ESP32 Driver Station backend for every port on every new Driver Station packet; compiled on the
// host as well so the mapping is unit-tested against a fake CoreOps table.

#pragma once

#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/platform/CoreOps.hpp"

namespace prosdk::platform {

/** WPILib Gamepad (SDL) button indices the conversion fills. */
enum CoreJoystickButton : int {
  kCoreJoystickFaceDown = 0,
  kCoreJoystickFaceRight = 1,
  kCoreJoystickFaceLeft = 2,
  kCoreJoystickFaceUp = 3,
  kCoreJoystickBack = 4,
  kCoreJoystickGuide = 5,
  kCoreJoystickStart = 6,
  kCoreJoystickLeftStick = 7,
  kCoreJoystickRightStick = 8,
  kCoreJoystickLeftBumper = 9,
  kCoreJoystickRightBumper = 10,
  kCoreJoystickDpadUp = 11,
  kCoreJoystickDpadDown = 12,
  kCoreJoystickDpadLeft = 13,
  kCoreJoystickDpadRight = 14,
};

/** WPILib Gamepad (SDL) axis indices the conversion fills. */
enum CoreJoystickAxis : int {
  kCoreJoystickLeftX = 0,
  kCoreJoystickLeftY = 1,
  kCoreJoystickRightX = 2,
  kCoreJoystickRightY = 3,
  kCoreJoystickLeftTrigger = 4,
  kCoreJoystickRightTrigger = 5,
};

/** One HAL joystick as the Driver Station backend stores it. */
struct CoreJoystickData {
  HAL_JoystickAxes axes;
  HAL_JoystickPOVs povs;
  HAL_JoystickButtons buttons;
  HAL_JoystickDescriptor descriptor;
};

/**
 * Converts one core gamepad snapshot into a HAL joystick (`out` is cleared first):
 *  - buttons and sticks through the snapshot's mapping profile into WPILib's Gamepad layout
 *    (Y axes down-positive);
 *  - trigger axes = core's trigger values (0..1, core's deadzone applied), raw = the analog
 *    button value scaled to 0..32767;
 *  - POV 0 and the D-pad buttons from core's 8-way POV (a direction is pressed on its two
 *    diagonals as well);
 *  - descriptor: core's gamepad name (UTF-8, at most 63 bytes); a gamepad the browser reports
 *    with the standard mapping is a gamepad of type HIDType::STANDARD, any other one a generic
 *    HID (not a gamepad, type UNKNOWN); no rumble or LED outputs.
 * A snapshot without axes and buttons (core neutralized the port) leaves the joystick empty,
 * i.e. disconnected.
 */
void ConvertCoreGamepad(const CoreGamepad& pad, CoreJoystickData* out) noexcept;

/**
 * Reads Driver Station port `port` (0..kCoreMaxGamepads-1) through `ops.rawGamepad` and converts
 * it. Returns false and leaves `out` empty when core has no frame for the port or the port is out
 * of range.
 */
bool ReadCoreJoystick(const CoreOps& ops, int port, CoreJoystickData* out) noexcept;

/** core POV degrees (-1, 0..315 in steps of 45) -> HAL POV bit mask; anything else is centered. */
HAL_JoystickPOV CorePovToHal(int povDegrees) noexcept;

/** True when `data` is a connected joystick (it has an axis, a button or a POV). */
bool CoreJoystickConnected(const CoreJoystickData& data) noexcept;

/**
 * The ports of `connected` (bit n = port n) that no longer read as connected joysticks. Core
 * neutralizes a port when its frames stop for PROBOT_INPUT_TIMEOUT_MS (500 ms: a backgrounded
 * Driver Station tab, an unplugged gamepad) or the Driver Station releases the robot, without a
 * new packet; the Driver Station backend checks this every cycle so that it never keeps the
 * last stick values of a lost gamepad. `scratch` is overwritten.
 */
uint8_t LostCoreJoysticks(const CoreOps& ops, uint8_t connected,
                          CoreJoystickData* scratch) noexcept;

}  // namespace prosdk::platform
