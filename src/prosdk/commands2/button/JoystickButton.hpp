// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "prosdk/commands2/button/Trigger.hpp"
#include "prosdk/driverstation/GenericHID.hpp"
#include "prosdk/driverstation/HIDDevice.hpp"

namespace prosdk::cmd {
/**
 * A class used to bind command scheduling to joystick button presses.  Can be
 * composed with other buttons with the operators in Trigger.
 *
 * This class is provided by the Commands v2 VendorDep
 *
 * @see Trigger
 */
class JoystickButton : public Trigger {
 public:
  /**
   * Creates a JoystickButton that commands can be bound to.
   *
   * @param joystick The joystick on which the button is located.
   * @param buttonNumber The number of the button on the joystick.
   */
  explicit JoystickButton(prosdk::GenericHID* joystick, int buttonNumber)
      : Trigger([joystick, buttonNumber] {
          return joystick->GetRawButton(buttonNumber);
        }) {}

  /**
   * Creates a JoystickButton that commands can be bound to.
   *
   * @param joystick The HID device on which the button is located.
   * @param buttonNumber The number of the button on the HID device.
   */
  explicit JoystickButton(prosdk::HIDDevice* joystick, int buttonNumber)
      : JoystickButton(&joystick->GetHID(), buttonNumber) {}
};
}  // namespace prosdk::cmd
