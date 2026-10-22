// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/commands2/button/Trigger.hpp"
#include "prosdk/driverstation/GenericHID.hpp"
#include "prosdk/driverstation/HIDDevice.hpp"
#include "prosdk/driverstation/POVDirection.hpp"

namespace prosdk::cmd {
/**
 * A class used to bind command scheduling to joystick POV presses.  Can be
 * composed with other buttons with the operators in Trigger.
 *
 * This class is provided by the Commands v2 VendorDep
 *
 * @see Trigger
 */
class POVButton : public Trigger {
 public:
  /**
   * Creates a POVButton that commands can be bound to.
   *
   * @param joystick The joystick on which the button is located.
   * @param angle The angle of the POV corresponding to a button press.
   * @param povNumber The number of the POV on the joystick.
   */
  POVButton(prosdk::GenericHID* joystick, prosdk::POVDirection angle,
            int povNumber = 0)
      : Trigger([joystick, angle, povNumber] {
          return joystick->GetPOV(povNumber) == angle;
        }) {}

  /**
   * Creates a POVButton that commands can be bound to.
   *
   * @param joystick The HID device on which the POV is located.
   * @param angle The angle of the POV corresponding to a button press.
   * @param povNumber The number of the POV on the HID device.
   */
  POVButton(prosdk::HIDDevice* joystick, prosdk::POVDirection angle,
            int povNumber = 0)
      : POVButton(&joystick->GetHID(), angle, povNumber) {}
};
}  // namespace prosdk::cmd
