// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/commands2/CommandScheduler.hpp"
#include "prosdk/commands2/button/Trigger.hpp"
#include "prosdk/driverstation/DriverStation.hpp"
#include "prosdk/driverstation/GenericHID.hpp"
#include "prosdk/driverstation/POVDirection.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"

namespace prosdk::cmd {

/**
 * A version of {@link prosdk::GenericHID} with {@link Trigger} factories for
 * command-based.
 *
 * @see GenericHID
 */
class CommandGenericHID final {
 public:
  /**
   * Construct an instance of a device.
   *
   * @param port The port index on the Driver Station that the device is plugged
   * into.
   */
  explicit CommandGenericHID(int port);

  /**
   * Construct an instance of a device with a GenericHID object.
   *
   * @param hid The GenericHID object to use for this command HID.
   */
  explicit CommandGenericHID(prosdk::GenericHID& hid);

  /**
   * Gets the CommandGenericHID object for the given port. CommandGenericHID
   * objects are cached, so this will always return the same object for the same
   * port.
   *
   * @param port The port index on the Driver Station that the device is
   * plugged into.
   * @return The CommandGenericHID object for the given port.
   */
  static CommandGenericHID& GetCommandGenericHID(int port);

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  prosdk::GenericHID& GetHID();

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @param loop the event loop instance to attach the event to. Defaults to the
   * CommandScheduler's default loop.
   * @return an event instance representing the button's digital signal attached
   * to the given loop.
   */
  Trigger Button(int button,
                 prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                            .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around this angle of a POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @param angle POV angle.
   * @return a Trigger instance based around this angle of a POV on the HID.
   */
  Trigger POV(prosdk::POVDirection angle,
              prosdk::EventLoop* loop =
                  CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around this angle of a POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @param pov   index of the POV to read (starting at 0). Defaults to 0.
   * @param angle POV angle.
   * @return a Trigger instance based around this angle of a POV on the HID.
   */
  Trigger POV(int pov, prosdk::POVDirection angle,
              prosdk::EventLoop* loop =
                  CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the up direction of the
   * default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the up direction of a POV on the
   * HID.
   */
  Trigger POVUp(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                           .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the up right direction
   * of the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the up right direction of a POV on
   * the HID.
   */
  Trigger POVUpRight(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                                .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the right direction of
   * the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the right direction of a POV on the
   * HID.
   */
  Trigger POVRight(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                              .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the down right direction
   * of the default (index 0) POV on the HID.
   *
   * @return a Trigger instance based around the down right direction of a POV
   * on the HID.
   */
  Trigger POVDownRight(
      prosdk::EventLoop* loop =
          CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the down direction of
   * the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the down direction of a POV on
   * the HID.
   */
  Trigger POVDown(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                             .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the down left direction
   * of the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the down left direction of a POV on
   * the HID.
   */
  Trigger POVDownLeft(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                                 .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the left direction of
   * the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the left direction of a POV on
   * the HID.
   */
  Trigger POVLeft(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                             .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the up left direction
   * of the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the up left direction of a POV on
   * the HID.
   */
  Trigger POVUpLeft(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                               .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance based around the center (not pressed)
   * position of the default (index 0) POV on the HID.
   *
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance based around the center position of a POV on the
   * HID.
   */
  Trigger POVCenter(prosdk::EventLoop* loop = CommandScheduler::GetInstance()
                                               .GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance that is true when the axis value is less than
   * {@code threshold}, attached to {@link
   * CommandScheduler::GetDefaultButtonLoop() the default command scheduler
   * button loop}.
   * @param axis The axis to read, starting at 0.
   * @param threshold The value below which this trigger should return true.
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance that is true when the axis value is less than
   * the provided threshold.
   */
  Trigger AxisLessThan(
      int axis, double threshold,
      prosdk::EventLoop* loop =
          CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance that is true when the axis value is greater
   * than {@code threshold}, attached to {@link
   * CommandScheduler::GetDefaultButtonLoop() the default command scheduler
   * button loop}.
   * @param axis The axis to read, starting at 0.
   * @param threshold The value below which this trigger should return true.
   * @param loop  the event loop instance to attach the event to. Defaults to
   * {@link CommandScheduler::GetDefaultButtonLoop() the default command
   * scheduler button loop}.
   * @return a Trigger instance that is true when the axis value is greater than
   * the provided threshold.
   */
  Trigger AxisGreaterThan(
      int axis, double threshold,
      prosdk::EventLoop* loop =
          CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

  /**
   * Constructs a Trigger instance that is true when the axis magnitude value is
   * greater than {@code threshold}, attached to the given loop.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value above which this trigger should return true.
   * @param loop the event loop instance to attach the trigger to.
   * @return a Trigger instance that is true when the axis magnitude value is
   * greater than the provided threshold.
   */
  Trigger AxisMagnitudeGreaterThan(
      int axis, double threshold,
      prosdk::EventLoop* loop =
          CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

  /**
   * Set the rumble output for the HID.
   *
   * The DS currently supports 2 rumble values, left rumble and right rumble.
   *
   * @param type  Which rumble value to set
   * @param value The normalized value (0 to 1) to set the rumble to
   */
  void SetRumble(prosdk::GenericHID::RumbleType type, double value);

  /**
   * Get if the HID is connected.
   *
   * @return true if the HID is connected
   */
  bool IsConnected() const;

 private:
  prosdk::GenericHID* m_hid;
};
}  // namespace prosdk::cmd
