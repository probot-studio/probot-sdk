// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string_view>

#include "prosdk/commands2/button/Trigger.hpp"

namespace prosdk::cmd {
/**
 * A Button that uses a NetworkTable boolean field.
 *
 * prosdk: there are no NetworkTables. The button is a mutable boolean tunable
 * (default false) published at the given path when the button is constructed,
 * so it appears under /Tunables on the Driver Station, which sets and clears
 * it. The condition is true while a Driver Station is attached and the tunable
 * is true (upstream: while NetworkTables is connected and the topic is true).
 * All NetworkButtons with the same path share one tunable; it is removed when
 * the last of them (including the Trigger copies held by bindings) is
 * destroyed. Publishing needs a registered tunables backend (the robot program
 * registers it when the robot class is constructed). The constructors that
 * take NetworkTables types (BooleanTopic, BooleanSubscriber, NetworkTable,
 * NetworkTableInstance) are not available.
 *
 * This class is provided by the Commands v2 VendorDep
 */
class NetworkButton : public Trigger {
 public:
  /**
   * Creates a NetworkButton that commands can be bound to.
   *
   * prosdk: replaces the BooleanTopic overload.
   *
   * @param path The tunable path of the value (for example "Buttons/Shoot").
   */
  explicit NetworkButton(std::string_view path);

  /**
   * Creates a NetworkButton that commands can be bound to.
   *
   * @param table The table where the value is located (tunable path prefix).
   * @param field The field that is the value.
   */
  NetworkButton(std::string_view table, std::string_view field);
};
}  // namespace prosdk::cmd
