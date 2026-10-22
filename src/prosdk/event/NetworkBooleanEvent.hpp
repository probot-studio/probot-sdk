// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string_view>

#include "prosdk/event/BooleanEvent.hpp"

// prosdk: NetworkTables is not ported. The boolean is a tunable
// (prosdk::tunables), which the Driver Station shows under /Tunables and can
// toggle; the NetworkTables types of the upstream constructors are replaced by
// a tunable name, a tunable table or a boolean tunable.
namespace prosdk::tunables {
template <typename T, typename... I>
class Tunable;
class TunableTable;
}  // namespace prosdk::tunables

namespace prosdk {
/**
 * A Button that uses a NetworkTable boolean field.
 *
 * prosdk: the field is a boolean tunable. The event is active while a Driver
 * Station is attached (upstream: while the NetworkTables instance is
 * connected) and the tunable is true. Events created from the same name share
 * one tunable, published with the value false by the first of them; it stays
 * published while one of them exists. Driver Station edits are applied by
 * prosdk::tunables::TunableRegistry::Update() (every robot loop), so poll the
 * event loop on the robot thread.
 */
class NetworkBooleanEvent : public BooleanEvent {
 public:
  /**
   * Creates a new event with the given boolean topic determining whether it is
   * active.
   *
   * prosdk: replaces the BooleanTopic constructor.
   *
   * @param loop the loop that polls this event
   * @param name The tunable name (for example "Buttons/Shoot"; Driver Station
   *     path /Tunables/Buttons/Shoot)
   */
  NetworkBooleanEvent(EventLoop* loop, std::string_view name);

  /**
   * Creates a new event with the given boolean subscriber determining whether
   * it is active.
   *
   * prosdk: replaces the BooleanSubscriber constructor. The event takes over
   * the tunable (for example prosdk::tunables::Add<bool>("Buttons/Shoot")); it
   * stays published while the event exists.
   *
   * @param loop the loop that polls this event
   * @param tunable The boolean tunable that provides the value
   */
  NetworkBooleanEvent(EventLoop* loop,
                      prosdk::tunables::Tunable<bool> tunable);

  /**
   * Creates a new event with the given boolean topic determining whether it is
   * active.
   *
   * prosdk: replaces the NetworkTable constructor.
   *
   * @param loop the loop that polls this event
   * @param table The tunable table that contains the tunable
   * @param topicName The tunable name within the table
   */
  NetworkBooleanEvent(EventLoop* loop, prosdk::tunables::TunableTable table,
                      std::string_view topicName);

  /**
   * Creates a new event with the given boolean topic determining whether it is
   * active.
   *
   * @param loop the loop that polls this event
   * @param tableName The tunable table name that contains the tunable
   * @param topicName The tunable name within the table
   */
  NetworkBooleanEvent(EventLoop* loop, std::string_view tableName,
                      std::string_view topicName);
};
}  // namespace prosdk
