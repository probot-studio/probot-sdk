// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <string>
#include <string_view>
#include <vector>

namespace prosdk {

/**
 * The preferences class provides a relatively simple way to save important
 * values to the roboRIO to access the next time the roboRIO is booted.
 *
 * prosdk: the values are stored by prosdk/platform/PreferencesStore.hpp (no
 * NetworkTables): in NVS flash on the robot, which is rewritten on every change
 * (do not set changing values every loop), in memory or a JSON file on the host.
 * Not to be confused with the Arduino-ESP32 Preferences library (global class
 * ::Preferences); with `using namespace prosdk;` write prosdk::Preferences.
 *
 * This class is thread safe.
 */
class Preferences {
 public:
  /**
   * Returns a vector of all the keys.
   *
   * @return a vector of the keys
   */
  static std::vector<std::string> GetKeys();

  /**
   * Returns the string at the given key.  If this table does not have a value
   * for that position, then the given defaultValue will be returned.
   *
   * @param key          the key
   * @param defaultValue the value to return if none exists in the table
   * @return either the value in the table, or the defaultValue
   */
  static std::string GetString(std::string_view key,
                               std::string_view defaultValue = "");

  /**
   * Returns the int at the given key.  If this table does not have a value for
   * that position, then the given defaultValue value will be returned.
   *
   * @param key          the key
   * @param defaultValue the value to return if none exists in the table
   * @return either the value in the table, or the defaultValue
   */
  static int GetInt(std::string_view key, int defaultValue = 0);

  /**
   * Returns the double at the given key.  If this table does not have a value
   * for that position, then the given defaultValue value will be returned.
   *
   * @param key          the key
   * @param defaultValue the value to return if none exists in the table
   * @return either the value in the table, or the defaultValue
   */
  static double GetDouble(std::string_view key, double defaultValue = 0.0);

  /**
   * Returns the float at the given key.  If this table does not have a value
   * for that position, then the given defaultValue value will be returned.
   *
   * @param key          the key
   * @param defaultValue the value to return if none exists in the table
   * @return either the value in the table, or the defaultValue
   */
  static float GetFloat(std::string_view key, float defaultValue = 0.0);

  /**
   * Returns the boolean at the given key.  If this table does not have a value
   * for that position, then the given defaultValue value will be returned.
   *
   * @param key          the key
   * @param defaultValue the value to return if none exists in the table
   * @return either the value in the table, or the defaultValue
   */
  static bool GetBoolean(std::string_view key, bool defaultValue = false);

  /**
   * Returns the long (int64_t) at the given key.  If this table does not have a
   * value for that position, then the given defaultValue value will be
   * returned.
   *
   * @param key          the key
   * @param defaultValue the value to return if none exists in the table
   * @return either the value in the table, or the defaultValue
   */
  static int64_t GetLong(std::string_view key, int64_t defaultValue = 0);

  /**
   * Puts the given string into the preferences table.
   *
   * The value may not have quotation marks, nor may the key have any whitespace
   * nor an equals sign.
   *
   * @param key   the key
   * @param value the value
   */
  static void SetString(std::string_view key, std::string_view value);

  /**
   * Puts the given string into the preferences table if it doesn't
   * already exist.
   */
  static void InitString(std::string_view key, std::string_view value);

  /**
   * Puts the given int into the preferences table.
   *
   * The key may not have any whitespace nor an equals sign.
   *
   * @param key   the key
   * @param value the value
   */
  static void SetInt(std::string_view key, int value);

  /**
   * Puts the given int into the preferences table if it doesn't
   * already exist.
   */
  static void InitInt(std::string_view key, int value);

  /**
   * Puts the given double into the preferences table.
   *
   * The key may not have any whitespace nor an equals sign.
   *
   * @param key   the key
   * @param value the value
   */
  static void SetDouble(std::string_view key, double value);

  /**
   * Puts the given double into the preferences table if it doesn't
   * already exist.
   */
  static void InitDouble(std::string_view key, double value);

  /**
   * Puts the given float into the preferences table.
   *
   * The key may not have any whitespace nor an equals sign.
   *
   * @param key   the key
   * @param value the value
   */
  static void SetFloat(std::string_view key, float value);

  /**
   * Puts the given float into the preferences table if it doesn't
   * already exist.
   */
  static void InitFloat(std::string_view key, float value);

  /**
   * Puts the given boolean into the preferences table.
   *
   * The key may not have any whitespace nor an equals sign.
   *
   * @param key   the key
   * @param value the value
   */
  static void SetBoolean(std::string_view key, bool value);

  /**
   * Puts the given boolean into the preferences table if it doesn't
   * already exist.
   */
  static void InitBoolean(std::string_view key, bool value);

  /**
   * Puts the given long (int64_t) into the preferences table.
   *
   * The key may not have any whitespace nor an equals sign.
   *
   * @param key   the key
   * @param value the value
   */
  static void SetLong(std::string_view key, int64_t value);

  /**
   * Puts the given long into the preferences table if it doesn't
   * already exist.
   */
  static void InitLong(std::string_view key, int64_t value);

  /**
   * Returns whether or not there is a key with the given name.
   *
   * @param key the key
   * @return if there is a value at the given key
   */
  static bool ContainsKey(std::string_view key);

  /**
   * Remove a preference.
   *
   * @param key the key
   */
  static void Remove(std::string_view key);

  /**
   * Remove all preferences.
   */
  static void RemoveAll();

 private:
  Preferences() = default;
};

}  // namespace wpi
