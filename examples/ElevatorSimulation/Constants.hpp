// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/mass.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace Constants {

inline constexpr int MOTOR_PORT = 4;
inline constexpr int ENCODER_A_CHANNEL = 6;
inline constexpr int ENCODER_B_CHANNEL = 7;
inline constexpr int JOYSTICK_PORT = 0;

inline constexpr double ELEVATOR_KP = 5.0;
inline constexpr double ELEVATOR_KI = 0.0;
inline constexpr double ELEVATOR_KD = 0.0;

inline constexpr prosdk::units::volt_t ELEVATOR_KS = 0.0_V;
inline constexpr prosdk::units::volt_t ELEVATOR_KG = 0.762_V;
inline constexpr auto ELEVATOR_KV = 0.762_V / 1_mps;
inline constexpr auto ELEVATOR_KA = 0.0_V / 1_mps_sq;

inline constexpr double ELEVATOR_GEARING = 10.0;
inline constexpr prosdk::units::meter_t ELEVATOR_DRUM_RADIUS = 2_in;
inline constexpr prosdk::units::kilogram_t CARRIAGE_MASS = 4.0_kg;

inline constexpr prosdk::units::meter_t SETPOINT = 75_cm;
inline constexpr prosdk::units::meter_t MIN_ELEVATOR_HEIGHT = 0_cm;
inline constexpr prosdk::units::meter_t MAX_ELEVATOR_HEIGHT = 1.25_m;

// distance per pulse = (distance per revolution) / (pulses per revolution)
//  = (Pi * D) / ppr
inline constexpr double ARM_ENCODER_DIST_PER_PULSE =
    2.0 * std::numbers::pi * ELEVATOR_DRUM_RADIUS.value() / 4096.0;

}  // namespace Constants
