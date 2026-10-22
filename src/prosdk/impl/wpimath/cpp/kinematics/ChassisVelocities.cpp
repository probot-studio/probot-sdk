// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/ChassisVelocities.hpp"

#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json,
                        const ChassisVelocities& velocities) {
  json = prosdk::util::json::object("vx", velocities.vx.value(), "vy",
                                 velocities.vy.value(), "omega",
                                 velocities.omega.value());
}

void prosdk::math::from_json(const prosdk::util::json& json,
                          ChassisVelocities& velocities) {
  velocities = ChassisVelocities{
      prosdk::units::meters_per_second_t{json.at("vx").get_number()},
      prosdk::units::meters_per_second_t{json.at("vy").get_number()},
      prosdk::units::radians_per_second_t{json.at("omega").get_number()}};
}
