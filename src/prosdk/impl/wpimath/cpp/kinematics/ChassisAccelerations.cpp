// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/ChassisAccelerations.hpp"

#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angular_acceleration.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json,
                        const ChassisAccelerations& accel) {
  json = prosdk::util::json::object("ax", accel.ax.value(), "ay", accel.ay.value(),
                                 "alpha", accel.alpha.value());
}

void prosdk::math::from_json(const prosdk::util::json& json,
                          ChassisAccelerations& accel) {
  accel = ChassisAccelerations{
      units::meters_per_second_squared_t{json.at("ax").get_number()},
      units::meters_per_second_squared_t{json.at("ay").get_number()},
      units::radians_per_second_squared_t{json.at("alpha").get_number()}};
}
