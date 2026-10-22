// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/Rotation2d.hpp"

#include "prosdk/units/angle.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json, const Rotation2d& rotation) {
  json = prosdk::util::json::object("radians", rotation.Radians().value());
}

void prosdk::math::from_json(const prosdk::util::json& json, Rotation2d& rotation) {
  rotation = Rotation2d{prosdk::units::radian_t{json.at("radians").get_number()}};
}
