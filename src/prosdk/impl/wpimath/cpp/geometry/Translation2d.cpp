// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/Translation2d.hpp"

#include "prosdk/units/length.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json,
                        const Translation2d& translation) {
  json.set_object();
  json["x"] = translation.X().value();
  json["y"] = translation.Y().value();
}

void prosdk::math::from_json(const prosdk::util::json& json,
                          Translation2d& translation) {
  translation = Translation2d{prosdk::units::meter_t{json.at("x").get_number()},
                              prosdk::units::meter_t{json.at("y").get_number()}};
}
