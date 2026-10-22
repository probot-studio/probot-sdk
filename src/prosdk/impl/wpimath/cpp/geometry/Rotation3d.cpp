// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/Rotation3d.hpp"

#include "prosdk/math/geometry/Quaternion.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json, const Rotation3d& rotation) {
  json = prosdk::util::json::object("quaternion", rotation.GetQuaternion());
}

void prosdk::math::from_json(const prosdk::util::json& json, Rotation3d& rotation) {
  rotation = Rotation3d{json.at("quaternion").get<Quaternion>()};
}
