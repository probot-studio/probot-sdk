// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/Pose3d.hpp"

#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/geometry/Translation3d.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json, const Pose3d& pose) {
  json.set_object();
  json["translation"] = pose.Translation();
  json["rotation"] = pose.Rotation();
}

void prosdk::math::from_json(const prosdk::util::json& json, Pose3d& pose) {
  pose = Pose3d{json.at("translation").get<Translation3d>(),
                json.at("rotation").get<Rotation3d>()};
}
