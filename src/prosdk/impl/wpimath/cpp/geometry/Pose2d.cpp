// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/geometry/Pose2d.hpp"

#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json, const Pose2d& pose) {
  json.set_object();
  json["translation"] = pose.Translation();
  json["rotation"] = pose.Rotation();
}

void prosdk::math::from_json(const prosdk::util::json& json, Pose2d& pose) {
  pose = Pose2d{json.at("translation").get<Translation2d>(),
                json.at("rotation").get<Rotation2d>()};
}
