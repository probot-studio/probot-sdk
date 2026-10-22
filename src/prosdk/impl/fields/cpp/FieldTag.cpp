// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/fields/FieldTag.hpp"

#include "prosdk/util/json.hpp"

using namespace prosdk::fields;

void prosdk::fields::to_json(prosdk::util::json& json, const FieldTag& tag) {
  json = prosdk::util::json::object("ID", tag.ID, "pose", tag.pose);
}

void prosdk::fields::from_json(const prosdk::util::json& json, FieldTag& tag) {
  tag.ID = json.at("ID").get_int();
  tag.pose = json.at("pose").get<prosdk::math::Pose3d>();
}
