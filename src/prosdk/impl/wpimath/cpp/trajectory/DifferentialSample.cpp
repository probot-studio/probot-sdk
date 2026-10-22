// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/trajectory/DifferentialSample.hpp"

#include <vector>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/kinematics/ChassisAccelerations.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/json.hpp"

void prosdk::math::to_json(prosdk::util::json& json,
                        const DifferentialSample& sample) {
  json = prosdk::util::json::object("time", sample.time.value(), "pose",
                                 sample.pose, "velocity", sample.velocity,
                                 "acceleration", sample.acceleration,
                                 "leftVelocity", sample.leftVelocity.value(),
                                 "rightVelocity", sample.rightVelocity.value());
}

void prosdk::math::from_json(const prosdk::util::json& json,
                          DifferentialSample& sample) {
  sample = DifferentialSample{
      prosdk::units::second_t{json.at("time").get_number()},
      json.at("pose").get<Pose2d>(),
      json.at("velocity").get<ChassisVelocities>(),
      json.at("acceleration").get<ChassisAccelerations>(),
      prosdk::units::meters_per_second_t{json.at("leftVelocity").get_number()},
      prosdk::units::meters_per_second_t{json.at("rightVelocity").get_number()}};
}

void prosdk::math::to_json(prosdk::util::json& json,
                        const std::vector<DifferentialSample>& samples) {
  json.set_array();
  for (const auto& sample : samples) {
    json.emplace_back(sample);
  }
}

void prosdk::math::from_json(const prosdk::util::json& json,
                          std::vector<DifferentialSample>& samples) {
  samples.clear();
  for (const auto& sample : json.get_array()) {
    samples.emplace_back(sample.get<DifferentialSample>());
  }
}
