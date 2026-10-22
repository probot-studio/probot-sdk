// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/util/ComputerVisionUtil.hpp"

#include <catch2/catch_test_macros.hpp>

#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/math/geometry/Translation3d.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/length.hpp"

TEST_CASE("ComputerVisionUtilTest ObjectToRobotPose", "[wpimath]") {
  prosdk::math::Pose3d robot{1_m, 2_m, 0_m,
                          prosdk::math::Rotation3d{0_deg, 0_deg, 30_deg}};
  prosdk::math::Transform3d cameraToObject{
      prosdk::math::Translation3d{1_m, 1_m, 1_m},
      prosdk::math::Rotation3d{0_deg, -20_deg, 45_deg}};
  prosdk::math::Transform3d robotToCamera{
      prosdk::math::Translation3d{1_m, 0_m, 2_m},
      prosdk::math::Rotation3d{0_deg, 0_deg, 25_deg}};
  prosdk::math::Pose3d object = robot + robotToCamera + cameraToObject;

  CHECK(robot ==
        prosdk::math::ObjectToRobotPose(object, cameraToObject, robotToCamera));
}
