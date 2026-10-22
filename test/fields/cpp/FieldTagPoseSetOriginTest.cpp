// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/fields/FieldTag.hpp"
#include "prosdk/fields/fields.hpp"
#include "prosdk/math/geometry/Pose3d.hpp"
#include "prosdk/util/json.hpp"

using namespace prosdk::fields;

TEST_CASE("FieldTagPoseSetOriginTest TransformationMatches",
          "[field-tag][field-layout][pose]") {
  auto field = Field{
      "2027 FRC Test Field",
      "2027",
      "Test Field",
      std::nullopt,
      54_ft,
      27_ft,
      "frc",
      std::vector<FieldTag>{
          FieldTag{
              1, prosdk::math::Pose3d{0_ft, 0_ft, 0_ft,
                                   prosdk::math::Rotation3d{0_deg, 0_deg, 0_deg}}},
          FieldTag{2, prosdk::math::Pose3d{
                          4_ft, 4_ft, 4_ft,
                          prosdk::math::Rotation3d{0_deg, 0_deg, 180_deg}}}}};

  field.SetOrigin(Field::OriginPosition::RED_ALLIANCE_WALL_RIGHT_SIDE);

  auto mirrorPose = prosdk::math::Pose3d{
      54_ft, 27_ft, 0_ft, prosdk::math::Rotation3d{0_deg, 0_deg, 180_deg}};
  CHECK(mirrorPose == *field.GetTagPose(1));
  mirrorPose = prosdk::math::Pose3d{50_ft, 23_ft, 4_ft,
                                 prosdk::math::Rotation3d{0_deg, 0_deg, 0_deg}};
  CHECK(mirrorPose == *field.GetTagPose(2));
}
