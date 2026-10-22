// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/smartdashboard/Field2d.hpp"

#include <memory>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/units/angle.hpp"

struct Field2dTest {
  Field2dTest() {
    prosdk::tunables::TunableRegistry::Reset();
    prosdk::tunables::TunableRegistry::RegisterBackend("", mock);
  }

  ~Field2dTest() { prosdk::tunables::TunableRegistry::Reset(); }

  std::shared_ptr<prosdk::tunables::MockTunableBackend> mock =
      std::make_shared<prosdk::tunables::MockTunableBackend>();
};

static void CheckPose(const prosdk::math::Pose2d& actual,
                      const prosdk::math::Pose2d& expected) {
  CHECK(actual.X() == expected.X());
  CHECK(actual.Y() == expected.Y());
  CHECK(actual.Rotation().Degrees() == expected.Rotation().Degrees());
}

TEST_CASE_METHOD(Field2dTest,
                 "Field2dTest DashboardEditsRobotPoseThroughTunable",
                 "[wpilibc][smartdashboard]") {
  prosdk::Field2d field;
  prosdk::tunables::Publish("field", field);

  prosdk::math::Pose2d pose{1_m, 2_m, 30_deg};
  mock->SetStructVector<prosdk::math::Pose2d>("/field/Robot", std::vector{pose});
  prosdk::tunables::TunableRegistry::Update();

  CheckPose(field.GetRobotPose(), pose);
}

TEST_CASE_METHOD(Field2dTest,
                 "Field2dTest DashboardEditsObjectPosesThroughTunable",
                 "[wpilibc][smartdashboard]") {
  prosdk::Field2d field;
  prosdk::FieldObject2d* existing = field.GetObject("target");
  prosdk::tunables::Publish("field", field);

  prosdk::math::Pose2d first{1_m, 2_m, 30_deg};
  prosdk::math::Pose2d second{3_m, 4_m, 60_deg};

  mock->SetStructVector<prosdk::math::Pose2d>("/field/target",
                                           std::vector{first, second});
  prosdk::tunables::TunableRegistry::Update();

  auto existingPoses = existing->GetPoses();
  REQUIRE(existingPoses.size() == 2u);
  CheckPose(existingPoses[0], first);
  CheckPose(existingPoses[1], second);
}

TEST_CASE_METHOD(Field2dTest,
                 "Field2dTest DashboardEditsObjectCreatedAfterTunablePublish",
                 "[wpilibc][smartdashboard]") {
  prosdk::Field2d field;
  prosdk::tunables::Publish("field", field);
  prosdk::FieldObject2d* late = field.GetObject("target");

  prosdk::math::Pose2d pose{1_m, 2_m, 30_deg};
  mock->SetStructVector<prosdk::math::Pose2d>("/field/target", std::vector{pose});
  prosdk::tunables::TunableRegistry::Update();

  auto latePoses = late->GetPoses();
  REQUIRE(latePoses.size() == 1u);
  CheckPose(latePoses[0], pose);
}

TEST_CASE_METHOD(Field2dTest,
                 "Field2dTest MoveAssignPublishedFromUnpublishedRepublishes"
                 "ObjectTunables",
                 "[wpilibc][smartdashboard]") {
  prosdk::Field2d field;
  field.GetObject("stale");
  prosdk::tunables::Publish("field", field);

  {
    prosdk::Field2d replacement;
    prosdk::math::Pose2d robotPose{1_m, 2_m, 30_deg};
    prosdk::math::Pose2d targetPose{3_m, 4_m, 60_deg};
    replacement.SetRobotPose(robotPose);
    replacement.GetObject("target")->SetPose(targetPose);

    field = std::move(replacement);
  }

  CHECK_FALSE(mock->GetUid("/field/stale").has_value());

  auto robotPoses = mock->GetStructVector<prosdk::math::Pose2d>("/field/Robot");
  REQUIRE(robotPoses.size() == 1u);
  CheckPose(robotPoses[0], {1_m, 2_m, 30_deg});

  auto targetPoses = mock->GetStructVector<prosdk::math::Pose2d>("/field/target");
  REQUIRE(targetPoses.size() == 1u);
  CheckPose(targetPoses[0], {3_m, 4_m, 60_deg});

  prosdk::math::Pose2d editedPose{5_m, 6_m, 90_deg};
  mock->SetStructVector<prosdk::math::Pose2d>("/field/target",
                                           std::vector{editedPose});
  prosdk::tunables::TunableRegistry::Update();

  auto movedTargetPoses = field.GetObject("target")->GetPoses();
  REQUIRE(movedTargetPoses.size() == 1u);
  CheckPose(movedTargetPoses[0], editedPose);
}
