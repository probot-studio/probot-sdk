// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/kinematics/proto/SwerveDriveKinematicsProto.hpp"

#include <catch2/catch_test_macros.hpp>

#include "../../ProtoTestBase.hpp"
#include "prosdk/math/kinematics/SwerveDriveKinematics.hpp"

using namespace prosdk::math;

struct SwerveDriveKinematicsProtoTestData {
  using Type = SwerveDriveKinematics<4>;

  inline static const Type TEST_DATA{prosdk::math::Translation2d{1.0_m, 0.9_m},
                                     prosdk::math::Translation2d{1.1_m, -0.8_m},
                                     prosdk::math::Translation2d{-1.2_m, 0.7_m},
                                     prosdk::math::Translation2d{-1.3_m, -0.6_m}};

  static void CheckEq(const Type& testData, const Type& data) {
    CHECK(testData.GetModules() == data.GetModules());
  }
};

INSTANTIATE_CATCH_TYPED_TEST_SUITE_P(SwerveDriveKinematics, ProtoTest,
                                     SwerveDriveKinematicsProtoTestData);
