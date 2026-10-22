// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// prosdk parts of the fields and apriltag libraries that the upstream C++ tests do not cover:
// Field(path) file loading (patch fields/0001), the built-in field resources, the empty field
// image registry (platform/FieldImages.cpp) and a full AprilTagDetector + AprilTagPoseEstimator
// run on a synthetic image (the upstream C++ detector test only checks the configuration).

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "prosdk/apriltag/AprilTagDetector.hpp"
#include "prosdk/apriltag/AprilTagPoseEstimator.hpp"
#include "prosdk/fields/Field.hpp"
#include "prosdk/fields/field_images.hpp"
#include "prosdk/fields/fields.hpp"
#include "prosdk/thirdparty/apriltag/apriltag.h"
#include "prosdk/thirdparty/apriltag/tag36h11.h"
#include "prosdk/util/json.hpp"

using namespace prosdk::fields;
using Catch::Approx;

TEST_CASE("Fields DefaultFieldFromResources", "[fields]") {
  Field field = GetField(FieldId::DEFAULT_FIELD);
  CHECK(field.GetName() == "2026 FRC Rebuilt Welded");
  CHECK(field.GetName() == GetFieldName(FieldId::FRC_2026_REBUILT_WELDED));
  REQUIRE(field.HasTags());
  CHECK(field.GetTags().size() == 32);
  CHECK(field.GetResourceFile() == "frc/2026-rebuilt-welded.json");

  // the compiled-in JSON resource describes the same field as the generated constants
  std::string_view json = field.GetJson();
  REQUIRE_FALSE(json.empty());
  Field fromJson = prosdk::util::json::parse(json).value().get<Field>();
  CHECK(fromJson.GetName() == field.GetName());
  CHECK(fromJson.GetLength() == field.GetLength());
  CHECK(fromJson.GetTags() == field.GetTags());

  CHECK(GetFields().size() == static_cast<size_t>(FieldId::NUM_FIELDS));
}

TEST_CASE("Fields NoImagesButImageMetadata", "[fields]") {
  Field field = GetField(FieldId::FTC_2025_2026_DECODE);
  REQUIRE(field.HasImage());
  CHECK(field.GetImage()->GetPath() == "ftc/2025-2026-decode.png");
  CHECK(GetFieldImages().empty());
  CHECK(GetFieldImage(field.GetImage()->GetPath()).empty());
}

TEST_CASE("Fields SerializeAndLoadFile", "[fields]") {
  std::string path = "prosdk_fields_test_layout.json";
  Field field = GetField(FieldId::FRC_2025_REEFSCAPE_ANDY_MARK);
  field.Serialize(path);

  Field loaded{path};
  std::remove(path.c_str());
  CHECK(loaded.GetName() == field.GetName());
  CHECK(loaded.GetSeason() == field.GetSeason());
  CHECK(loaded.GetImage() == field.GetImage());
  CHECK(loaded.GetTags() == field.GetTags());
  CHECK_FALSE(loaded.GetJson().empty());
  CHECK(loaded.GetTagPose(18) == field.GetTagPose(18));

  CHECK_THROWS(Field{"prosdk_fields_test_missing.json"});
}

TEST_CASE("AprilTag DetectAndEstimateSyntheticTag", "[apriltag]") {
  // tag36h11 id 7: 10x10 cells (8 cells between the outer black edges), 20 px per cell, centered
  // in a white 400x400 image
  constexpr int kSize = 400;
  constexpr int kCell = 20;
  apriltag_family_t* family = tag36h11_create();
  image_u8_t* tag = apriltag_to_image(family, 7);
  REQUIRE(tag->width == 10);
  std::vector<uint8_t> image(kSize * kSize, 255);
  const int origin = kSize / 2 - tag->width * kCell / 2;
  for (int y = 0; y < tag->height * kCell; ++y) {
    for (int x = 0; x < tag->width * kCell; ++x) {
      image[(origin + y) * kSize + origin + x] =
          tag->buf[(y / kCell) * tag->stride + x / kCell];
    }
  }
  image_u8_destroy(tag);
  tag36h11_destroy(family);

  prosdk::apriltag::AprilTagDetector detector;
  REQUIRE(detector.AddFamily("tag36h11"));
  auto results = detector.Detect(kSize, kSize, image.data());
  REQUIRE(results.size() == 1);
  const auto& det = *results[0];
  CHECK(det.GetId() == 7);
  CHECK(det.GetFamily() == "tag36h11");
  CHECK(det.GetCenter().x == Approx(kSize / 2.0).margin(1.0));

  // fx = 500 px, tag edge 8 * 20 = 160 px, tag size 0.16 m -> 0.5 m straight ahead
  prosdk::apriltag::AprilTagPoseEstimator estimator{
      {prosdk::units::meter_t{0.16}, 500.0, 500.0, kSize / 2.0, kSize / 2.0}};
  auto pose = estimator.Estimate(det);
  CHECK(pose.Z().value() == Approx(0.5).margin(0.01));
  CHECK(pose.X().value() == Approx(0.0).margin(0.01));
  CHECK(pose.Y().value() == Approx(0.0).margin(0.01));

  auto homography = estimator.EstimateHomography(det.GetHomography());
  CHECK(homography.Z().value() == Approx(0.5).margin(0.02));
}
