// AprilTagFieldPose: the fields and apriltag libraries on the robot.
//
// Loads the current-season field (prosdk::fields::GetField(FieldId::DEFAULT_FIELD); the field
// metadata and tag poses are compiled into the library) and turns one AprilTag detection into a
// robot pose on that field with AprilTagPoseEstimator and ComputerVisionUtil.
//
// The kit has no camera, so the detection is a fixed homography and corner set, as
// AprilTagDetector::Detect() reports it for tag 26 seen straight on from 0.52 m by a
// 400x400-pixel camera (fx = fy = 500 px). With a camera board, pass each grayscale frame to
// m_detector.Detect(width, height, pixels) and use the detections it returns instead.
//
// The tag ID, tag pose and estimated robot pose go to the Driver Station (/Telemetry/Vision).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/apriltag/AprilTagDetector.hpp>
#include <prosdk/apriltag/AprilTagPoseEstimator.hpp>
#include <prosdk/fields/fields.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/math/geometry/CoordinateSystem.hpp>
#include <prosdk/math/util/ComputerVisionUtil.hpp>

namespace {

constexpr int kTagId = 26;

// Homography (row-major 3x3) and corners (x, y of the four corners) of the detection
constexpr double kHomography[9] = {80.0, 0.0, 200.0, 0.0, 80.0, 200.0, 0.0, 0.0, 1.0};
constexpr double kCorners[8] = {120.0, 280.0, 280.0, 280.0, 280.0, 120.0, 120.0, 120.0};

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    m_detector.AddFamily("tag36h11");
    prosdk::telemetry::Log("Vision/Field", std::string{m_field.GetName()});
  }

  void RobotPeriodic() override {
    auto tagPose = m_field.GetTagPose(kTagId);
    if (!tagPose) {
      return;
    }
    // camera frame (x right, y down, z forward) to WPILib (x forward, y left, z up)
    auto cameraToTag = prosdk::math::CoordinateSystem::Convert(
        m_estimator.Estimate(kHomography, kCorners), prosdk::math::CoordinateSystem::EDN(),
        prosdk::math::CoordinateSystem::NWU());
    // the estimator's tag frame has +X into the tag; a field tag pose has +X out of the tag:
    // turn the tag frame 180 degrees about its own Z axis
    cameraToTag = prosdk::math::Transform3d{
        cameraToTag.Translation(),
        prosdk::math::Rotation3d{0_deg, 0_deg, 180_deg}.RotateBy(cameraToTag.Rotation())};
    auto robotPose = prosdk::math::ObjectToRobotPose(*tagPose, cameraToTag, m_robotToCamera);

    prosdk::telemetry::Log("Vision/Tag ID", kTagId);
    prosdk::telemetry::Log("Vision/Tag pose", tagPose->ToPose2d());
    prosdk::telemetry::Log("Vision/Robot pose", robotPose.ToPose2d());
  }

 private:
  prosdk::fields::Field m_field = prosdk::fields::GetField(prosdk::fields::FieldId::DEFAULT_FIELD);
  prosdk::apriltag::AprilTagDetector m_detector;
  // 6.5 in tags (FRC); camera intrinsics in pixels
  prosdk::apriltag::AprilTagPoseEstimator m_estimator{{6.5_in, 500.0, 500.0, 200.0, 200.0}};
  // camera 10 cm ahead of the robot center, 20 cm up, looking forward
  prosdk::math::Transform3d m_robotToCamera{0.1_m, 0_m, 0.2_m, prosdk::math::Rotation3d{}};
};

PROSDK_ROBOT_MAIN(Robot)
