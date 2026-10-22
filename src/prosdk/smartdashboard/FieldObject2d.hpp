// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <initializer_list>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/trajectory/TrajectorySample.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/SmallVector.hpp"
#include "prosdk/util/mutex.hpp"

namespace prosdk::math {
template <TrajectorySample SampleType>
class Trajectory;
}

namespace prosdk {

class Field2d;

/**
 * Game field object on a Field2d.
 */
class FieldObject2d {
  friend class Field2d;
  struct private_init {};

 public:
  FieldObject2d(std::string_view name, const private_init&)
      : m_name{name}, m_posesTunable{m_mutex} {}

  FieldObject2d(FieldObject2d&& rhs);
  FieldObject2d& operator=(FieldObject2d&& rhs);

  /**
   * Set the pose from a Pose object.
   *
   * @param pose 2D pose
   */
  void SetPose(const prosdk::math::Pose2d& pose);

  /**
   * Set the pose from x, y, and rotation.
   *
   * @param x X location
   * @param y Y location
   * @param rotation rotation
   */
  void SetPose(prosdk::units::meter_t x, prosdk::units::meter_t y,
               prosdk::math::Rotation2d rotation);

  /**
   * Get the pose.
   *
   * @return 2D pose, or 0,0,0 if unknown / does not exist
   */
  prosdk::math::Pose2d GetPose() const;

  /**
   * Set multiple poses from an array of Pose objects.
   * The total number of poses is limited to 85.
   *
   * @param poses array of 2D poses
   */
  void SetPoses(std::span<const prosdk::math::Pose2d> poses);

  /**
   * Set multiple poses from an array of Pose objects.
   * The total number of poses is limited to 85.
   *
   * @param poses array of 2D poses
   */
  void SetPoses(std::initializer_list<prosdk::math::Pose2d> poses);

  /**
   * Sets poses from a trajectory.
   *
   * @param trajectory The trajectory from which poses should be added.
   */
  template <prosdk::math::TrajectorySample SampleType>
  void SetTrajectory(const prosdk::math::Trajectory<SampleType>& trajectory) {
    std::vector<prosdk::math::Pose2d> poses;
    poses.reserve(trajectory.Samples().size());
    for (auto&& state : trajectory.Samples()) {
      poses.push_back(state.pose);
    }
    m_posesTunable.Set(std::move(poses));
  }

  /**
   * Get multiple poses.
   *
   * @return vector of 2D poses
   */
  std::vector<prosdk::math::Pose2d> GetPoses() const;

  /**
   * Get multiple poses.
   *
   * @param out output prosdk::util::SmallVector to hold 2D poses
   * @return span referring to output prosdk::util::SmallVector
   */
  std::span<const prosdk::math::Pose2d> GetPoses(
      prosdk::util::SmallVectorImpl<prosdk::math::Pose2d>& out) const;

 private:
  class PosesTunable
      : public prosdk::tunables::Tunable<std::vector<prosdk::math::Pose2d>> {
   public:
    explicit PosesTunable(prosdk::util::mutex& mutex);
    PosesTunable(prosdk::util::mutex& mutex, PosesTunable&& rhs);
    PosesTunable& operator=(PosesTunable&& rhs);

    std::vector<prosdk::math::Pose2d> Get() const;
    void Set(std::span<const prosdk::math::Pose2d> poses);
    void Set(std::vector<prosdk::math::Pose2d> poses);

   private:
    using Base = prosdk::tunables::Tunable<std::vector<prosdk::math::Pose2d>>;

    size_t GetStructSize() const override;
    bool UnpackStruct(std::span<const uint8_t> data) override;
    void PackStruct(std::span<uint8_t> buf) const override;

    prosdk::util::mutex* m_mutex;
  };

  mutable prosdk::util::mutex m_mutex;
  std::string m_name;
  PosesTunable m_posesTunable;
};

}  // namespace wpi
