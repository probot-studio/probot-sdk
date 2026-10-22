// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/math/geometry/Pose2d.hpp"
#include "prosdk/math/kinematics/ChassisVelocities.hpp"
#include "prosdk/math/trajectory/HolonomicSample.hpp"
#include "prosdk/math/util/StateSpaceUtil.hpp"
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/math.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/util/SymbolExports.hpp"
#include "prosdk/util/array.hpp"

namespace prosdk::math {

/**
 * The linear time-varying unicycle controller has a similar form to the LQR,
 * but the model used to compute the controller gain is the nonlinear unicycle
 * model linearized around the drivetrain's current state.
 *
 * See section 8.9 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 8.9.1.
 */
class WPILIB_DLLEXPORT LTVUnicycleController {
 public:
  /**
   * Constructs a linear time-varying unicycle controller with default maximum
   * desired error tolerances of (x = 0.0625 m, y = 0.125 m, heading = 2 rad)
   * and default maximum desired control effort of (linear velocity = 1 m/s,
   * angular velocity = 2 rad/s).
   *
   * @param dt Discretization timestep.
   */
  explicit LTVUnicycleController(prosdk::units::second_t dt)
      : LTVUnicycleController{{0.0625, 0.125, 2.0}, {1.0, 2.0}, dt} {}

  /**
   * Constructs a linear time-varying unicycle controller.
   *
   * See
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
   * for how to select the tolerances.
   *
   * @param Qelems The maximum desired error tolerance for each state (x, y,
   *               heading).
   * @param Relems The maximum desired control effort for each input (linear
   *               velocity, angular velocity).
   * @param dt     Discretization timestep.
   */
  LTVUnicycleController(const prosdk::util::array<double, 3>& Qelems,
                        const prosdk::util::array<double, 2>& Relems,
                        prosdk::units::second_t dt)
      : m_Q{prosdk::math::CostMatrix(Qelems)},
        m_R{prosdk::math::CostMatrix(Relems)},
        m_dt{dt} {}

  /**
   * Move constructor.
   */
  LTVUnicycleController(LTVUnicycleController&&) = default;

  /**
   * Move assignment operator.
   */
  LTVUnicycleController& operator=(LTVUnicycleController&&) = default;

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  bool AtReference() const {
    const auto& eTranslate = m_poseError.Translation();
    const auto& eRotate = m_poseError.Rotation();
    const auto& tolTranslate = m_poseTolerance.Translation();
    const auto& tolRotate = m_poseTolerance.Rotation();
    return prosdk::units::math::abs(eTranslate.X()) < tolTranslate.X() &&
           prosdk::units::math::abs(eTranslate.Y()) < tolTranslate.Y() &&
           prosdk::units::math::abs(eRotate.Radians()) < tolRotate.Radians();
  }

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   */
  void SetTolerance(const Pose2d& poseTolerance) {
    m_poseTolerance = poseTolerance;
  }

  /**
   * Returns the linear and angular velocity outputs of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose        The current pose.
   * @param poseRef            The desired pose.
   * @param linearVelocityRef  The desired linear velocity.
   * @param angularVelocityRef The desired angular velocity.
   */
  ChassisVelocities Calculate(
      const Pose2d& currentPose, const Pose2d& poseRef,
      prosdk::units::meters_per_second_t linearVelocityRef,
      prosdk::units::radians_per_second_t angularVelocityRef);

  /**
   * Returns the linear and angular velocity outputs of the LTV controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * @param currentPose  The current pose.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   */
  ChassisVelocities Calculate(const Pose2d& currentPose,
                              const HolonomicSample& desiredState) {
    // The sample velocity is field-relative; the controller needs the
    // robot-relative forward (linear) velocity.
    auto linearVelocity =
        desiredState.velocity.ToRobotRelative(desiredState.pose.Rotation()).vx;
    return Calculate(currentPose, desiredState.pose, linearVelocity,
                     desiredState.velocity.omega);
  }

  /**
   * Enables and disables the controller for troubleshooting purposes.
   *
   * @param enabled If the controller is enabled or not.
   */
  void SetEnabled(bool enabled) { m_enabled = enabled; }

 private:
  // LQR cost matrices
  Eigen::Matrix<double, 3, 3> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  prosdk::units::second_t m_dt;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;
  bool m_enabled = true;
};

}  // namespace prosdk::math
