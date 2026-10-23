// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetFlywheelBangBangController: flywheel velocity with a BangBangController plus
// feedforward (upstream wpilibcExamples/src/main/cpp/snippets/FlywheelBangBangController), ported
// to prosdk. The controller is on the
// Driver Station (/Tunables and /Telemetry BangBangController). SimulationPeriodic (FlywheelSim)
// only runs on the host; on the robot it is inert.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 38 flywheel PWMSparkMax, GPIO 39/40 encoder A/B.
// Joystick 0, axis 0 sets the velocity.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/BangBangController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/system/Models.hpp>
#include <prosdk/simulation/EncoderSim.hpp>
#include <prosdk/simulation/FlywheelSim.hpp>
#include <prosdk/system/RobotController.hpp>
#include <prosdk/telemetry/Telemetry.hpp>
#include <prosdk/tunables/Tunables.hpp>
#include <prosdk/units/moment_of_inertia.hpp>

/**
 * This is a sample program to demonstrate the use of a
 * prosdk::math::BangBangController with a flywheel to control velocity.
 */
class Robot : public prosdk::TimedRobot {
 public:
  /**
   * Controls flywheel to a set velocity (RPM) controlled by a joystick.
   */
  void TeleopPeriodic() override {
    // Scale setpoint value between 0 and maxSetpointValue
    prosdk::units::radians_per_second_t setpoint = prosdk::units::math::max(
        0_rpm, joystick.GetRawAxis(0) * MAX_SETPOINT_VALUE);

    // Set setpoint and measurement of the bang-bang controller
    prosdk::units::volt_t bangOutput =
        bangBangController.Calculate(encoder.GetRate(), setpoint.value()) *
        12_V;

    // Controls a motor with the output of the BangBang controller and a
    // feedforward. The feedforward is reduced slightly to avoid overspeeding
    // the shooter.
    flywheelMotor.SetVoltage(bangOutput +
                             0.9 * feedforward.Calculate(setpoint));

    prosdk::telemetry::Log("BangBangController", bangBangController);
  }

  Robot() {
    // Add bang-bang controller to SmartDashboard and networktables.
    prosdk::tunables::Publish("BangBangController", bangBangController);
  }

  /**
   * Update our simulation. This should be run every robot loop in simulation.
   */
  void SimulationPeriodic() override {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated velocities to our simulated encoder
    flywheelSim.SetInputVoltage(
        flywheelMotor.GetThrottle() *
        prosdk::units::volt_t{prosdk::RobotController::GetInputVoltage()});
    flywheelSim.Update(20_ms);
    encoderSim.SetRate(flywheelSim.GetAngularVelocity().value());
  }

 private:
  static constexpr int MOTOR_PORT = 38;
  static constexpr int ENCODER_A_CHANNEL = 39;
  static constexpr int ENCODER_B_CHANNEL = 40;

  // Max setpoint for joystick control
  static constexpr prosdk::units::radians_per_second_t MAX_SETPOINT_VALUE =
      6000_rpm;

  // Joystick to control setpoint
  prosdk::Joystick joystick{0};

  prosdk::PWMSparkMax flywheelMotor{MOTOR_PORT};
  prosdk::Encoder encoder{ENCODER_A_CHANNEL, ENCODER_B_CHANNEL};

  prosdk::math::BangBangController bangBangController;

  // Gains are for example purposes only - must be determined for your own
  // robot!
  static constexpr prosdk::units::volt_t FLYWHEEL_KS = 0.0001_V;
  static constexpr decltype(1_V / 1_rad_per_s) FLYWHEEL_KV = 0.000195_V / 1_rpm;
  static constexpr decltype(1_V / 1_rad_per_s_sq) FLYWHEEL_KA =
      0.0003_V / 1_rev_per_m_per_s;
  prosdk::math::SimpleMotorFeedforward<prosdk::units::radians> feedforward{
      FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA};

  // Simulation classes help us simulate our robot

  // Reduction between motors and encoder, as output over input. If the flywheel
  // spins slower than the motors, this number should be greater than one.
  static constexpr double FLYWHEEL_GEARING = 1.0;

  // 1/2 MR²
  static constexpr prosdk::units::kilogram_square_meter_t
      FLYWHEEL_MOMENT_OF_INERTIA = 0.5 * 1.5_lb * 4_in * 4_in;

  prosdk::math::DCMotor gearbox = prosdk::math::DCMotor::NEO(1);
  prosdk::math::LinearSystem<1, 1, 1> plant{
      prosdk::math::Models::FlywheelFromPhysicalConstants(
          gearbox, FLYWHEEL_MOMENT_OF_INERTIA, FLYWHEEL_GEARING)};

  prosdk::sim::FlywheelSim flywheelSim{plant, gearbox};
  prosdk::sim::EncoderSim encoderSim{encoder};
};

PROSDK_ROBOT_MAIN(Robot)
