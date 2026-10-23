// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetProfiledPIDFeedforward: ProfiledPIDController with SimpleMotorFeedforward
// (upstream wpilibcExamples/src/main/cpp/snippets/ProfiledPIDFeedforward), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 39/40 encoder A/B, GPIO 38 PWMSparkMax (or any PWM
// ESC).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/ProfiledPIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

/**
 * prosdk::math::ProfiledPIDController with feedforward snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() { encoder.SetDistancePerPulse(1.0 / 256.0); }

  // Controls a simple motor's position using a
  // prosdk::math::SimpleMotorFeedforward and a prosdk::math::ProfiledPIDController
  void GoToPosition(prosdk::units::meter_t goalPosition) {
    auto pidVal = controller.Calculate(
        prosdk::units::meter_t{encoder.GetDistance()}, goalPosition);
    motor.SetVoltage(
        prosdk::units::volt_t{pidVal} +
        feedforward.Calculate(lastVelocity, controller.GetSetpoint().velocity));
    lastVelocity = controller.GetSetpoint().velocity;
  }

  void TeleopPeriodic() override {
    // Example usage: move to position 10.0 meters
    GoToPosition(10.0_m);
  }

 private:
  prosdk::math::ProfiledPIDController<prosdk::units::meters> controller{
      1.0, 0.0, 0.0, {5_mps, 10_mps_sq}};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> feedforward{
      0.5_V, 1.5_V / 1_mps, 0.3_V / 1_mps_sq};
  prosdk::Encoder encoder{39, 40};
  prosdk::PWMSparkMax motor{38};

  prosdk::units::meters_per_second_t lastVelocity = 0_mps;
};

PROSDK_ROBOT_MAIN(Robot)
