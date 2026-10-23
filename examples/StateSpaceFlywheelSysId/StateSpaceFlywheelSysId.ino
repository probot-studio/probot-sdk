// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// StateSpaceFlywheelSysId: a flywheel velocity loop built from a LinearSystemLoop (LQR + Kalman
// filter) on a
// flywheel model identified with SysId (kV, kA) (upstream
// wpilibcExamples/src/main/cpp/examples/StateSpaceFlywheelSysId), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): PWMSparkMax (or any PWM ESC) on GPIO 38,
// quadrature encoder A/B on GPIO 39/40 (upstream: PWM 0, DIO 0/1).
// Gamepad 0: hold the right bumper to spin up to 500 rpm, release to spin down.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/LinearQuadraticRegulator.hpp>
#include <prosdk/math/estimator/KalmanFilter.hpp>
#include <prosdk/math/system/LinearSystemLoop.hpp>
#include <prosdk/math/system/Models.hpp>

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control a flywheel.
 */
class Robot : public prosdk::TimedRobot {
  static constexpr int MOTOR_PORT = 38;
  static constexpr int ENCODER_A_CHANNEL = 39;
  static constexpr int ENCODER_B_CHANNEL = 40;
  static constexpr int JOYSTICK_PORT = 0;
  static constexpr prosdk::units::radians_per_second_t SPINUP = 500_rpm;

  // Volts per (radian per second)
  static constexpr auto FLYWHEEL_KV = 0.023_V / 1_rad_per_s;

  // Volts per (radian per second squared)
  static constexpr auto FLYWHEEL_KA = 0.001_V / 1_rad_per_s_sq;

  // The plant holds a state-space model of our flywheel. This system has the
  // following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  prosdk::math::LinearSystem<1, 1, 1> flywheelPlant =
      prosdk::math::Models::FlywheelFromSysId(FLYWHEEL_KV, FLYWHEEL_KA);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  prosdk::math::KalmanFilter<1, 1, 1> observer{
      flywheelPlant,
      {3.0},   // How accurate we think our model is
      {0.01},  // How accurate we think our encoder data is
      20_ms};

  // A LQR uses feedback to create voltage commands.
  prosdk::math::LinearQuadraticRegulator<1, 1> controller{
      flywheelPlant,
      // qelms. Velocity error tolerance, in radians per second. Decrease this
      // to more heavily penalize state excursion, or make the controller behave
      // more aggressively.
      {8.0},
      // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less
      // aggressive. 12 is a good starting point because that is the
      // (approximate) maximum voltage of a battery.
      {12.0},
      // Nominal time between loops. 20ms for TimedRobot, but can be lower if
      // using notifiers.
      20_ms};

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  prosdk::math::LinearSystemLoop<1, 1, 1> loop{flywheelPlant, controller, observer,
                                            12_V, 20_ms};

  // An encoder set up to measure flywheel velocity in radians per second.
  prosdk::Encoder encoder{ENCODER_A_CHANNEL, ENCODER_B_CHANNEL};

  prosdk::PWMSparkMax motor{MOTOR_PORT};
  prosdk::Gamepad joystick{JOYSTICK_PORT};

 public:
  Robot() {
    // We go 2 pi radians per 4096 clicks.
    encoder.SetDistancePerPulse(2.0 * std::numbers::pi / 4096.0);
  }

  void TeleopInit() override {
    loop.Reset(prosdk::math::Vectord<1>{encoder.GetRate()});
  }

  void TeleopPeriodic() override {
    // Sets the target velocity of our flywheel. This is similar to setting the
    // setpoint of a PID controller.
    if (joystick.GetRightBumperButton()) {
      // We pressed the bumper, so let's set our next reference
      loop.SetNextR(prosdk::math::Vectord<1>{SPINUP.value()});
    } else {
      // We released the bumper, so let's spin down
      loop.SetNextR(prosdk::math::Vectord<1>{0.0});
    }

    // Correct our Kalman filter's state vector estimate with encoder data.
    loop.Correct(prosdk::math::Vectord<1>{encoder.GetRate()});

    // Update our LQR to generate new voltage commands and use the voltages to
    // predict the next state with out Kalman filter.
    loop.Predict(20_ms);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    motor.SetVoltage(prosdk::units::volt_t{loop.U(0)});
  }
};

PROSDK_ROBOT_MAIN(Robot)
