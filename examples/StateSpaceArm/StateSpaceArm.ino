// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// StateSpaceArm: an arm position loop built from a LinearSystemLoop (LQR + Kalman filter) with a
// trapezoid profile (upstream wpilibcExamples/src/main/cpp/examples/StateSpaceArm), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): PWMSparkMax (or any PWM ESC) on GPIO 4, quadrature
// encoder A/B on GPIO 6/7 (upstream: PWM 0, DIO 0/1).
// Gamepad 0: hold the right bumper to raise the arm to 90 deg, release to lower it.

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
#include <prosdk/math/system/DCMotor.hpp>
#include <prosdk/math/system/LinearSystemLoop.hpp>
#include <prosdk/math/system/Models.hpp>
#include <prosdk/math/trajectory/TrapezoidProfile.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/moment_of_inertia.hpp>

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an arm.
 */
class Robot : public prosdk::TimedRobot {
  static constexpr int MOTOR_PORT = 4;
  static constexpr int ENCODER_A_CHANNEL = 6;
  static constexpr int ENCODER_B_CHANNEL = 7;
  static constexpr int JOYSTICK_PORT = 0;

  static constexpr prosdk::units::radian_t RAISED_POSITION = 90_deg;
  static constexpr prosdk::units::radian_t LOWERED_POSITION = 0_deg;

  // Moment of inertia of the arm. Can be estimated with CAD. If finding this
  // constant is difficult, prosdk::math::LinearSystem.identifyPositionSystem may
  // be better.
  static constexpr prosdk::units::kilogram_square_meter_t ARM_MOI = 1.2_kg_sq_m;

  // Reduction between motors and encoder, as output over input. If the arm
  // spins slower than the motors, this number should be greater than one.
  static constexpr double ARM_GEARING = 10.0;

  // The plant holds a state-space model of our arm. This system has the
  // following properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  prosdk::math::LinearSystem<2, 1, 1> armPlant =
      prosdk::math::Models::SingleJointedArmFromPhysicalConstants(
          prosdk::math::DCMotor::NEO(2), ARM_MOI, ARM_GEARING)
          .Slice(0);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  prosdk::math::KalmanFilter<2, 1, 1> observer{
      armPlant,
      {0.015, 0.17},  // How accurate we think our model is
      {0.01},         // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position
      // reading.
      20_ms};

  // A LQR uses feedback to create voltage commands.
  prosdk::math::LinearQuadraticRegulator<2, 1> controller{
      armPlant,
      // qelms. Velocity error tolerance, in radians and radians per second.
      // Decrease this to more heavily penalize state excursion, or make the
      // controller behave more aggressively.
      {1.0 * 2.0 * std::numbers::pi / 360.0,
       10.0 * 2.0 * std::numbers::pi / 360.0},
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
  prosdk::math::LinearSystemLoop<2, 1, 1> loop{armPlant, controller, observer,
                                            12_V, 20_ms};

  // An encoder set up to measure arm position in radians per second.
  prosdk::Encoder encoder{ENCODER_A_CHANNEL, ENCODER_B_CHANNEL};

  prosdk::PWMSparkMax motor{MOTOR_PORT};
  prosdk::Gamepad joystick{JOYSTICK_PORT};

  prosdk::math::TrapezoidProfile<prosdk::units::radians> profile{
      {45_deg_per_s, 90_deg_per_s / 1_s}};

  prosdk::math::TrapezoidProfile<prosdk::units::radians>::State lastProfiledReference;

 public:
  Robot() {
    // We go 2 pi radians per 4096 clicks.
    encoder.SetDistancePerPulse(2.0 * std::numbers::pi / 4096.0);
  }

  void TeleopInit() override {
    loop.Reset(prosdk::math::Vectord<2>{encoder.GetDistance(), encoder.GetRate()});

    lastProfiledReference = {
        prosdk::units::radian_t{encoder.GetDistance()},
        prosdk::units::radians_per_second_t{encoder.GetRate()}};
  }

  void TeleopPeriodic() override {
    // Sets the target position of our arm. This is similar to setting the
    // setpoint of a PID controller.
    prosdk::math::TrapezoidProfile<prosdk::units::radians>::State goal;
    if (joystick.GetRightBumperButton()) {
      // We pressed the bumper, so let's set our next reference
      goal = {RAISED_POSITION, 0_rad_per_s};
    } else {
      // We released the bumper, so let's spin down
      goal = {LOWERED_POSITION, 0_rad_per_s};
    }
    lastProfiledReference =
        profile.Calculate(20_ms, lastProfiledReference, goal);

    loop.SetNextR(
        prosdk::math::Vectord<2>{lastProfiledReference.position.value(),
                              lastProfiledReference.velocity.value()});

    // Correct our Kalman filter's state vector estimate with encoder data.
    loop.Correct(prosdk::math::Vectord<1>{encoder.GetDistance()});

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
