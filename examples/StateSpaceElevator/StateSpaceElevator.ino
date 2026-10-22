// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// StateSpaceElevator: an elevator position loop built from a LinearSystemLoop (LQR + Kalman filter)
// with a
// trapezoid profile (upstream wpilibcExamples/src/main/cpp/examples/StateSpaceElevator), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): PWMSparkMax (or any PWM ESC) on GPIO 4, quadrature
// encoder A/B on GPIO 6/7 (upstream: PWM 0, DIO 0/1).
// Gamepad 0: hold the right bumper to raise the elevator, release to lower it.

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
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/mass.hpp>
#include <prosdk/units/velocity.hpp>

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an elevator.
 */
class Robot : public prosdk::TimedRobot {
  static constexpr int MOTOR_PORT = 4;
  static constexpr int ENCODER_A_CHANNEL = 6;
  static constexpr int ENCODER_B_CHANNEL = 7;
  static constexpr int JOYSTICK_PORT = 0;

  static constexpr prosdk::units::meter_t RAISED_POSITION = 2_ft;
  static constexpr prosdk::units::meter_t LOWERED_POSITION = 0_ft;

  static constexpr prosdk::units::meter_t DRUM_RADIUS = 0.75_in;
  static constexpr prosdk::units::kilogram_t CARRIAGE_MASS = 4.5_kg;
  static constexpr double GEAR_RATIO = 6.0;

  // The plant holds a state-space model of our elevator. This system has the
  // following properties:
  //
  // States: [position, velocity], in meters and meters per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in meters.
  prosdk::math::LinearSystem<2, 1, 1> elevatorPlant =
      prosdk::math::Models::ElevatorFromPhysicalConstants(
          prosdk::math::DCMotor::NEO(2), CARRIAGE_MASS, DRUM_RADIUS, GEAR_RATIO)
          .Slice(0);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  prosdk::math::KalmanFilter<2, 1, 1> observer{
      elevatorPlant,
      {prosdk::units::meter_t{2_in}.value(),
       prosdk::units::meters_per_second_t{40_in / 1_s}
           .value()},  // How accurate we think our model is
      {0.001},         // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position
      // reading.
      20_ms};

  // A LQR uses feedback to create voltage commands.
  prosdk::math::LinearQuadraticRegulator<2, 1> controller{
      elevatorPlant,
      // qelms. State error tolerance, in meters and meters per second.
      // Decrease this to more heavily penalize state excursion, or make the
      // controller behave more aggressively.
      {prosdk::units::meter_t{1_in}.value(),
       prosdk::units::meters_per_second_t{10_in / 1_s}.value()},
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
  prosdk::math::LinearSystemLoop<2, 1, 1> loop{elevatorPlant, controller, observer,
                                            12_V, 20_ms};

  // An encoder set up to measure elevator height in meters.
  prosdk::Encoder encoder{ENCODER_A_CHANNEL, ENCODER_B_CHANNEL};

  prosdk::PWMSparkMax motor{MOTOR_PORT};
  prosdk::Gamepad joystick{JOYSTICK_PORT};

  prosdk::math::TrapezoidProfile<prosdk::units::meters> profile{{3_fps, 6_fps_sq}};

  prosdk::math::TrapezoidProfile<prosdk::units::meters>::State lastProfiledReference;

 public:
  Robot() {
    // Circumference = pi * d, so distance per click = pi * d / counts
    encoder.SetDistancePerPulse(2.0 * std::numbers::pi * DRUM_RADIUS.value() /
                                4096.0);
  }

  void TeleopInit() override {
    // Reset our loop to make sure it's in a known state.
    loop.Reset(prosdk::math::Vectord<2>{encoder.GetDistance(), encoder.GetRate()});

    lastProfiledReference = {
        prosdk::units::meter_t{encoder.GetDistance()},
        prosdk::units::meters_per_second_t{encoder.GetRate()}};
  }

  void TeleopPeriodic() override {
    // Sets the target height of our elevator. This is similar to setting the
    // setpoint of a PID controller.
    prosdk::math::TrapezoidProfile<prosdk::units::meters>::State goal;
    if (joystick.GetRightBumperButton()) {
      // We pressed the bumper, so let's set our next reference
      goal = {RAISED_POSITION, 0_fps};
    } else {
      // We released the bumper, so let's spin down
      goal = {LOWERED_POSITION, 0_fps};
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
