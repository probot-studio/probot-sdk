// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ElevatorExponentialSimulation: an elevator with ElevatorSim physics followed with an
// ExponentialProfile, PID and ElevatorFeedforward, plus a Mechanism2d view
// (upstream wpilibcExamples/src/main/cpp/examples/ElevatorExponentialSimulation), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): PWMSparkMax (or any PWM ESC) on GPIO 4, quadrature
// encoder A/B on GPIO 6/7 (upstream: PWM 0, DIO 0/1).
// Joystick 0: hold the trigger to go to the upper setpoint, release to go to the lower one.
// The physics model only runs in the host simulator (SimulationPeriodic); on the robot it is
// inert and the real motor and encoder are used. The Driver Station's "Veri" page draws the
// Mechanism2d as a widget (probot-core #40).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
