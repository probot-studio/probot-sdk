// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ArmSimulation: a single-jointed arm with SingleJointedArmSim physics, PID control with the
// setpoint and kP read from Preferences, and a Mechanism2d view (upstream
// wpilibcExamples/src/main/cpp/examples/ArmSimulation),
// ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): PWMSparkMax (or any PWM ESC) on GPIO 4, quadrature
// encoder A/B on GPIO 6/7 (upstream: PWM 0, DIO 0/1).
// Joystick 0: hold the trigger to move the arm to the setpoint, release to stop the motor.
// GetTrigger() reads raw button 1, which on the Probot DS gamepad is FACE_RIGHT (B / Circle).
// Preferences "ArmPosition" (deg) and "ArmP" are tunables under /Preferences on the Driver
// Station, stored in NVS; they are read when teleop starts.
// The physics model only runs in the host simulator (SimulationPeriodic); on the robot it is
// inert and the real motor and encoder are used. The Driver Station's "Veri" page draws the
// Mechanism2d as a widget (probot-core #40).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "Robot.hpp"

PROSDK_ROBOT_MAIN(Robot)
