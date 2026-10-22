// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Mechanism2d: an elevator with a wrist shown as a Mechanism2d (upstream
// wpilibcExamples/src/main/cpp/examples/Mechanism2d), ported
// to prosdk. The mechanism goes to /Telemetry/Mech2d (the Driver Station has no Mechanism2d
// widget yet and shows it as a topic tree, design/core-requests.md R5).
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 4  elevator PWMSparkMax    GPIO 5  wrist PWMSparkMax
//   GPIO 6/7 elevator encoder A/B   GPIO 1  wrist potentiometer (analog, 0-3.1 V)
// Joystick 0: axis 0 elevator, axis 1 wrist.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/AnalogPotentiometer.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/smartdashboard/Mechanism2d.hpp>
#include <prosdk/smartdashboard/MechanismLigament2d.hpp>
#include <prosdk/telemetry/Telemetry.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/util/Color.hpp>
#include <prosdk/util/Color8Bit.hpp>

/**
 * This sample program shows how to use Mechanism2d - a visual representation of
 * arms, elevators, and other mechanisms on dashboards; driven by a node-based
 * API.
 *
 * <p>Ligaments are based on other ligaments or roots, and roots are contained
 * in the base Mechanism2d object. Use pointers for nodes, and beware not to let
 * the container out of scope - the appended nodes will be recursively
 * destructed!
 */
class Robot : public prosdk::TimedRobot {
  static constexpr double METERS_PER_PULSE = 0.01;
  static constexpr double ELEVATOR_MINIMUM_LENGTH = 0.5;

 public:
  Robot() { elevatorEncoder.SetDistancePerPulse(METERS_PER_PULSE); }

  void RobotPeriodic() override {
    // update the dashboard mechanism's state
    elevator->SetLength(ELEVATOR_MINIMUM_LENGTH +
                        elevatorEncoder.GetDistance());
    wrist->SetAngle(prosdk::units::degree_t{wristPotentiometer.Get()});

    // publish to telemetry
    prosdk::telemetry::Log("Mech2d", mech);
  }

  void TeleopPeriodic() override {
    elevatorMotor.SetThrottle(joystick.GetRawAxis(0));
    wristMotor.SetThrottle(joystick.GetRawAxis(1));
  }

 private:
  prosdk::PWMSparkMax elevatorMotor{4};
  prosdk::PWMSparkMax wristMotor{5};
  prosdk::Encoder elevatorEncoder{6, 7};
  prosdk::AnalogPotentiometer wristPotentiometer{1, 90};
  prosdk::Joystick joystick{0};

  // the main mechanism object
  prosdk::Mechanism2d mech{3, 3};
  // the mechanism root node
  prosdk::MechanismRoot2d* root = mech.GetRoot("climber", 2, 0);
  // MechanismLigament2d objects represent each "section"/"stage" of the
  // mechanism, and are based off the root node or another ligament object
  prosdk::MechanismLigament2d* elevator =
      root->Append<prosdk::MechanismLigament2d>("elevator", 1, 90_deg);
  prosdk::MechanismLigament2d* wrist = elevator->Append<prosdk::MechanismLigament2d>(
      "wrist", 0.5, 90_deg, 6, prosdk::util::Color8Bit{prosdk::util::Color::PURPLE});
};

PROSDK_ROBOT_MAIN(Robot)
