// OpModeRobotExample: an OpModeRobot on probot-core (design/PORT.md section 8). The opmodes are
// listed on the Driver Station: two autonomous routines, one teleop and one utility opmode. Pick
// one, INIT, then start it; the robot constructs the selected opmode while disabled, starts it on
// enable and ends it on disable, as with WPILib's Driver Station.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   GPIO 4  left ESC signal    GPIO 5  right ESC signal    GPIO 6/7  arm encoder A/B

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/OpModeRobot.hpp>
#include <prosdk/hardware/motor/PWMMotorController.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/opmode/PeriodicOpMode.hpp>
#include <prosdk/system/Timer.hpp>

namespace {

// A hobby ESC or continuous servo on one PWM pin: 1000-2000 us, 1500 us stop.
class PwmEsc : public prosdk::PWMMotorController {
 public:
  explicit PwmEsc(int pin) : PWMMotorController(pin) {
    SetBounds(2000_us, 1510_us, 1500_us, 1490_us, 1000_us);
  }
};

}  // namespace

class Robot : public prosdk::OpModeRobot<Robot> {
 public:
  Robot();

  PwmEsc left{4};
  PwmEsc right{5};
  prosdk::DifferentialDrive drive{left, right};
  prosdk::Encoder encoder{6, 7};
  prosdk::Gamepad gamepad{0};
};

// Autonomous 1: drive forward for 1.5 s.
class DriveForward : public prosdk::PeriodicOpMode {
 public:
  explicit DriveForward(Robot& robot) : m_robot{robot} {}
  void Start() override { m_timer.Restart(); }
  void Periodic() override {
    if (m_timer.Get() < 1.5_s) {
      m_robot.drive.ArcadeDrive(0.5, 0.0, false);
    } else {
      m_robot.drive.StopMotor();
    }
  }
  void End() override { m_robot.drive.StopMotor(); }

 private:
  Robot& m_robot;
  prosdk::Timer m_timer;
};

// Autonomous 2: turn in place for 1 s, then stop.
class TurnInPlace : public prosdk::PeriodicOpMode {
 public:
  explicit TurnInPlace(Robot& robot) : m_robot{robot} {}
  void Start() override { m_timer.Restart(); }
  void Periodic() override {
    m_robot.drive.ArcadeDrive(0.0, m_timer.Get() < 1_s ? 0.4 : 0.0, false);
  }
  void End() override { m_robot.drive.StopMotor(); }

 private:
  Robot& m_robot;
  prosdk::Timer m_timer;
};

// Teleop: arcade drive from gamepad 0.
class ArcadeTeleop : public prosdk::PeriodicOpMode {
 public:
  explicit ArcadeTeleop(Robot& robot) : m_robot{robot} {}
  void Periodic() override {
    m_robot.drive.ArcadeDrive(-m_robot.gamepad.GetLeftY(), -m_robot.gamepad.GetRightX());
  }
  void End() override { m_robot.drive.StopMotor(); }

 private:
  Robot& m_robot;
};

// Utility: zero the encoder, then report its distance every 0.5 s.
class ZeroEncoder : public prosdk::PeriodicOpMode {
 public:
  explicit ZeroEncoder(Robot& robot) : m_robot{robot} {
    AddPeriodic([this] { prosdk::telemetry::Log("Encoder", m_robot.encoder.GetDistance()); },
                0.5_s);
  }
  void Start() override { m_robot.encoder.Reset(); }

 private:
  Robot& m_robot;
};

Robot::Robot() {
  right.SetInverted(true);
  AddOpMode<DriveForward>(prosdk::RobotMode::AUTONOMOUS, "Drive Forward", "Autos",
                          "Drives forward for 1.5 s");
  AddOpMode<TurnInPlace>(prosdk::RobotMode::AUTONOMOUS, "Turn In Place", "Autos",
                         "Turns for 1 s");
  AddOpMode<ArcadeTeleop>(prosdk::RobotMode::TELEOPERATED, "Arcade Drive");
  AddOpMode<ZeroEncoder>(prosdk::RobotMode::UTILITY, "Zero Encoder", "",
                         "Zeroes the encoder and reports its distance");
  PublishOpModes();
}

PROSDK_ROBOT_MAIN(Robot)
