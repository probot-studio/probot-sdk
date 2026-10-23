// TimedRobotDifferential: a WPILib-style TimedRobot on probot-core (design/PORT.md section 8).
// Two-motor differential drive on hobby ESCs, arcade drive from gamepad 0, wheel encoders,
// odometry, and a heading-hold PID controller whose gains can be tuned from the Driver Station
// (/Tunables/Heading PID). Pose and wheel distances go to the Driver Station (/Telemetry).
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   GPIO 38     left ESC signal         GPIO 37     right ESC signal
//   GPIO 39/40  left encoder A/B        GPIO 41/42  right encoder A/B
//
// Teleop: left stick Y drives, right stick X turns. Hold the bottom face button (A on Xbox) to keep the heading.
// Autonomous: drives forward for 2 s.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <numbers>

#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/motor/PWMMotorController.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/kinematics/DifferentialDriveOdometry.hpp>
#include <prosdk/system/Timer.hpp>

namespace {

constexpr double kWheelDiameterMeters = 0.065;
constexpr int kEncoderCountsPerRev = 360;
constexpr double kTrackWidthMeters = 0.15;

// A hobby ESC or continuous servo on one PWM pin: 1000-2000 us, 1500 us stop.
class PwmEsc : public prosdk::PWMMotorController {
 public:
  explicit PwmEsc(int pin) : PWMMotorController(pin) {
    SetBounds(2000_us, 1510_us, 1500_us, 1490_us, 1000_us);
  }
};

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    m_right.SetInverted(true);
    const double metersPerCount = kWheelDiameterMeters * std::numbers::pi / kEncoderCountsPerRev;
    m_leftEncoder.SetDistancePerPulse(metersPerCount);
    m_rightEncoder.SetDistancePerPulse(metersPerCount);
    m_headingPid.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
    prosdk::tunables::Publish("Heading PID", m_headingPid);
  }

  void RobotPeriodic() override {
    const auto& pose = m_odometry.Update(Heading(), Left(), Right());
    prosdk::telemetry::Log("Pose/X", pose.X().value());
    prosdk::telemetry::Log("Pose/Y", pose.Y().value());
    prosdk::telemetry::Log("Pose/Heading (deg)", pose.Rotation().Degrees().value());
    prosdk::telemetry::Log("Encoders/Left", m_leftEncoder.GetDistance());
    prosdk::telemetry::Log("Encoders/Right", m_rightEncoder.GetDistance());
  }

  void AutonomousInit() override { m_timer.Restart(); }

  void AutonomousPeriodic() override {
    if (m_timer.Get() < 2_s) {
      m_drive.ArcadeDrive(0.5, 0.0, false);
    } else {
      m_drive.StopMotor();
    }
  }

  void TeleopPeriodic() override {
    double turn = -m_gamepad.GetRightX();
    if (m_gamepad.GetFaceDownButton()) {
      if (!m_holding) {
        m_holding = true;
        m_target = Heading().Radians().value();
        m_headingPid.Reset();
      }
      turn = m_headingPid.Calculate(Heading().Radians().value(), m_target);
    } else {
      m_holding = false;
    }
    m_drive.ArcadeDrive(-m_gamepad.GetLeftY(), turn);
  }

 private:
  prosdk::units::meter_t Left() const { return prosdk::units::meter_t{m_leftEncoder.GetDistance()}; }
  prosdk::units::meter_t Right() const {
    return prosdk::units::meter_t{m_rightEncoder.GetDistance()};
  }
  // no gyro: heading from the wheel distance difference
  prosdk::math::Rotation2d Heading() const {
    return prosdk::units::radian_t{(Right() - Left()).value() / kTrackWidthMeters};
  }

  PwmEsc m_left{38};
  PwmEsc m_right{37};
  prosdk::DifferentialDrive m_drive{m_left, m_right};
  prosdk::Encoder m_leftEncoder{39, 40};
  prosdk::Encoder m_rightEncoder{41, 42, true};
  prosdk::Gamepad m_gamepad{0};
  prosdk::math::PIDController m_headingPid{1.0, 0.0, 0.05};
  prosdk::math::DifferentialDriveOdometry m_odometry{Heading(), Left(), Right()};
  prosdk::Timer m_timer;
  bool m_holding = false;
  double m_target = 0.0;
};

PROSDK_ROBOT_MAIN(Robot)
