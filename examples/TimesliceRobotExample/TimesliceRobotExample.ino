// TimesliceRobotExample: a TimesliceRobot on probot-core (design/PORT.md section 8). The robot
// loop runs every 20 ms (TimedRobot's default period) and two controllers get their own 2 ms
// slices of every 10 ms controller period, so probot-core runs 2 ms loop passes (the greatest
// common divisor of the periods and offsets).
//
//   | Function        | Offset | Allocation |
//   |-----------------|--------|------------|
//   | RobotPeriodic() | 0 ms   | 4 ms       |
//   | Arm controller  | 4 ms   | 2 ms       |
//   | Drive feedback  | 6 ms   | 2 ms       |
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   GPIO 38 arm servo/ESC signal    GPIO 39/40 arm encoder A/B

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimesliceRobot.hpp>
#include <prosdk/hardware/motor/PWMMotorController.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>

namespace {

// A hobby ESC or continuous servo on one PWM pin: 1000-2000 us, 1500 us stop.
class PwmEsc : public prosdk::PWMMotorController {
 public:
  explicit PwmEsc(int pin) : PWMMotorController(pin) {
    SetBounds(2000_us, 1510_us, 1500_us, 1490_us, 1000_us);
  }
};

}  // namespace

class Robot : public prosdk::TimesliceRobot {
 public:
  // the robot loop gets the first 4 ms; the controllers are scheduled within a 10 ms period
  Robot() : TimesliceRobot{4_ms, 10_ms} {
    m_encoder.SetDistancePerPulse(1.0 / 360.0);  // rotations
    prosdk::tunables::Publish("Arm PID", m_armPid);
    Schedule([this] { RunArm(); }, 2_ms);
    Schedule([this] { m_samples++; }, 2_ms);
  }

  void RobotPeriodic() override {
    prosdk::telemetry::Log("Arm/Position", m_encoder.GetDistance());
    prosdk::telemetry::Log("Arm/Setpoint", m_setpoint);
    prosdk::telemetry::Log("Feedback samples", static_cast<int64_t>(m_samples));
  }

  void TeleopInit() override { m_setpoint = 0.25; }
  void DisabledInit() override { m_arm.StopMotor(); }

 private:
  void RunArm() {
    if (IsEnabled()) {
      m_arm.SetThrottle(m_armPid.Calculate(m_encoder.GetDistance(), m_setpoint));
    }
  }

  PwmEsc m_arm{38};
  prosdk::Encoder m_encoder{39, 40};
  prosdk::math::PIDController m_armPid{2.0, 0.0, 0.0, 10_ms};
  double m_setpoint = 0.0;
  uint32_t m_samples = 0;
};

PROSDK_ROBOT_MAIN(Robot)
