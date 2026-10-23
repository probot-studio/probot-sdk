// CommandBasedRobot: the WPILib command-based framework (prosdk::cmd) on probot-core.
// A TimedRobot runs the CommandScheduler every loop. One drive subsystem on two PWM motor
// controllers (hobby ESCs) with wheel encoders, driven by a CommandXboxController:
//   - default command: arcade drive (left stick Y drives, right stick X turns)
//   - A: drive 1 m forward with a PID command whose gains are tunable from the Driver Station
//     (/Tunables/Drive Distance PID)
//   - hold the left bumper plus A/B/X/Y: SysId quasistatic forward/reverse, dynamic
//     forward/reverse (the SysId data goes to the on-robot DataLog)
// Autonomous: chosen on the Driver Station (/Tunables/Auto, a Selectable).
// The running commands and the scheduler (cancel by id) are on the Driver Station under
// /Telemetry/Scheduler and /Tunables/Scheduler.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   GPIO 38     left ESC signal         GPIO 37     right ESC signal
//   GPIO 39/40  left encoder A/B        GPIO 41/42  right encoder A/B

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <numbers>

#include <prosdk/commands2/CommandHelper.hpp>
#include <prosdk/commands2/CommandScheduler.hpp>
#include <prosdk/commands2/Commands.hpp>
#include <prosdk/commands2/SubsystemBase.hpp>
#include <prosdk/commands2/button/CommandXboxController.hpp>
#include <prosdk/commands2/sysid/SysIdRoutine.hpp>
#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/motor/PWMMotorController.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/tunables/Selectable.hpp>

constexpr double kWheelDiameterMeters = 0.065;
constexpr int kEncoderCountsPerRev = 360;

// A hobby ESC or continuous servo on one PWM pin: 1000-2000 us, 1500 us stop. Not in an
// anonymous namespace: Drive (below) has PwmEsc members, and GCC warns (-Wsubobject-linkage)
// about a class whose field type has internal linkage when the .ino is not the main file.
class PwmEsc : public prosdk::PWMMotorController {
 public:
  explicit PwmEsc(int pin) : PWMMotorController(pin) {
    SetBounds(2000_us, 1510_us, 1500_us, 1490_us, 1000_us);
  }
};

class Drive : public prosdk::cmd::SubsystemBase {
 public:
  Drive() : SubsystemBase{"Drive"} {
    m_right.SetInverted(true);
    const double metersPerCount = kWheelDiameterMeters * std::numbers::pi / kEncoderCountsPerRev;
    m_leftEncoder.SetDistancePerPulse(metersPerCount);
    m_rightEncoder.SetDistancePerPulse(metersPerCount);
  }

  void Periodic() override {
    prosdk::telemetry::Log("Drive/Left (m)", m_leftEncoder.GetDistance());
    prosdk::telemetry::Log("Drive/Right (m)", m_rightEncoder.GetDistance());
  }

  void Arcade(double forward, double turn) { m_drive.ArcadeDrive(forward, turn); }
  void Stop() { m_drive.StopMotor(); }

  double Distance() const { return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2; }
  void ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
  }

  prosdk::cmd::CommandPtr ArcadeCommand(std::function<double()> forward,
                                        std::function<double()> turn) {
    return Run([this, forward, turn] { Arcade(forward(), turn()); }).WithName("Arcade");
  }

  prosdk::cmd::CommandPtr SysIdQuasistatic(prosdk::cmd::sysid::Direction direction) {
    return m_sysId.Quasistatic(direction);
  }
  prosdk::cmd::CommandPtr SysIdDynamic(prosdk::cmd::sysid::Direction direction) {
    return m_sysId.Dynamic(direction);
  }

 private:
  void SetVoltage(prosdk::units::volt_t voltage) {
    m_left.SetVoltage(voltage);
    m_right.SetVoltage(voltage);
    m_drive.Feed();  // the motors are driven directly: keep the drive's motor safety fed
  }

  void LogSysId(prosdk::sysid::SysIdRoutineLog* log) {
    log->Motor("drive-left")
        .voltage(m_left.GetVoltage())
        .position(prosdk::units::meter_t{m_leftEncoder.GetDistance()})
        .velocity(prosdk::units::meters_per_second_t{m_leftEncoder.GetRate()});
    log->Motor("drive-right")
        .voltage(m_right.GetVoltage())
        .position(prosdk::units::meter_t{m_rightEncoder.GetDistance()})
        .velocity(prosdk::units::meters_per_second_t{m_rightEncoder.GetRate()});
  }

  PwmEsc m_left{38};
  PwmEsc m_right{37};
  prosdk::DifferentialDrive m_drive{m_left, m_right};
  prosdk::Encoder m_leftEncoder{39, 40};
  prosdk::Encoder m_rightEncoder{41, 42, true};
  prosdk::cmd::sysid::SysIdRoutine m_sysId{
      prosdk::cmd::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
      prosdk::cmd::sysid::Mechanism{[this](prosdk::units::volt_t v) { SetVoltage(v); },
                                    [this](prosdk::sysid::SysIdRoutineLog* log) { LogSysId(log); },
                                    this}};
};

// Drives a distance with a PID controller on the averaged wheel distance. The gains are
// published once as /Tunables/Drive Distance PID and shared by every DriveDistance command.
class DriveDistance : public prosdk::cmd::CommandHelper<prosdk::cmd::Command, DriveDistance> {
 public:
  DriveDistance(Drive& drive, double meters) : m_drive{drive}, m_meters{meters} {
    AddRequirements(&drive);
  }

  void Initialize() override {
    m_drive.ResetEncoders();
    Pid().Reset();
    Pid().SetSetpoint(m_meters);
  }
  void Execute() override { m_drive.Arcade(Pid().Calculate(m_drive.Distance()), 0.0); }
  bool IsFinished() override { return Pid().AtSetpoint(); }
  void End(bool interrupted) override { m_drive.Stop(); }

  static prosdk::math::PIDController& Pid() {
    static prosdk::math::PIDController pid{2.0, 0.0, 0.1};
    return pid;
  }

 private:
  Drive& m_drive;
  double m_meters;
};

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    using prosdk::cmd::sysid::Direction;
    m_drive.SetDefaultCommand(m_drive.ArcadeCommand([this] { return -m_controller.GetLeftY(); },
                                                    [this] { return -m_controller.GetRightX(); }));

    m_controller.A().OnTrue(DriveDistance{m_drive, 1.0}.ToPtr());
    auto sysIdLayer = m_controller.LeftBumper();
    (sysIdLayer && m_controller.A()).WhileTrue(m_drive.SysIdQuasistatic(Direction::FORWARD));
    (sysIdLayer && m_controller.B()).WhileTrue(m_drive.SysIdQuasistatic(Direction::REVERSE));
    (sysIdLayer && m_controller.X()).WhileTrue(m_drive.SysIdDynamic(Direction::FORWARD));
    (sysIdLayer && m_controller.Y()).WhileTrue(m_drive.SysIdDynamic(Direction::REVERSE));

    m_auto.AddDefault("Drive 1 m", m_driveAuto.get());
    m_auto.Add("Spin 2 s", m_spinAuto.get());
    m_auto.Add("Nothing", m_noAuto.get());

    DriveDistance::Pid().SetTolerance(0.02);
    prosdk::tunables::Publish("Drive Distance PID", DriveDistance::Pid());
    prosdk::tunables::Publish("Auto", m_auto);
    prosdk::tunables::Publish("Scheduler", prosdk::cmd::CommandScheduler::GetInstance());
  }

  void RobotPeriodic() override {
    auto& scheduler = prosdk::cmd::CommandScheduler::GetInstance();
    scheduler.Run();
    prosdk::telemetry::Log("Scheduler", scheduler);
  }

  void AutonomousInit() override {
    if (prosdk::cmd::Command* command = m_auto.GetSelected()) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  }

  void TeleopInit() override {
    if (prosdk::cmd::Command* command = m_auto.GetSelected()) {
      command->Cancel();
    }
  }

 private:
  Drive m_drive;
  prosdk::cmd::CommandXboxController m_controller{0};
  prosdk::cmd::CommandPtr m_driveAuto = DriveDistance{m_drive, 1.0}.ToPtr();
  prosdk::cmd::CommandPtr m_spinAuto =
      m_drive.ArcadeCommand([] { return 0.0; }, [] { return 0.5; }).WithTimeout(2_s);
  prosdk::cmd::CommandPtr m_noAuto = prosdk::cmd::None();
  prosdk::tunables::Selectable<prosdk::cmd::Command*> m_auto;
};

PROSDK_ROBOT_MAIN(Robot)
