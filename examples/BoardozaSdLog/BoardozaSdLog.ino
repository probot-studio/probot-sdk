// BoardozaSdLog: a TimedRobot that writes its DataLog (.wpilog files) to a microSD card on the
// Boardoza MicroSD board (design/boardoza/P9-storage.md). The card is mounted at /sd with
// prosdk::platform::MountSdCard on SPI bus 0, and DataLogManager writes to /sd/logs. The battery
// voltage is logged every loop; the Driver Station shows whether the card is mounted.
//
// Wiring (change the pins to match your robot; none may be a probot-core pin). These are the
// SPI2 IO_MUX pins of the ESP32-S3:
//   MicroSD J1: 1 MISO -> GPIO 13, 2 SCK -> GPIO 12, 3 MOSI -> GPIO 11, 4 CS -> GPIO 10,
//               5 GND -> GND, 6 +5V -> 5 V
// The board has no 3.3 V input: it needs 5 V on pin 6 (its regulator feeds the card 3.3 V). The
// SPI lines go straight to the card, so they are 3.3 V signals; never drive them from 5 V logic.
// The board has no card-detect line: insert the card before power-up. The card must carry a FAT16
// or FAT32 file system (exFAT is not supported, so format cards over 32 GB as FAT32); it is never
// formatted.
//
// Without a card (or on the simulation) the log goes to /littlefs/logs when LittleFS is mounted,
// and is discarded otherwise. Old logs on the card are not deleted automatically.
//
// While the robot is idle the .wpilog files in the log folder can be downloaded from the Driver
// Station's Logs page, "Dosyalar" card ("logs"); no card reader needed.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/datalog/DataLog.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/platform/HalConfig.hpp>
#include <prosdk/platform/SdCard.hpp>
#include <prosdk/platform/Storage.hpp>
#include <prosdk/system/DataLogManager.hpp>
#include <prosdk/system/RobotController.hpp>

namespace {

constexpr int kSpiBus = 0;  // SPI2
constexpr int kSck = 12;
constexpr int kMosi = 11;
constexpr int kMiso = 13;
constexpr int kSdChipSelect = 10;

// Mounts the card, points the log directory at it and starts DataLogManager.
bool StartSdLog() {
  bool mounted = prosdk::platform::SetSPIPins(kSpiBus, kSck, kMosi, kMiso) &&
                 prosdk::platform::MountSdCard(kSpiBus, kSdChipSelect, "/sd");
  if (mounted) {
    prosdk::platform::SetLogDirectory("/sd/logs");
  }
  prosdk::DataLogManager::Start();
  prosdk::DataLogManager::Log(mounted ? "logging to the SD card" : "no SD card");
  return mounted;
}

}  // namespace

class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    m_battery.Append(prosdk::RobotController::GetBatteryVoltage().value());
    prosdk::telemetry::Log("SD card/Mounted", m_sdMounted);
  }

 private:
  // first member: the card and the log before anything logs
  bool m_sdMounted = StartSdLog();

  prosdk::log::DoubleLogEntry m_battery{prosdk::DataLogManager::GetLog(), "/robot/battery"};
};

PROSDK_ROBOT_MAIN(Robot)
