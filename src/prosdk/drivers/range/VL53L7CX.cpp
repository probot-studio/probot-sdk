// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// VL53L7CX driver (VL53L7CX.hpp) on ST's ULD 2.0.0 (st/vl53l7cx_api.h, unmodified) and the prosdk
// platform layer (st/platform.cpp). The setup order follows ST's ULD and its multi-sensor example
// (x-cube-tof1, Projects/NUCLEO-F401RE/Examples/53L7A1/53L7A1_MultiSensorRanging/Src/app_tof.c):
// LPn high, 2 ms, vl53l7cx_init() at the default address, then vl53l7cx_set_i2c_address().

#include "prosdk/drivers/range/VL53L7CX.hpp"

#include <algorithm>
#include <cmath>
#include <format>

// platform.h first: it includes the C headers outside the extern "C" block below
#include "prosdk/drivers/range/st/platform.h"
#include "prosdk/drivers/range/st/VL53L7CXPlatform.hpp"
extern "C" {
#include "prosdk/drivers/range/st/vl53l7cx_api.h"
}

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;
using namespace std::chrono_literals;

struct VL53L7CX::Uld {
  Uld(I2C::Port port, int address) : bus{port, address} {}

  detail::VL53L7CXBus bus;
  VL53L7CX_Configuration dev{};      // about 2.3 KB (offset, crosstalk and temporary buffers)
  VL53L7CX_ResultsData results{};
};

namespace {

// Wait after LPn goes high before the first transfer (ST's multi-sensor example: HAL_Delay(2)).
constexpr auto kLpnDelay = 2ms;
// Data-ready poll period: a frame lasts 16.7 ms at 60 Hz.
constexpr auto kPollPeriod = 5ms;
// Pause before the setup is tried again after a failure.
constexpr auto kRetryPeriod = 1000ms;

// One sensor per port at a time uses the default address (setup until its address is moved).
std::mutex gDefaultAddressMutex[2];

bool ValidAddress(int address) {
  return address >= 0x08 && address <= 0x77;
}

}  // namespace

VL53L7CX::VL53L7CX(I2C::Port port, int lpnChannel, int address, Resolution resolution,
                   units::hertz_t rangingFrequency)
    : m_port{port},
      m_resolution{static_cast<int>(resolution)},
      m_targetAddress{ValidAddress(address) ? address : kDefaultAddress} {
  if (!ValidAddress(address)) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] VL53L7CX: address 0x{:02X} is not 0x08-0x77, using 0x{:02X}\n",
                           address, kDefaultAddress);
  }
  // vl53l7cx_api.h, vl53l7cx_set_ranging_frequency_hz: 1-60 Hz (4x4), 1-15 Hz (8x8)
  double hz = std::round(rangingFrequency.value());
  double maxHz = resolution == Resolution::k8x8 ? 15.0 : 60.0;
  m_frequencyHz = std::isnan(hz) ? 15 : static_cast<int>(std::clamp(hz, 1.0, maxHz));
  if (lpnChannel >= 0) {
    m_lpn.emplace(lpnChannel);
    m_lpn->Set(false);  // I2C off until this sensor's turn
  }
  // opens the port at the default address; the thread moves it
  m_uld = std::make_unique<Uld>(port, kDefaultAddress);
  util::ReportUsage("VL53L7CX", m_targetAddress, "");
}

VL53L7CX::~VL53L7CX() {
  m_stop = true;
  m_uld->bus.abort = true;  // ends a ULD call in progress (transfers fail, waits return)
  {
    std::scoped_lock lock{m_waitMutex};
  }
  m_wake.notify_all();
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void VL53L7CX::Start() {
  std::call_once(m_started, [this] {
    platform::ScopedThreadConfig threadConfig{{.name = "vl53l7cx", .stackSize = 6144}};
    m_thread = std::thread([this] { Run(); });
  });
}

bool VL53L7CX::Wait(std::chrono::milliseconds duration) {
  std::unique_lock lock{m_waitMutex};
  return !m_wake.wait_for(lock, duration, [this] { return m_stop.load(); });
}

bool VL53L7CX::Fail(const std::string& problem) {
  if (problem != m_problem && !m_stop) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] VL53L7CX 0x{:02X} on I2C port {}: {}\n",
                           m_targetAddress.load(), static_cast<int>(m_port), problem);
  }
  m_problem = problem;
  return false;
}

bool VL53L7CX::IsAlive() {
  uint8_t alive = 0;
  return vl53l7cx_is_alive(&m_uld->dev, &alive) == VL53L7CX_STATUS_OK && alive;
}

void VL53L7CX::Run() {
  while (!m_stop) {
    if (BringUp()) {
      m_problem.clear();
      Range();
    }
    {
      std::scoped_lock lock{m_mutex};
      m_ready = false;
    }
    if (m_lpn && !m_stop) {
      // I2C off until the next turn: a sensor that lost power is back at the default address,
      // where it must not answer during another sensor's turn
      m_lpn->Set(false);
    }
    if (!Wait(kRetryPeriod)) {
      return;
    }
  }
}

bool VL53L7CX::BringUp() {
  VL53L7CX_Configuration& dev = m_uld->dev;
  const int target = m_targetAddress;
  {
    // wait for the turn without blocking the destructor
    std::unique_lock lock{gDefaultAddressMutex[static_cast<int>(m_port)], std::defer_lock};
    while (!lock.try_lock()) {
      if (!Wait(kPollPeriod)) {
        return false;
      }
    }
    if (m_lpn) {
      m_lpn->Set(true);
      if (!Wait(kLpnDelay)) {
        return false;
      }
    }
    // A sensor that kept its address over a program restart (no power cycle) is set up there.
    m_uld->bus.Attach(&dev.platform, target);
    if (target == kDefaultAddress || !IsAlive()) {
      m_uld->bus.Attach(&dev.platform, kDefaultAddress);
      if (!IsAlive()) {
        return Fail("no answer (ID 0xF0/0x02 not read)");
      }
    }
    uint8_t status = vl53l7cx_init(&dev);  // reboot and firmware upload
    if (status != VL53L7CX_STATUS_OK) {
      return Fail(std::format("firmware upload failed (ULD status {})", status));
    }
    if (dev.platform.address != (target << 1)) {
      status = vl53l7cx_set_i2c_address(&dev, static_cast<uint16_t>(target << 1));
      if (status != VL53L7CX_STATUS_OK || !IsAlive()) {
        return Fail(std::format("no answer after the move to 0x{:02X}", target));
      }
    }
  }
  uint8_t status = vl53l7cx_set_resolution(&dev, static_cast<uint8_t>(m_resolution));
  status |= vl53l7cx_set_ranging_frequency_hz(&dev, static_cast<uint8_t>(m_frequencyHz));
  status |= vl53l7cx_set_ranging_mode(&dev, VL53L7CX_RANGING_MODE_CONTINUOUS);
  status |= vl53l7cx_start_ranging(&dev);
  if (status != VL53L7CX_STATUS_OK) {
    return Fail(std::format("configuration failed (ULD status {})", status));
  }
  return true;
}

void VL53L7CX::Range() {
  VL53L7CX_Configuration& dev = m_uld->dev;
  // no frame for 3 frame periods (at least 1 s): the sensor lost power or hung; set it up again
  const auto timeout = std::max<std::chrono::steady_clock::duration>(
      1s, std::chrono::milliseconds{3000 / m_frequencyHz});
  auto lastFrame = std::chrono::steady_clock::now();
  while (!m_stop) {
    const int target = m_targetAddress;
    if (dev.platform.address != (target << 1)) {
      uint8_t status = vl53l7cx_stop_ranging(&dev);
      status |= vl53l7cx_set_i2c_address(&dev, static_cast<uint16_t>(target << 1));
      status |= vl53l7cx_start_ranging(&dev);
      if (status != VL53L7CX_STATUS_OK) {
        Fail(std::format("address change to 0x{:02X} failed (ULD status {})", target, status));
        return;
      }
    }
    uint8_t ready = 0;
    if (vl53l7cx_check_data_ready(&dev, &ready) == VL53L7CX_STATUS_OK && ready &&
        vl53l7cx_get_ranging_data(&dev, &m_uld->results) == VL53L7CX_STATUS_OK) {
      Publish();
      lastFrame = std::chrono::steady_clock::now();
    }
    if (std::chrono::steady_clock::now() - lastFrame > timeout) {
      Fail("no frame, setting the sensor up again");
      return;
    }
    if (!Wait(kPollPeriod)) {
      return;
    }
  }
}

void VL53L7CX::Publish() {
  const VL53L7CX_ResultsData& results = m_uld->results;
  std::scoped_lock lock{m_mutex};
  m_zones = m_resolution;  // one target per zone: entry i is zone i
  std::copy_n(results.distance_mm, m_zones, m_distances.begin());
  std::copy_n(results.target_status, m_zones, m_status.begin());
  m_ready = true;
}

bool VL53L7CX::IsReady() {
  Start();
  std::scoped_lock lock{m_mutex};
  return m_ready;
}

int VL53L7CX::GetDistances(std::span<int16_t> distances, std::span<uint8_t> status) {
  Start();
  std::scoped_lock lock{m_mutex};
  std::copy_n(m_distances.begin(), std::min<std::size_t>(distances.size(), m_zones),
              distances.begin());
  std::copy_n(m_status.begin(), std::min<std::size_t>(status.size(), m_zones), status.begin());
  return m_zones;
}

int VL53L7CX::GetTargetStatus(std::span<uint8_t> status) {
  Start();
  std::scoped_lock lock{m_mutex};
  std::copy_n(m_status.begin(), std::min<std::size_t>(status.size(), m_zones), status.begin());
  return m_zones;
}

void VL53L7CX::SetAddress(int newAddress) {
  if (!ValidAddress(newAddress)) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] VL53L7CX: address 0x{:02X} is not 0x08-0x77, ignored\n",
                           newAddress);
    return;
  }
  m_targetAddress = newAddress;
  Start();
}

void VL53L7CX::LogTo(telemetry::TelemetryTable& table) const {
  std::array<int16_t, kMaxZones> distances;
  int zones;
  bool ready;
  {
    std::scoped_lock lock{m_mutex};
    distances = m_distances;
    zones = m_zones;
    ready = m_ready;
  }
  table.Log("Ready", ready);
  table.Log("Distances",
            std::span<const int16_t>{distances.data(), static_cast<std::size_t>(zones)});
}

std::string_view VL53L7CX::GetTelemetryType() const {
  return "Distance Sensor";
}
