// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// MLX90640 driver (MLX90640.hpp) on the vendored Melexis API. Registers: MLX90640 datasheet
// rev 12 (3 December 2019), section 10.7.1: status register 0x8000 (Figure 11; bit 3 new data in
// RAM, bits 2:0 last measured subpage) and control register 1 0x800D (Figure 12; bits 9:7 refresh
// rate, bits 11:10 resolution, bit 12 reading pattern). The reflected temperature is the sensor
// temperature minus 8 C, the open-air shift of the Melexis driver document (MLX90640 driver.pdf,
// TA_SHIFT).

#include "prosdk/drivers/thermal/MLX90640.hpp"

#include <algorithm>
#include <chrono>

#include "prosdk/drivers/thermal/melexis/MLX90640_API.h"
#include "prosdk/drivers/thermal/melexis/MelexisI2C.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint16_t kRegStatus = 0x8000;
constexpr uint16_t kStatusNewData = 0x0008;
constexpr double kTaShift = 8.0;  // C, Melexis driver document, MLX90640 in open air
constexpr double kInitRetryPeriod = 1.0;  // s
constexpr std::size_t kThreadStack = 12 * 1024;  // the API keeps 3 KiB arrays on the stack

}  // namespace

struct MLX90640::Impl {
  melexis::paramsMLX90640 params;
  uint16_t eeprom[MLX90640_EEPROM_DUMP_NUM];
  uint16_t frame[834];  // 768 pixels, 64 aux words, control register, subpage (API layout)
  float work[kPixelCount];       // both subpages are converted into this buffer
  float published[kPixelCount];  // guarded by m_mutex
  uint8_t subpagesSeen = 0;      // bit n: subpage n converted since the last publish
};

MLX90640::MLX90640(I2C::Port port, int address, RefreshRate rate,
                   units::millisecond_t updatePeriod)
    : m_i2c{port, address},
      m_rate{rate},
      m_updatePeriod{updatePeriod},
      m_impl{std::make_unique<Impl>()} {
  util::ReportUsage("MLX90640", address, "");
  if (updatePeriod.value() > 0) {
    platform::ScopedThreadConfig threadConfig{{.name = "mlx90640", .stackSize = kThreadStack}};
    m_thread = std::thread([this] { Run(); });
  }
}

MLX90640::~MLX90640() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

bool MLX90640::Init() {
  auto address = static_cast<uint8_t>(m_i2c.GetDeviceAddress());
  m_impl->subpagesSeen = 0;
  int error;
  {
    melexis::ScopedI2C bind{m_i2c};
    error = melexis::MLX90640_DumpEE(address, m_impl->eeprom);
    if (error == 0) {
      error = melexis::MLX90640_SetRefreshRate(address, static_cast<uint8_t>(m_rate));
    }
  }
  if (error == 0) {
    // nonzero: more than 4 broken or outlier pixels, or two adjacent ones (Melexis API)
    error = melexis::MLX90640_ExtractParameters(m_impl->eeprom, &m_impl->params);
  }
  std::scoped_lock lock{m_mutex};
  m_initialized = error == 0;
  m_connected = m_initialized;
  if (!m_initialized && !m_warned) {
    m_warned = true;
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] MLX90640 0x{:02X} on I2C port {}: init failed ({}), retrying\n",
                           m_i2c.GetDeviceAddress(), static_cast<int>(m_i2c.GetPort()), error);
  }
  return m_initialized;
}

void MLX90640::Update() {
  std::scoped_lock updateLock{m_updateMutex};
  double emissivity;
  bool initialized;
  {
    std::scoped_lock lock{m_mutex};
    initialized = m_initialized;
    if (!initialized) {
      double now = Timer::GetMonotonicTimestamp().value();
      if (m_lastInitAttempt >= 0.0 && now - m_lastInitAttempt < kInitRetryPeriod) {
        return;
      }
      m_lastInitAttempt = now;
    }
    emissivity = m_emissivity;
  }
  if (!initialized && !Init()) {
    return;
  }

  auto address = static_cast<uint8_t>(m_i2c.GetDeviceAddress());
  Impl& impl = *m_impl;
  int subpage;
  {
    melexis::ScopedI2C bind{m_i2c};
    uint16_t status = 0;
    if (melexis::MLX90640_I2CRead(address, kRegStatus, 1, &status) != 0) {
      std::scoped_lock lock{m_mutex};
      m_connected = false;
      m_initialized = false;  // set the camera up again (it may have lost power)
      return;
    }
    if ((status & kStatusNewData) == 0) {
      std::scoped_lock lock{m_mutex};
      m_connected = true;
      return;
    }
    // reads the subpage, clears the new-data bit; returns the subpage or a negative error
    subpage = melexis::MLX90640_GetFrameData(address, impl.frame);
  }
  if (subpage < 0) {
    std::scoped_lock lock{m_mutex};
    m_connected = subpage != -1;  // -1: transfer failed; -8: frame data invalid
    m_initialized = m_connected;
    return;
  }

  float ta = melexis::MLX90640_GetTa(impl.frame, &impl.params);
  melexis::MLX90640_CalculateTo(impl.frame, &impl.params, static_cast<float>(emissivity),
                                static_cast<float>(ta - kTaShift), impl.work);
  impl.subpagesSeen |= static_cast<uint8_t>(1 << (subpage & 1));
  bool complete = impl.subpagesSeen == 0x3;
  if (complete) {
    impl.subpagesSeen = 0;
    int mode = (impl.frame[832] & 0x1000) >> 12;  // 0x800D bit 12: 1 = chess pattern
    melexis::MLX90640_BadPixelsCorrection(impl.params.brokenPixels, impl.work, mode, &impl.params);
    melexis::MLX90640_BadPixelsCorrection(impl.params.outlierPixels, impl.work, mode,
                                          &impl.params);
  }

  std::scoped_lock lock{m_mutex};
  m_connected = true;
  m_ambient = ta;
  if (complete) {
    std::copy(std::begin(impl.work), std::end(impl.work), std::begin(impl.published));
    auto [lo, hi] = std::minmax_element(std::begin(impl.work), std::end(impl.work));
    m_minimum = *lo;
    m_maximum = *hi;
    ++m_frameCount;
  }
}

void MLX90640::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

bool MLX90640::GetFrame(std::span<float, kPixelCount> out) const {
  std::scoped_lock lock{m_mutex};
  if (m_frameCount == 0) {
    return false;
  }
  std::copy(std::begin(m_impl->published), std::end(m_impl->published), out.begin());
  return true;
}

units::celsius_t MLX90640::GetAmbientTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_ambient};
}

uint32_t MLX90640::GetFrameCount() const {
  std::scoped_lock lock{m_mutex};
  return m_frameCount;
}

void MLX90640::SetEmissivity(double emissivity) {
  std::scoped_lock lock{m_mutex};
  m_emissivity = std::clamp(emissivity, 0.01, 1.0);
}

bool MLX90640::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void MLX90640::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Ambient", m_ambient);
  table.Log("Minimum", static_cast<double>(m_minimum));
  table.Log("Maximum", static_cast<double>(m_maximum));
  table.Log("Frames", static_cast<int64_t>(m_frameCount));
  table.Log("Connected", m_connected);
}

std::string_view MLX90640::GetTelemetryType() const {
  return "Thermal Camera";
}
