// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// API of the WPILib 2026 frc::SPI (WPILib BSD license), re-ported on the prosdk SPI backend.
//
// SPI (SPI.hpp). All hardware access goes through prosdk/platform/hal/SPI.hpp, so the class runs
// unchanged on the host simulation.

#include "prosdk/drivers/bus/SPI.hpp"

#include <format>
#include <utility>

#include "prosdk/system/Errors.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

SPI::SPI(Port bus, int csGpio) : m_port{bus}, m_cs{csGpio} {
  int32_t status = 0;
  m_handle = platform::hal::OpenSpiDevice(static_cast<int>(bus), csGpio, m_config, &status);
  WPILIB_CheckErrorStatus(status, "SPI bus {} CS {}", static_cast<int>(bus), csGpio);
  util::ReportUsage(std::format("SPI[{}]", static_cast<int>(bus)), csGpio, "");
}

SPI::SPI(SPI&& other) noexcept
    : m_port{other.m_port},
      m_cs{other.m_cs},
      m_config{other.m_config},
      m_handle{std::exchange(other.m_handle, platform::hal::kInvalidSpiHandle)} {}

SPI& SPI::operator=(SPI&& other) noexcept {
  if (this != &other) {
    platform::hal::CloseSpiDevice(m_handle);
    m_port = other.m_port;
    m_cs = other.m_cs;
    m_config = other.m_config;
    m_handle = std::exchange(other.m_handle, platform::hal::kInvalidSpiHandle);
  }
  return *this;
}

SPI::~SPI() {
  platform::hal::CloseSpiDevice(m_handle);
}

SPI::Port SPI::GetPort() const {
  return m_port;
}

int SPI::GetChipSelect() const {
  return m_cs;
}

void SPI::SetClockRate(int hz) {
  if (hz < kMinClockRate || hz > kMaxClockRate) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "SPI clock rate {} Hz (valid: {} to {} Hz)",
                           hz, kMinClockRate, kMaxClockRate);
  }
  auto config = m_config;
  config.clockHz = hz;
  Configure(config);
}

void SPI::SetMode(Mode mode) {
  if (mode < kMode0 || mode > kMode3) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "SPI mode {}", static_cast<int>(mode));
  }
  auto config = m_config;
  config.mode = static_cast<int>(mode);
  Configure(config);
}

void SPI::SetChipSelectActiveHigh() {
  auto config = m_config;
  config.csActiveHigh = true;
  Configure(config);
}

void SPI::SetChipSelectActiveLow() {
  auto config = m_config;
  config.csActiveHigh = false;
  Configure(config);
}

void SPI::Configure(const platform::hal::SpiDeviceConfig& config) {
  int32_t status = 0;
  platform::hal::ConfigureSpiDevice(m_handle, config, &status);
  WPILIB_CheckErrorStatus(status, "SPI bus {} CS {}", static_cast<int>(m_port), m_cs);
  m_config = config;  // only what the device accepted (a refused rate keeps the previous one)
}

int SPI::Write(const uint8_t* data, int size) {
  const SpiSegment segment{data, nullptr, size};
  return TransactionSegments({&segment, 1});
}

int SPI::Read(bool initiate, uint8_t* dataReceived, int size) {
  (void)initiate;
  const SpiSegment segment{nullptr, dataReceived, size};
  return TransactionSegments({&segment, 1});
}

int SPI::Transaction(const uint8_t* dataToSend, uint8_t* dataReceived, int size) {
  const SpiSegment segment{dataToSend, dataReceived, size};
  return TransactionSegments({&segment, 1});
}

int SPI::TransactionSegments(std::span<const SpiSegment> segments) {
  int total = 0;
  for (const SpiSegment& s : segments) {
    if (s.size < 0) {
      return -1;
    }
    total += s.size;
  }
  return platform::hal::SpiTransfer(m_handle, segments) ? total : -1;
}
