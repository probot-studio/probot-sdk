// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Clocked addressable LED driver (ClockedAddressableLED.hpp). Frame format: Harvatek
// T3A33BRG-H9C0002X1U1930 datasheet v2.0 (2024-10-24), page 7 (CIN maximum 15 MHz), page 9
// "Cascading data structure" (MSB first, DIN latched on the rising CIN edge: SPI mode 0) and
// page 10 (fields):
//   - start: 32 zero bits;
//   - per LED, MSB first: FLAG[2:0] = 111, DIMMING[4:0], BLUE[7:0], GREEN[7:0], RED[7:0], so the
//     bytes 0xE0 | dimming, B, G, R; the first LED keeps the first word and passes the rest on;
//   - end: 32 one bits ("End of Frame", optional), and the host must send N/2 more clocks for N
//     LEDs so the last LED latches. Both are sent: 4 bytes 0xFF, then ceil(N / 16) bytes 0xFF.
// The sleep word (0xA0 00 00 00, page 10) is never sent: the first byte is always 0xE0 or more.

#include "prosdk/drivers/led/ClockedAddressableLED.hpp"

#include <algorithm>

#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kStartFrameBytes = 4;  // 32 zero bits
constexpr int kEndFrameBytes = 4;    // 32 one bits
constexpr uint8_t kFlag = 0xE0;      // FLAG[2:0] = 111 in bits 7..5 of the first byte

// The N/2 extra clocks for N LEDs, rounded up to whole bytes
int ExtraClockBytes(int length) {
  return (length + 15) / 16;
}

}  // namespace

ClockedAddressableLED::ClockedAddressableLED(SPI::Port bus, int length, int clockHz)
    : m_spi{bus, -1}, m_length{std::max(length, 0)} {
  m_spi.SetMode(SPI::kMode0);
  m_spi.SetClockRate(std::clamp(clockHz, SPI::kMinClockRate, kMaxClockRate));
  m_pixels.resize(static_cast<size_t>(m_length));
  {
    std::scoped_lock lock{m_mutex};
    SendLocked();  // all off
  }
  util::ReportUsage("ClockedAddressableLED", static_cast<int>(bus), "");
}

int ClockedAddressableLED::GetLength() const {
  return m_length;
}

void ClockedAddressableLED::SetData(std::span<const AddressableLED::LEDData> ledData) {
  std::scoped_lock lock{m_mutex};
  const size_t n = std::min(ledData.size(), m_pixels.size());
  std::copy_n(ledData.begin(), n, m_pixels.begin());
  std::fill(m_pixels.begin() + static_cast<std::ptrdiff_t>(n), m_pixels.end(),
            AddressableLED::LEDData{});
  SendLocked();
}

void ClockedAddressableLED::SetData(std::initializer_list<AddressableLED::LEDData> ledData) {
  SetData(std::span<const AddressableLED::LEDData>{ledData.begin(), ledData.size()});
}

void ClockedAddressableLED::SetBrightness(int level) {
  std::scoped_lock lock{m_mutex};
  m_brightness = std::clamp(level, 0, kMaxBrightness);
  SendLocked();
}

int ClockedAddressableLED::GetBrightness() const {
  std::scoped_lock lock{m_mutex};
  return m_brightness;
}

// m_mutex held
void ClockedAddressableLED::SendLocked() {
  m_frame.assign(kStartFrameBytes, 0x00);
  const auto header = static_cast<uint8_t>(kFlag | m_brightness);
  for (const AddressableLED::LEDData& pixel : m_pixels) {
    m_frame.push_back(header);
    m_frame.push_back(pixel.b);
    m_frame.push_back(pixel.g);
    m_frame.push_back(pixel.r);
  }
  m_frame.insert(m_frame.end(), kEndFrameBytes + ExtraClockBytes(m_length), 0xFF);
  // a failed transfer leaves the LEDs as they were; the next SetData sends everything again
  m_spi.Write(m_frame.data(), static_cast<int>(m_frame.size()));
}

void ClockedAddressableLED::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Length", m_length);
  table.Log("Brightness", m_brightness);
}

std::string_view ClockedAddressableLED::GetTelemetryType() const {
  return "Addressable LED";
}
