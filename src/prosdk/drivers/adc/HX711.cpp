// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// HX711 driver (HX711.hpp). Protocol: Avia Semiconductor HX711 data sheet (English, 9 pages):
// "Serial Interface" and Table 3 (25/26/27 pulses select channel A gain 128 / channel B gain 32 /
// channel A gain 64; Fig. 2 labels the 27-pulse case "CH.B, Gain:64", Table 3 and the "Analog
// Inputs" text say channel A), the timing table under Fig. 2 (T1-T4), "Reset and Power-Down" and
// Fig. 3 (PD_SCK high > 60 us), "Output Data Rate and Format" and the output settling time row
// of the electrical table.

#include "prosdk/drivers/adc/HX711.hpp"

#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kDataBits = 24;

// 400 ms at 10 SPS and 50 ms at 80 SPS: 4 conversion periods at either rate
constexpr int kSettleConversions = 4;

int Pulses(HX711::Gain gain) {
  switch (gain) {
    case HX711::Gain::k32B:
      return 26;
    case HX711::Gain::k64A:
      return 27;
    case HX711::Gain::k128A:
    default:
      return 25;
  }
}

double Now() {
  return Timer::GetMonotonicTimestamp().value();
}

}  // namespace

HX711::HX711(int dataGpio, int clockGpio, Gain gain)
    : m_clock{clockGpio}, m_data{dataGpio}, m_shift{m_clock, m_data}, m_gain{gain} {
  m_clock.Set(false);  // PD_SCK low: normal operation (high > 60 us powers the chip down)
  m_lastReadyTime = Now();
  util::ReportUsage("HX711", clockGpio, "");
}

void HX711::SetGain(Gain gain) {
  std::scoped_lock lock{m_mutex};
  m_gain = gain;
}

HX711::Gain HX711::GetGain() const {
  std::scoped_lock lock{m_mutex};
  return m_gain;
}

std::optional<int32_t> HX711::Update() {
  std::scoped_lock lock{m_mutex};
  if (m_poweredDown || m_shiftFailed) {
    return std::nullopt;
  }
  double now = Now();
  if (m_data.Get()) {  // DOUT high: no conversion ready
    if (now - m_lastReadyTime > kReadyTimeoutSeconds) {
      m_connected = false;
    }
    return std::nullopt;
  }
  m_lastReadyTime = now;
  // 24 data bits MSB first, then 1 to 3 pulses that select the next conversion
  int pulses = Pulses(m_gain);
  int extra = pulses - kDataBits;
  auto bits = m_shift.ShiftIn(pulses);
  if (!bits) {
    // reported to the console by DigitalShift. With valid arguments and owned pins the only
    // cause is an emergency stop, which lasts until reboot: stop clocking, so the console gets
    // one line and not one per call
    m_shiftFailed = true;
    m_connected = false;
    return std::nullopt;
  }
  // the 25th pulse pulls DOUT back high (Serial Interface), so the bits after the data are all
  // ones; zeros there mean DOUT is stuck low (no chip, or a broken line)
  uint64_t ones = (uint64_t{1} << extra) - 1;
  if ((*bits & ones) != ones) {
    m_connected = false;
    return std::nullopt;
  }
  m_connected = true;

  bool drop = false;
  if (m_discard > 0) {
    --m_discard;
    drop = true;
  }
  if (m_chipPulses != pulses) {
    // this conversion used another (or an unknown) selection; the pulses just sent switch the
    // next one, which then settles
    drop = true;
    m_discard = kSettleConversions;
  }
  m_chipPulses = pulses;
  if (drop) {
    return std::nullopt;
  }
  auto word = static_cast<int32_t>((*bits >> extra) & 0xFFFFFF);
  m_raw = (word & 0x800000) ? word - 0x1000000 : word;  // two's complement
  return m_raw;
}

int32_t HX711::GetRaw() const {
  std::scoped_lock lock{m_mutex};
  return m_raw;
}

bool HX711::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void HX711::PowerDown() {
  std::scoped_lock lock{m_mutex};
  m_poweredDown = true;
  m_clock.Set(true);
}

void HX711::PowerUp() {
  std::scoped_lock lock{m_mutex};
  m_clock.Set(false);
  if (m_poweredDown) {
    // the chip resets to channel A, gain 128 when PD_SCK returns low
    m_chipPulses = Pulses(Gain::k128A);
    m_discard = kSettleConversions;
    m_lastReadyTime = Now();
  }
  m_poweredDown = false;
}

void HX711::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Raw", m_raw);
  table.Log("Connected", m_connected);
}

std::string_view HX711::GetTelemetryType() const {
  return "ADC";
}
