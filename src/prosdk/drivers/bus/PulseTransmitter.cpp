// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/drivers/bus/PulseTransmitter.hpp"

#include <cmath>

#include "prosdk/system/Errors.hpp"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
// Rounded to whole hertz; NaN and out-of-range values map to a value the HAL refuses.
uint32_t ToHz(units::hertz_t f) {
  double hz = std::round(f.value());
  return hz >= 0.0 && hz <= 4e9 ? static_cast<uint32_t>(hz) : UINT32_MAX;
}
}  // namespace

PulseTransmitter::PulseTransmitter(int channel, units::hertz_t carrier, double carrierDuty)
    : m_channel{channel}, m_carrier{carrier} {
  int32_t status = 0;
  auto stack = util::GetStackTrace(1);
  m_handle = platform::hal::OpenPulseTx(channel, ToHz(carrier), carrierDuty, stack.c_str(),
                                        &status);
  WPILIB_CheckErrorStatus(status, "Pulse transmitter on GPIO {} (carrier {:.0f} Hz)", channel,
                          carrier.value());
  util::ReportUsage("IO", channel, "PulseTransmitter");
}

bool PulseTransmitter::Send(std::span<const Pulse> pulses) {
  int32_t status = 0;
  platform::hal::SendPulses(m_handle, pulses, &status);
  if (status != 0) {
    WPILIB_ReportError(status, "Pulse transmitter on GPIO {} ({} pulses)", m_channel,
                       pulses.size());
    return false;
  }
  return true;
}

bool PulseTransmitter::WaitSent(units::second_t timeout) {
  double ms = std::ceil(timeout.value() * 1000.0);
  int32_t timeoutMs = ms > 0.0 ? (ms < 3600000.0 ? static_cast<int32_t>(ms) : 3600000) : 0;
  int32_t status = 0;
  bool sent = platform::hal::WaitPulsesSent(m_handle, timeoutMs, &status);
  WPILIB_ReportError(status, "Pulse transmitter on GPIO {}", m_channel);
  return sent;
}

int PulseTransmitter::GetChannel() const {
  return m_channel;
}

units::hertz_t PulseTransmitter::GetCarrier() const {
  return m_carrier;
}
