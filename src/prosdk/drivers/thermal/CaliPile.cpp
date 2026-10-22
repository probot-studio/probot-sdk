// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// CaliPile driver (CaliPile.hpp). Sources: Excelitas TPiS 1S 1385 product specification
// (revised 04/01/2021) and TPiS 1T 1386 L5.5H product specification (revised 20/06/2024); the
// section numbers are 1S / 1T:
// - power-up and general call: 5.3 / 6.3 (only the general call is answered after power-up) and
//   5.5 / 6.5 (general call 0x00 then 0x04; up to 300 us to load the address);
// - registers: 6.1 / 7.1 (Table 12; values undefined after power-up, so all control registers are
//   written) and 6.2 / 7.2 (bit layouts, Table 25 low-pass codes); reads auto-increment;
// - EEPROM: 5.8 / 6.8 (ECR register 31 = 0x80 to read, 0x00 after) and 8.2-8.3 / 9.2-9.3
//   (Table 37: PROTOCOL 3, CHKSUM, LOOKUP#, PTAT25, M, U0, UOUT1, TOBJ1);
// - temperatures: 8.4 / 9.4 (ambient) and 8.5 / 9.5 (object; f(x) = x^3.8 for LOOKUP# 1,
//   x^4.2 for LOOKUP# 2; k scaled by emissivity times field-of-view coverage);
// - presence, motion, shock and TPOT: 7.2-7.6 / 8.2-8.6 (recommended start settings).
// Multi-byte values are MSB first (6.2 / 7.2). CHKSUM: the 16-bit sum of cells 0 and 3-31,
// cell 1 the high byte (the sample code of 1T 9.3).

#include "prosdk/drivers/thermal/CaliPile.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kReloadCommand = 0x04;
constexpr int kRegObject = 1;         // registers 1-19 are read in one burst
constexpr int kReadCount = 19;
constexpr int kRegSlp = 20;           // registers 20-29 are written in one burst
constexpr int kRegTpotThreshold = 28;
constexpr int kRegEepromControl = 31;
constexpr int kRegEeprom = 32;        // EEPROM cells 0-31 = registers 32-63
constexpr uint8_t kEepromRead = 0x80;
constexpr uint8_t kEepromIdle = 0x00;
constexpr uint8_t kProtocol = 3;

// Recommended start settings (1S 7.2-7.4): presence SLP1 = 1 s, SLP2 = 8 s, source
// TPobjLP1 - TPobjLP2, threshold 50; motion cycle 120 ms, threshold 10 (SLP1 is shared with
// presence: 1 s instead of the motion section's 0.5 s); shock SLP3 = 2 s, threshold 10. Interrupt
// mask 0 (INT pin unused), TPOT direction 1 (flag when TPobject exceeds the threshold), timer 0,
// TPOT threshold 0xFFFF (never) until one is set.
constexpr uint8_t kSlp12 = 0x8B;          // register 20: SLP2 8h (8 s), SLP1 Bh (1 s)
constexpr uint8_t kSlp3 = 0x0A;           // register 21: SLP3 Ah (2 s)
constexpr uint8_t kPresenceThreshold = 50;
constexpr uint8_t kMotionThreshold = 10;
constexpr uint8_t kShockThreshold = 10;
constexpr uint8_t kInterruptMask = 0x00;
constexpr uint8_t kMultiple = 0x16;       // register 26: TPOT dir 1, SRC 01b, cycle 10b (120 ms)
constexpr uint8_t kTimer = 0x00;

constexpr uint8_t kStatusOverTemperature = 0x10;  // chip status (register 19) flags
constexpr uint8_t kStatusPresence = 0x08;
constexpr uint8_t kStatusMotion = 0x04;

constexpr double kKelvin25 = 298.15;
constexpr double kInitRetryPeriod = 1.0;  // s
constexpr int kThresholdTolerance = 8;   // register counts (16 thermopile counts)

}  // namespace

CaliPile::CaliPile(I2C::Port port, Variant variant, int address)
    : m_i2c{port, address}, m_generalCall{port, 0x00}, m_variant{variant} {
  util::ReportUsage("CaliPile", address, variant == Variant::k1S1385 ? "1S1385" : "1T1386");
}

bool CaliPile::Reload() {
  uint8_t command = kReloadCommand;
  bool ok = !m_generalCall.WriteBulk(&command, 1);
  std::this_thread::sleep_for(std::chrono::microseconds{300});
  return ok;
}

bool CaliPile::Init() {
  const char* failure = nullptr;
  uint8_t ee[32] = {};
  if (!Reload()) {
    failure = "no answer to the general call";
  } else {
    bool read = !m_i2c.Write(kRegEepromControl, kEepromRead) &&
                !m_i2c.Read(kRegEeprom, sizeof(ee), ee);
    // back to the normal supply current, also after a failed read
    bool idle = !m_i2c.Write(kRegEepromControl, kEepromIdle);
    if (!read || !idle) {
      failure = "no answer";
    }
  }

  double exponent = 0.0;
  if (failure == nullptr) {
    unsigned sum = ee[0];
    for (int i = 3; i < 32; ++i) {
      sum += ee[i];
    }
    unsigned checksum = (static_cast<unsigned>(ee[1]) << 8) | ee[2];
    int lookup = ee[9];
    if ((sum & 0xFFFF) != checksum) {
      failure = "EEPROM checksum mismatch";
    } else if (lookup == 1) {
      exponent = 3.8;
    } else if (lookup == 2) {
      exponent = 4.2;
    } else {
      failure = "unknown EEPROM LOOKUP#";
    }
    int slope = (ee[12] << 8) | ee[13];
    int u0 = ((ee[14] << 8) | ee[15]) + 32768;
    int uout1 = ((ee[16] << 8) | ee[17]) * 2;
    if (failure == nullptr && (slope == 0 || uout1 == u0 || ee[18] == 25)) {
      failure = "invalid EEPROM calibration";  // the math below would divide by zero
    }
    if (failure == nullptr) {
      if (ee[0] != kProtocol) {
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] CaliPile 0x{:02X}: EEPROM PROTOCOL {} (expected {})\n",
                               m_i2c.GetDeviceAddress(), ee[0], kProtocol);
      }
      int expected = m_variant == Variant::k1S1385 ? 1 : 2;
      if (lookup != expected) {
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] CaliPile 0x{:02X}: EEPROM LOOKUP# {} does not match the "
                               "variant (expected {}); using the EEPROM\n",
                               m_i2c.GetDeviceAddress(), lookup, expected);
      }
    }
  }

  if (failure == nullptr) {
    uint8_t config[] = {kRegSlp,
                        kSlp12,
                        kSlp3,
                        kPresenceThreshold,
                        kMotionThreshold,
                        kShockThreshold,
                        kInterruptMask,
                        kMultiple,
                        kTimer,
                        0xFF,
                        0xFF};
    if (m_i2c.WriteBulk(config, sizeof(config))) {
      failure = "no answer";
    }
  }

  std::scoped_lock lock{m_mutex};
  if (failure != nullptr) {
    m_connected = false;
    if (!m_warned) {
      m_warned = true;
      platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                             "[prosdk] CaliPile 0x{:02X} on I2C port {}: {}, retrying\n",
                             m_i2c.GetDeviceAddress(), static_cast<int>(m_i2c.GetPort()),
                             failure);
    }
    return false;
  }
  m_exponent = exponent;
  m_ptat25 = ((ee[10] & 0x7F) << 8) | ee[11];                    // 15 bits
  m_slope = ((ee[12] << 8) | ee[13]) / 100.0;                    // M = RegVal / 100
  m_u0 = ((ee[14] << 8) | ee[15]) + 32768.0;                     // U0 = RegVal + 32768
  double uout1 = ((ee[16] << 8) | ee[17]) * 2.0;                 // UOUT1 = RegVal * 2
  double tobj1 = ee[18] + 273.15;                                // TOBJ1 in C
  m_k = (uout1 - m_u0) / (std::pow(tobj1, exponent) - std::pow(kKelvin25, exponent));
  m_thresholdRegister = 0xFFFF;
  m_thresholdDirty = m_threshold > 0.0;
  m_initialized = true;
  m_connected = true;
  return true;
}

uint16_t CaliPile::ThresholdRegister(double objectKelvin, double ambientKelvin) const {
  // inverse of the object formula: TPobject = k * (f(Tobj) - f(Tamb)) + U0; the register holds
  // TPobject / 2 (TPOT threshold, 16 bits)
  double counts = m_k * m_emissivity *
                      (std::pow(objectKelvin, m_exponent) - std::pow(ambientKelvin, m_exponent)) +
                  m_u0;
  return static_cast<uint16_t>(std::clamp(std::lround(counts / 2.0), 0L, 0xFFFFL));
}

void CaliPile::Update() {
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
  }
  if (!initialized && !Init()) {
    return;
  }

  uint8_t r[kReadCount];  // registers 1 to 19; reading 18 clears the interrupt status
  if (m_i2c.Read(kRegObject, kReadCount, r)) {
    // after a power loss the chip answers only the general call again: set it up anew
    std::scoped_lock lock{m_mutex};
    m_connected = false;
    m_initialized = false;
    return;
  }
  // TPobject: 17 bits in registers 1, 2, 3[7]; TPambient: 15 bits in 3[6:0], 4
  uint32_t tpObject = (static_cast<uint32_t>(r[0]) << 9) | (r[1] << 1) | (r[2] >> 7);
  uint32_t tpAmbient = (static_cast<uint32_t>(r[2] & 0x7F) << 8) | r[3];

  bool writeThreshold = false;
  uint16_t threshold = 0;
  {
    std::scoped_lock lock{m_mutex};
    m_connected = true;
    m_chipStatus = r[18];
    m_ambientK = kKelvin25 + (tpAmbient - m_ptat25) / m_slope;
    double fObject = (tpObject - m_u0) / (m_k * m_emissivity) +
                     std::pow(m_ambientK, m_exponent);
    m_objectK = fObject > 0.0 ? std::pow(fObject, 1.0 / m_exponent) : 0.0;
    m_haveReading = true;
    if (m_threshold > 0.0) {
      threshold = ThresholdRegister(m_threshold, m_ambientK);
      writeThreshold = m_thresholdDirty ||
                       std::abs(threshold - m_thresholdRegister) >= kThresholdTolerance;
    }
  }
  if (writeThreshold) {
    uint8_t frame[3] = {kRegTpotThreshold, static_cast<uint8_t>(threshold >> 8),
                        static_cast<uint8_t>(threshold)};
    bool ok = !m_i2c.WriteBulk(frame, 3);
    std::scoped_lock lock{m_mutex};
    m_connected = ok;
    if (ok) {
      m_thresholdRegister = threshold;
      m_thresholdDirty = false;
    }
  }
}

units::celsius_t CaliPile::GetObjectTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_haveReading ? m_objectK - 273.15 : 0.0};
}

units::celsius_t CaliPile::GetAmbientTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_haveReading ? m_ambientK - 273.15 : 0.0};
}

bool CaliPile::IsPresence() const {
  std::scoped_lock lock{m_mutex};
  return (m_chipStatus & kStatusPresence) != 0;
}

bool CaliPile::IsMotion() const {
  std::scoped_lock lock{m_mutex};
  return (m_chipStatus & kStatusMotion) != 0;
}

bool CaliPile::IsOverTemperature() const {
  std::scoped_lock lock{m_mutex};
  // the chip sets the flag at power-up (1S 7.5); it only means something once a threshold is
  // written
  return m_threshold > 0.0 && !m_thresholdDirty && (m_chipStatus & kStatusOverTemperature) != 0;
}

void CaliPile::SetEmissivity(double emissivity) {
  std::scoped_lock lock{m_mutex};
  m_emissivity = std::clamp(emissivity, 0.01, 1.0);
  m_thresholdDirty = m_threshold > 0.0;
}

void CaliPile::SetOverTemperatureThreshold(units::celsius_t threshold) {
  std::scoped_lock lock{m_mutex};
  m_threshold = threshold.value() + 273.15;
  m_thresholdDirty = true;
}

bool CaliPile::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void CaliPile::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Object", m_haveReading ? m_objectK - 273.15 : 0.0);
  table.Log("Ambient", m_haveReading ? m_ambientK - 273.15 : 0.0);
  table.Log("Presence", (m_chipStatus & kStatusPresence) != 0);
  table.Log("Motion", (m_chipStatus & kStatusMotion) != 0);
  table.Log("Connected", m_connected);
}

std::string_view CaliPile::GetTelemetryType() const {
  return "Thermopile Sensor";
}
