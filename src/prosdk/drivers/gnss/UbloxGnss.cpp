// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// u-blox GNSS driver (UbloxGnss.hpp). Configuration sequence:
//
//   1. probe, every second until UBX-MON-VER answers: UBX-CFG-PRT for the port in use (output UBX
//      only; a u-blox 8 DDC port may start with NMEA output only, SAM-M8Q data sheet table 9) and
//      a UBX-MON-VER poll. A receiver without UBX-CFG-PRT (M10) is expected to answer it with
//      ACK-NAK (unverified on hardware); any answer to it is ignored.
//   2. configure: protocol version < 24 (u-blox 8 lists 15 to 23.01): UBX-CFG-MSG NAV-PVT rate 1
//      and UBX-CFG-RATE; otherwise one UBX-CFG-VALSET (RAM layer) with CFG-<port>OUTPROT-NMEA 0,
//      CFG-MSGOUT-UBX_NAV_PVT_<port> 1 and CFG-RATE-MEAS. Each must be acknowledged (ACK-ACK);
//      no answer within a second resends, three failures go back to the probe.
//   3. running: NAV-PVT updates the fix; no NAV-PVT within the timeout goes back to the probe.

#include "prosdk/drivers/gnss/UbloxGnss.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegBytesAvailableHigh = 0xFD;  // then 0xFE, low byte (M8 section 11.5.1)
constexpr int kRegStream = 0xFF;
constexpr int kI2CChunk = 128;          // bytes per I2C read (well inside the 50 ms HAL timeout)
constexpr int kMaxBytesPerUpdate = 2048;
constexpr double kRetryPeriod = 1.0;    // s between probes or configuration attempts
constexpr int kMaxConfigAttempts = 3;
constexpr int kWarnAfterProbes = 5;
constexpr double kMinTimeout = 2.0;     // s
constexpr int kMinPeriodMs = 50;        // CFG-RATE measRate >= 50 ms below protocol 24 (M8 32.10.27)
constexpr int kMaxPeriodMs = 65535;     // U2
constexpr int kFirstValsetProtocol = 24;

// pending acknowledgements (bit per configuration message)
constexpr uint8_t kAckCfgMsg = 0x01;
constexpr uint8_t kAckCfgRate = 0x02;
constexpr uint8_t kAckCfgValset = 0x04;

uint8_t AckBit(uint8_t messageClass, uint8_t messageId) {
  if (messageClass != ubx::kClassCfg) {
    return 0;
  }
  switch (messageId) {
    case ubx::kIdCfgMsg:
      return kAckCfgMsg;
    case ubx::kIdCfgRate:
      return kAckCfgRate;
    case ubx::kIdCfgValset:
      return kAckCfgValset;
    default:
      return 0;
  }
}

}  // namespace

UbloxGnss::UbloxGnss(SerialPort::Port port, int baudRate, units::millisecond_t updatePeriod)
    : m_baudRate{baudRate},
      m_name{std::format("serial port {} ({} baud)", static_cast<int>(port), baudRate)} {
  m_serial.emplace(baudRate, port);
  m_serial->SetTimeout(0_s);
  util::ReportUsage("UbloxGnss", static_cast<int>(port), "serial");
  Start(updatePeriod);
}

UbloxGnss::UbloxGnss(I2C::Port port, int address, units::millisecond_t updatePeriod)
    : m_address{address},
      m_name{std::format("I2C port {} 0x{:02X}", static_cast<int>(port), address)} {
  m_i2c.emplace(port, address);
  util::ReportUsage("UbloxGnss", address, "I2C");
  Start(updatePeriod);
}

void UbloxGnss::Start(units::millisecond_t updatePeriod) {
  m_updatePeriod = updatePeriod;
  m_periodMs = static_cast<int>(1000.0 / kDefaultRate.value());
  if (m_updatePeriod > 0_ms) {
    platform::ScopedThreadConfig threadConfig{{.name = "ublox-gnss"}};
    m_thread = std::thread([this] { Run(); });
  }
}

UbloxGnss::~UbloxGnss() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void UbloxGnss::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

void UbloxGnss::Update() {
  if (m_serial) {
    ReadSerial();
  } else {
    ReadI2C();
  }
  for (const auto& frame : Step(Timer::GetMonotonicTimestamp().value())) {
    Send(frame);
  }
}

void UbloxGnss::ReadSerial() {
  std::array<char, 256> chunk;
  int budget = kMaxBytesPerUpdate;
  int available = m_serial->GetBytesReceived();
  while (available > 0 && budget > 0) {
    int n = m_serial->Read(chunk.data(), std::min({available, budget, static_cast<int>(chunk.size())}));
    if (n <= 0) {
      break;
    }
    Feed({reinterpret_cast<const uint8_t*>(chunk.data()), static_cast<size_t>(n)});
    available -= n;
    budget -= n;
  }
}

void UbloxGnss::ReadI2C() {
  uint8_t count[2];
  if (m_i2c->Read(kRegBytesAvailableHigh, 2, count)) {
    return;  // aborted (no answer)
  }
  int available = (count[0] << 8) | count[1];
  if (available == 0xFFFF) {
    available = 0;  // 0xFF fill: nothing waiting
  }
  available = std::min(available, kMaxBytesPerUpdate);
  std::array<uint8_t, kI2CChunk> chunk;
  while (available > 0) {
    int n = std::min(available, kI2CChunk);
    if (m_i2c->Read(kRegStream, n, chunk.data())) {
      return;
    }
    Feed({chunk.data(), static_cast<size_t>(n)});
    available -= n;
  }
}

void UbloxGnss::Send(const std::vector<uint8_t>& frame) {
  if (m_serial) {
    m_serial->Write(reinterpret_cast<const char*>(frame.data()), static_cast<int>(frame.size()));
  } else {
    std::vector<uint8_t> copy = frame;  // WriteBulk takes a mutable buffer
    m_i2c->WriteBulk(copy.data(), static_cast<int>(copy.size()));
  }
}

void UbloxGnss::Feed(std::span<const uint8_t> bytes) {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock parse{m_parseMutex};
  for (uint8_t b : bytes) {
    if (m_parser.Push(b)) {
      HandleMessage(m_parser.GetMessage(), now);
    }
  }
}

void UbloxGnss::HandleMessage(const ubx::Parser::Message& message, double now) {
  bool warnNak = false;
  std::unique_lock lock{m_mutex};
  m_lastFrame = now;
  const auto& payload = message.payload;
  if (message.messageClass == ubx::kClassMon && message.messageId == ubx::kIdMonVer) {
    if (m_stage == Stage::kProbe) {
      auto version = ubx::ParseProtocolVersion(payload);
      m_configInterface = version && *version >= kFirstValsetProtocol;
      m_stage = Stage::kConfigure;
      m_configSent = false;
      m_attempts = 0;
    }
  } else if (message.messageClass == ubx::kClassAck && payload.size() >= 2) {
    uint8_t bit = AckBit(payload[0], payload[1]);
    if (m_stage != Stage::kConfigure || (m_pendingAcks & bit) == 0) {
      return;  // not ours (for example the NAK of the probe's UBX-CFG-PRT on an M10)
    }
    if (message.messageId == ubx::kIdAckAck) {
      m_pendingAcks &= ~bit;
      if (m_pendingAcks == 0) {
        m_stage = Stage::kRunning;
        m_lastPvt = now;
      }
    } else if (message.messageId == ubx::kIdAckNak && !m_warnedNak) {
      m_warnedNak = true;
      warnNak = true;
    }
  } else if (message.messageClass == ubx::kClassNav && message.messageId == ubx::kIdNavPvt) {
    auto pvt = ubx::DecodeNavPvt(payload);
    if (!pvt) {
      return;
    }
    m_lastPvt = now;
    GnssFix fix;
    bool positionFix = pvt->fixType >= 2 && pvt->fixType <= 4;
    fix.valid = (pvt->flags & 0x01) != 0 && positionFix;  // gnssFixOK
    fix.fixType = pvt->fixType <= 5 ? static_cast<GnssFixType>(pvt->fixType) : GnssFixType::kNoFix;
    fix.latitude = units::degree_t{pvt->lat * 1e-7};
    fix.longitude = units::degree_t{pvt->lon * 1e-7};
    fix.heightMsl = units::meter_t{pvt->hMsl * 1e-3};
    fix.groundSpeed = units::meters_per_second_t{pvt->gSpeed * 1e-3};
    fix.heading = units::degree_t{pvt->headMot * 1e-5};
    fix.numSatellites = pvt->numSv;
    fix.hAcc = units::meter_t{pvt->hAcc * 1e-3};
    fix.utc.year = pvt->year;
    fix.utc.month = pvt->month;
    fix.utc.day = pvt->day;
    fix.utc.hour = pvt->hour;
    fix.utc.minute = pvt->minute;
    fix.utc.second = pvt->second + pvt->nano * 1e-9;  // nano may be negative (M10 3.15.11)
    fix.utc.dateValid = (pvt->valid & 0x01) != 0;
    fix.utc.timeValid = (pvt->valid & 0x02) != 0;
    fix.timestamp = units::second_t{now};
    m_fix = fix;
  }
  if (warnNak) {
    int periodMs = m_periodMs;
    lock.unlock();  // GetFix() never waits for the console
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] UbloxGnss on {}: receiver refused UBX 0x06 0x{:02X} "
                           "(navigation period {} ms)\n",
                           m_name, message.payload[1], periodMs);
  }
}

std::vector<std::vector<uint8_t>> UbloxGnss::Step(double now) {
  std::vector<std::vector<uint8_t>> frames;
  bool warnSilent = false;
  std::unique_lock lock{m_mutex};
  switch (m_stage) {
    case Stage::kProbe:
      if (m_attempts > 0 && now - m_lastSend < kRetryPeriod) {
        break;
      }
      if (++m_attempts == kWarnAfterProbes && !m_warnedSilent) {
        m_warnedSilent = true;
        warnSilent = true;
      }
      m_lastSend = now;
      frames.push_back(m_serial ? ubx::EncodeCfgPrtUart1(static_cast<uint32_t>(m_baudRate))
                                : ubx::EncodeCfgPrtDdc(m_address));
      frames.push_back(ubx::Encode(ubx::kClassMon, ubx::kIdMonVer));
      break;
    case Stage::kConfigure:
      if (m_configSent && now - m_lastSend < kRetryPeriod) {
        break;
      }
      if (m_attempts >= kMaxConfigAttempts) {
        m_stage = Stage::kProbe;  // detect the receiver again
        m_attempts = 0;
        break;
      }
      ++m_attempts;
      m_configSent = true;
      m_lastSend = now;
      if (m_configInterface) {
        const ubx::ConfigItem items[] = {
            {m_serial ? ubx::key::kUart1OutprotNmea : ubx::key::kI2cOutprotNmea, 0},
            {m_serial ? ubx::key::kMsgoutNavPvtUart1 : ubx::key::kMsgoutNavPvtI2c, 1},
            {ubx::key::kRateMeas, static_cast<uint64_t>(m_periodMs)},
        };
        frames.push_back(ubx::EncodeValset(ubx::kLayerRam, items));
        m_pendingAcks = kAckCfgValset;
      } else {
        frames.push_back(ubx::EncodeCfgMsg(ubx::kClassNav, ubx::kIdNavPvt, 1));
        frames.push_back(ubx::EncodeCfgRate(static_cast<uint16_t>(m_periodMs)));
        m_pendingAcks = kAckCfgMsg | kAckCfgRate;
      }
      break;
    case Stage::kRunning:
      if (now - m_lastPvt > TimeoutSeconds()) {
        m_stage = Stage::kProbe;  // receiver silent: restarted (configuration lost) or unplugged
        m_attempts = 0;
      }
      break;
  }
  lock.unlock();  // GetFix() never waits for the console
  if (warnSilent) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] UbloxGnss on {}: no answer to UBX-MON-VER\n", m_name);
  }
  return frames;
}

double UbloxGnss::TimeoutSeconds() const {
  return std::max(kMinTimeout, 3.0 * m_periodMs / 1000.0);
}

GnssFix UbloxGnss::GetFix() const {
  std::scoped_lock lock{m_mutex};
  return m_fix;
}

void UbloxGnss::SetRate(units::hertz_t rate) {
  double periodMs = rate.value() > 0.0 ? 1000.0 / rate.value() : kMaxPeriodMs;
  int clamped = static_cast<int>(std::lround(std::clamp(periodMs, double{kMinPeriodMs},
                                                        double{kMaxPeriodMs})));
  std::scoped_lock lock{m_mutex};
  if (clamped == m_periodMs) {
    return;
  }
  m_periodMs = clamped;
  m_warnedNak = false;
  if (m_stage != Stage::kProbe) {
    m_stage = Stage::kConfigure;  // send the new period
    m_configSent = false;
    m_attempts = 0;
  }
}

bool UbloxGnss::IsConnected() const {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  return m_lastFrame >= 0.0 && now - m_lastFrame <= TimeoutSeconds();
}

bool UbloxGnss::IsConfigured() const {
  std::scoped_lock lock{m_mutex};
  return m_stage == Stage::kRunning;
}

uint32_t UbloxGnss::GetChecksumErrors() const {
  std::scoped_lock parse{m_parseMutex};
  return m_parser.GetErrorCount();
}

void UbloxGnss::LogTo(telemetry::TelemetryTable& table) const {
  GnssFix fix = GetFix();
  table.Log("Valid", fix.valid);
  table.Log("Fix Type", static_cast<int32_t>(fix.fixType));
  table.Log("Latitude", fix.latitude.value());
  table.Log("Longitude", fix.longitude.value());
  table.Log("Height MSL", fix.heightMsl.value());
  table.Log("Ground Speed", fix.groundSpeed.value());
  table.Log("Heading", fix.heading.value());
  table.Log("Satellites", static_cast<int32_t>(fix.numSatellites));
  table.Log("Horizontal Accuracy", fix.hAcc.value());
}

std::string_view UbloxGnss::GetTelemetryType() const {
  return "GNSS";
}
