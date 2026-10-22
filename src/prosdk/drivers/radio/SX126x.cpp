// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// SX126x LoRa driver (SX126x.hpp). Semtech SX1261/2 datasheet rev. 1.2 (June 2019): section 6.1.1.4
// (low data rate optimization from 16.38 ms symbols), 6.1.4 (time on air), 8.1 (NRESET low
// about 100 us), 8.2 (SPI mode 0, up to 16 MHz), 8.3.1 (BUSY: wait for low before every command;
// it rises up to 600 ns after NSS), 9.2.1 and table 9-2 (image calibration bands; a TCXO needs a
// full calibration after SetDIO3AsTcxoCtrl), table 12-1 (registers), section 13 (commands and
// opcodes, tables 13-1 to 13-86), 14.2 and 14.3 (TX and RX sequences), 14.4 (SetPacketType, then
// SetModulationParams, then SetPacketParams) and section 15 (known limitations 15.1, 15.2, 15.4).
// Ebyte E22-900M22S user manual v1.4: TCXO powered from DIO3, RXEN and TXEN high-active.

#include "prosdk/drivers/radio/SX126x.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// Opcodes (section 13)
constexpr uint8_t kClearIrqStatus = 0x02;
constexpr uint8_t kClearDeviceErrors = 0x07;
constexpr uint8_t kSetDioIrqParams = 0x08;
constexpr uint8_t kWriteRegister = 0x0D;
constexpr uint8_t kWriteBuffer = 0x0E;
constexpr uint8_t kGetIrqStatus = 0x12;
constexpr uint8_t kGetRxBufferStatus = 0x13;
constexpr uint8_t kGetPacketStatus = 0x14;
constexpr uint8_t kGetDeviceErrors = 0x17;
constexpr uint8_t kReadRegister = 0x1D;
constexpr uint8_t kReadBuffer = 0x1E;
constexpr uint8_t kSetStandby = 0x80;
constexpr uint8_t kSetRx = 0x82;
constexpr uint8_t kSetTx = 0x83;
constexpr uint8_t kSetRfFrequency = 0x86;
constexpr uint8_t kCalibrate = 0x89;
constexpr uint8_t kSetPacketType = 0x8A;
constexpr uint8_t kSetModulationParams = 0x8B;
constexpr uint8_t kSetPacketParams = 0x8C;
constexpr uint8_t kSetTxParams = 0x8E;
constexpr uint8_t kSetBufferBaseAddress = 0x8F;
constexpr uint8_t kSetPaConfig = 0x95;
constexpr uint8_t kSetDio3AsTcxoCtrl = 0x97;
constexpr uint8_t kCalibrateImage = 0x98;
constexpr uint8_t kSetDio2AsRfSwitchCtrl = 0x9D;
constexpr uint8_t kGetStatus = 0xC0;

// Registers (table 12-1)
constexpr uint16_t kRegIqPolarity = 0x0736;
constexpr uint16_t kRegLoRaSyncWord = 0x0740;  // MSB; LSB at 0x0741
constexpr uint16_t kRegTxModulation = 0x0889;
constexpr uint16_t kRegTxClampConfig = 0x08D8;

// IRQ bits (table 13-29)
constexpr uint16_t kIrqTxDone = 1 << 0;
constexpr uint16_t kIrqRxDone = 1 << 1;
constexpr uint16_t kIrqHeaderErr = 1 << 5;
constexpr uint16_t kIrqCrcErr = 1 << 6;
constexpr uint16_t kIrqTimeout = 1 << 9;
constexpr uint16_t kIrqAll = 0x03FF;
constexpr uint16_t kIrqUsed = kIrqTxDone | kIrqRxDone | kIrqHeaderErr | kIrqCrcErr | kIrqTimeout;

constexpr uint8_t kStandbyRc = 0x00;          // table 13-4
constexpr uint8_t kChipModeStandbyRc = 0x2;   // status bits 6:4, table 13-76
constexpr uint8_t kCalibrateAll = 0x7F;       // table 13-18: bits 0-6
constexpr uint16_t kCalibrationErrors = 0x7F;  // table 13-85: bits 0-6 (incl. XOSC_START_ERR)
constexpr uint8_t kPacketTypeLoRa = 0x01;     // table 13-38
constexpr uint8_t kRamp200us = 0x04;          // table 13-41
constexpr uint32_t kRxContinuous = 0xFFFFFF;  // table 13-9
constexpr uint32_t kMaxTimeout = 0xFFFFFE;    // 262 s, table 13-7
constexpr double kTimeoutStepMs = 0.015625;   // 15.625 us per timeout and delay step

// SX1262 high-power PA at +22 dBm (table 13-21): paDutyCycle 0x04 (the maximum allowed for the
// SX1262), hpMax 0x07, deviceSel 0 (SX1262), paLut 1. SetTxParams then sets -9 to +22 dBm.
constexpr uint8_t kPaDutyCycle = 0x04;
constexpr uint8_t kHpMax = 0x07;

// TCXO start-up delay given to SetDIO3AsTcxoCtrl. The E22 manual gives none; 5 ms is a common
// TCXO settling time (unverified for the E22's TCXO).
constexpr uint32_t kTcxoDelaySteps = 320;  // 320 x 15.625 us = 5 ms

// BUSY is high at most 3.5 ms for a calibration or a cold start (sections 13.1.12, table 8-2)
// plus the TCXO delay; the margin covers a slow image calibration (no time is given for it).
constexpr auto kBusyTimeout = std::chrono::milliseconds{100};

// Table 13-48 codes and the real bandwidths, in the order of SX126xConfig::Bandwidth.
constexpr uint8_t kBandwidthCode[] = {0x00, 0x08, 0x01, 0x09, 0x02, 0x0A, 0x03, 0x04, 0x05, 0x06};
constexpr double kBandwidthHz[] = {125000.0 / 16, 125000.0 / 12, 125000.0 / 8, 125000.0 / 6,
                                   125000.0 / 4,  125000.0 / 3,  62500.0,      125000.0,
                                   250000.0,      500000.0};

// Table 9-2: image calibration bands, MHz, with their freq1/freq2 codes.
struct ImageBand {
  double lowMHz, highMHz;
  uint8_t freq1, freq2;
};
constexpr ImageBand kImageBands[] = {{430, 440, 0x6B, 0x6F},
                                     {470, 510, 0x75, 0x81},
                                     {779, 787, 0xC1, 0xC5},
                                     {863, 870, 0xD7, 0xDB},
                                     {902, 928, 0xE1, 0xE9}};

int BandwidthIndex(SX126xConfig::Bandwidth bandwidth) {
  return std::clamp(static_cast<int>(bandwidth), 0, 9);
}

double SymbolTimeMs(const SX126xConfig& config) {
  return std::ldexp(1.0, config.spreadingFactor) / kBandwidthHz[BandwidthIndex(config.bandwidth)] *
         1000.0;
}

// Section 6.1.1.4: on when a symbol lasts 16.38 ms or more (SF11 and SF12 at 125 kHz, ...).
bool LowDataRateOptimize(const SX126xConfig& config) {
  return SymbolTimeMs(config) >= 16.38;
}

uint8_t High(uint32_t value) {
  return static_cast<uint8_t>(value >> 8);
}

uint8_t Low(uint32_t value) {
  return static_cast<uint8_t>(value);
}

}  // namespace

SX126x::SX126x(SPI::Port bus, int csGpio, int busy, int dio1, int reset, int rxEn, int txEn,
               const Config& config)
    : m_spi{bus, csGpio}, m_busy{busy}, m_dio1{dio1}, m_config{config} {
  m_spi.SetMode(SPI::kMode0);
  m_spi.SetClockRate(config.spiClockHz);
  if (reset >= 0) {
    m_reset.emplace(reset);
    m_reset->Set(true);  // released
  }
  if (rxEn >= 0) {
    m_rxEn.emplace(rxEn);
    m_rxEn->Set(false);
  }
  if (txEn >= 0) {
    m_txEn.emplace(txEn);
    m_txEn->Set(false);
  }
  util::ReportUsage("SX126x", csGpio, "");
  platform::ScopedThreadConfig threadConfig{{.name = "sx126x"}};
  m_thread = std::thread([this] { Run(); });
}

SX126x::~SX126x() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
  std::scoped_lock lock{m_mutex};
  if (m_ready) {
    SetStandby();  // stop a transmission or reception
  }
  SetRfPath(RfPath::kOff);
}

bool SX126x::Begin() {
  const char* failed = nullptr;
  uint16_t deviceErrors = 0;
  bool ok;
  {
    std::scoped_lock lock{m_mutex};
    m_begun = false;
    m_ready = false;
    m_txPending = false;
    m_rxWanted = false;
    m_packets.clear();
    m_warnedBusy = true;   // a BUSY timeout here is reported by the line below
    m_lastReadyOk = true;  // wait the full BUSY timeout (reset, calibration)
    m_busyHigh = false;
    SetRfPath(RfPath::kOff);
    ok = Configure(&failed, &deviceErrors);
    m_ready = ok;
    m_warnedBusy = false;
  }
  m_begun = ok;
  if (!ok) {
    if (deviceErrors != 0) {
      platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                             "[prosdk] SX126x on SPI bus {} CS {}: {} (device errors 0x{:04X})\n",
                             static_cast<int>(m_spi.GetPort()), m_spi.GetChipSelect(), failed,
                             deviceErrors);
    } else {
      platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                             "[prosdk] SX126x on SPI bus {} CS {}: {}\n",
                             static_cast<int>(m_spi.GetPort()), m_spi.GetChipSelect(), failed);
    }
  }
  return ok;
}

// m_mutex held. Section 14.2/14.3 order; every step after the reset needs BUSY low.
bool SX126x::Configure(const char** failed, uint16_t* deviceErrors) {
  const Config& c = m_config;
  const double hz = c.frequency.value();
  if (!(hz >= 150e6 && hz <= 960e6)) {
    *failed = "frequency outside 150-960 MHz";
    return false;
  }
  if (c.spreadingFactor < 5 || c.spreadingFactor > 12) {
    *failed = "spreading factor outside 5-12";
    return false;
  }
  if (c.codingRate < 5 || c.codingRate > 8) {
    *failed = "coding rate outside 4/5-4/8";
    return false;
  }
  if (c.powerDbm < -9 || c.powerDbm > 22) {
    *failed = "power outside -9 to +22 dBm";
    return false;
  }
  if (c.preambleLength < 1 || c.preambleLength > 0xFFFF) {
    *failed = "preamble length outside 1-65535";
    return false;
  }

  if (m_reset) {
    // section 8.1: NRESET low for typically 100 us; the chip then boots and calibrates with
    // BUSY high (cold start about 3.5 ms, table 8-2)
    m_reset->Set(false);
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
    m_reset->Set(true);
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
    m_lastCommand = std::chrono::steady_clock::now();
  }

  // presence: in STDBY_RC after SetStandby (a chip left in RX or TX by an earlier Begin() too)
  uint8_t status[2] = {};
  if (!Command(std::array<uint8_t, 2>{kSetStandby, kStandbyRc}) ||
      !Read(std::array<uint8_t, 2>{kGetStatus, 0}, status) ||
      ((status[1] >> 4) & 0x07) != kChipModeStandbyRc) {
    *failed = "no answer (BUSY stuck or not in STDBY_RC)";
    return false;
  }

  // TCXO on DIO3 (13.3.6), then a full calibration (9.2.1: the power-on calibration fails with
  // a TCXO); XOSC_START_ERR from the power-on start is expected and cleared first
  bool ok = true;
  if (c.tcxoVoltage != Config::TcxoVoltage::kNone) {
    const uint8_t voltage = static_cast<uint8_t>(static_cast<int>(c.tcxoVoltage) - 1);
    ok = Command(std::array<uint8_t, 5>{kSetDio3AsTcxoCtrl, voltage,
                                        static_cast<uint8_t>(kTcxoDelaySteps >> 16),
                                        High(kTcxoDelaySteps), Low(kTcxoDelaySteps)});
  }
  ok = ok && Command(std::array<uint8_t, 3>{kClearDeviceErrors, 0, 0});
  ok = ok && Command(std::array<uint8_t, 2>{kCalibrate, kCalibrateAll});
  const double mhz = hz / 1e6;
  for (const ImageBand& band : kImageBands) {
    if (ok && mhz >= band.lowMHz && mhz <= band.highMHz) {
      ok = Command(std::array<uint8_t, 3>{kCalibrateImage, band.freq1, band.freq2});
    }
  }
  if (!ok || !GetDeviceErrors(deviceErrors)) {
    *failed = "calibration command failed";
    return false;
  }
  if ((*deviceErrors & kCalibrationErrors) != 0) {
    *failed = "calibration failed (TCXO voltage?)";
    return false;
  }
  *deviceErrors = 0;

  if (c.dio2AsRfSwitch) {
    ok = Command(std::array<uint8_t, 2>{kSetDio2AsRfSwitchCtrl, 0x01});
  }
  ok = ok && Command(std::array<uint8_t, 2>{kSetPacketType, kPacketTypeLoRa});
  // 13.4.1: RfFreq = frequency x 2^25 / 32 MHz
  const uint32_t frf = static_cast<uint32_t>(std::llround(hz * 33554432.0 / 32e6));
  ok = ok && Command(std::array<uint8_t, 5>{kSetRfFrequency, static_cast<uint8_t>(frf >> 24),
                                            static_cast<uint8_t>(frf >> 16), High(frf), Low(frf)});
  ok = ok && Command(std::array<uint8_t, 5>{kSetPaConfig, kPaDutyCycle, kHpMax, 0x00, 0x01});
  ok = ok && Command(std::array<uint8_t, 3>{
                 kSetTxParams, static_cast<uint8_t>(static_cast<int8_t>(c.powerDbm)), kRamp200us});
  // 15.2: TxClampConfig bits 4-1 to 1111 after a power-on reset (SX1262 antenna mismatch)
  uint8_t clamp = 0;
  ok = ok && ReadRegister(kRegTxClampConfig, std::span{&clamp, 1});
  clamp |= 0x1E;
  ok = ok && WriteRegister(kRegTxClampConfig, std::span{&clamp, 1});
  ok = ok && Command(std::array<uint8_t, 3>{kSetBufferBaseAddress, 0x00, 0x00});
  ok = ok && Command(std::array<uint8_t, 5>{
                 kSetModulationParams, static_cast<uint8_t>(c.spreadingFactor),
                 kBandwidthCode[BandwidthIndex(c.bandwidth)],
                 static_cast<uint8_t>(c.codingRate - 4),
                 static_cast<uint8_t>(LowDataRateOptimize(c) ? 0x01 : 0x00)});
  ok = ok && SetPacketParams(kMaxPayload);
  // 15.4: IQ polarity bit 2 set for standard IQ
  uint8_t iq = 0;
  ok = ok && ReadRegister(kRegIqPolarity, std::span{&iq, 1});
  iq |= 0x04;
  ok = ok && WriteRegister(kRegIqPolarity, std::span{&iq, 1});
  const uint8_t syncWord[2] = {High(c.syncWord), Low(c.syncWord)};
  ok = ok && WriteRegister(kRegLoRaSyncWord, syncWord);
  // 13.3.1: the used IRQs on DIO1 only
  ok = ok && Command(std::array<uint8_t, 9>{kSetDioIrqParams, High(kIrqUsed), Low(kIrqUsed),
                                            High(kIrqUsed), Low(kIrqUsed), 0, 0, 0, 0});
  ok = ok && ClearIrq(kIrqAll);
  if (!ok) {
    *failed = "configuration command failed";
  }
  return ok;
}

// m_mutex held. Section 8.3.1: BUSY must be low before every command, read or write; it rises
// at most 600 ns (TSW) after NSS, so it is not sampled within 1 us of the last command. Once BUSY
// was stuck high for kBusyTimeout, commands fail at once until BUSY is seen low again, so a lost
// chip does not block every call for the full timeout.
bool SX126x::WaitReady() {
  while (std::chrono::steady_clock::now() - m_lastCommand < std::chrono::microseconds{1}) {
  }
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = m_lastReadyOk ? kBusyTimeout : std::chrono::milliseconds{0};
  while (m_busy.Get()) {
    const auto waited = std::chrono::steady_clock::now() - start;
    if (waited >= timeout) {
      BusyStuck();
      return false;
    }
    // short polls for mode switches; 1 ms sleeps for a calibration or the TCXO start (ESP-IDF
    // busy-waits sleeps shorter than a tick)
    std::this_thread::sleep_for(waited < std::chrono::milliseconds{1}
                                    ? std::chrono::microseconds{100}
                                    : std::chrono::microseconds{1000});
  }
  m_lastReadyOk = true;
  m_busyHigh = false;
  return true;
}

// m_mutex held
void SX126x::BusyStuck() {
  m_lastReadyOk = false;
  if (!m_warnedBusy) {
    m_warnedBusy = true;
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] SX126x on SPI bus {} CS {}: BUSY stuck high\n",
                           static_cast<int>(m_spi.GetPort()), m_spi.GetChipSelect());
  }
}

// m_mutex held. An opcode and its parameters (the replies are status bytes only).
bool SX126x::Command(std::span<const uint8_t> command) {
  if (!WaitReady()) {
    return false;
  }
  const int size = static_cast<int>(command.size());
  const bool ok = m_spi.Write(command.data(), size) == size;
  m_lastCommand = std::chrono::steady_clock::now();
  return ok;
}

// m_mutex held. An opcode with NOPs; reply has the same length (byte 1 is the status).
bool SX126x::Read(std::span<const uint8_t> command, std::span<uint8_t> reply) {
  if (!WaitReady() || reply.size() != command.size()) {
    return false;
  }
  const int size = static_cast<int>(command.size());
  const bool ok = m_spi.Transaction(command.data(), reply.data(), size) == size;
  m_lastCommand = std::chrono::steady_clock::now();
  return ok;
}

// m_mutex held. Table 13-24: opcode, address MSB first, data.
bool SX126x::WriteRegister(uint16_t address, std::span<const uint8_t> data) {
  if (!WaitReady()) {
    return false;
  }
  const uint8_t header[3] = {kWriteRegister, High(address), Low(address)};
  const SpiSegment segments[] = {{header, nullptr, 3},
                                 {data.data(), nullptr, static_cast<int>(data.size())}};
  const bool ok = m_spi.TransactionSegments(segments) == 3 + static_cast<int>(data.size());
  m_lastCommand = std::chrono::steady_clock::now();
  return ok;
}

// m_mutex held. Table 13-25: opcode, address, one NOP, then the data.
bool SX126x::ReadRegister(uint16_t address, std::span<uint8_t> data) {
  if (!WaitReady()) {
    return false;
  }
  const uint8_t header[4] = {kReadRegister, High(address), Low(address), 0};
  const SpiSegment segments[] = {{header, nullptr, 4},
                                 {nullptr, data.data(), static_cast<int>(data.size())}};
  const bool ok = m_spi.TransactionSegments(segments) == 4 + static_cast<int>(data.size());
  m_lastCommand = std::chrono::steady_clock::now();
  return ok;
}

// m_mutex held
bool SX126x::SetStandby() {
  return Command(std::array<uint8_t, 2>{kSetStandby, kStandbyRc});
}

// m_mutex held. Table 13-66 to 13-70: preamble, explicit header, payload length, CRC on,
// standard IQ.
bool SX126x::SetPacketParams(int payloadLength) {
  const uint32_t preamble = static_cast<uint32_t>(m_config.preambleLength);
  return Command(std::array<uint8_t, 7>{kSetPacketParams, High(preamble), Low(preamble), 0x00,
                                        static_cast<uint8_t>(payloadLength), 0x01, 0x00});
}

// m_mutex held
bool SX126x::ClearIrq(uint16_t flags) {
  return Command(std::array<uint8_t, 3>{kClearIrqStatus, High(flags), Low(flags)});
}

// m_mutex held. Table 13-30: status, then IrqStatus MSB first.
bool SX126x::GetIrqStatus(uint16_t* flags) {
  uint8_t reply[4] = {};
  if (!Read(std::array<uint8_t, 4>{kGetIrqStatus, 0, 0, 0}, reply)) {
    return false;
  }
  *flags = static_cast<uint16_t>((reply[2] << 8) | reply[3]);
  return true;
}

// m_mutex held. Table 13-84: status, then OpError MSB first.
bool SX126x::GetDeviceErrors(uint16_t* errors) {
  uint8_t reply[4] = {};
  if (!Read(std::array<uint8_t, 4>{kGetDeviceErrors, 0, 0, 0}, reply)) {
    return false;
  }
  *errors = static_cast<uint16_t>((reply[2] << 8) | reply[3]);
  return true;
}

// m_mutex held. The RF switch of the E22 (RXEN, TXEN high-active); with dio2AsRfSwitch the chip
// drives TXEN itself.
void SX126x::SetRfPath(RfPath path) {
  if (m_txEn) {
    m_txEn->Set(path == RfPath::kTx);
  }
  if (m_rxEn) {
    m_rxEn->Set(path == RfPath::kRx);
  }
}

bool SX126x::Send(std::span<const uint8_t> data) {
  if (data.empty() || data.size() > static_cast<size_t>(kMaxPayload)) {
    return false;
  }
  // 13.1.4: the chip ends the send itself after timeout x 15.625 us (at most 262 s)
  const double timeoutMs = 2.0 * GetTimeOnAir(static_cast<int>(data.size())).value() + 100.0;
  const uint32_t timeout = static_cast<uint32_t>(
      std::min(std::ceil(timeoutMs / kTimeoutStepMs), static_cast<double>(kMaxTimeout)));
  std::scoped_lock lock{m_mutex};
  if (!m_ready || m_txPending) {
    return false;
  }
  // 14.2: standby (stops a reception), payload, then TX with the RF switch on the TX path.
  // 15.1: before each transmission TxModulation bit 2 is cleared at 500 kHz and set otherwise.
  bool ok = SetStandby() && SetPacketParams(static_cast<int>(data.size()));
  uint8_t modulation = 0;
  ok = ok && ReadRegister(kRegTxModulation, std::span{&modulation, 1});
  if (m_config.bandwidth == Config::Bandwidth::k500kHz) {
    modulation &= static_cast<uint8_t>(~0x04);
  } else {
    modulation |= 0x04;
  }
  ok = ok && WriteRegister(kRegTxModulation, std::span{&modulation, 1});
  if (ok && WaitReady()) {
    const uint8_t header[2] = {kWriteBuffer, 0x00};
    const SpiSegment segments[] = {{header, nullptr, 2},
                                   {data.data(), nullptr, static_cast<int>(data.size())}};
    ok = m_spi.TransactionSegments(segments) == 2 + static_cast<int>(data.size());
    m_lastCommand = std::chrono::steady_clock::now();
  } else {
    ok = false;
  }
  ok = ok && ClearIrq(kIrqAll);
  if (ok) {
    SetRfPath(RfPath::kTx);
    ok = Command(std::array<uint8_t, 4>{kSetTx, static_cast<uint8_t>(timeout >> 16), High(timeout),
                                        Low(timeout)});
  }
  if (!ok) {
    SetRfPath(RfPath::kOff);
    if (m_rxWanted) {
      StartRxLocked();  // the standby above stopped it
    }
    return false;
  }
  m_txPending = true;
  return true;
}

bool SX126x::IsTxDone() const {
  std::scoped_lock lock{m_mutex};
  return !m_txPending;
}

bool SX126x::StartReceive() {
  std::scoped_lock lock{m_mutex};
  if (!m_ready) {
    return false;
  }
  m_rxWanted = true;
  if (m_txPending) {
    return true;  // Update() starts the reception when the send ends
  }
  return StartRxLocked();
}

// m_mutex held. 14.3: standby, packet length limit, IRQs cleared, RF switch, continuous RX.
bool SX126x::StartRxLocked() {
  bool ok = SetStandby() && SetPacketParams(kMaxPayload) && ClearIrq(kIrqAll);
  if (ok) {
    SetRfPath(RfPath::kRx);
    ok = Command(std::array<uint8_t, 4>{kSetRx, static_cast<uint8_t>(kRxContinuous >> 16),
                                        High(kRxContinuous), Low(kRxContinuous)});
  }
  if (!ok) {
    SetRfPath(RfPath::kOff);
  }
  return ok;
}

void SX126x::StopReceive() {
  std::scoped_lock lock{m_mutex};
  m_rxWanted = false;
  if (m_ready && !m_txPending) {
    SetStandby();
    SetRfPath(RfPath::kOff);
  }
}

std::optional<SX126x::Packet> SX126x::Poll() {
  std::scoped_lock lock{m_mutex};
  if (m_packets.empty()) {
    return std::nullopt;
  }
  Packet packet = std::move(m_packets.front());
  m_packets.pop_front();
  return packet;
}

// Section 6.1.4: ToA = 2^SF / BW x Nsymbol, with N_bit_CRC 16 and N_symbol_header 20.
units::millisecond_t SX126x::GetTimeOnAir(int payloadBytes) const {
  const int sf = m_config.spreadingFactor;
  const int cr = m_config.codingRate - 4;
  const int payload = std::clamp(payloadBytes, 0, kMaxPayload);
  int bits;
  int perSymbol;
  double fixedSymbols;
  if (sf <= 6) {
    bits = 8 * payload + 16 - 4 * sf + 20;
    perSymbol = 4 * sf;
    fixedSymbols = 6.25 + 8;
  } else {
    bits = 8 * payload + 16 - 4 * sf + 8 + 20;
    perSymbol = 4 * (LowDataRateOptimize(m_config) ? sf - 2 : sf);
    fixedSymbols = 4.25 + 8;
  }
  const int blocks = perSymbol > 0 ? (std::max(bits, 0) + perSymbol - 1) / perSymbol : 0;
  const double symbols = m_config.preambleLength + fixedSymbols + blocks * (cr + 4);
  return units::millisecond_t{SymbolTimeMs(m_config) * symbols};
}

void SX126x::Run() {
  const auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{kPollPeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

void SX126x::Update() {
  if (!m_begun || !m_dio1.Get()) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (!m_ready) {
    return;
  }
  // BUSY high: the chip is still switching modes, or it is gone (an unplugged board reads BUSY
  // and DIO1 high through the input pull-ups). Try again on the next poll instead of holding the
  // lock for the BUSY timeout; after kBusyTimeout the chip counts as not connected.
  if (m_busy.Get()) {
    const auto now = std::chrono::steady_clock::now();
    if (!m_busyHigh) {
      m_busyHigh = true;
      m_busySince = now;
    } else if (now - m_busySince > kBusyTimeout) {
      BusyStuck();
    }
    return;
  }
  m_busyHigh = false;
  uint16_t irq = 0;
  if (!GetIrqStatus(&irq) || irq == 0) {
    return;
  }
  // 14.3 step 12: clear first, then read the packet
  ClearIrq(irq);
  if (m_txPending && (irq & (kIrqTxDone | kIrqTimeout)) != 0) {
    m_txPending = false;
    SetRfPath(RfPath::kOff);
    if ((irq & kIrqTxDone) != 0) {
      ++m_sent;
    } else {
      ++m_txTimeouts;
    }
    if (m_rxWanted) {
      StartRxLocked();
    }
    return;
  }
  if ((irq & kIrqRxDone) != 0) {
    // 14.3 step 11: RxDone comes with CrcErr when the payload CRC is wrong
    if ((irq & (kIrqCrcErr | kIrqHeaderErr)) != 0) {
      ++m_crcErrors;
    } else {
      ReadPacketLocked();
    }
  } else if ((irq & kIrqHeaderErr) != 0) {
    ++m_crcErrors;
  }
}

// m_mutex held. Tables 13-78 (length and start), 13-27 (buffer: one NOP after the offset) and
// 13-79/13-80 (RSSI = -RssiPkt/2 dBm, SNR = SnrPkt/4 dB, two's complement).
void SX126x::ReadPacketLocked() {
  uint8_t bufferStatus[4] = {};
  if (!Read(std::array<uint8_t, 4>{kGetRxBufferStatus, 0, 0, 0}, bufferStatus)) {
    return;
  }
  Packet packet;
  packet.data.resize(bufferStatus[2]);
  if (!packet.data.empty()) {
    if (!WaitReady()) {
      return;
    }
    const uint8_t header[3] = {kReadBuffer, bufferStatus[3], 0};
    const SpiSegment segments[] = {{header, nullptr, 3},
                                   {nullptr, packet.data.data(),
                                    static_cast<int>(packet.data.size())}};
    const bool ok =
        m_spi.TransactionSegments(segments) == 3 + static_cast<int>(packet.data.size());
    m_lastCommand = std::chrono::steady_clock::now();
    if (!ok) {
      return;
    }
  }
  uint8_t packetStatus[5] = {};
  if (Read(std::array<uint8_t, 5>{kGetPacketStatus, 0, 0, 0, 0}, packetStatus)) {
    packet.rssi = -packetStatus[2] / 2.0;
    packet.snr = static_cast<int8_t>(packetStatus[3]) / 4.0;
  }
  m_lastRssi = packet.rssi;
  m_lastSnr = packet.snr;
  ++m_received;
  if (static_cast<int>(m_packets.size()) >= kMaxQueuedPackets) {
    m_packets.pop_front();
    ++m_dropped;
  }
  m_packets.push_back(std::move(packet));
}

bool SX126x::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_ready && m_lastReadyOk;
}

int SX126x::GetSentCount() const {
  std::scoped_lock lock{m_mutex};
  return m_sent;
}

int SX126x::GetTxTimeoutCount() const {
  std::scoped_lock lock{m_mutex};
  return m_txTimeouts;
}

int SX126x::GetReceivedCount() const {
  std::scoped_lock lock{m_mutex};
  return m_received;
}

int SX126x::GetCrcErrorCount() const {
  std::scoped_lock lock{m_mutex};
  return m_crcErrors;
}

int SX126x::GetDroppedCount() const {
  std::scoped_lock lock{m_mutex};
  return m_dropped;
}

void SX126x::LogTo(telemetry::TelemetryTable& table) const {
  double rssi, snr;
  int32_t sent, received, crcErrors;
  bool connected;
  {
    std::scoped_lock lock{m_mutex};
    rssi = m_lastRssi;
    snr = m_lastSnr;
    sent = m_sent;
    received = m_received;
    crcErrors = m_crcErrors;
    connected = m_ready && m_lastReadyOk;
  }
  table.Log("Connected", connected);
  table.Log("RSSI", rssi);
  table.Log("SNR", snr);
  table.Log("Sent", sent);
  table.Log("Received", received);
  table.Log("CRC errors", crcErrors);
}

std::string_view SX126x::GetTelemetryType() const {
  return "LoRa";
}
