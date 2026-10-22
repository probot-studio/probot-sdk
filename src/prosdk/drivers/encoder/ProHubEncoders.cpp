// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ProHub encoder stream reader (ProHubEncoders.hpp). Replaces Probot Lib 0.2.6's UartEncoder:
// a prosdk::SerialPort instead of HardwareSerial, a reader thread below probot-core's loop
// priority instead of a max-priority task, and the latest frame copied under a mutex instead of
// the tearing double buffer.

#include "prosdk/drivers/encoder/ProHubEncoders.hpp"

#include <chrono>

#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;

uint32_t ReadU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int64_t ReadI64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) {
    v |= static_cast<uint64_t>(p[i]) << (8 * i);
  }
  return static_cast<int64_t>(v);
}

void WriteLe(uint8_t* p, uint64_t v, int bytes) {
  for (int i = 0; i < bytes; ++i) {
    p[i] = static_cast<uint8_t>(v >> (8 * i));
  }
}

uint8_t Crc(const uint8_t* p, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; ++i) {
    c ^= p[i];
  }
  return c;
}

}  // namespace

ProHubEncoders::ProHubEncoders(SerialPort::Port port, int baudRate,
                               units::millisecond_t pollPeriod)
    : m_pollPeriod{pollPeriod} {
  if (m_pollPeriod > 0_ms) {
    m_serial.emplace(baudRate, port);
    m_serial->SetTimeout(0_s);
    util::ReportUsage("ProHubEncoders", static_cast<int>(port), "");
    platform::ScopedThreadConfig threadConfig{{.name = "prohub-enc"}};
    m_thread = std::thread([this] { Run(); });
  }
}

ProHubEncoders::~ProHubEncoders() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void ProHubEncoders::Feed(std::span<const uint8_t> bytes) {
  std::scoped_lock parse{m_parseMutex};
  m_bytes += static_cast<uint32_t>(bytes.size());
  for (uint8_t b : bytes) {
    if (m_len == 0) {
      if (b == kHeader0) {
        m_buf[m_len++] = b;
      }
      continue;
    }
    if (m_len == 1) {
      if (b == kHeader1) {
        m_buf[m_len++] = b;
      } else if (b != kHeader0) {
        m_len = 0;  // (a repeated 0xAA keeps waiting for 0x55)
      }
      continue;
    }
    m_buf[m_len++] = b;
    if (m_len < kFrameSize) {
      continue;
    }
    m_len = 0;
    if (Crc(m_buf.data(), kFrameSize - 1) != m_buf[kFrameSize - 1]) {
      ++m_crcErrors;
      continue;
    }
    Frame f;
    const uint8_t* p = &m_buf[2];
    f.seq = *p++;
    f.timestampUs = ReadU32(p);
    p += 4;
    f.dtUs = ReadU32(p);
    p += 4;
    for (auto& t : f.totalTicks) {
      t = ReadI64(p);
      p += 8;
    }
    for (auto& d : f.deltaTicks) {
      d = static_cast<int32_t>(ReadU32(p));
      p += 4;
    }
    double now = Timer::GetMonotonicTimestamp().value();
    {
      std::scoped_lock lock{m_mutex};
      m_frame = f;
      m_frameTime = now;
    }
    ++m_frames;
  }
}

std::array<uint8_t, ProHubEncoders::kFrameSize> ProHubEncoders::Encode(const Frame& frame) {
  std::array<uint8_t, kFrameSize> out{};
  out[0] = kHeader0;
  out[1] = kHeader1;
  uint8_t* p = &out[2];
  *p++ = frame.seq;
  WriteLe(p, frame.timestampUs, 4);
  p += 4;
  WriteLe(p, frame.dtUs, 4);
  p += 4;
  for (int64_t t : frame.totalTicks) {
    WriteLe(p, static_cast<uint64_t>(t), 8);
    p += 8;
  }
  for (int32_t d : frame.deltaTicks) {
    WriteLe(p, static_cast<uint32_t>(d), 4);
    p += 4;
  }
  out[kFrameSize - 1] = Crc(out.data(), kFrameSize - 1);
  return out;
}

std::optional<ProHubEncoders::Frame> ProHubEncoders::GetFrame() const {
  std::scoped_lock lock{m_mutex};
  return m_frame;
}

void ProHubEncoders::CheckIndex(int index) const {
  if (index < 0 || index >= kNumEncoders) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "ProHub encoder {} (valid: 0 to {})",
                           index, kNumEncoders - 1);
  }
}

int64_t ProHubEncoders::GetTicks(int index) const {
  CheckIndex(index);
  std::scoped_lock lock{m_mutex};
  return m_frame ? m_frame->totalTicks[index] : 0;
}

double ProHubEncoders::GetTicksPerSecond(int index) const {
  CheckIndex(index);
  std::scoped_lock lock{m_mutex};
  if (!m_frame || m_frame->dtUs == 0) {
    return 0.0;
  }
  return m_frame->deltaTicks[index] * 1e6 / m_frame->dtUs;
}

bool ProHubEncoders::IsConnected() const {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  return m_frame.has_value() && now - m_frameTime <= units::second_t{kTimeout}.value();
}

uint32_t ProHubEncoders::GetFrameCount() const {
  return m_frames;
}

uint32_t ProHubEncoders::GetCrcErrors() const {
  return m_crcErrors;
}

uint32_t ProHubEncoders::GetBytesReceived() const {
  return m_bytes;
}

void ProHubEncoders::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_pollPeriod}.value())};
  std::array<char, 256> chunk;
  while (!m_stop) {
    int available = m_serial->GetBytesReceived();
    while (available > 0 && !m_stop) {
      int n = m_serial->Read(chunk.data(),
                             available < static_cast<int>(chunk.size())
                                 ? available
                                 : static_cast<int>(chunk.size()));
      if (n <= 0) {
        break;
      }
      Feed({reinterpret_cast<const uint8_t*>(chunk.data()), static_cast<size_t>(n)});
      available -= n;
    }
    std::this_thread::sleep_for(period);
  }
}

ProHubEncoder::ProHubEncoder(ProHubEncoders& encoders, int index)
    : m_encoders{encoders}, m_index{index} {
  m_offset = m_encoders.GetTicks(index);  // validates the index
}

int64_t ProHubEncoder::Get() const {
  int64_t ticks = m_encoders.GetTicks(m_index) - m_offset;
  return m_reverse ? -ticks : ticks;
}

void ProHubEncoder::Reset() {
  m_offset = m_encoders.GetTicks(m_index);
}

double ProHubEncoder::GetDistance() const {
  return static_cast<double>(Get()) * m_distancePerPulse;
}

double ProHubEncoder::GetRate() const {
  double rate = m_encoders.GetTicksPerSecond(m_index) * m_distancePerPulse;
  return m_reverse ? -rate : rate;
}

void ProHubEncoder::SetDistancePerPulse(double distancePerPulse) {
  m_distancePerPulse = distancePerPulse;
}

double ProHubEncoder::GetDistancePerPulse() const {
  return m_distancePerPulse;
}

void ProHubEncoder::SetReverseDirection(bool reverseDirection) {
  m_reverse = reverseDirection;
}

int ProHubEncoder::GetIndex() const {
  return m_index;
}

void ProHubEncoder::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Velocity", GetRate());
  table.Log("Distance", GetDistance());
  table.Log("Distance per Tick", GetDistancePerPulse());
}

std::string_view ProHubEncoder::GetTelemetryType() const {
  return "Encoder";
}
