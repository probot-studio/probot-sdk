// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// UBX protocol helpers (Ubx.hpp). Message layouts: u-blox M10 SPG 5.10 Interface Description
// UBX-21035062 R03 (frame 3.2, checksum 3.4, CFG-VALSET 3.10.5, MON-VER 3.14.9, NAV-PVT 3.15.11,
// configuration items 4.2/4.5) and u-blox 8 / M8 Receiver Description UBX-13003221 R28 (CFG-MSG
// 32.10.18, CFG-PRT 32.10.25, CFG-RATE 32.10.27; bit fields from the figures of those sections).

#include "prosdk/drivers/gnss/Ubx.hpp"

#include <algorithm>
#include <string_view>

using namespace prosdk;

namespace {

void PutLe(uint8_t* p, uint64_t v, size_t bytes) {
  for (size_t i = 0; i < bytes; ++i) {
    p[i] = static_cast<uint8_t>(v >> (8 * i));
  }
}

uint32_t GetLe(const uint8_t* p, size_t bytes) {
  uint32_t v = 0;
  for (size_t i = 0; i < bytes; ++i) {
    v |= static_cast<uint32_t>(p[i]) << (8 * i);
  }
  return v;
}

int32_t GetI4(const uint8_t* p) {
  return static_cast<int32_t>(GetLe(p, 4));
}

// CFG-PRT protocol masks (M8 section 32.10.25, figures "Bitfield inProtoMask/outProtoMask")
constexpr uint16_t kProtoUbx = 0x0001;
constexpr uint16_t kProtoNmea = 0x0002;
constexpr uint16_t kProtoRtcm = 0x0004;  // input only
constexpr uint16_t kInProtoDefault = kProtoUbx | kProtoNmea | kProtoRtcm;  // M8 appendix C.15
constexpr uint16_t kOutProtoUbxOnly = kProtoUbx;

// CFG-PRT UART mode for 8N1: charLen 0b11 (bits 7..6), parity 0b10x = none (bits 11..9),
// nStopBits 0b00 = 1 (bits 13..12) (M8 section 32.10.25.2, figure "Bitfield mode")
constexpr uint32_t kUartMode8N1 = (0b11u << 6) | (0b100u << 9);

std::vector<uint8_t> CfgPrt(uint8_t portId, uint32_t mode, uint32_t baudRate) {
  std::array<uint8_t, 20> p{};
  p[0] = portId;
  PutLe(&p[4], mode, 4);
  PutLe(&p[8], baudRate, 4);  // reserved (0) on the DDC port
  PutLe(&p[12], kInProtoDefault, 2);
  PutLe(&p[14], kOutProtoUbxOnly, 2);
  return ubx::Encode(ubx::kClassCfg, ubx::kIdCfgPrt, p);
}

}  // namespace

std::array<uint8_t, 2> ubx::Checksum(std::span<const uint8_t> classToPayload) {
  uint8_t a = 0;
  uint8_t b = 0;
  for (uint8_t byte : classToPayload) {
    a = static_cast<uint8_t>(a + byte);
    b = static_cast<uint8_t>(b + a);
  }
  return {a, b};
}

std::vector<uint8_t> ubx::Encode(uint8_t messageClass, uint8_t messageId,
                                 std::span<const uint8_t> payload) {
  std::vector<uint8_t> out(8 + payload.size());
  out[0] = kSync1;
  out[1] = kSync2;
  out[2] = messageClass;
  out[3] = messageId;
  PutLe(&out[4], payload.size(), 2);
  std::copy(payload.begin(), payload.end(), out.begin() + 6);
  auto ck = Checksum({out.data() + 2, payload.size() + 4});
  out[6 + payload.size()] = ck[0];
  out[7 + payload.size()] = ck[1];
  return out;
}

size_t ubx::KeyValueSize(uint32_t key) {
  switch ((key >> 28) & 0x07) {
    case 1:  // one bit, stored in one byte
    case 2:
      return 1;
    case 3:
      return 2;
    case 4:
      return 4;
    case 5:
      return 8;
    default:
      return 0;
  }
}

std::vector<uint8_t> ubx::EncodeValset(uint8_t layers, std::span<const ConfigItem> items) {
  std::vector<uint8_t> payload{0x00, layers, 0x00, 0x00};  // version 0, layers, reserved0[2]
  for (const ConfigItem& item : items) {
    size_t size = KeyValueSize(item.key);
    size_t at = payload.size();
    payload.resize(at + 4 + size);
    PutLe(&payload[at], item.key, 4);
    PutLe(&payload[at + 4], item.value, size);
  }
  return Encode(kClassCfg, kIdCfgValset, payload);
}

std::vector<uint8_t> ubx::EncodeCfgPrtDdc(int address) {
  // mode: slaveAddr in bits 7..1 (M8 section 32.10.25.5, figure "Bitfield mode")
  return CfgPrt(kPortDdc, static_cast<uint32_t>(address & 0x7F) << 1, 0);
}

std::vector<uint8_t> ubx::EncodeCfgPrtUart1(uint32_t baudRate) {
  return CfgPrt(kPortUart1, kUartMode8N1, baudRate);
}

std::vector<uint8_t> ubx::EncodeCfgMsg(uint8_t messageClass, uint8_t messageId, uint8_t rate) {
  const uint8_t payload[] = {messageClass, messageId, rate};
  return Encode(kClassCfg, kIdCfgMsg, payload);
}

std::vector<uint8_t> ubx::EncodeCfgRate(uint16_t measurementPeriodMs) {
  std::array<uint8_t, 6> p{};
  PutLe(&p[0], measurementPeriodMs, 2);  // measRate, ms
  PutLe(&p[2], 1, 2);                    // navRate, measurements per solution
  PutLe(&p[4], 1, 2);                    // timeRef 1 = GPS time
  return Encode(kClassCfg, kIdCfgRate, p);
}

std::optional<ubx::NavPvt> ubx::DecodeNavPvt(std::span<const uint8_t> payload) {
  if (payload.size() < kNavPvtLength) {
    return std::nullopt;
  }
  const uint8_t* p = payload.data();
  NavPvt v;
  v.iTow = GetLe(&p[0], 4);
  v.year = static_cast<uint16_t>(GetLe(&p[4], 2));
  v.month = p[6];
  v.day = p[7];
  v.hour = p[8];
  v.minute = p[9];
  v.second = p[10];
  v.valid = p[11];
  v.nano = GetI4(&p[16]);
  v.fixType = p[20];
  v.flags = p[21];
  v.numSv = p[23];
  v.lon = GetI4(&p[24]);
  v.lat = GetI4(&p[28]);
  v.height = GetI4(&p[32]);
  v.hMsl = GetI4(&p[36]);
  v.hAcc = GetLe(&p[40], 4);
  v.vAcc = GetLe(&p[44], 4);
  v.velN = GetI4(&p[48]);
  v.velE = GetI4(&p[52]);
  v.velD = GetI4(&p[56]);
  v.gSpeed = GetI4(&p[60]);
  v.headMot = GetI4(&p[64]);
  v.pDop = static_cast<uint16_t>(GetLe(&p[76], 2));
  return v;
}

std::array<uint8_t, ubx::kNavPvtLength> ubx::EncodeNavPvt(const NavPvt& v) {
  std::array<uint8_t, kNavPvtLength> p{};
  PutLe(&p[0], v.iTow, 4);
  PutLe(&p[4], v.year, 2);
  p[6] = v.month;
  p[7] = v.day;
  p[8] = v.hour;
  p[9] = v.minute;
  p[10] = v.second;
  p[11] = v.valid;
  PutLe(&p[16], static_cast<uint32_t>(v.nano), 4);
  p[20] = v.fixType;
  p[21] = v.flags;
  p[23] = v.numSv;
  PutLe(&p[24], static_cast<uint32_t>(v.lon), 4);
  PutLe(&p[28], static_cast<uint32_t>(v.lat), 4);
  PutLe(&p[32], static_cast<uint32_t>(v.height), 4);
  PutLe(&p[36], static_cast<uint32_t>(v.hMsl), 4);
  PutLe(&p[40], v.hAcc, 4);
  PutLe(&p[44], v.vAcc, 4);
  PutLe(&p[48], static_cast<uint32_t>(v.velN), 4);
  PutLe(&p[52], static_cast<uint32_t>(v.velE), 4);
  PutLe(&p[56], static_cast<uint32_t>(v.velD), 4);
  PutLe(&p[60], static_cast<uint32_t>(v.gSpeed), 4);
  PutLe(&p[64], static_cast<uint32_t>(v.headMot), 4);
  PutLe(&p[76], v.pDop, 2);
  return p;
}

std::optional<double> ubx::ParseProtocolVersion(std::span<const uint8_t> payload) {
  // swVersion CH[30], hwVersion CH[10], then extension CH[30] strings (M10 section 3.14.9)
  constexpr size_t kFirstExtension = 40;
  constexpr size_t kExtensionLength = 30;
  constexpr std::string_view kTag = "PROTVER";
  for (size_t at = kFirstExtension; at + kExtensionLength <= payload.size();
       at += kExtensionLength) {
    std::string_view s{reinterpret_cast<const char*>(&payload[at]), kExtensionLength};
    s = s.substr(0, s.find('\0'));
    if (s.substr(0, kTag.size()) != kTag || s.size() <= kTag.size() + 1) {
      continue;
    }
    // "PROTVER=34.10" (protocol 18 and later) or "PROTVER 15.00"
    double value = 0.0;
    double scale = 0.0;
    bool any = false;
    for (char c : s.substr(kTag.size() + 1)) {
      if (c == '.' && scale == 0.0) {
        scale = 0.1;
      } else if (c >= '0' && c <= '9') {
        any = true;
        if (scale == 0.0) {
          value = value * 10.0 + (c - '0');
        } else {
          value += (c - '0') * scale;
          scale /= 10.0;
        }
      } else {
        break;
      }
    }
    if (any) {
      return value;
    }
  }
  return std::nullopt;
}

bool ubx::Parser::Push(uint8_t byte) {
  switch (m_state) {
    case State::kSync1:
      if (byte == kSync1) {
        m_state = State::kSync2;
      }
      return false;
    case State::kSync2:
      if (byte == kSync2) {
        m_state = State::kHeader;
        m_headerLen = 0;
        m_ckA = 0;
        m_ckB = 0;
      } else if (byte != kSync1) {
        m_state = State::kSync1;
      }
      return false;
    case State::kHeader:
      m_header[m_headerLen++] = byte;
      m_ckA = static_cast<uint8_t>(m_ckA + byte);
      m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
      if (m_headerLen == m_header.size()) {
        m_length = m_header[2] | (static_cast<size_t>(m_header[3]) << 8);
        m_payloadLen = 0;
        if (m_length > kMaxPayload) {
          ++m_errors;
          m_state = State::kSync1;
        } else {
          m_state = m_length == 0 ? State::kCkA : State::kPayload;
        }
      }
      return false;
    case State::kPayload:
      m_payload[m_payloadLen++] = byte;
      m_ckA = static_cast<uint8_t>(m_ckA + byte);
      m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
      if (m_payloadLen == m_length) {
        m_state = State::kCkA;
      }
      return false;
    case State::kCkA:
      m_rxCkA = byte;
      m_state = State::kCkB;
      return false;
    case State::kCkB:
      m_state = State::kSync1;
      if (m_rxCkA != m_ckA || byte != m_ckB) {
        ++m_errors;
        return false;
      }
      ++m_frames;
      return true;
  }
  return false;
}

ubx::Parser::Message ubx::Parser::GetMessage() const {
  return {m_header[0], m_header[1], {m_payload.data(), m_payloadLen}};
}
