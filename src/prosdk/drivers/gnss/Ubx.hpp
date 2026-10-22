// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// u-blox UBX protocol: frame encoder and parser, the NAV-PVT decoder, MON-VER protocol version,
// and the configuration messages prosdk::UbloxGnss sends (no HAL; design/boardoza/P7-gnss.md).
// Sources: u-blox M10 SPG 5.10 Interface Description UBX-21035062 R03 (M10) and u-blox 8 / M8
// Receiver Description UBX-13003221 R28 (M8):
//
//   B5 62 | class u8 | id u8 | length u16 LE | payload | CK_A CK_B
//
// The checksum is the 8-bit Fletcher sum over class, id, length and payload (M10 sections 3.2
// and 3.4). All multi-byte fields are little endian.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <optional>
#include <span>
#include <vector>

namespace prosdk::ubx {

inline constexpr uint8_t kSync1 = 0xB5;
inline constexpr uint8_t kSync2 = 0x62;

// Classes and ids (M10 section 3.8; M8 section 32)
inline constexpr uint8_t kClassNav = 0x01;
inline constexpr uint8_t kClassAck = 0x05;
inline constexpr uint8_t kClassCfg = 0x06;
inline constexpr uint8_t kClassMon = 0x0A;
inline constexpr uint8_t kIdNavPvt = 0x07;    // NAV-PVT, 92-byte payload
inline constexpr uint8_t kIdAckNak = 0x00;    // payload: class and id of the message
inline constexpr uint8_t kIdAckAck = 0x01;
inline constexpr uint8_t kIdCfgPrt = 0x00;    // M8 only (not in the M10 message list)
inline constexpr uint8_t kIdCfgMsg = 0x01;    // M8 only
inline constexpr uint8_t kIdCfgRate = 0x08;   // M8 only
inline constexpr uint8_t kIdCfgValset = 0x8A; // configuration interface (M10)
inline constexpr uint8_t kIdMonVer = 0x04;

/// NAV-PVT payload length (M10 section 3.15.11, M8 section 32.17.17).
inline constexpr size_t kNavPvtLength = 92;

/// UBX-CFG-PRT port ids (M8 section 11, "Port Number assignment").
inline constexpr uint8_t kPortDdc = 0;
inline constexpr uint8_t kPortUart1 = 1;

/// UBX-CFG-VALSET layers bit field (M10 section 3.10.5).
inline constexpr uint8_t kLayerRam = 0x01;
inline constexpr uint8_t kLayerBbr = 0x02;
inline constexpr uint8_t kLayerFlash = 0x04;

/// Configuration keys (M10 section 4.9: CFG-RATE, CFG-I2COUTPROT, CFG-UART1OUTPROT, CFG-MSGOUT).
namespace key {
inline constexpr uint32_t kRateMeas = 0x30210001;           // U2, ms between measurements
inline constexpr uint32_t kI2cOutprotNmea = 0x10720002;     // L, NMEA output on I2C
inline constexpr uint32_t kUart1OutprotNmea = 0x10740002;   // L, NMEA output on UART1
inline constexpr uint32_t kMsgoutNavPvtI2c = 0x20910006;    // U1, NAV-PVT rate on I2C
inline constexpr uint32_t kMsgoutNavPvtUart1 = 0x20910007;  // U1, NAV-PVT rate on UART1
}  // namespace key

/** Fletcher-8 checksum (CK_A, CK_B) over the bytes from class to the end of the payload. */
std::array<uint8_t, 2> Checksum(std::span<const uint8_t> classToPayload);

/** Builds a complete frame (sync, header, payload, checksum). */
std::vector<uint8_t> Encode(uint8_t messageClass, uint8_t messageId,
                            std::span<const uint8_t> payload = {});

/**
 * Value size in bytes of a configuration key: bits 30..28 of the key (M10 section 4.2;
 * 1 = one bit stored in one byte, 2 = 1, 3 = 2, 4 = 4, 5 = 8 bytes). 0 for an invalid key.
 */
size_t KeyValueSize(uint32_t key);

/** One key-value pair of UBX-CFG-VALSET. */
struct ConfigItem {
  uint32_t key;
  uint64_t value;  ///< the low KeyValueSize(key) bytes are sent
};

/**
 * UBX-CFG-VALSET version 0 (no transaction) frame: version 0, layers, 2 reserved bytes, then the
 * keys (U4) and values concatenated without padding (M10 sections 3.10.5 and 4.5).
 */
std::vector<uint8_t> EncodeValset(uint8_t layers, std::span<const ConfigItem> items);

/**
 * UBX-CFG-PRT frame for the DDC (I2C) port (M8 section 32.10.25.5): mode = 7-bit address in bits
 * 7..1; input UBX, NMEA and RTCM2 (the defaults, M8 appendix C.15.4); output UBX only.
 */
std::vector<uint8_t> EncodeCfgPrtDdc(int address);

/**
 * UBX-CFG-PRT frame for UART1 (M8 section 32.10.25.2): 8N1 (mode charLen 0b11 in bits 7..6,
 * parity 0b100 in bits 11..9, one stop bit), the given baud rate, input UBX, NMEA and RTCM2,
 * output UBX only.
 */
std::vector<uint8_t> EncodeCfgPrtUart1(uint32_t baudRate);

/** UBX-CFG-MSG frame setting a message's rate on the port it is received on (M8 32.10.18.3). */
std::vector<uint8_t> EncodeCfgMsg(uint8_t messageClass, uint8_t messageId, uint8_t rate);

/**
 * UBX-CFG-RATE frame (M8 section 32.10.27): measurement period, one navigation solution per
 * measurement, aligned to GPS time (timeRef 1, the default of M8 appendix C.16).
 */
std::vector<uint8_t> EncodeCfgRate(uint16_t measurementPeriodMs);

/** The NAV-PVT fields prosdk uses, in the message's units (M10 section 3.15.11). */
struct NavPvt {
  uint32_t iTow = 0;      ///< ms, GPS time of week
  uint16_t year = 0;      ///< UTC
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint8_t valid = 0;      ///< bit 0 validDate, bit 1 validTime
  int32_t nano = 0;       ///< ns, -1e9 to 1e9
  uint8_t fixType = 0;    ///< 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR, 5 time only
  uint8_t flags = 0;      ///< bit 0 gnssFixOK
  uint8_t numSv = 0;
  int32_t lon = 0;        ///< 1e-7 deg
  int32_t lat = 0;        ///< 1e-7 deg
  int32_t height = 0;     ///< mm above the ellipsoid
  int32_t hMsl = 0;       ///< mm above mean sea level
  uint32_t hAcc = 0;      ///< mm
  uint32_t vAcc = 0;      ///< mm
  int32_t velN = 0;       ///< mm/s
  int32_t velE = 0;
  int32_t velD = 0;
  int32_t gSpeed = 0;     ///< mm/s, 2-D ground speed
  int32_t headMot = 0;    ///< 1e-5 deg, heading of motion
  uint16_t pDop = 0;      ///< 0.01
};

/** Decodes a NAV-PVT payload; nullopt when it is shorter than 92 bytes. */
std::optional<NavPvt> DecodeNavPvt(std::span<const uint8_t> payload);

/** Encodes a NAV-PVT payload (unused and reserved fields 0; for tests and simulators). */
std::array<uint8_t, kNavPvtLength> EncodeNavPvt(const NavPvt& pvt);

/**
 * The protocol version from a MON-VER payload: the "PROTVER=xx.yy" (or "PROTVER xx.yy") extension
 * string (M10 section 1.2, M8 section 2). nullopt when absent.
 */
std::optional<double> ParseProtocolVersion(std::span<const uint8_t> monVerPayload);

/**
 * Frame parser. Push bytes one at a time; garbage between frames (NMEA text, 0xFF fill from the
 * I2C stream) is skipped. A header announcing more than kMaxPayload bytes is dropped at once and
 * the parser hunts for the next sync, so a corrupted length cannot swallow the stream.
 */
class Parser {
 public:
  /// Largest payload kept (MON-VER is 40 + 30 n bytes).
  static constexpr size_t kMaxPayload = 512;

  /** A complete frame; payload points into the parser and is valid until the next Push. */
  struct Message {
    uint8_t messageClass;
    uint8_t messageId;
    std::span<const uint8_t> payload;
  };

  /** Adds one byte. Returns true when it completed a frame with a valid checksum. */
  bool Push(uint8_t byte);

  /** The last complete frame (after Push returned true). */
  Message GetMessage() const;

  /** Frames that passed the checksum. */
  uint32_t GetFrameCount() const { return m_frames; }
  /** Frames dropped: checksum mismatch or payload too long. */
  uint32_t GetErrorCount() const { return m_errors; }

 private:
  enum class State { kSync1, kSync2, kHeader, kPayload, kCkA, kCkB };
  State m_state = State::kSync1;
  std::array<uint8_t, 4> m_header{};  // class, id, length LE
  size_t m_headerLen = 0;
  size_t m_length = 0;
  size_t m_payloadLen = 0;
  std::array<uint8_t, kMaxPayload> m_payload{};
  uint8_t m_ckA = 0;
  uint8_t m_ckB = 0;
  uint8_t m_rxCkA = 0;
  uint32_t m_frames = 0;
  uint32_t m_errors = 0;
};

}  // namespace prosdk::ubx
