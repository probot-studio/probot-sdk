// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NMEA 0183 reader and decoders (Nmea.hpp). Field layouts: u-blox M10 SPG 5.10 Interface
// Description UBX-21035062 R03, sections 2.7.5 (GGA), 2.7.13 (GSA) and 2.7.17 (RMC).

#include "prosdk/drivers/gnss/Nmea.hpp"

#include <cmath>

using namespace prosdk;

namespace {

constexpr char kHex[] = "0123456789ABCDEF";
constexpr int kMaxFields = 24;

struct Fields {
  std::array<std::string_view, kMaxFields> f{};
  int count = 0;
  std::string_view operator[](int i) const { return i < count ? f[i] : std::string_view{}; }
};

// Splits a body at the commas (fields past kMaxFields are dropped).
Fields Split(std::string_view body) {
  Fields out;
  size_t start = 0;
  while (out.count < kMaxFields) {
    size_t comma = body.find(',', start);
    out.f[out.count++] = body.substr(start, comma == std::string_view::npos ? comma : comma - start);
    if (comma == std::string_view::npos) {
      break;
    }
    start = comma + 1;
  }
  return out;
}

int HexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool IsDigit(char c) {
  return c >= '0' && c <= '9';
}

// A plain decimal ("-12.345"); NMEA numbers have no exponent. False for an empty or bad field.
bool ParseDecimal(std::string_view s, double* out) {
  size_t i = 0;
  bool negative = false;
  if (i < s.size() && (s[i] == '-' || s[i] == '+')) {
    negative = s[i] == '-';
    ++i;
  }
  int64_t mantissa = 0;
  int fractionDigits = 0;
  bool anyDigit = false;
  bool point = false;
  for (; i < s.size(); ++i) {
    char c = s[i];
    if (c == '.' && !point) {
      point = true;
      continue;
    }
    if (!IsDigit(c) || mantissa > (int64_t{1} << 53)) {
      return false;
    }
    mantissa = mantissa * 10 + (c - '0');
    fractionDigits += point ? 1 : 0;
    anyDigit = true;
  }
  if (!anyDigit) {
    return false;
  }
  double v = static_cast<double>(mantissa) / std::pow(10.0, fractionDigits);
  *out = negative ? -v : v;
  return true;
}

double Number(std::string_view s) {
  double v;
  return ParseDecimal(s, &v) ? v : nmea::kNoValue;
}

// A whole number that fits an int (a garbled field gives the fallback, never an overflow).
int Integer(std::string_view s, int fallback) {
  double v;
  if (!ParseDecimal(s, &v) || v < -1e9 || v > 1e9) {
    return fallback;
  }
  return static_cast<int>(v);
}

// (d)ddmm.mmmm with hemisphere N/S/E/W -> signed degrees (M10 section 2.5.3).
double Coordinate(std::string_view value, std::string_view hemisphere) {
  double v;
  if (!ParseDecimal(value, &v) || v < 0.0 || hemisphere.size() != 1) {
    return nmea::kNoValue;
  }
  double degrees = std::floor(v / 100.0);
  double result = degrees + (v - degrees * 100.0) / 60.0;
  char h = hemisphere[0];
  if (h == 'S' || h == 'W') {
    return -result;
  }
  return h == 'N' || h == 'E' ? result : nmea::kNoValue;
}

// hhmmss(.ss)
bool Time(std::string_view s, int* hour, int* minute, double* second) {
  if (s.size() < 6 || !IsDigit(s[0]) || !IsDigit(s[1]) || !IsDigit(s[2]) || !IsDigit(s[3])) {
    return false;
  }
  double sec;
  if (!ParseDecimal(s.substr(4), &sec)) {
    return false;
  }
  *hour = (s[0] - '0') * 10 + (s[1] - '0');
  *minute = (s[2] - '0') * 10 + (s[3] - '0');
  *second = sec;
  return true;
}

// ddmmyy; the two-digit year is read as 20yy
bool Date(std::string_view s, int* year, int* month, int* day) {
  if (s.size() != 6) {
    return false;
  }
  for (char c : s) {
    if (!IsDigit(c)) {
      return false;
    }
  }
  *day = (s[0] - '0') * 10 + (s[1] - '0');
  *month = (s[2] - '0') * 10 + (s[3] - '0');
  *year = 2000 + (s[4] - '0') * 10 + (s[5] - '0');
  return true;
}

}  // namespace

uint8_t nmea::Checksum(std::string_view body) {
  uint8_t c = 0;
  for (char ch : body) {
    c ^= static_cast<uint8_t>(ch);
  }
  return c;
}

std::string nmea::Encode(std::string_view body) {
  uint8_t c = Checksum(body);
  std::string out;
  out.reserve(body.size() + 6);
  out += '$';
  out += body;
  out += '*';
  out += kHex[c >> 4];
  out += kHex[c & 0x0F];
  out += "\r\n";
  return out;
}

std::string_view nmea::SentenceType(std::string_view body) {
  std::string_view address = body.substr(0, body.find(','));
  if (address.size() != 5 || address[0] == 'P') {
    return {};
  }
  return address.substr(2);
}

std::optional<nmea::Gga> nmea::ParseGga(std::string_view body) {
  if (SentenceType(body) != "GGA") {
    return std::nullopt;
  }
  Fields f = Split(body);
  if (f.count < 10) {
    return std::nullopt;
  }
  Gga g;
  g.hasTime = Time(f[1], &g.hour, &g.minute, &g.second);
  g.latitude = Coordinate(f[2], f[3]);
  g.longitude = Coordinate(f[4], f[5]);
  g.quality = Integer(f[6], 0);
  g.numSatellites = Integer(f[7], -1);
  g.hdop = Number(f[8]);
  g.altitudeMsl = Number(f[9]);
  g.geoidSeparation = Number(f[11]);
  return g;
}

std::optional<nmea::Rmc> nmea::ParseRmc(std::string_view body) {
  if (SentenceType(body) != "RMC") {
    return std::nullopt;
  }
  Fields f = Split(body);
  if (f.count < 10) {
    return std::nullopt;
  }
  Rmc r;
  r.hasTime = Time(f[1], &r.hour, &r.minute, &r.second);
  r.valid = f[2] == "A";
  r.latitude = Coordinate(f[3], f[4]);
  r.longitude = Coordinate(f[5], f[6]);
  r.speedKnots = Number(f[7]);
  r.courseDegrees = Number(f[8]);
  r.hasDate = Date(f[9], &r.year, &r.month, &r.day);
  if (f[12].size() == 1) {
    r.posMode = f[12][0];
  }
  return r;
}

std::optional<nmea::Gsa> nmea::ParseGsa(std::string_view body) {
  if (SentenceType(body) != "GSA") {
    return std::nullopt;
  }
  Fields f = Split(body);
  if (f.count < 18) {
    return std::nullopt;
  }
  Gsa g;
  g.navMode = Integer(f[2], 0);
  g.pdop = Number(f[15]);
  g.hdop = Number(f[16]);
  g.vdop = Number(f[17]);
  return g;
}

bool nmea::Reader::Push(uint8_t byte) {
  if (byte == '$') {
    if (m_inSentence) {
      ++m_errors;  // the previous sentence never ended
    }
    m_inSentence = true;
    m_len = 0;
    return false;
  }
  if (!m_inSentence) {
    return false;
  }
  if (byte == '\r' || byte == '\n') {
    m_inSentence = false;
    // body, then '*' and two hex digits
    if (m_len < 4 || m_buf[m_len - 3] != '*') {
      ++m_errors;
      return false;
    }
    int hi = HexDigit(m_buf[m_len - 2]);
    int lo = HexDigit(m_buf[m_len - 1]);
    size_t bodyLen = m_len - 3;
    std::string_view body{m_buf.data(), bodyLen};
    if (hi < 0 || lo < 0 || Checksum(body) != ((hi << 4) | lo)) {
      ++m_errors;
      return false;
    }
    m_bodyLen = bodyLen;
    ++m_sentences;
    return true;
  }
  if (byte < 0x20 || byte > 0x7E || m_len >= m_buf.size()) {
    ++m_errors;  // binary data or an overlong line: drop the sentence
    m_inSentence = false;
    return false;
  }
  m_buf[m_len++] = static_cast<char>(byte);
  return false;
}

std::string_view nmea::Reader::Body() const {
  return {m_buf.data(), m_bodyLen};
}
