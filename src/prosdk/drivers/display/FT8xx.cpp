// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// FT800 / FT810 EVE driver (FT8xx.hpp). Sources (clean-room, no vendor code):
// - FT800 Embedded Video Engine Datasheet v1.4 (BRT_000039): 4.1.4/4.1.5 (memory read: 00b +
//   22-bit address + dummy byte; write: 10b + address), 4.1.6 (host command: 01b + code, 00h,
//   00h; table 4-4), 4.1.7 (interrupt flags), 4.2.3 (SPI at most 11 MHz until the clock runs),
//   4.8.2 (PD_N low at least 5 ms), 5 (memory map, ROM_CHIPID, register table), feature list
//   (at most 512x512).
// - FT81X Embedded Video Engine Datasheet v1.2 (FT_001165): 4.1.1 (SPI mode 0, 30 MHz), table
//   4-5 (host commands), 4.8.3.5 (20 ms after waking), 4.8.4 (boot up to 300 ms), 5.1 (memory
//   map, register table), 5.2 (chip ID 08h 10h-13h at 0C0000h).
// - FT800 Series Programmer Guide v2.1 (BRT_000030) 2.2.5 and FT81X Series Programmers Guide v1.2
//   (BRT_000031) 2.3: start-up order (CLKEXT, ACTIVE, timing registers except REG_PCLK, first
//   display list, REG_DLSWAP = 2, DISP on GPIO bit 7, REG_PCLK). Chapter 4 (display-list
//   encodings), 5.1 (command FIFO: 4 KiB ring, free space = 4092 - fullness), 5.6 (fault
//   recovery), 5.8 (options), 5.11-5.52 (co-processor command layouts), registers
//   REG_TOUCH_SCREEN_XY (8000h per axis when not touched).

#include "prosdk/drivers/display/FT8xx.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <format>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

// Register and memory addresses of one chip (FT800 DS section 5, FT81X DS section 5.1).
struct FT8xx::Map {
  Chip chip;
  int maxWidth, maxHeight;
  uint32_t ramDl, ramCmd;
  uint32_t regId, regCpuReset;
  uint32_t regHcycle, regHoffset, regHsize, regHsync0, regHsync1;
  uint32_t regVcycle, regVoffset, regVsize, regVsync0, regVsync1;
  uint32_t regDlswap, regDither, regSwizzle, regCspread, regPclkPol, regPclk;
  uint32_t regGpioDir, regGpio;
  uint32_t regIntFlags, regIntEn, regIntMask;
  uint32_t regPwmDuty;
  uint32_t regCmdRead, regCmdWrite;
  uint32_t regTouchScreenXy, regTouchTag, regTouchTransformA;
};

namespace {

// Host commands (FT800 DS table 4-4, FT81X DS table 4-5)
constexpr uint8_t kHostActive = 0x00;
constexpr uint8_t kHostClkExt = 0x44;

constexpr uint32_t kRomChipId = 0x0C0000;  // both chips
constexpr uint8_t kChipIdFamily = 0x08;     // byte at 0C0000h
constexpr uint8_t kRegIdValue = 0x7C;
constexpr uint32_t kCmdFifoSize = 4096;
constexpr uint32_t kCmdFifoMaxFill = kCmdFifoSize - 4;  // FT81X PG 5.1.1
constexpr uint32_t kCmdReadFault = 0xFFF;               // FT81X PG 5.6
constexpr uint8_t kDlswapFrame = 2;
constexpr uint8_t kGpioDisp = 0x80;  // GPIO bit 7: the panel's DISP line (FT800 PG 2.2)
constexpr uint8_t kIntTouch = 0x02;  // FT800 DS table 4-5
constexpr uint8_t kIntTag = 0x04;
constexpr uint32_t kNotTouched = 0x8000;
constexpr int kMaxDuty = 128;  // REG_PWM_DUTY: 0 to 128

constexpr int kRomFontCalibrate = 28;  // a ROM font of both chips (16-31)

constexpr auto kServicePeriod = std::chrono::milliseconds{10};
constexpr auto kRetryPeriod = std::chrono::milliseconds{1000};
constexpr auto kPdLowTime = std::chrono::milliseconds{20};   // >= 5 ms (FT800 DS 4.8.2)
constexpr auto kPdHighTime = std::chrono::milliseconds{20};  // >= 20 ms (FT81X PG 2.3)
constexpr auto kBootPoll = std::chrono::milliseconds{10};
constexpr int kBootPolls = 40;  // 400 ms: boot takes up to 300 ms (FT81X DS 4.8.4)
constexpr auto kFifoTimeout = std::chrono::milliseconds{1000};

// Display-list encodings (FT81X PG chapter 4, FT800 PG chapter 4). The guides' bit diagrams of
// CLEAR_COLOR_RGB and COLOR_RGB label bits 15-8 "Blue" and 7-0 "Green"; red-green-blue (as the
// parameter order, and as CMD_FGCOLOR's 0xRRGGBB in PG 5.30) is used here. Unverified on
// hardware.
constexpr uint32_t DlRgb(uint32_t op, uint8_t r, uint8_t g, uint8_t b) {
  return (op << 24) | (uint32_t{r} << 16) | (uint32_t{g} << 8) | b;
}
constexpr uint32_t DlClearColorRgb(uint8_t r, uint8_t g, uint8_t b) {
  return DlRgb(0x02, r, g, b);
}
constexpr uint32_t DlColorRgb(uint8_t r, uint8_t g, uint8_t b) {
  return DlRgb(0x04, r, g, b);
}
constexpr uint32_t DlClear(bool c, bool s, bool t) {
  return (0x26u << 24) | (c ? 4u : 0u) | (s ? 2u : 0u) | (t ? 1u : 0u);
}
constexpr uint32_t DlTag(uint8_t tag) {
  return (0x03u << 24) | tag;
}
constexpr uint32_t DlPointSize(uint32_t sixteenths) {
  return (0x0Du << 24) | (sixteenths & 0x1FFF);
}
constexpr uint32_t DlLineWidth(uint32_t sixteenths) {
  return (0x0Eu << 24) | (sixteenths & 0xFFF);
}
constexpr uint32_t DlBegin(int primitive) {
  return (0x1Fu << 24) | (static_cast<uint32_t>(primitive) & 0xF);
}
constexpr uint32_t DlEnd() {
  return 0x21u << 24;
}
constexpr uint32_t DlDisplay() {
  return 0;
}
// x, y in 1/16 pixel (the FT800's only format and the FT81x default, VERTEX_FORMAT 4)
constexpr uint32_t DlVertex2f(int32_t x, int32_t y) {
  return (1u << 30) | ((static_cast<uint32_t>(x) & 0x7FFF) << 15) |
         (static_cast<uint32_t>(y) & 0x7FFF);
}

// Co-processor commands (FT81X PG 5.11-5.52; the same codes in FT800 PG chapter 5)
constexpr uint32_t kCmdDlstart = 0xFFFFFF00;
constexpr uint32_t kCmdSwap = 0xFFFFFF01;
constexpr uint32_t kCmdBgcolor = 0xFFFFFF09;
constexpr uint32_t kCmdFgcolor = 0xFFFFFF0A;
constexpr uint32_t kCmdText = 0xFFFFFF0C;
constexpr uint32_t kCmdButton = 0xFFFFFF0D;
constexpr uint32_t kCmdGauge = 0xFFFFFF13;
constexpr uint32_t kCmdCalibrate = 0xFFFFFF15;
constexpr uint32_t kCmdNumber = 0xFFFFFF2E;

int32_t Sixteenths(double pixels, int32_t min, int32_t max) {
  if (!std::isfinite(pixels)) {
    return 0;
  }
  return static_cast<int32_t>(std::clamp(std::lround(pixels * 16.0), long{min}, long{max}));
}

void PutWord(std::vector<uint8_t>* out, uint32_t word) {
  for (int i = 0; i < 4; ++i) {
    out->push_back(static_cast<uint8_t>(word >> (8 * i)));
  }
}

}  // namespace

// FT800 DS section 5 (RAM_REG at 102400h, RAM_DL 100000h, RAM_CMD 108000h)
const FT8xx::Map FT8xx::kMapFT800{
    .chip = Chip::kFT800, .maxWidth = 512, .maxHeight = 512,
    .ramDl = 0x100000, .ramCmd = 0x108000,
    .regId = 0x102400, .regCpuReset = 0x10241C,
    .regHcycle = 0x102428, .regHoffset = 0x10242C, .regHsize = 0x102430,
    .regHsync0 = 0x102434, .regHsync1 = 0x102438,
    .regVcycle = 0x10243C, .regVoffset = 0x102440, .regVsize = 0x102444,
    .regVsync0 = 0x102448, .regVsync1 = 0x10244C,
    .regDlswap = 0x102450, .regDither = 0x10245C, .regSwizzle = 0x102460,
    .regCspread = 0x102464, .regPclkPol = 0x102468, .regPclk = 0x10246C,
    .regGpioDir = 0x10248C, .regGpio = 0x102490,
    .regIntFlags = 0x102498, .regIntEn = 0x10249C, .regIntMask = 0x1024A0,
    .regPwmDuty = 0x1024C4,
    .regCmdRead = 0x1024E4, .regCmdWrite = 0x1024E8,
    .regTouchScreenXy = 0x102510, .regTouchTag = 0x102518, .regTouchTransformA = 0x10251C,
};

// FT81X DS section 5.1 (RAM_REG at 302000h, RAM_DL 300000h, RAM_CMD 308000h)
const FT8xx::Map FT8xx::kMapFT81x{
    .chip = Chip::kFT81x, .maxWidth = 800, .maxHeight = 600,
    .ramDl = 0x300000, .ramCmd = 0x308000,
    .regId = 0x302000, .regCpuReset = 0x302020,
    .regHcycle = 0x30202C, .regHoffset = 0x302030, .regHsize = 0x302034,
    .regHsync0 = 0x302038, .regHsync1 = 0x30203C,
    .regVcycle = 0x302040, .regVoffset = 0x302044, .regVsize = 0x302048,
    .regVsync0 = 0x30204C, .regVsync1 = 0x302050,
    .regDlswap = 0x302054, .regDither = 0x302060, .regSwizzle = 0x302064,
    .regCspread = 0x302068, .regPclkPol = 0x30206C, .regPclk = 0x302070,
    .regGpioDir = 0x302090, .regGpio = 0x302094,
    .regIntFlags = 0x3020A8, .regIntEn = 0x3020AC, .regIntMask = 0x3020B0,
    .regPwmDuty = 0x3020D4,
    .regCmdRead = 0x3020F8, .regCmdWrite = 0x3020FC,
    .regTouchScreenXy = 0x302124, .regTouchTag = 0x30212C, .regTouchTransformA = 0x302150,
};

FT8xx::FT8xx(SPI::Port bus, int csGpio, int pdGpio, const FT8xxPanel& panel, int intGpio,
             int clockHz)
    : m_spi{bus, csGpio},
      m_clockHz{std::clamp(clockHz, SPI::kMinClockRate, kMaxClockRate)},
      m_panel{panel},
      m_duty{kMaxDuty} {
  m_spi.SetMode(SPI::kMode0);  // the only mode the chips support
  m_spi.SetClockRate(kStartupClockRate);
  if (pdGpio >= 0) {
    m_pd.emplace(pdGpio);
    m_pd->Set(true);
  }
  if (intGpio >= 0) {
    m_int.emplace(intGpio);
  }
  util::ReportUsage("FT8xx", csGpio, "");
  platform::ScopedThreadConfig threadConfig{{.name = "ft8xx"}};
  m_thread = std::thread([this] { Run(); });
}

FT8xx::~FT8xx() {
  {
    std::scoped_lock lock{m_waitMutex};
    m_stop = true;
  }
  m_wake.notify_all();
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

bool FT8xx::IsReady() const {
  std::scoped_lock lock{m_mutex};
  return m_ready;
}

FT8xx::Chip FT8xx::GetChip() const {
  std::scoped_lock lock{m_mutex};
  return m_chip;
}

const FT8xxPanel& FT8xx::GetPanel() const {
  return m_panel;
}

void FT8xx::SetBacklight(double brightness) {
  if (!std::isfinite(brightness)) {
    brightness = 0.0;
  }
  m_duty = static_cast<int>(std::lround(std::clamp(brightness, 0.0, 1.0) * kMaxDuty));
}

// ---- thread ----

// Returns false when the driver stops.
bool FT8xx::Wait(std::chrono::milliseconds duration, bool wakeOnSwap) {
  std::unique_lock lock{m_waitMutex};
  m_wake.wait_for(lock, duration, [&] { return m_stop.load() || (wakeOnSwap && m_swapped); });
  if (wakeOnSwap) {
    m_swapped = false;
  }
  return !m_stop;
}

// Prints a problem once (until it changes or the chip came up). Returns false.
bool FT8xx::Fail(const std::string& problem) {
  if (problem != m_problem && !m_stop) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning, "[prosdk] FT8xx on SPI bus {} CS {}: {}\n",
                           static_cast<int>(m_spi.GetPort()), m_spi.GetChipSelect(), problem);
    m_problem = problem;
  }
  return false;
}

void FT8xx::Run() {
  while (!m_stop) {
    bool ready;
    {
      std::scoped_lock lock{m_mutex};
      ready = m_ready;
    }
    if (!ready && !BringUp()) {
      Wait(kRetryPeriod);
      continue;
    }
    Service();
    Wait(kServicePeriod, true);
  }
}

bool FT8xx::BringUp() {
  m_map = nullptr;
  try {
    m_spi.SetClockRate(kStartupClockRate);
  } catch (const std::exception& e) {
    return Fail(e.what());
  }
  // a PD pulse resets the chip (FT800 DS 4.8.2: low at least 5 ms; FT81X PG 2.3: 20 ms after
  // PD high)
  if (m_pd) {
    m_pd->Set(false);
    if (!Wait(kPdLowTime)) {
      return false;
    }
    m_pd->Set(true);
    if (!Wait(kPdHighTime)) {
      return false;
    }
  }
  // the boards have a 12 MHz crystal: CLKEXT, then ACTIVE (both programmer guides)
  if (!HostCommand(kHostClkExt) || !HostCommand(kHostActive)) {
    return Fail("SPI transfer failed");
  }
  // the chip boots in up to 300 ms; the chip ID tells the memory map, REG_ID 7Ch that it runs
  const Map* map = nullptr;
  uint8_t id[4] = {};
  for (int i = 0; i < kBootPolls && map == nullptr; ++i) {
    if (Read(kRomChipId, id, sizeof(id)) && id[0] == kChipIdFamily) {
      // FT81X DS 5.2: 10h-13h for FT810-FT813. FT800 DS 5 gives the ID as "0800" without a byte
      // order; 08h at 0C0000h, 00h at 0C0001h as on the FT81x is assumed (unverified).
      const Map* candidate = id[1] == 0x00                     ? &kMapFT800
                             : (id[1] >= 0x10 && id[1] <= 0x13) ? &kMapFT81x
                                                                : nullptr;
      uint8_t regId = 0;
      if (candidate != nullptr && Read8(candidate->regId, &regId) && regId == kRegIdValue) {
        map = candidate;
        break;
      }
    }
    if (!Wait(kBootPoll)) {
      return false;
    }
  }
  if (map == nullptr) {
    return Fail(std::format("no FT800/FT81x answered (chip ID {:02X} {:02X} {:02X} {:02X})",
                            id[0], id[1], id[2], id[3]));
  }
  if (m_panel.width > map->maxWidth || m_panel.height > map->maxHeight ||
      m_panel.width <= 0 || m_panel.height <= 0 || m_panel.pclkDivider <= 0) {
    return Fail(std::format("panel {}x{} (divider {}) not supported by the {} (at most {}x{})",
                            m_panel.width, m_panel.height, m_panel.pclkDivider,
                            map->chip == Chip::kFT800 ? "FT800" : "FT81x", map->maxWidth,
                            map->maxHeight));
  }
  m_map = map;

  // programmer guides 2.2.5 / 2.3: timing registers with the pixel clock off, a first display
  // list, swap, DISP on, then REG_PCLK starts the video output
  bool ok = Write8(map->regPclk, 0) && Write8(map->regPwmDuty, 0);
  ok = ok && Write16(map->regHcycle, static_cast<uint16_t>(m_panel.hcycle)) &&
       Write16(map->regHoffset, static_cast<uint16_t>(m_panel.hoffset)) &&
       Write16(map->regHsync0, static_cast<uint16_t>(m_panel.hsync0)) &&
       Write16(map->regHsync1, static_cast<uint16_t>(m_panel.hsync1)) &&
       Write16(map->regVcycle, static_cast<uint16_t>(m_panel.vcycle)) &&
       Write16(map->regVoffset, static_cast<uint16_t>(m_panel.voffset)) &&
       Write16(map->regVsync0, static_cast<uint16_t>(m_panel.vsync0)) &&
       Write16(map->regVsync1, static_cast<uint16_t>(m_panel.vsync1)) &&
       Write8(map->regSwizzle, static_cast<uint8_t>(m_panel.swizzle)) &&
       Write8(map->regPclkPol, static_cast<uint8_t>(m_panel.pclkPolarity)) &&
       Write8(map->regCspread, static_cast<uint8_t>(m_panel.cspread)) &&
       Write8(map->regDither, static_cast<uint8_t>(m_panel.dither)) &&
       Write16(map->regHsize, static_cast<uint16_t>(m_panel.width)) &&
       Write16(map->regVsize, static_cast<uint16_t>(m_panel.height));
  ok = ok && Write32(map->ramDl + 0, DlClearColorRgb(0, 0, 0)) &&
       Write32(map->ramDl + 4, DlClear(true, true, true)) && Write32(map->ramDl + 8, DlDisplay()) &&
       Write8(map->regDlswap, kDlswapFrame);
  uint8_t gpioDir = 0, gpio = 0;
  ok = ok && Read8(map->regGpioDir, &gpioDir) && Write8(map->regGpioDir, gpioDir | kGpioDisp) &&
       Read8(map->regGpio, &gpio) && Write8(map->regGpio, gpio | kGpioDisp) &&
       Write8(map->regPclk, static_cast<uint8_t>(m_panel.pclkDivider));
  // start the co-processor with an empty FIFO (the fault recovery of FT81X PG 5.6; it also
  // releases every engine, whose REG_CPURESET reset value is 2 on the FT81x per its DS table)
  ok = ok && RecoverCoprocessor();
  if (m_int) {
    ok = ok && Write8(map->regIntMask, kIntTouch | kIntTag) && Write8(map->regIntEn, 1);
  }
  std::optional<std::array<int32_t, 6>> transform;
  int transformVersion;
  {
    std::scoped_lock lock{m_mutex};
    transform = m_transform;
    transformVersion = m_transformVersion;
  }
  if (transform) {
    for (int i = 0; i < 6 && ok; ++i) {
      ok = Write32(map->regTouchTransformA + 4 * i, static_cast<uint32_t>((*transform)[i]));
    }
  }
  if (!ok) {
    m_map = nullptr;
    return Fail("SPI transfer failed during start-up");
  }
  m_appliedTransformVersion = transformVersion;
  m_appliedDuty = -1;  // Service() turns the backlight on
  m_touchActive = false;
  m_calibrationSent = false;
  try {
    m_spi.SetClockRate(m_clockHz);
  } catch (const std::exception& e) {
    Fail(std::format("SPI clock {} Hz refused, staying at {} Hz: {}", m_clockHz,
                     kStartupClockRate, e.what()));
  }
  m_problem.clear();
  std::scoped_lock lock{m_mutex};
  m_ready = true;
  m_chip = map->chip;
  return true;
}

void FT8xx::Service() {
  const Map* map = m_map;
  uint8_t regId = 0;
  if (!Read8(map->regId, &regId) || regId != kRegIdValue) {
    {
      std::scoped_lock lock{m_mutex};
      m_ready = false;
      m_touch.reset();
      m_touchTag = 0;
    }
    Fail(std::format("chip lost (REG_ID {:02X}), starting it again", regId));
    return;
  }

  const int duty = m_duty;
  if (duty != m_appliedDuty && Write8(map->regPwmDuty, static_cast<uint8_t>(duty))) {
    m_appliedDuty = duty;
  }

  std::optional<std::array<int32_t, 6>> transform;
  int transformVersion;
  bool calibrate;
  {
    std::scoped_lock lock{m_mutex};
    transform = m_transform;
    transformVersion = m_transformVersion;
    calibrate = m_calibrate;
  }
  if (transformVersion != m_appliedTransformVersion && transform) {
    bool ok = true;
    for (int i = 0; i < 6 && ok; ++i) {
      ok = Write32(map->regTouchTransformA + 4 * i, static_cast<uint32_t>((*transform)[i]));
    }
    if (ok) {
      m_appliedTransformVersion = transformVersion;
    }
  }

  uint32_t read = 0;
  if (!ReadCommandRead(&read)) {
    return;
  }
  if (read == kCmdReadFault) {
    if (RecoverCoprocessor()) {
      std::scoped_lock lock{m_mutex};
      ++m_faults;
    }
    m_calibrationSent = false;  // a calibration in progress starts over
    Fail("co-processor fault (display list too long or bad command); FIFO reset");
    return;
  }
  if (read == m_cmdWrite) {  // idle: the previous frame or calibration is done
    if (m_calibrationSent) {
      // CMD_CALIBRATE's result word, the last word sent, is 0 on failure (FT81X PG 5.52)
      uint32_t outcome = 0;
      if (!Read32(map->ramCmd + ((m_cmdWrite - 4) & (kCmdFifoSize - 1)), &outcome)) {
        return;
      }
      std::array<uint8_t, 24> raw{};
      if (outcome == 0) {
        m_calibrationSent = false;
        m_appliedTransformVersion = -1;  // write a stored transform again
        {
          std::scoped_lock lock{m_mutex};
          m_calibrate = false;
        }
        Fail("touch calibration failed; transform unchanged");
      } else if (Read(map->regTouchTransformA, raw.data(), static_cast<int>(raw.size()))) {
        std::array<int32_t, 6> result{};
        for (int i = 0; i < 6; ++i) {
          result[i] = static_cast<int32_t>(
              uint32_t{raw[4 * i]} | (uint32_t{raw[4 * i + 1]} << 8) |
              (uint32_t{raw[4 * i + 2]} << 16) | (uint32_t{raw[4 * i + 3]} << 24));
        }
        m_calibrationSent = false;
        std::scoped_lock lock{m_mutex};
        m_transform = result;
        m_calibrate = false;
        m_appliedTransformVersion = m_transformVersion;  // the chip has it already
      }
    } else if (calibrate) {
      // FT81X PG 5.52 example: a new display list with a prompt, then CMD_CALIBRATE
      std::vector<uint8_t> stream;
      PutWord(&stream, kCmdDlstart);
      PutWord(&stream, DlClearColorRgb(0, 0, 0));
      PutWord(&stream, DlClear(true, true, true));
      PutWord(&stream, DlColorRgb(255, 255, 255));
      PutWord(&stream, kCmdText);
      PutWord(&stream, (static_cast<uint32_t>(m_panel.height / 2) << 16) |
                           static_cast<uint32_t>(m_panel.width / 2));
      PutWord(&stream, (uint32_t{kOptCenter} << 16) | kRomFontCalibrate);
      for (char c : std::string_view{"Tap the dots"}) {
        stream.push_back(static_cast<uint8_t>(c));
      }
      stream.resize(stream.size() + 4 - stream.size() % 4, 0);  // NUL and padding
      PutWord(&stream, kCmdCalibrate);
      PutWord(&stream, 0);  // result, written by the co-processor
      m_calibrationSent = SendCommands(stream);
    } else {
      std::optional<std::vector<uint8_t>> frame;
      {
        std::scoped_lock lock{m_mutex};
        frame.swap(m_pending);
      }
      if (frame && SendCommands(*frame)) {
        std::scoped_lock lock{m_mutex};
        ++m_framesSent;
      }
    }
  }

  ReadTouch();
}

// REG_TOUCH_SCREEN_XY: x in bits 31-16, y in bits 15-0, 8000h when not touched.
void FT8xx::ReadTouch() {
  const Map* map = m_map;
  if (m_int && !m_touchActive && m_int->Get()) {
    return;  // INT_N high: no touch since the last read
  }
  if (m_int) {
    uint8_t flags = 0;
    Read8(map->regIntFlags, &flags);  // clears the flags (and INT_N)
  }
  uint32_t xy = 0;
  uint8_t tag = 0;
  if (!Read32(map->regTouchScreenXy, &xy) || !Read8(map->regTouchTag, &tag)) {
    return;
  }
  const bool touched = (xy >> 16) != kNotTouched && (xy & 0xFFFF) != kNotTouched;
  m_touchActive = touched;
  std::scoped_lock lock{m_mutex};
  if (touched) {
    m_touch = TouchPoint{static_cast<int16_t>(xy >> 16), static_cast<int16_t>(xy & 0xFFFF)};
    m_touchTag = tag;
  } else {
    m_touch.reset();
    m_touchTag = 0;
  }
}

// Writes a command stream (a multiple of 4 bytes) into the RAM_CMD ring and moves
// REG_CMD_WRITE after each piece. Pieces end at the ring's end (no reliance on the chip's
// wrap of continuous writes) and never fill more than 4092 bytes (FT81X PG 5.1.1).
bool FT8xx::SendCommands(const std::vector<uint8_t>& stream) {
  const Map* map = m_map;
  const auto start = std::chrono::steady_clock::now();
  size_t done = 0;
  while (done < stream.size()) {
    uint32_t read = 0;
    if (!ReadCommandRead(&read) || read == kCmdReadFault) {
      return false;
    }
    const uint32_t fullness = (m_cmdWrite - read) & (kCmdFifoSize - 1);
    const uint32_t space = kCmdFifoMaxFill - fullness;
    if (space == 0) {
      if (std::chrono::steady_clock::now() - start > kFifoTimeout ||
          !Wait(std::chrono::milliseconds{1})) {
        return false;
      }
      continue;
    }
    const size_t n = std::min<size_t>(
        {space, stream.size() - done, kCmdFifoSize - m_cmdWrite});
    if (!Write(map->ramCmd + m_cmdWrite, stream.data() + done, static_cast<int>(n))) {
      return false;
    }
    done += n;
    m_cmdWrite = (m_cmdWrite + static_cast<uint32_t>(n)) & (kCmdFifoSize - 1);
    if (!Write32(map->regCmdWrite, m_cmdWrite)) {
      return false;
    }
  }
  return true;
}

bool FT8xx::ReadCommandRead(uint32_t* read) {
  uint32_t value = 0;
  if (!Read32(m_map->regCmdRead, &value)) {
    return false;
  }
  *read = value & 0xFFF;  // 12-bit register
  return true;
}

// FT81X PG 5.6: hold the co-processor in reset, zero both FIFO pointers, release it.
bool FT8xx::RecoverCoprocessor() {
  const Map* map = m_map;
  const bool ok = Write8(map->regCpuReset, 1) && Write16(map->regCmdRead, 0) &&
                  Write16(map->regCmdWrite, 0) && Write8(map->regCpuReset, 0);
  m_cmdWrite = 0;
  return ok;
}

// ---- SPI framing ----

// Host command: 01b + 6-bit code, then the parameter and 00h (FT81X DS 4.1.5); ACTIVE is 00h.
bool FT8xx::HostCommand(uint8_t command, uint8_t parameter) {
  const uint8_t tx[3] = {command, parameter, 0};
  return m_spi.Write(tx, sizeof(tx)) == static_cast<int>(sizeof(tx));
}

// Memory read: 00b + 22-bit address, one dummy byte, then the data (little endian).
bool FT8xx::Read(uint32_t address, uint8_t* data, int size) {
  const uint8_t header[4] = {static_cast<uint8_t>((address >> 16) & 0x3F),
                             static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address),
                             0};
  const SpiSegment segments[] = {{header, nullptr, sizeof(header)}, {nullptr, data, size}};
  return m_spi.TransactionSegments(segments) >= 0;
}

// Memory write: 10b + 22-bit address, then the data (little endian).
bool FT8xx::Write(uint32_t address, const uint8_t* data, int size) {
  const uint8_t header[3] = {static_cast<uint8_t>(0x80 | ((address >> 16) & 0x3F)),
                             static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address)};
  const SpiSegment segments[] = {{header, nullptr, sizeof(header)}, {data, nullptr, size}};
  return m_spi.TransactionSegments(segments) >= 0;
}

bool FT8xx::Read8(uint32_t address, uint8_t* value) {
  return Read(address, value, 1);
}

bool FT8xx::Read32(uint32_t address, uint32_t* value) {
  uint8_t b[4] = {};
  if (!Read(address, b, sizeof(b))) {
    return false;
  }
  *value = uint32_t{b[0]} | (uint32_t{b[1]} << 8) | (uint32_t{b[2]} << 16) | (uint32_t{b[3]} << 24);
  return true;
}

bool FT8xx::Write8(uint32_t address, uint8_t value) {
  return Write(address, &value, 1);
}

bool FT8xx::Write16(uint32_t address, uint16_t value) {
  const uint8_t b[2] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
  return Write(address, b, sizeof(b));
}

bool FT8xx::Write32(uint32_t address, uint32_t value) {
  const uint8_t b[4] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
                        static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)};
  return Write(address, b, sizeof(b));
}

// ---- frame building (caller) ----

// m_buildMutex held
void FT8xx::AppendWord(uint32_t word) {
  if (m_building.size() + 4 > static_cast<size_t>(kMaxFrameBytes)) {
    m_buildOverflow = true;
    return;
  }
  PutWord(&m_building, word);
}

// m_buildMutex held. One 16-bit field; the callers append them in pairs, so the stream stays a
// multiple of 4 bytes.
void FT8xx::AppendHalf(int value) {
  if (m_building.size() + 2 > static_cast<size_t>(kMaxFrameBytes)) {
    m_buildOverflow = true;
    return;
  }
  m_building.push_back(static_cast<uint8_t>(value));
  m_building.push_back(static_cast<uint8_t>(value >> 8));
}

// m_buildMutex held. The text up to its first NUL, a NUL, and padding to 4 bytes (PG 5.1.1:
// the co-processor continues at the next 4-byte boundary).
void FT8xx::AppendString(std::string_view text) {
  text = text.substr(0, text.find('\0'));
  const size_t padded = (text.size() + 4) & ~size_t{3};
  if (m_building.size() + padded > static_cast<size_t>(kMaxFrameBytes)) {
    m_buildOverflow = true;
    return;
  }
  m_building.insert(m_building.end(), text.begin(), text.end());
  m_building.resize(m_building.size() + padded - text.size(), 0);
}

// m_buildMutex held
void FT8xx::StartFrameIfEmpty() {
  if (m_building.empty()) {
    AppendWord(kCmdDlstart);
  }
}

void FT8xx::ClearColorRGB(uint8_t red, uint8_t green, uint8_t blue) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlClearColorRgb(red, green, blue));
}

void FT8xx::Clear(bool color, bool stencil, bool tag) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlClear(color, stencil, tag));
}

void FT8xx::ColorRGB(uint8_t red, uint8_t green, uint8_t blue) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlColorRgb(red, green, blue));
}

void FT8xx::PointSize(double radius) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlPointSize(static_cast<uint32_t>(Sixteenths(radius, 0, 8191))));
}

void FT8xx::LineWidth(double width) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlLineWidth(static_cast<uint32_t>(Sixteenths(width, 0, 4095))));
}

void FT8xx::Tag(uint8_t tag) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlTag(tag));
}

void FT8xx::Begin(Primitive primitive) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlBegin(primitive));
}

void FT8xx::Vertex2f(double x, double y) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlVertex2f(Sixteenths(x, -16384, 16383), Sixteenths(y, -16384, 16383)));
}

void FT8xx::End() {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(DlEnd());
}

void FT8xx::FgColor(uint32_t rgb) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(kCmdFgcolor);
  AppendWord(rgb & 0xFFFFFF);
}

void FT8xx::BgColor(uint32_t rgb) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(kCmdBgcolor);
  AppendWord(rgb & 0xFFFFFF);
}

// CMD_TEXT: x, y, font, options (16 bits each), string (FT81X PG 5.41)
void FT8xx::Text(int x, int y, int font, uint16_t options, std::string_view text) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(kCmdText);
  AppendHalf(x);
  AppendHalf(y);
  AppendHalf(font);
  AppendHalf(options);
  AppendString(text);
}

// CMD_BUTTON: x, y, w, h, font, options (16 bits each), string (FT81X PG 5.28)
void FT8xx::Button(int x, int y, int w, int h, int font, uint16_t options,
                   std::string_view label) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(kCmdButton);
  AppendHalf(x);
  AppendHalf(y);
  AppendHalf(w);
  AppendHalf(h);
  AppendHalf(font);
  AppendHalf(options);
  AppendString(label);
}

// CMD_NUMBER: x, y, font, options (16 bits each), n (32 bits) (FT81X PG 5.43)
void FT8xx::Number(int x, int y, int font, uint16_t options, int32_t value) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(kCmdNumber);
  AppendHalf(x);
  AppendHalf(y);
  AppendHalf(font);
  AppendHalf(options);
  AppendWord(static_cast<uint32_t>(value));
}

// CMD_GAUGE: x, y, r, options, major, minor, val, range (16 bits each) (FT81X PG 5.33)
void FT8xx::Gauge(int x, int y, int r, uint16_t options, int major, int minor, int value,
                  int range) {
  std::scoped_lock lock{m_buildMutex};
  StartFrameIfEmpty();
  AppendWord(kCmdGauge);
  AppendHalf(x);
  AppendHalf(y);
  AppendHalf(r);
  AppendHalf(options);
  AppendHalf(major);
  AppendHalf(minor);
  AppendHalf(value);
  AppendHalf(range);
}

void FT8xx::Swap() {
  std::vector<uint8_t> frame;
  {
    std::scoped_lock lock{m_buildMutex};
    StartFrameIfEmpty();
    AppendWord(DlDisplay());
    AppendWord(kCmdSwap);
    const bool overflow = m_buildOverflow;
    frame.swap(m_building);
    m_buildOverflow = false;
    if (overflow) {
      if (!m_overflowReported) {
        m_overflowReported = true;
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] FT8xx on SPI bus {} CS {}: frame over {} bytes "
                               "discarded\n",
                               static_cast<int>(m_spi.GetPort()), m_spi.GetChipSelect(),
                               kMaxFrameBytes);
      }
      return;
    }
  }
  {
    std::scoped_lock lock{m_mutex};
    if (m_pending) {
      ++m_framesDropped;
    }
    m_pending = std::move(frame);
  }
  {
    std::scoped_lock lock{m_waitMutex};
    m_swapped = true;
  }
  m_wake.notify_all();
}

// ---- touch ----

std::optional<FT8xx::TouchPoint> FT8xx::GetTouch() const {
  std::scoped_lock lock{m_mutex};
  return m_touch;
}

int FT8xx::GetTouchTag() const {
  std::scoped_lock lock{m_mutex};
  return m_touchTag;
}

void FT8xx::CalibrateTouch() {
  std::scoped_lock lock{m_mutex};
  m_calibrate = true;
}

bool FT8xx::IsCalibrating() const {
  std::scoped_lock lock{m_mutex};
  return m_calibrate;
}

std::optional<std::array<int32_t, 6>> FT8xx::GetTouchTransform() const {
  std::scoped_lock lock{m_mutex};
  return m_transform;
}

void FT8xx::SetTouchTransform(const std::array<int32_t, 6>& transform) {
  std::scoped_lock lock{m_mutex};
  m_transform = transform;
  ++m_transformVersion;
}

int FT8xx::GetCoprocessorFaults() const {
  std::scoped_lock lock{m_mutex};
  return m_faults;
}

void FT8xx::LogTo(telemetry::TelemetryTable& table) const {
  bool ready;
  int frames, dropped, faults, tag;
  std::optional<TouchPoint> touch;
  {
    std::scoped_lock lock{m_mutex};
    ready = m_ready;
    frames = m_framesSent;
    dropped = m_framesDropped;
    faults = m_faults;
    touch = m_touch;
    tag = m_touchTag;
  }
  table.Log("Ready", ready);
  table.Log("Frames", frames);
  table.Log("Dropped frames", dropped);
  table.Log("Faults", faults);
  table.Log("Touched", touch.has_value());
  table.Log("Touch X", touch ? touch->x : 0);
  table.Log("Touch Y", touch ? touch->y : 0);
  table.Log("Touch tag", tag);
}

std::string_view FT8xx::GetTelemetryType() const {
  return "Display";
}
