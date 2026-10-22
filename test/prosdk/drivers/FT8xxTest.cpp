// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "prosdk/drivers/display/FT8xx.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "FakeSPIDevice.hpp"
#include "prosdk/simulation/DIOSim.hpp"
#include "prosdk/simulation/SpiSim.hpp"

using namespace prosdk;
using namespace std::chrono_literals;

namespace {
using Bytes = std::vector<uint8_t>;

constexpr int kCs = 10;
constexpr int kPd = 4;
constexpr int kInt = 5;

// An FT800 or FT810 seen from SPI (FT800 DS 4.1.4-4.1.6, FT81X DS 4.1): a 4 MiB memory, host
// commands recorded, every memory write recorded. A write to REG_CMD_WRITE runs the
// co-processor at once: the bytes from REG_CMD_READ to the new write pointer are appended to
// `executed` and REG_CMD_READ follows (or turns FFFh when `faultOnce` is set). CMD_CALIBRATE
// in the executed bytes stores `calibration` in REG_TOUCH_TRANSFORM_A-F and writes its result word
// in RAM_CMD: 1, or 0 when `calibrationFails` is set (FT81X PG 5.52).
struct EveModel {
  explicit EveModel(bool ft81x) : is81x{ft81x} {
    const uint32_t reg = ft81x ? 0x302000 : 0x102400;
    regCmdRead = reg + (ft81x ? 0xF8 : 0xE4);
    regCmdWrite = reg + (ft81x ? 0xFC : 0xE8);
    regTouchXy = ft81x ? 0x302124 : 0x102510;
    regTouchTag = ft81x ? 0x30212C : 0x102518;
    regTransformA = ft81x ? 0x302150 : 0x10251C;
    ramCmd = ft81x ? 0x308000 : 0x108000;
    mem[0x0C0000] = 0x08;
    mem[0x0C0001] = ft81x ? 0x10 : 0x00;
    mem[0x0C0002] = 0x01;
    mem[reg] = 0x7C;  // REG_ID
    Put32(regTouchXy, 0x80008000);
    mem[reg + (ft81x ? 0x90 : 0x8C)] = 0x80;  // REG_GPIO_DIR reset value
  }

  void Put32(uint32_t address, uint32_t value) {
    for (int i = 0; i < 4; ++i) {
      mem[address + i] = static_cast<uint8_t>(value >> (8 * i));
    }
  }
  uint32_t Get(uint32_t address, int size) const {
    uint32_t value = 0;
    for (int i = 0; i < size; ++i) {
      value |= uint32_t{mem[address + i]} << (8 * i);
    }
    return value;
  }

  void Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
    std::scoped_lock lock{mutex};
    frames.emplace_back(tx.begin(), tx.end());
    const uint32_t address = ((uint32_t{tx[0]} & 0x3F) << 16) | (uint32_t{tx[1]} << 8) | tx[2];
    if (tx.size() == 3 && (tx[0] & 0x80) == 0) {
      hostCommands.emplace_back(tx.begin(), tx.end());
    } else if ((tx[0] & 0xC0) == 0x00) {  // read: address, dummy byte, data
      for (size_t i = 4; i < rx.size(); ++i) {
        rx[i] = mem[(address + i - 4) & 0x3FFFFF];
      }
    } else if ((tx[0] & 0xC0) == 0x80) {  // write
      writes.emplace_back(address, Bytes(tx.begin() + 3, tx.end()));
      for (size_t i = 3; i < tx.size(); ++i) {
        mem[(address + i - 3) & 0x3FFFFF] = tx[i];
      }
      if (address == regCmdWrite) {
        RunCoprocessor();
      }
    }
  }

  void RunCoprocessor() {
    const uint32_t write = Get(regCmdWrite, 2) & 0xFFF;
    uint32_t read = Get(regCmdRead, 2) & 0xFFF;
    if (faultOnce) {
      faultOnce = false;
      Put32(regCmdRead, 0xFFF);
      return;
    }
    const size_t start = executed.size();
    const uint32_t startRead = read;
    while (read != write) {
      executed.push_back(mem[ramCmd + read]);
      read = (read + 1) & 0xFFF;
    }
    Put32(regCmdRead, read);
    for (size_t i = start; i + 4 <= executed.size(); i += 4) {
      const uint32_t word = uint32_t{executed[i]} | (uint32_t{executed[i + 1]} << 8) |
                            (uint32_t{executed[i + 2]} << 16) | (uint32_t{executed[i + 3]} << 24);
      if (word == 0xFFFFFF15) {  // CMD_CALIBRATE
        for (int k = 0; k < 6; ++k) {
          Put32(regTransformA + 4 * k, static_cast<uint32_t>(calibration[k]));
        }
        const uint32_t resultAt = (startRead + static_cast<uint32_t>(i - start) + 4) & 0xFFF;
        Put32(ramCmd + resultAt, calibrationFails ? 0 : 1);
      }
    }
  }

  // the values written to address (up to 4 bytes each), in order
  std::vector<uint32_t> ValuesWritten(uint32_t address) {
    std::scoped_lock lock{mutex};
    std::vector<uint32_t> out;
    for (const auto& [at, data] : writes) {
      if (at == address) {
        uint32_t value = 0;
        for (size_t i = 0; i < data.size() && i < 4; ++i) {
          value |= uint32_t{data[i]} << (8 * i);
        }
        out.push_back(value);
      }
    }
    return out;
  }

  uint32_t Read(uint32_t address, int size) {
    std::scoped_lock lock{mutex};
    return Get(address, size);
  }

  size_t ExecutedSize() {
    std::scoped_lock lock{mutex};
    return executed.size();
  }

  std::mutex mutex;
  bool is81x;
  std::vector<uint8_t> mem = std::vector<uint8_t>(0x400000, 0);
  std::vector<Bytes> frames;  // every transaction's sent bytes
  std::vector<Bytes> hostCommands;
  std::vector<std::pair<uint32_t, Bytes>> writes;
  Bytes executed;
  bool faultOnce = false;
  bool calibrationFails = false;
  std::array<int32_t, 6> calibration{0x8000, 0x10, -0x200000, -0x20, 0x9000, 0x1000000};
  uint32_t regCmdRead, regCmdWrite, regTouchXy, regTouchTag, regTransformA, ramCmd;
};

// An FT8xx fake on SPI bus 0, CS 10.
struct Fixture {
  explicit Fixture(bool ft81x) : model{ft81x}, dev{0, kCs} {
    dev.handler = [this](std::span<const uint8_t> tx, std::span<uint8_t> rx) {
      model.Transfer(tx, rx);
    };
  }
  EveModel model;
  test::FakeSPIDevice dev;
};

bool WaitFor(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 3s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

void Word(Bytes* out, uint32_t word) {
  for (int i = 0; i < 4; ++i) {
    out->push_back(static_cast<uint8_t>(word >> (8 * i)));
  }
}

// VERTEX2F hand-assembled (programmer guides 4.47 / 4.43): 01b, x and y in 1/16 pixel, 15 bits
uint32_t Vertex(int x, int y) {
  return (1u << 30) | ((static_cast<uint32_t>(x * 16) & 0x7FFF) << 15) |
         (static_cast<uint32_t>(y * 16) & 0x7FFF);
}

}  // namespace

TEST_CASE("FT8xx brings up an FT810 with an 800x480 panel", "[ft8xx]") {
  Fixture fx{true};
  sim::DIOSim pd{kPd};
  std::vector<bool> pdLevels;
  std::mutex pdMutex;
  auto cb = pd.RegisterValueCallback(
      [&](std::string_view, const HAL_Value* v) {
        std::scoped_lock lock{pdMutex};
        pdLevels.push_back(v->data.v_boolean);
      },
      false);

  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));
  CHECK(display.GetChip() == FT8xx::Chip::kFT81x);
  {
    std::scoped_lock lock{pdMutex};
    REQUIRE(pdLevels.size() >= 2);
    CHECK_FALSE(pdLevels[pdLevels.size() - 2]);  // the reset pulse: low, then high
    CHECK(pdLevels.back());
  }

  // CLKEXT, then ACTIVE (FT81X PG 2.3)
  {
    std::scoped_lock lock{fx.model.mutex};
    REQUIRE(fx.model.hostCommands.size() >= 2);
    CHECK(fx.model.hostCommands[0] == Bytes{0x44, 0x00, 0x00});
    CHECK(fx.model.hostCommands[1] == Bytes{0x00, 0x00, 0x00});
  }

  // timing registers (FT81X DS 5.1 register table)
  CHECK(fx.model.Read(0x30202C, 2) == 1056);  // REG_HCYCLE
  CHECK(fx.model.Read(0x302030, 2) == 46);    // REG_HOFFSET
  CHECK(fx.model.Read(0x302034, 2) == 800);   // REG_HSIZE
  CHECK(fx.model.Read(0x30203C, 2) == 40);    // REG_HSYNC1
  CHECK(fx.model.Read(0x302040, 2) == 525);   // REG_VCYCLE
  CHECK(fx.model.Read(0x302044, 2) == 23);    // REG_VOFFSET
  CHECK(fx.model.Read(0x302048, 2) == 480);   // REG_VSIZE
  CHECK(fx.model.Read(0x302050, 2) == 1);     // REG_VSYNC1
  CHECK(fx.model.Read(0x30206C, 1) == 1);     // REG_PCLK_POL
  CHECK(fx.model.Read(0x302054, 1) == 2);     // REG_DLSWAP = DLSWAP_FRAME
  CHECK((fx.model.Read(0x302090, 1) & 0x80) != 0);  // REG_GPIO_DIR: DISP output
  CHECK((fx.model.Read(0x302094, 1) & 0x80) != 0);  // REG_GPIO: DISP high
  // first display list: CLEAR_COLOR_RGB(0,0,0), CLEAR(1,1,1), DISPLAY
  CHECK(fx.model.Read(0x300000, 4) == 0x02000000);
  CHECK(fx.model.Read(0x300004, 4) == 0x26000007);
  CHECK(fx.model.Read(0x300008, 4) == 0x00000000);
  // the pixel clock is off during the set-up and starts last (REG_PCLK 0, then 2)
  CHECK(fx.model.ValuesWritten(0x302070) == std::vector<uint32_t>{0, 2});

  // framing: a write is 10b + 22-bit address + data, a read 00b + address + dummy byte
  {
    std::scoped_lock lock{fx.model.mutex};
    bool pclkWrite = false, idRead = false;
    for (const Bytes& f : fx.model.frames) {
      pclkWrite = pclkWrite || f == Bytes{0xB0, 0x20, 0x70, 0x02};
      idRead = idRead || f == Bytes{0x30, 0x20, 0x00, 0x00, 0x00};
    }
    CHECK(pclkWrite);
    CHECK(idRead);
  }

  platform::hal::SpiDeviceConfig config;
  REQUIRE(sim::GetSpiDeviceConfig(0, kCs, &config));
  CHECK(config.mode == 0);
  CHECK(config.clockHz == FT8xx::kDefaultClockRate);

  // backlight: full after start-up, then half (REG_PWM_DUTY 0-128)
  CHECK(WaitFor([&] { return fx.model.Read(0x3020D4, 1) == 128; }));
  display.SetBacklight(0.5);
  CHECK(WaitFor([&] { return fx.model.Read(0x3020D4, 1) == 64; }));
  CHECK(display.GetTelemetryType() == "Display");
}

TEST_CASE("FT8xx uses the FT800 memory map and rejects panels it cannot drive", "[ft8xx]") {
  {
    Fixture fx{false};
    FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kBoardozaFT800Bundle};
    REQUIRE(WaitFor([&] { return display.IsReady(); }));
    CHECK(display.GetChip() == FT8xx::Chip::kFT800);
    // FT800 DS section 5 register table
    CHECK(fx.model.Read(0x102428, 2) == 548);  // REG_HCYCLE
    CHECK(fx.model.Read(0x102430, 2) == 480);  // REG_HSIZE
    CHECK(fx.model.Read(0x102444, 2) == 272);  // REG_VSIZE
    CHECK(fx.model.Read(0x102464, 1) == 1);    // REG_CSPREAD
    CHECK(fx.model.ValuesWritten(0x10246C) == std::vector<uint32_t>{0, 5});  // REG_PCLK
    CHECK((fx.model.Read(0x102490, 1) & 0x80) != 0);                         // REG_GPIO DISP
    CHECK(fx.model.Read(0x100004, 4) == 0x26000007);                         // RAM_DL + 4
  }
  {
    Fixture fx{false};
    FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};  // the FT800 stops at 512
    CHECK_FALSE(WaitFor([&] { return display.IsReady(); }, 300ms));
    CHECK(display.GetChip() == FT8xx::Chip::kNone);
    CHECK(fx.model.ValuesWritten(0x10246C).empty());  // video never started
  }
}

TEST_CASE("FT8xx frame bytes match hand-assembled programmer guide words", "[ft8xx]") {
  Fixture fx{true};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));

  display.ClearColorRGB(5, 45, 110);
  display.Clear();
  display.ColorRGB(160, 22, 22);
  display.PointSize(20);
  display.Begin(FT8xx::kPoints);
  display.Vertex2f(192, 133);
  display.End();
  display.Text(80, 60, 31, FT8xx::kOptCenterX, "Text!");
  display.Number(20, 60, 31, 0, 42);
  display.Gauge(80, 60, 50, 0, 5, 4, 30, 100);
  display.FgColor(0x703800);
  display.Button(10, 20, 140, 100, 31, FT8xx::kOptFlat, std::string{"OK\0ignored", 10});
  display.Swap();

  Bytes expected;
  Word(&expected, 0xFFFFFF00);  // CMD_DLSTART
  Word(&expected, 0x02052D6E);  // CLEAR_COLOR_RGB(5, 45, 110)
  Word(&expected, 0x26000007);  // CLEAR(1, 1, 1)
  Word(&expected, 0x04A01616);  // COLOR_RGB(160, 22, 22)
  Word(&expected, 0x0D000140);  // POINT_SIZE(320): 20 pixels
  Word(&expected, 0x1F000002);  // BEGIN(POINTS)
  Word(&expected, 0x46000850);  // VERTEX2F(192 * 16, 133 * 16)
  Word(&expected, 0x21000000);  // END
  Word(&expected, 0xFFFFFF0C);  // CMD_TEXT
  Word(&expected, 0x003C0050);  // x 80, y 60
  Word(&expected, 0x0200001F);  // font 31, OPT_CENTERX
  for (uint8_t c : Bytes{'T', 'e', 'x', 't', '!', 0, 0, 0}) {
    expected.push_back(c);
  }
  Word(&expected, 0xFFFFFF2E);  // CMD_NUMBER
  Word(&expected, 0x003C0014);  // x 20, y 60
  Word(&expected, 0x0000001F);  // font 31, options 0
  Word(&expected, 42);
  Word(&expected, 0xFFFFFF13);  // CMD_GAUGE
  Word(&expected, 0x003C0050);  // x 80, y 60
  Word(&expected, 0x00000032);  // r 50, options 0
  Word(&expected, 0x00040005);  // major 5, minor 4
  Word(&expected, 0x0064001E);  // value 30, range 100
  Word(&expected, 0xFFFFFF0A);  // CMD_FGCOLOR
  Word(&expected, 0x00703800);
  Word(&expected, 0xFFFFFF0D);  // CMD_BUTTON
  Word(&expected, 0x0014000A);  // x 10, y 20
  Word(&expected, 0x0064008C);  // w 140, h 100
  Word(&expected, 0x0100001F);  // font 31, OPT_FLAT
  for (uint8_t c : Bytes{'O', 'K', 0, 0}) {  // the label ends at its first NUL
    expected.push_back(c);
  }
  Word(&expected, 0x00000000);  // DISPLAY
  Word(&expected, 0xFFFFFF01);  // CMD_SWAP

  REQUIRE(WaitFor([&] { return fx.model.ExecutedSize() >= expected.size(); }));
  std::scoped_lock lock{fx.model.mutex};
  CHECK(fx.model.executed == expected);
}

TEST_CASE("FT8xx feeds long frames through the command FIFO in pieces", "[ft8xx]") {
  Fixture fx{true};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));

  auto frame = [&](int vertices, Bytes* expected) {
    Word(expected, 0xFFFFFF00);
    for (int i = 0; i < vertices; ++i) {
      display.Vertex2f(i % 400, i % 200);
      Word(expected, Vertex(i % 400, i % 200));
    }
    Word(expected, 0);
    Word(expected, 0xFFFFFF01);
    display.Swap();
  };
  Bytes expected;
  frame(1000, &expected);  // 4012 bytes: the next frame wraps at the end of RAM_CMD
  REQUIRE(WaitFor([&] { return fx.model.ExecutedSize() == expected.size(); }));
  frame(1500, &expected);  // 6012 bytes: more than the 4092 the FIFO holds
  REQUIRE(WaitFor([&] { return fx.model.ExecutedSize() == expected.size(); }));

  std::scoped_lock lock{fx.model.mutex};
  CHECK(fx.model.executed == expected);
  int pieces = 0;
  for (const auto& [at, data] : fx.model.writes) {
    if (at >= 0x308000 && at < 0x309000) {
      ++pieces;
      CHECK(at + data.size() <= 0x309000);  // no piece crosses the end of RAM_CMD
      CHECK(data.size() <= 4092);
      CHECK(data.size() % 4 == 0);
    }
  }
  CHECK(pieces >= 4);
}

TEST_CASE("FT8xx discards a frame over the size limit", "[ft8xx]") {
  Fixture fx{true};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));

  for (int i = 0; i < 2100; ++i) {  // 8400 bytes of vertices
    display.Vertex2f(1, 1);
  }
  display.Swap();
  display.Clear();
  display.Swap();

  Bytes expected;
  Word(&expected, 0xFFFFFF00);
  Word(&expected, 0x26000007);
  Word(&expected, 0);
  Word(&expected, 0xFFFFFF01);
  REQUIRE(WaitFor([&] { return fx.model.ExecutedSize() >= expected.size(); }));
  std::this_thread::sleep_for(30ms);
  std::scoped_lock lock{fx.model.mutex};
  CHECK(fx.model.executed == expected);
}

TEST_CASE("FT8xx recovers from a co-processor fault", "[ft8xx]") {
  Fixture fx{true};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));
  const size_t cpuResetsBefore = fx.model.ValuesWritten(0x302020).size();

  {
    std::scoped_lock lock{fx.model.mutex};
    fx.model.faultOnce = true;
  }
  display.Clear();
  display.Swap();
  REQUIRE(WaitFor([&] { return display.GetCoprocessorFaults() == 1; }));
  // FT81X PG 5.6: REG_CPURESET 1, REG_CMD_READ 0, REG_CMD_WRITE 0, REG_CPURESET 0
  const auto resets = fx.model.ValuesWritten(0x302020);
  REQUIRE(resets.size() == cpuResetsBefore + 2);
  CHECK(resets[resets.size() - 2] == 1);
  CHECK(resets.back() == 0);
  CHECK(fx.model.Read(0x3020F8, 2) == 0);
  CHECK(fx.model.Read(0x3020FC, 2) == 0);
  CHECK(display.IsReady());

  // the next frame goes from the start of RAM_CMD
  display.ColorRGB(1, 2, 3);
  display.Swap();
  Bytes expected;
  Word(&expected, 0xFFFFFF00);
  Word(&expected, 0x04010203);
  Word(&expected, 0);
  Word(&expected, 0xFFFFFF01);
  REQUIRE(WaitFor([&] { return fx.model.ExecutedSize() >= expected.size(); }));
  std::scoped_lock lock{fx.model.mutex};
  CHECK(fx.model.executed == expected);
}

TEST_CASE("FT8xx reports touches and tags", "[ft8xx]") {
  Fixture fx{true};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));
  std::this_thread::sleep_for(30ms);
  CHECK_FALSE(display.GetTouch().has_value());  // 8000h 8000h: not touched
  CHECK(display.GetTouchTag() == 0);

  {
    std::scoped_lock lock{fx.model.mutex};
    fx.model.Put32(0x302124, (100u << 16) | 50u);  // REG_TOUCH_SCREEN_XY
    fx.model.mem[0x30212C] = 7;                    // REG_TOUCH_TAG
  }
  REQUIRE(WaitFor([&] { return display.GetTouch().has_value(); }));
  CHECK(display.GetTouch()->x == 100);
  CHECK(display.GetTouch()->y == 50);
  CHECK(display.GetTouchTag() == 7);

  {
    std::scoped_lock lock{fx.model.mutex};
    fx.model.Put32(0x302124, 0x80008000);
  }
  CHECK(WaitFor([&] { return !display.GetTouch().has_value(); }));
  CHECK(display.GetTouchTag() == 0);
}

TEST_CASE("FT8xx reads the touch registers after INT only", "[ft8xx]") {
  Fixture fx{false};
  sim::DIOSim intPin{kInt};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel480x272, kInt};
  intPin.SetValue(true);  // INT_N idle high
  REQUIRE(WaitFor([&] { return display.IsReady(); }));
  CHECK(fx.model.Read(0x1024A0, 1) == 0x06);  // REG_INT_MASK: TOUCH and TAG
  CHECK(fx.model.Read(0x10249C, 1) == 1);     // REG_INT_EN

  {
    std::scoped_lock lock{fx.model.mutex};
    fx.model.Put32(0x102510, (10u << 16) | 20u);
  }
  std::this_thread::sleep_for(50ms);
  CHECK_FALSE(display.GetTouch().has_value());  // INT_N high: not read
  intPin.SetValue(false);
  REQUIRE(WaitFor([&] { return display.GetTouch().has_value(); }));
  CHECK(display.GetTouch()->x == 10);
  CHECK(display.GetTouch()->y == 20);
}

TEST_CASE("FT8xx calibrates the touch screen and restores a transform", "[ft8xx]") {
  Fixture fx{true};
  FT8xx display{SPI::kPort0, kCs, kPd, ft8xx::kPanel800x480};
  REQUIRE(WaitFor([&] { return display.IsReady(); }));
  CHECK_FALSE(display.GetTouchTransform().has_value());

  display.CalibrateTouch();
  CHECK(display.IsCalibrating());
  REQUIRE(WaitFor([&] { return !display.IsCalibrating(); }));
  REQUIRE(display.GetTouchTransform().has_value());
  CHECK(*display.GetTouchTransform() == fx.model.calibration);
  {
    std::scoped_lock lock{fx.model.mutex};
    const Bytes& ex = fx.model.executed;
    // CMD_CALIBRATE and its result word end the calibration stream
    REQUIRE(ex.size() >= 8);
    CHECK(Bytes(ex.end() - 8, ex.end()) == Bytes{0x15, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0});
    CHECK(std::string(ex.begin(), ex.end()).find("Tap the dots") != std::string::npos);
  }

  const std::array<int32_t, 6> stored{1, 2, 3, 4, 5, 6};
  display.SetTouchTransform(stored);
  CHECK(WaitFor([&] {
    return fx.model.Read(0x302150, 4) == 1 && fx.model.Read(0x302164, 4) == 6;
  }));

  // a failed calibration (result 0) keeps the stored transform and writes it to the chip again
  {
    std::scoped_lock lock{fx.model.mutex};
    fx.model.calibrationFails = true;
  }
  display.CalibrateTouch();
  REQUIRE(WaitFor([&] { return !display.IsCalibrating(); }));
  CHECK(*display.GetTouchTransform() == stored);
  CHECK(WaitFor([&] {
    return fx.model.Read(0x302150, 4) == 1 && fx.model.Read(0x302164, 4) == 6;
  }));
}

TEST_CASE("FT8xx without a chip is not ready and stops at once", "[ft8xx]") {
  test::FakeSPIDevice dev{0, kCs};  // reads zeros
  auto display = std::make_unique<FT8xx>(SPI::kPort0, kCs, kPd, ft8xx::kPanel480x272);
  display->Clear();
  display->Swap();  // kept until a chip answers
  CHECK_FALSE(WaitFor([&] { return display->IsReady(); }, 500ms));
  CHECK(display->GetChip() == FT8xx::Chip::kNone);
  CHECK_FALSE(display->GetTouch().has_value());

  // the thread waits 1 s before the next attempt; the destructor must not wait it out
  const auto start = std::chrono::steady_clock::now();
  display.reset();
  CHECK(std::chrono::steady_clock::now() - start < 500ms);
}
