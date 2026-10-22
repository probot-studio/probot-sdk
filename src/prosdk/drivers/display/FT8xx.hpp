// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Bridgetek (FTDI) FT800 / FT810 EVE graphics controller over prosdk::SPI
// (design/boardoza/P13-eve.md). Clean-room from the Bridgetek data sheets and programmer guides;
// no vendor sample code. Written for the Boardoza FT800 and FT810 TFT driver boards (12 MHz
// crystal, 4-wire resistive touch) and the Ozdisan bundles of those boards with a panel.
//
// The robot code builds a frame (display-list commands and co-processor widgets) and hands it
// over with Swap(). A thread brings the chip up, feeds frames into the co-processor's command
// FIFO, applies the backlight and reads the touch screen; no SPI traffic runs on the caller.

#pragma once

#include <stdint.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "prosdk/drivers/bus/SPI.hpp"
#include "prosdk/drivers/display/FT8xxPanels.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * FT800 or FT810 (EVE) display controller with a TFT panel.
 *
 * Construction never blocks and does not use the bus. The thread pulses PD low, selects the
 * crystal, wakes the chip, detects FT800 or FT81x from the chip ID, programs the panel timing and
 * turns the display on; IsReady() is then true. A missing or reset chip is brought up again
 * about once a second.
 *
 * Drawing: call the frame methods (Clear, ColorRGB, Begin/Vertex2f/End, Text, Button, ...) from
 * one thread, for example the robot loop, then Swap(). Each call only appends to a buffer. The
 * thread sends the latest swapped frame when the co-processor is idle; a frame swapped while
 * another is still waiting replaces it (counted as dropped). Redraw at 10-20 Hz, not every loop.
 *
 * Coordinates are pixels from the top-left corner. Fonts are the ROM fonts 16-31 (both chips).
 */
class FT8xx : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default SPI clock after start-up. The chip allows 30 MHz (data sheets 4.1); 20 MHz leaves
  /// margin for flying leads and the boards' 74LCX125 input buffers (unverified on hardware).
  static constexpr int kDefaultClockRate = 20'000'000;
  /// Highest SPI clock (data sheets 4.1); a larger clockHz is limited to it.
  static constexpr int kMaxClockRate = 30'000'000;
  /// SPI clock during start-up: at most 11 MHz until the system clock runs (FT800 DS 4.2.3).
  static constexpr int kStartupClockRate = 10'000'000;
  /// Largest frame in bytes of command stream (twice the 4 KiB co-processor FIFO). A frame that
  /// grows beyond it is discarded at Swap().
  static constexpr int kMaxFrameBytes = 8192;

  /// The controller found at start-up.
  enum class Chip {
    /// Not detected (yet).
    kNone,
    /// FT800 (chip ID 08h 00h; 256 KiB RAM_G, registers at 102400h).
    kFT800,
    /// FT810 family (chip ID 08h 10h-13h; 1 MiB RAM_G, registers at 302000h).
    kFT81x
  };

  /// Graphics primitive for Begin() (programmer guides, BEGIN).
  enum Primitive {
    kBitmaps = 1,
    kPoints = 2,
    kLines = 3,
    kLineStrip = 4,
    kEdgeStripR = 5,
    kEdgeStripL = 6,
    kEdgeStripA = 7,
    kEdgeStripB = 8,
    kRects = 9
  };

  /// Widget options (programmer guides, parameter OPTION). Combine with |.
  enum Option : uint16_t {
    /// 3D look (the default).
    kOpt3D = 0,
    /// No 3D look (Button, Gauge).
    kOptFlat = 256,
    /// Number(): the value is signed.
    kOptSigned = 256,
    /// Text/Number: x is the horizontal center.
    kOptCenterX = 512,
    /// Text/Number: y is the vertical center.
    kOptCenterY = 1024,
    /// Text/Number: (x, y) is the center.
    kOptCenter = 1536,
    /// Text/Number: x is the right edge.
    kOptRightX = 2048,
    /// Gauge: no background.
    kOptNoBack = 4096,
    /// Gauge: no tick marks.
    kOptNoTicks = 8192,
    /// Gauge: no pointer.
    kOptNoPointer = 16384
  };

  /// A touch position in pixels (after calibration).
  struct TouchPoint {
    int x;
    int y;
  };

  /**
   * Opens the SPI device (mode 0) and the PD (and INT) pins and starts the thread. Does not
   * throw for a missing chip: IsReady() stays false and one console line says why.
   *
   * @param bus     The SPI bus (prosdk::platform::SetSPIPins sets its pins).
   * @param csGpio  The chip-select GPIO (the board's CS).
   * @param pdGpio  DIO channel (GPIO) wired to the board's PD pin (power-down, low = off), or -1
   *                when PD is not wired (the board then needs a power cycle to reset the chip).
   * @param panel   The panel timing, for example ft8xx::kPanel800x480.
   * @param intGpio DIO channel (GPIO) wired to the board's INT pin, or -1. When set, the touch
   *                registers are read only after the chip signals a touch.
   * @param clockHz The SPI clock after start-up, limited to SPI::kMinClockRate-kMaxClockRate.
   */
  FT8xx(prosdk::SPI::Port bus, int csGpio, int pdGpio, const FT8xxPanel& panel,
        int intGpio = -1, int clockHz = kDefaultClockRate);

  /** Stops the thread. The display keeps showing the last frame. */
  ~FT8xx() override;

  FT8xx(const FT8xx&) = delete;
  FT8xx& operator=(const FT8xx&) = delete;

  /** True when the chip answered and the panel is configured. */
  bool IsReady() const;

  /** The controller detected at the last start-up. */
  Chip GetChip() const;

  /** The panel timing in use. */
  const FT8xxPanel& GetPanel() const;

  /** Sets the backlight, 0 (off) to 1 (full, the default). Applied by the thread. */
  void SetBacklight(double brightness);

  // Frame building. Each call appends to the frame under construction; the first one starts a
  // new display list (CMD_DLSTART). Display-list commands go through the co-processor FIFO,
  // which copies them into the display list (FT81X PG 5.3).

  /** Sets the color used by Clear() (CLEAR_COLOR_RGB). The initial value is black. */
  void ClearColorRGB(uint8_t red, uint8_t green, uint8_t blue);

  /** Clears the color, stencil and tag buffers (CLEAR); start every frame with it. */
  void Clear(bool color = true, bool stencil = true, bool tag = true);

  /** Sets the drawing color (COLOR_RGB). The initial value is white. */
  void ColorRGB(uint8_t red, uint8_t green, uint8_t blue);

  /** Point radius in pixels, 0 to 511 in 1/16 steps (POINT_SIZE). The initial value is 1. */
  void PointSize(double radius);

  /** Line half-width in pixels, 0 to 255 in 1/16 steps (LINE_WIDTH). The initial value is 1. */
  void LineWidth(double width);

  /**
   * Tag 1-255 for the objects drawn next (TAG); GetTouchTag() reports it when they are touched.
   * Objects drawn before the first Tag() of a frame carry 255, TAG's initial value (programmer
   * guides, TAG). Tag(0) writes the cleared tag buffer's value, so the objects after it read as
   * untagged (unverified: the guides give 1-255 as the valid range).
   */
  void Tag(uint8_t tag);

  /** Starts drawing a primitive (BEGIN); every vertex after it is one point, line end, ... */
  void Begin(Primitive primitive);

  /** One vertex in pixels, -1024 to 1023 in 1/16 steps (VERTEX2F). */
  void Vertex2f(double x, double y);

  /** Ends the primitive (END). */
  void End();

  /** Foreground color of widgets, 0xRRGGBB (CMD_FGCOLOR): buttons, knobs. */
  void FgColor(uint32_t rgb);

  /** Background color of widgets, 0xRRGGBB (CMD_BGCOLOR): gauge face. */
  void BgColor(uint32_t rgb);

  /**
   * Text (CMD_TEXT) in the drawing color. (x, y) is the top-left corner unless options say
   * otherwise. The text ends at its first NUL character.
   */
  void Text(int x, int y, int font, uint16_t options, std::string_view text);

  /** A button of w x h pixels with a label (CMD_BUTTON); kOptFlat removes the 3D look. */
  void Button(int x, int y, int w, int h, int font, uint16_t options, std::string_view label);

  /** A decimal number (CMD_NUMBER); kOptSigned for negative values. */
  void Number(int x, int y, int font, uint16_t options, int32_t value);

  /**
   * A round gauge of radius r (CMD_GAUGE): `major` divisions with `minor` ticks each, the
   * pointer at value (0 to range).
   */
  void Gauge(int x, int y, int r, uint16_t options, int major, int minor, int value, int range);

  /**
   * Ends the frame (DISPLAY, CMD_SWAP) and hands it to the thread, which shows it at the next
   * frame boundary. A frame over kMaxFrameBytes is discarded with one console line.
   */
  void Swap();

  /** The latest touch position, or nullopt when the screen is not touched. */
  std::optional<TouchPoint> GetTouch() const;

  /**
   * The tag of the touched object (REG_TOUCH_TAG): its Tag() value, 255 for an object drawn
   * before any Tag() in the frame, 0 for the cleared background or when nothing is touched.
   */
  int GetTouchTag() const;

  /**
   * Starts the touch calibration (CMD_CALIBRATE): the screen asks to tap three dots. While it
   * runs no frame is sent, so call it only for a panel with resistive touch (it never ends on a
   * panel without). GetTouchTransform() returns the result afterwards; store it and pass it to
   * SetTouchTransform() at the next start to skip the calibration. A calibration the chip
   * reports as failed (result 0) prints one console line and leaves the transform unchanged.
   */
  void CalibrateTouch();

  /** True from CalibrateTouch() until the three dots were tapped (or the calibration failed). */
  bool IsCalibrating() const;

  /**
   * The touch transform (REG_TOUCH_TRANSFORM_A-F, s15.16) of the last calibration or
   * SetTouchTransform(), or nullopt when there was none.
   */
  std::optional<std::array<int32_t, 6>> GetTouchTransform() const;

  /** Writes a stored touch transform (applied now and after every start-up). */
  void SetTouchTransform(const std::array<int32_t, 6>& transform);

  /** Co-processor faults recovered so far (REG_CMD_READ = FFFh, FT81X PG 5.6). */
  int GetCoprocessorFaults() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  struct Map;  // register addresses of one chip
  static const Map kMapFT800;
  static const Map kMapFT81x;

  void Run();
  bool BringUp();
  void Service();
  bool Wait(std::chrono::milliseconds duration, bool wakeOnSwap = false);
  bool Fail(const std::string& problem);

  bool HostCommand(uint8_t command, uint8_t parameter = 0);
  bool Read(uint32_t address, uint8_t* data, int size);
  bool Write(uint32_t address, const uint8_t* data, int size);
  bool Read8(uint32_t address, uint8_t* value);
  bool Read32(uint32_t address, uint32_t* value);
  bool Write8(uint32_t address, uint8_t value);
  bool Write16(uint32_t address, uint16_t value);
  bool Write32(uint32_t address, uint32_t value);
  bool SendCommands(const std::vector<uint8_t>& stream);
  bool ReadCommandRead(uint32_t* read);
  bool RecoverCoprocessor();
  void ReadTouch();

  void AppendWord(uint32_t word);
  void AppendHalf(int value);
  void AppendString(std::string_view text);
  void StartFrameIfEmpty();

  prosdk::SPI m_spi;
  int m_clockHz;
  FT8xxPanel m_panel;
  std::optional<prosdk::DigitalOutput> m_pd;
  std::optional<prosdk::DigitalInput> m_int;

  // thread only
  const Map* m_map = nullptr;
  uint32_t m_cmdWrite = 0;  // our REG_CMD_WRITE (offset into RAM_CMD)
  int m_appliedDuty = -1;
  int m_appliedTransformVersion = 0;
  bool m_touchActive = false;
  bool m_calibrationSent = false;
  std::string m_problem;  // the last problem printed

  // frame under construction (caller)
  std::mutex m_buildMutex;
  std::vector<uint8_t> m_building;
  bool m_buildOverflow = false;
  bool m_overflowReported = false;

  mutable std::mutex m_mutex;  // the fields below
  bool m_ready = false;
  Chip m_chip = Chip::kNone;
  std::optional<std::vector<uint8_t>> m_pending;
  bool m_calibrate = false;  // requested or running
  std::optional<std::array<int32_t, 6>> m_transform;
  int m_transformVersion = 0;  // bumped by SetTouchTransform
  std::optional<TouchPoint> m_touch;
  int m_touchTag = 0;
  int m_framesSent = 0;
  int m_framesDropped = 0;
  int m_faults = 0;

  std::atomic<int> m_duty;  // REG_PWM_DUTY, 0-128

  std::mutex m_waitMutex;
  std::condition_variable m_wake;
  bool m_swapped = false;  // under m_waitMutex: a frame arrived
  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
