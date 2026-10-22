# P13: FT8xx EVE display (FT800, FT810)

Package P13 of the Boardoza plan. One class, `prosdk::FT8xx`, drives the Bridgetek (FTDI) EVE
graphics controllers on the two Boardoza TFT driver boards and the Ozdisan bundles of those
boards with a panel. Panel timings are presets in `FT8xxPanels.hpp`.

| Board | Ozdisan id | Chip | Panel preset | Touch (Ozdisan name) |
|---|---|---|---|---|
| FT800 TFT Driver Breakout Board (B-FT800-BRK-R1.0) | (board alone) | FT800Q | user-supplied | resistive, if the panel has it |
| FT810 TFT Driver Breakout Board (B-FT810-BRK-R1.0) | (board alone) | FT810Q | user-supplied | resistive, if the panel has it |
| FT800 board + 4.3 inch TFT-1, 4.3 inch TFT-2, 5 inch TFT-1 | 1585930, 1585931, 1585933 | FT800 | `kBoardozaFT800Bundle` = 480x272 (unverified) | none |
| FT800 board + 4.3 inch TFT-3 | 1585932 | FT800 | `kBoardozaFT800Bundle` (unverified) | capacitive: not supported |
| FT810 board + 5 inch TFT-1, 7 inch TFT-2 | 1585934, 1585936 | FT810 | `kBoardozaFT810Bundle` = 800x480 (unverified) | resistive |
| FT810 board + 7 inch TFT-1 | 1585935 | FT810 | `kBoardozaFT810Bundle` (unverified) | none |
| FT810 board + 7 inch TFT-3 | 1585938 | FT810 | `kBoardozaFT810Bundle` (unverified) | capacitive: not supported |

Ozdisan does not name the bundled panels or publish their timings. The FT800 drives at most
512x512, so its bundles are taken to be 480x272; the FT810 bundles are taken to be 800x480 as in
the Boardoza FT810 library. The touch column comes from the Ozdisan product names only. On the
capacitive ("cap. touch") bundles the display works but touch does not: the FT800 and FT810
have no capacitive touch engine, and the FT800 board's J9 "capacitive touch I2C" header is not
documented (unverified, no support). On a bundle without resistive touch `GetTouch()` stays
empty and `CalibrateTouch()` must not be called (it would wait for taps forever).

Sources (clean-room; no vendor or Boardoza code is copied):
- FT800 Embedded Video Engine Datasheet v1.4 (BRT_000039).
- FT81X Embedded Video Engine Datasheet v1.2 (FT_001165).
- FT800 Series Programmer Guide v2.1 (BRT_000030), FT81X Series Programmers Guide v1.2
  (BRT_000031).
- The Boardoza board READMEs and KiCad schematics (research records in the Boardoza plan). The
  800x480 timing values are taken from the Boardoza FT810 library (CC BY-SA); only the numbers are
  used, not code.

## Wiring and 5 V hazards

| Header | Pins |
|---|---|
| J1 (2.50 mm) | 1 3V3 (output), 2 5V (input), 3 GND, 4 PD, 5 INT |
| J6 (SPI) | 1 CS, 2 MISO, 3 MOSI, 4 SCK |
| J7 | auxiliary 5 V input |
| J2 | 40-pin FPC to the panel (RGB666, PCLK, DISP, sync, backlight, 4-wire resistive touch) |
| J3, J5 | speaker, audio line out (not used) |

- Supply: **5 V** on J1 pin 2 (or J7). The README gives about 150 mA plus the backlight; the
  backlight boost driver runs from the same supply, so a 7 inch panel draws noticeably more
  (not measured).
- The chip runs at 3.3 V. CS, SCK, MOSI and PD reach it through 74LCX125 buffers with 5 V tolerant
  inputs, so 3.3 V or 5 V hosts both work. The ESP32-S3 pins connect directly.
- MISO and INT are 3.3 V outputs: safe for the ESP32-S3.
- **J1 pin 1 (3V3) is an output.** Do not connect it to the controller's 3.3 V rail or feed it;
  leave it open.
- INT_N: open drain on the FT800 (DS pin table: pull up 1-10 kOhm to VCCIO) and open drain by
  default on the FT81x (DS 4.1.6). Whether the boards fit the pull-up is unverified. A prosdk
  `DigitalInput` on the ESP32-S3 enables the internal pull-up (design/hal-esp32.md, DIO), which
  is enough for the driver's 10 ms polling; if INT reads low all the time, wire a 10 kOhm pull-up
  to 3.3 V or pass `intGpio = -1`.
- PD is the chip's PD_N (low = power down). The driver pulses it low to reset the chip. Without
  PD wired (`pdGpio = -1`) a hung chip needs a power cycle.
- **E-stop**: prosdk drives every DIO output low on an e-stop and refuses later sets. PD goes
  low, the chip powers down and the display stays dark until the robot restarts.
- Bus sharing: whether the board releases MISO while CS is high (74LCX125 output enable) is
  unverified. Put the display alone on its SPI bus, or check before sharing.

## Class

`src/prosdk/drivers/display/FT8xx.hpp`, `.cpp`; presets in `FT8xxPanels.hpp`.

```cpp
prosdk::FT8xx display{prosdk::SPI::kPort0, /*csGpio=*/10, /*pdGpio=*/4,
                      prosdk::ft8xx::kBoardozaFT810Bundle, /*intGpio=*/5};
display.ClearColorRGB(10, 20, 40);
display.Clear();
display.Text(400, 30, 30, prosdk::FT8xx::kOptCenter, "Hello");
display.Swap();
```

- Constructor `FT8xx(SPI::Port bus, int csGpio, int pdGpio, const FT8xxPanel& panel,
  int intGpio = -1, int clockHz = kDefaultClockRate)`. It never blocks and never throws for a
  missing chip. It opens the SPI device (mode 0, the only mode the chips accept) and the PD
  output (driven high) and the optional INT input, then starts the thread `ft8xx`.
- **Deviation from the plan:** there is no `Begin()` power-up call. The thread performs the
  start-up and repeats it about once a second while no chip answers, or after the chip was lost
  (REG_ID no longer 7Ch). `IsReady()` tells when the display is up; `GetChip()` returns `kFT800`
  or `kFT81x` from the chip ID.
- Start-up (programmer guides 2.2.5 / 2.3, data sheets 4.2.3 / 4.8):
  1. SPI at 10 MHz (FT800 DS 4.2.3: at most 11 MHz until the system clock runs).
  2. PD low 20 ms (at least 5 ms, FT800 DS 4.8.2), then high and 20 ms.
  3. Host commands CLKEXT (44h, the boards have a 12 MHz crystal), then ACTIVE (00h).
  4. Poll every 10 ms, up to 400 ms (boot takes up to 300 ms, FT81X DS 4.8.4): the chip ID at
     0C0000h (08h 00h FT800, 08h 10h-13h FT81x) and REG_ID = 7Ch.
  5. Panel check against the chip (FT800 512x512, FT81x 800x600); a panel that does not fit
     leaves the driver not ready, with one console line.
  6. REG_PCLK 0, REG_PWM_DUTY 0, the timing registers, a first display list (CLEAR_COLOR_RGB 0,
     CLEAR, DISPLAY), REG_DLSWAP = 2, GPIO bit 7 (DISP) as output and high, REG_PCLK = divider.
  7. Co-processor reset with empty FIFO (REG_CPURESET 1, CMD_READ 0, CMD_WRITE 0, CPURESET 0),
     INT mask TOUCH | TAG and INT_EN when INT is wired, the stored touch transform, then the SPI
     clock `clockHz` (default 20 MHz, at most 30 MHz).
- Frame building, from one caller thread (the robot loop): `ClearColorRGB`, `Clear`, `ColorRGB`,
  `PointSize`, `LineWidth`, `Tag`, `Begin(Primitive)`, `Vertex2f`, `End` (display-list words),
  `FgColor`, `BgColor`, `Text`, `Button`, `Number`, `Gauge` (co-processor widgets). The first
  call of a frame inserts CMD_DLSTART; `Swap()` appends DISPLAY and CMD_SWAP and hands the
  frame to the thread. Coordinates are pixels (VERTEX2F in 1/16 pixel); fonts are ROM fonts
  16-31. Strings end at their first NUL and are NUL-terminated and padded to 4 bytes.
- Frames: the thread sends the latest swapped frame when the co-processor is idle (REG_CMD_READ
  = our write pointer). A frame swapped while the previous one still waits replaces it (counted
  in "Dropped frames"). A frame is at most `kMaxFrameBytes` = 8192 bytes of command stream; a
  larger one is discarded at `Swap()` with one console line. The FIFO is written in pieces that
  never cross the end of the 4 KiB ring and never fill more than 4092 bytes (PG 5.1.1); REG_CMD_WRITE
  is moved after each piece. Redraw at 10-20 Hz.
- Co-processor fault (REG_CMD_READ = FFFh, PG 5.6): the thread resets the co-processor as above,
  counts it in `GetCoprocessorFaults()` and prints one line; the next frame starts clean.
- Backlight: `SetBacklight(0..1)` sets REG_PWM_DUTY 0-128 (default full).
- Touch: `GetTouch()` returns the REG_TOUCH_SCREEN_XY position (nullopt at 8000h, not touched);
  `GetTouchTag()` returns REG_TOUCH_TAG: the `Tag()` value of the touched object, 255 for an
  object drawn before any `Tag()` in the frame (TAG's initial value, programmer guides TAG), 0
  for the cleared background, after `Tag(0)`, or when nothing is touched. Read every 10 ms;
  with INT wired, only after INT_N went low or while a touch is active, and REG_INT_FLAGS is read
  to clear it. The touch engine is left at its reset settings.
- Calibration: `CalibrateTouch()` sends a prompt and CMD_CALIBRATE from the thread (no frames
  meanwhile); `IsCalibrating()` is true until the three taps are done, then
  `GetTouchTransform()` holds REG_TOUCH_TRANSFORM_A-F. When the chip writes 0 into CMD_CALIBRATE's
  result word (failure, PG 5.52) the driver prints one line, keeps the previous transform and
  writes a stored one to the chip again. `SetTouchTransform()` writes a stored
  transform now and after every start-up.
- Telemetry type `Display`: Ready, Frames, Dropped frames, Faults, Touched, Touch X, Touch Y,
  Touch tag.
- `kPanel480x272`: the programmer guides' example values (HCYCLE 548, HOFFSET 43, HSYNC1 41,
  VCYCLE 292, VOFFSET 12, VSYNC1 10, PCLK 5, PCLK_POL 1, CSPREAD 1, DITHER 1). On the FT81x the
  same divider gives a 12 MHz pixel clock; check the panel.
- `kPanel800x480`: HCYCLE 1056, HOFFSET 46, HSYNC1 40, VCYCLE 525, VOFFSET 23, VSYNC1 1, PCLK 2
  (30 MHz), PCLK_POL 1, CSPREAD 0, DITHER 1; FT81x only.

Example: `examples/BoardozaEveDisplay/` (battery number and gauge, robot state, a tagged button,
the touch point; touch calibration is off by default and switched on with `kCalibrateTouch` for a
resistive-touch panel).

## Verified and unverified

Verified against the documents (cited in the source): the SPI framing (read 00b + 22-bit
address + dummy byte, write 10b + address, host command 3 bytes), the host command codes, the
memory maps and register addresses of both chips, the chip ID of the FT81x, REG_ID 7Ch, the
start-up order, the display-list and co-processor encodings and option values, the FIFO rules
and fault recovery, the interrupt flag bits, REG_PWM_DUTY 0-128, the 8000h "not touched" value.

Covered by host-simulation tests (`test/prosdk/drivers/FT8xxTest.cpp`, an EVE model behind
FakeSPIDevice). They are written and syntax-checked but not yet compiled into the test binary or
run, because no build ran in this package; nothing here counts as verified until they pass. The
cases: FT810 start-up order
and register values, FT800 memory map, a panel too large for the FT800, frame bytes against
hand-assembled words, long frames across the ring end, the frame size limit, fault recovery,
touch and tag, INT gating, calibration (success and a failed result) and a stored transform, a
missing chip and a fast destructor.

Unverified (marked in the code):
- Nothing has run on hardware.
- The bundled panels and both bundle presets; the 800x480 timing values.
- The FT800 chip ID byte order (DS gives "0800"; 08h at 0C0000h, 00h at 0C0001h is assumed).
- Color byte order of CLEAR_COLOR_RGB and COLOR_RGB: the guides' bit diagrams label bits 15-8
  "Blue" and 7-0 "Green"; red, green, blue (high to low) is used, as the parameter order and
  CMD_FGCOLOR's 0xRRGGBB suggest.
- The 20 MHz default SPI clock through the board buffers and flying leads.
- The INT_N pull-up on the boards; MISO release while CS is high.
- The touch engine at reset settings (no RZTHRESH tuning) with the bundle panels.
- The touch type of each bundle (taken from the Ozdisan product names only).
- `Tag(0)` to end tagging: the guides give 1-255 as TAG's valid range; 0 is the value CLEAR
  writes to the tag buffer, so the objects read as untagged.
- The boards' AUDIO_SHDN line and the chip's MODE strap; audio is not supported.

## Resource use

- Pins: one SPI bus (shared only if MISO is released, see above), CS, PD, optional INT.
- One thread (`ft8xx`, default stack). Idle traffic every 10 ms: REG_ID (5 bytes), REG_CMD_READ
  (8 bytes), touch (8 + 5 bytes, skipped with INT wired and no touch); about 3 KB/s.
- A frame of N bytes costs N + 3 bytes per piece plus pointer updates; 8 KiB at 20 MHz takes about
  3.5 ms of bus time on the thread, none on the caller.
- Memory: the frame under construction and the pending frame, each up to 8 KiB (heap), plus the
  thread stack. Callers never wait on SPI.
- Console: one line per new problem (no chip, panel not supported, chip lost, fault, failed
  touch calibration, oversized frame once).
