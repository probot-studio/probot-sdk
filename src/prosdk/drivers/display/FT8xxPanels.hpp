// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Panel timing presets for prosdk::FT8xx (design/boardoza/P13-eve.md). The FT800/FT810 drive a
// parallel RGB TFT; these values go into the chip's video timing registers (FT800 Series
// Programmer Guide BRT_000030 v2.1 section 2.2, FT81X Series Programmers Guide BRT_000031 v1.2
// section 2.3). They depend on the panel, not on the chip: take them from the panel's datasheet
// when it is not one of the presets below.

#pragma once

namespace prosdk {

/**
 * Video timing of one TFT panel, in the units of the FT8xx timing registers (PCLK cycles for the
 * horizontal values, lines for the vertical ones).
 */
struct FT8xxPanel {
  /// Visible pixels per line (REG_HSIZE). FT800: at most 512, FT81x: at most 800.
  int width;
  /// Visible lines (REG_VSIZE). FT800: at most 512, FT81x: at most 600.
  int height;
  /// Total PCLK cycles per line (REG_HCYCLE).
  int hcycle;
  /// PCLK cycles from the line start to the first visible pixel (REG_HOFFSET).
  int hoffset;
  /// HSYNC falling edge, in PCLK cycles from the line start (REG_HSYNC0).
  int hsync0;
  /// HSYNC rising edge (REG_HSYNC1).
  int hsync1;
  /// Total lines per frame (REG_VCYCLE).
  int vcycle;
  /// Lines from the frame start to the first visible line (REG_VOFFSET).
  int voffset;
  /// VSYNC falling edge, in lines (REG_VSYNC0).
  int vsync0;
  /// VSYNC rising edge (REG_VSYNC1).
  int vsync1;
  /// Pixel clock = system clock / pclkDivider (REG_PCLK). With the boards' 12 MHz crystal the
  /// system clock is 48 MHz on the FT800 and 60 MHz on the FT81x (data sheets section 4.2.4).
  int pclkDivider;
  /// 0: data clocked out on the rising PCLK edge, 1: on the falling edge (REG_PCLK_POL).
  int pclkPolarity;
  /// RGB output pin order (REG_SWIZZLE, 0 = as wired on the Boardoza boards).
  int swizzle;
  /// 1 staggers the color outputs around the PCLK edge to reduce noise (REG_CSPREAD).
  int cspread;
  /// 1 dithers the output to the 6-bit panel colors (REG_DITHER).
  int dither;
};

namespace ft8xx {

/**
 * 480x272 (WQVGA, 4.3 inch class) at about 60 Hz on the FT800 (PCLK 48 MHz / 5 = 9.6 MHz). These
 * are the example values of both programmer guides (FT800 PG 2.2.4/2.2.5, FT81X PG 2.3). On an
 * FT81x the same divider gives 12 MHz and about 75 Hz, as in the FT81X guide's example; check
 * the panel's maximum pixel clock.
 */
inline constexpr FT8xxPanel kPanel480x272{
    .width = 480,
    .height = 272,
    .hcycle = 548,
    .hoffset = 43,
    .hsync0 = 0,
    .hsync1 = 41,
    .vcycle = 292,
    .voffset = 12,
    .vsync0 = 0,
    .vsync1 = 10,
    .pclkDivider = 5,
    .pclkPolarity = 1,
    .swizzle = 0,
    .cspread = 1,
    .dither = 1,
};

/**
 * 800x480 (WVGA, 5 and 7 inch class), FT81x only: PCLK 60 MHz / 2 = 30 MHz, 1056 x 525 cycles,
 * about 54 Hz. Unverified: the values are those of the Boardoza FT810 library (repo
 * Boardoza_FT810_TFT_Driver_Breakout_Board), not from a panel datasheet; Ozdisan does not name
 * the bundled panels.
 */
inline constexpr FT8xxPanel kPanel800x480{
    .width = 800,
    .height = 480,
    .hcycle = 1056,
    .hoffset = 46,
    .hsync0 = 0,
    .hsync1 = 40,
    .vcycle = 525,
    .voffset = 23,
    .vsync0 = 0,
    .vsync1 = 1,
    .pclkDivider = 2,
    .pclkPolarity = 1,
    .swizzle = 0,
    .cspread = 0,
    .dither = 1,
};

/**
 * The Ozdisan FT800 bundles (FT800 board with a 4.3 inch TFT-1, TFT-2 or TFT-3, or a 5 inch
 * TFT-1; Ozdisan 1585930-1585933). The FT800 drives at most 512x512, so these panels are taken
 * to be 480x272. Unverified: the panel part numbers and timings are not published. Per the
 * Ozdisan names none has resistive touch (TFT-3 is "cap. touch", the others "no touch"), so the
 * driver reports no touches on them.
 */
inline constexpr FT8xxPanel kBoardozaFT800Bundle = kPanel480x272;

/**
 * The Ozdisan FT810 bundles (FT810 board with a 5 inch TFT-1 or a 7 inch TFT-1, TFT-2 or TFT-3;
 * Ozdisan 1585934-1585936, 1585938), taken to be 800x480 as in the Boardoza FT810 library.
 * Unverified: the panel part numbers and timings are not published. Per the Ozdisan names the
 * 5 inch TFT-1 and 7 inch TFT-2 have resistive touch, the 7 inch TFT-1 none and the 7 inch TFT-3
 * capacitive touch (not supported).
 */
inline constexpr FT8xxPanel kBoardozaFT810Bundle = kPanel800x480;

}  // namespace ft8xx

}  // namespace prosdk
