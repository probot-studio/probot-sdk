// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (d) Boardoza driver decode paths that are pure computation (no bus): the NMEA and UBX parsers,
// the Melexis MLX90640 library (parameters from the EEPROM, temperatures of a whole frame), the
// BME688 and MS5611 compensation formulas, the TMAG5170 frame CRC, the OPT4048 channel decode and
// CRC, and the VL53L7CX platform byte swap. Canned inputs come from the host tests under
// test/prosdk/drivers. Drivers left out, and why: design/qemu-bench.md.

#include <stdint.h>

#include <array>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <prosdk/drivers/environment/BME688.hpp>
#include <prosdk/drivers/environment/MS5611.hpp>
#include <prosdk/drivers/gnss/Nmea.hpp>
#include <prosdk/drivers/gnss/Ubx.hpp>
#include <prosdk/drivers/light/OPT4048.hpp>
#include <prosdk/drivers/magnetic/TMAG5170.hpp>
#include <prosdk/drivers/range/st/platform.h>
#include <prosdk/drivers/thermal/melexis/MLX90640_API.h>

#include "Bench.hpp"

namespace qemu::bench {

namespace {

namespace mlx = prosdk::melexis;

void Gnss() {
  // NmeaTest.cpp: the M10 interface description's GGA and RMC examples, and a GSA body
  const std::string epoch =
      std::string{"$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B\r\n"
                  "$GNRMC,014230.00,A,4722.80340,N,00831.68218,E,0.000,,120477,,,A,V*14\r\n"} +
      prosdk::nmea::Encode("GNGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54,1");
  prosdk::nmea::Reader reader;
  int fixes = 0;
  // one epoch: split the byte stream, check the checksums, decode the three sentences
  Run("nmea_epoch_gga_rmc_gsa", 1000, [&] {
    for (char c : epoch) {
      if (!reader.Push(static_cast<uint8_t>(c))) {
        continue;
      }
      std::string_view body = reader.Body();
      std::string_view type = prosdk::nmea::SentenceType(body);
      if (type == "GGA") {
        auto gga = prosdk::nmea::ParseGga(body);
        fixes += gga && gga->quality > 0;
        DoNotOptimize(gga);
      } else if (type == "RMC") {
        auto rmc = prosdk::nmea::ParseRmc(body);
        DoNotOptimize(rmc);
      } else if (type == "GSA") {
        auto gsa = prosdk::nmea::ParseGsa(body);
        DoNotOptimize(gsa);
      }
    }
    DoNotOptimize(fixes);
  });

  // UbxTest.cpp SamplePvt(): one NAV-PVT frame through the parser and the decoder
  prosdk::ubx::NavPvt pvt;
  pvt.iTow = 123456789;
  pvt.year = 2026;
  pvt.month = 10;
  pvt.day = 22;
  pvt.hour = 13;
  pvt.minute = 45;
  pvt.second = 59;
  pvt.valid = 0x07;
  pvt.nano = -250000;
  pvt.fixType = 3;
  pvt.flags = 0x01;
  pvt.numSv = 11;
  pvt.lon = 289784210;
  pvt.lat = 410082560;
  pvt.height = 75123;
  pvt.hMsl = 37456;
  pvt.hAcc = 1830;
  pvt.vAcc = 2950;
  pvt.velN = -120;
  pvt.velE = 340;
  pvt.velD = -5;
  pvt.gSpeed = 361;
  pvt.headMot = 10955000;
  pvt.pDop = 145;
  const auto payload = prosdk::ubx::EncodeNavPvt(pvt);
  const std::vector<uint8_t> frame =
      prosdk::ubx::Encode(prosdk::ubx::kClassNav, prosdk::ubx::kIdNavPvt, payload);
  prosdk::ubx::Parser parser;
  int32_t lat = 0;
  Run("ubx_navpvt_frame", 2000, [&] {
    for (uint8_t b : frame) {
      if (parser.Push(b)) {
        auto msg = parser.GetMessage();
        auto decoded = prosdk::ubx::DecodeNavPvt(msg.payload);
        lat += decoded ? decoded->lat : 0;
      }
    }
    DoNotOptimize(lat);
  });
}

// MLX90640Test.cpp: the datasheet (rev 12, section 11.2) calculation example. EEPROM words
// 0x2410-0x243F, 0x240A bit 11 (calibrated in interleaved mode), per-pixel word 0x08A0; RAM: raw
// pixels 0x0261 and the auxiliary words of the example; control register 0x0901.
constexpr std::array<uint16_t, 48> kEeprom2410 = {
    0x4210, 0xFFBB, 0x0202, 0xF202, 0xF2F2, 0xE2E2, 0xD1E1, 0xB1D1,  // 0x2410
    0xF10F, 0xF00F, 0xE0EF, 0xE0EF, 0xE1E1, 0xF3F2, 0xF404, 0xE504,  // 0x2418
    0x79A6, 0x2F44, 0xFFDD, 0x2210, 0x3333, 0x2233, 0xEF01, 0x9ACC,  // 0x2420
    0xEEDC, 0x10FF, 0x2221, 0x3333, 0x2333, 0x0112, 0xEEFF, 0xBBDD,  // 0x2428
    0x18EF, 0x2FF1, 0x5952, 0x9D68, 0x5454, 0x0994, 0x6956, 0x5354,  // 0x2430
    0x2363, 0xE446, 0xFBB5, 0x044B, 0xF020, 0x9797, 0x9797, 0x2889,  // 0x2438
};

struct Mlx90640Data {
  uint16_t eeprom[832];  // 0x2400..0x273F
  uint16_t frame[834];   // 768 pixels, 64 auxiliary words, control register, subpage
  mlx::paramsMLX90640 params;
  float to[768];
};

void Thermal() {
  auto data = std::make_unique<Mlx90640Data>();
  Mlx90640Data& d = *data;  // value-initialized: all zero
  for (size_t i = 0; i < kEeprom2410.size(); ++i) {
    d.eeprom[0x10 + i] = kEeprom2410[i];
  }
  d.eeprom[0x0A] = 0x0800;
  for (int p = 0; p < 768; ++p) {
    d.eeprom[0x40 + p] = 0x08A0;
    d.frame[p] = 0x0261;
  }
  d.frame[768 + 0x00] = 0x4BF2;  // VBE
  d.frame[768 + 0x08] = 0xFFCA;  // CP subpage 0
  d.frame[768 + 0x28] = 0xFFC8;  // CP subpage 1
  d.frame[768 + 0x0A] = 0x1881;  // gain
  d.frame[768 + 0x20] = 0x06AF;  // PTAT
  d.frame[768 + 0x2A] = 0xCCC5;  // VDD
  d.frame[832] = 0x0901;         // control register 1: interleaved, 18 bit, 2 Hz

  int error = 0;
  // once per camera start (MLX90640::Update after a reset)
  Run("mlx90640_extract_parameters", 20, [&] {
    error |= mlx::MLX90640_ExtractParameters(d.eeprom, &d.params);
    DoNotOptimize(d.params);
  });
  // one full frame: Ta, then To of both subpages (768 pixels), as MLX90640::Update does
  float ta = 0.0f;
  Run("mlx90640_frame_to_768px", 20, [&] {
    for (uint16_t subpage = 0; subpage < 2; ++subpage) {
      d.frame[833] = subpage;
      ta = mlx::MLX90640_GetTa(d.frame, &d.params);
      mlx::MLX90640_CalculateTo(d.frame, &d.params, 1.0f, ta - 8.0f, d.to);
    }
    DoNotOptimize(d.to);
  });
  // the datasheet example: Ta 39.184 C, pixel 367 80.18 C (MLX90640Test.cpp)
  Linef("BENCH-CHECK mlx90640 extract_error={} ta={:.3f} to367={:.2f}", error, ta, d.to[367]);
}

void Environment() {
  // BME688Test.cpp: calibration and the ADC values of field 0
  constexpr prosdk::BME688::Calibration kCal = {
      .t1 = 25829, .t2 = 26431, .t3 = 3,     .p1 = 36379, .p2 = -10417, .p3 = 88,
      .p4 = 6936,  .p5 = -48,   .p6 = 30,    .p7 = 33,    .p8 = -1350,  .p9 = -2768,
      .p10 = 30,   .h1 = 751,   .h2 = 1022,  .h3 = 0,     .h4 = 45,     .h5 = 20,
      .h6 = 120,   .h7 = -100,  .g1 = -53,   .g2 = -11536, .g3 = 18,    .resHeatRange = 1,
      .resHeatVal = 40};
  uint32_t adc = 0;
  double sum = 0.0;
  // one sample: temperature, pressure, humidity, gas resistance (double precision formulas)
  Run("bme688_compensate_sample", 2000, [&] {
    adc = (adc + 1) & 0xFF;
    double tFine = 0.0;
    double t = prosdk::BME688::CompensateTemperature(kCal, 500000 + adc, tFine);
    double p = prosdk::BME688::CompensatePressure(kCal, 400000 + adc, tFine);
    double h = prosdk::BME688::CompensateHumidity(kCal, 25000 + adc, t);
    double g = prosdk::BME688::GasResistance(600 + adc, 5);
    sum += t + p + h + g;
    DoNotOptimize(sum);
  });

  // MS5611Test.cpp: PROM words; first and second order compensation (integer)
  constexpr std::array<uint16_t, 8> kProm = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
  int32_t acc = 0;
  Run("ms5611_compensate_sample", 10000, [&] {
    adc = (adc + 1) & 0xFFFF;
    int32_t temperature = 0;
    int32_t pressure = 0;
    prosdk::MS5611::Compensate(kProm, 9085466 + adc, 8569150 - adc * 16, temperature, pressure);
    acc += temperature + pressure;
    DoNotOptimize(acc);
  });
}

void Sensors() {
  uint32_t frame = 0x0F000400;
  uint32_t crc = 0;
  // the CRC of every 32-bit SPI answer (TMAG5170 datasheet 7.5.2.5)
  Run("tmag5170_crc4_frame", 20000, [&] {
    frame += 0x10;
    crc += prosdk::TMAG5170::Crc4(frame);
    DoNotOptimize(crc);
  });

  uint16_t msb = 0x2123;
  // one channel: mantissa/exponent decode plus the CRC check (OPT4048Test.cpp registers)
  Run("opt4048_decode_channel_crc", 20000, [&] {
    ++msb;
    auto ch = prosdk::OPT4048::DecodeChannel(msb, 0x4552);
    crc += prosdk::OPT4048::ChannelCrc(ch);
    DoNotOptimize(crc);
  });

  // the ULD's byte-order swap of a ranging-data read; the block parse that follows it is inside
  // vl53l7cx_get_ranging_data, behind the I2C read (no bus in QEMU): not measured
  std::vector<uint8_t> buffer(1024);
  for (size_t i = 0; i < buffer.size(); ++i) {
    buffer[i] = static_cast<uint8_t>(i * 7 + 1);
  }
  Run("vl53l7cx_swap_buffer_1k", 2000, [&] {
    VL53L7CX_SwapBuffer(buffer.data(), static_cast<uint16_t>(buffer.size()));
    DoNotOptimize(buffer[0]);
  });
}

}  // namespace

void BenchDrivers() {
  Gnss();
  Thermal();
  Environment();
  Sensors();
}

}  // namespace qemu::bench
