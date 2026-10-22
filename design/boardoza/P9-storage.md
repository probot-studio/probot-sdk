# P9: SPI storage

Package P9 of the Boardoza plan. It adds two flash chip drivers and an SD card mount on the
prosdk SPI bus (F1, `design/boardoza/F1-spi.md`):

- `prosdk::W25QFlash`: Winbond W25Q serial NOR flash (W25Q40CL);
- `prosdk::W25NFlash`: Winbond W25N01GV serial SLC NAND flash, a raw page and block API;
- `prosdk::platform::MountSdCard`: an SD card in SPI mode with a FAT file system, mounted into
  the file system (plan gap G7).

| Board | Ozdisan id | Chip | prosdk | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| W25Q40 | 1206520 | Winbond W25Q40CLSNIG (4 Mbit NOR) | `W25QFlash` | W25Q40CLSNIG_SPI_Flash_Memory_Breakout_Board |
| W25N01GV | 1598830 | Winbond W25N01GVZEIG (1 Gbit SLC NAND) | `W25NFlash` | W25N01GVZEIG_Serial_SLC_NAND_Flash_Breakout_Board |
| MicroSD | 1202712 | microSD socket (no chip) | `platform::MountSdCard` | MicroSD_Breakout_Board |

Files:
- `src/prosdk/drivers/memory/W25QFlash.hpp` and `.cpp`
- `src/prosdk/drivers/memory/W25NFlash.hpp` and `.cpp`
- `src/prosdk/platform/SdCard.hpp` and `.cpp` (platform glob; ESP32 code under
  `PROSDK_TARGET_HW`, a host stub otherwise)
- Tests (host only): `test/prosdk/drivers/W25QFlashTest.cpp`, `W25NFlashTest.cpp`
- Example: `examples/BoardozaSdLog/BoardozaSdLog.ino`

Sources:
- Winbond W25Q40CL datasheet rev. E, 2017-07-28 (in the board repo's assets).
- Winbond W25N01GVxxIG/IT datasheet rev. L, 2018-05-09 (in the board repo's assets).
- ESP-IDF 5.5.4 headers shipped with Arduino-ESP32 3.3.10: `esp_vfs_fat.h`, `driver/sdspi_host.h`,
  `sd_protocol_types.h`, `ffconf.h`, `sdkconfig.h`.
- The three board READMEs, schematics and the MicroSD PCB netlist (as summarized in the plan's
  research records).

No vendor code was used. The Boardoza W25Q40 library (CC BY-SA 4.0) and W25N01GV library
(BSD-3-Clause) were not copied; the drivers are written from the datasheets.

## 1. Wiring and hazards

All three boards share one SPI bus and differ only in the chip select. Set the bus pins once with
`prosdk::platform::SetSPIPins(bus, sck, mosi, miso)` before the first device opens. The SPI2
IO_MUX pins (SCK 12, MOSI 11, MISO 13, CS 10) are a good default. Tie every board's GND to the
controller's GND.

| Board | Connector | Pins |
|---|---|---|
| W25Q40 | J1 | 1 5V (supply in, to the TLV73333 3.3 V regulator), 2 CLK, 3 DI (MOSI), 4 GND |
| W25Q40 | J2 | 1 /CS, 2 DO (MISO), 3 /WP, 4 /HOLD |
| W25N01GV | J1 | 1 5V/3V3 supply in, 2 nHOLD, 3 nWP, 4 GND |
| W25N01GV | J2 | 1 nCS, 2 DO (MISO), 3 CLK, 4 DI (MOSI) |
| MicroSD | J1 | 1 MISO, 2 SCK, 3 MOSI, 4 CS, 5 GND, 6 +5V |

**W25Q40.** The supply pin takes 3.3 to 5 V (README). The signals go straight to the flash, which
is a 3.3 V part (VCC 2.3 to 3.6 V): drive them only from 3.3 V logic, which the ESP32-S3 is. /WP
and /HOLD must be high for standard SPI. The schematic has two 10 kOhm resistors that are probably
pull-ups on /WP and /HOLD, but their nets were not verified, so tie both pins to 3.3 V (not to
5 V). With 3.3 V on the supply pin, the regulator's dropout leaves the flash slightly below 3.3 V;
that is inside 2.3 to 3.6 V, but the datasheet's 104 MHz rating then no longer applies (80 MHz
does), far above the driver's 10 MHz default.

**W25N01GV.** The board has a TXB0106 level shifter between the host pins and the 3.3 V flash.
The README says the board "operates directly from 3.3V or 5V"; the shifter's host-side reference
is assumed to follow the supply pin (**unverified** from the schematic nets). So with 5 V on J1
pin 1 the shifter may drive DO (MISO) at 5 V, and the ESP32-S3 pins are not 5 V tolerant.
**Supply this board from 3.3 V when it is wired to the ESP32-S3.** The flash itself then gets a
little less than 3.3 V from the regulator, inside its 2.7 to 3.6 V range. Tie nWP and nHOLD high
(3.3 V). The board has three 10 kOhm resistors, likely pull-ups on CS, WP and HOLD (nets not
verified). An auto-direction shifter such as the TXB0106 limits the clock and dislikes strong
loads; the reliable clock through it was not measured.

**MicroSD.** The board has no 3.3 V input: it needs 5 V on pin 6 (its regulator feeds the card).
The SPI lines are wired straight to the card with no level shifter, so they are 3.3 V signals;
never drive them from 5 V logic (the README's "compatible with 5V MCUs" is misleading). There are
no pull-ups on the SPI lines and no card-detect line (both from the PCB netlist). `MountSdCard`
enables the ESP32-S3's internal pull-up on MISO; it is weak, and long wires may need an external
10 kOhm pull-up to 3.3 V (**unverified** on hardware). Insert the card before power-up.

## 2. `prosdk::W25QFlash`

```cpp
W25QFlash(SPI::Port bus, int csGpio, int clockHz = 10'000'000);
uint32_t ReadJedecId();            // 0xEF4013 for a W25Q40CL
bool IsConnected() const;          // Winbond ID (EFh) read at construction
uint32_t GetSize() const;          // bytes: 2^(capacity byte of the JEDEC ID); 0 without a chip
bool Read(uint32_t address, std::span<uint8_t> data);
bool Program(uint32_t address, std::span<const uint8_t> data);
bool EraseSector(uint32_t address);   // 4 KiB that contains address
bool EraseBlock64(uint32_t address);  // 64 KiB that contains address
bool EraseChip();
bool IsBusy();
bool WaitReady(units::millisecond_t timeout);
// TelemetryLoggable, type "Flash Memory": Connected, Busy (as last read)
```

- Construction: SPI mode 0 and the given clock (both modes 0 and 3 are supported by the chip),
  then Release Power-down (ABh; a chip left in power-down ignores every other instruction), a
  3 us wait (tRES1), and JEDEC ID (9Fh). A missing chip prints one console line and leaves
  `IsConnected()` false and `GetSize()` 0; every access then returns false. The SPI constructor
  itself throws for a bus without pins or a chip select in use, as `prosdk::SPI` does.
- `Read` uses Fast Read (0Bh, one dummy byte), which is valid at any clock; Read Data (03h) is
  limited to 50 MHz (33 MHz below 2.7 V) and is not used.
- `Program` splits the data on 256-byte page boundaries: a Page Program that runs past the end
  of its page wraps to the page start (datasheet 9.1.19). Each page is a Write Enable (06h,
  checked: WEL must read back as 1), a Page Program (02h) and a wait for BUSY (tPP 0.8 ms maximum,
  10 ms timeout). Programming only clears bits; erase first.
- Erases do not wait: Sector Erase (20h) takes up to 300 ms, Block Erase 64 KiB (D8h) 1 s and Chip
  Erase (C7h) 4 s, which would stall the robot loop. Poll `IsBusy()` or call `WaitReady()`.
  `Read`, `Program` and the erases return false while the chip is busy.
- Ranges are checked against `GetSize()`. The driver does not touch the status registers: the
  block protection bits are non-volatile and their factory default is 0 (nothing protected). A
  chip with protection set ignores programs and erases of the protected range; the driver does
  not detect this.
- No thread. Every chip access holds one mutex; `WaitReady` holds it for up to its timeout.
  `IsConnected`, `GetSize` and the telemetry read atomics and never wait for an access.
- A chip that is still erasing when the controller restarts ignores the JEDEC ID read (only the
  status register answers while BUSY, 8.1.1), so the driver then reports no chip. Construct it
  again after the erase has ended (at most 4 s for a chip erase).
- Other W25Q parts: the instructions are the same across the family, and 2^capacity matches the
  W25Q40CL (13h = 512 KiB). Other capacities are **unverified**; capacities beyond 16 MiB (which
  need 4-byte addresses) give `GetSize()` 0.

## 3. `prosdk::W25NFlash`

```cpp
W25NFlash(SPI::Port bus, int csGpio, int clockHz = 10'000'000);
uint32_t ReadId();                 // 0xEFAA21
bool IsConnected() const;
bool ReadPage(int page, std::span<uint8_t> data, int column = 0);
bool ProgramPage(int page, std::span<const uint8_t> data, int column = 0);
bool EraseBlock(int block);
bool IsBadBlock(int block);
EccStatus GetEccStatus() const;    // kOk, kCorrected, kUncorrectable, kUncorrectableMultiple
bool IsBusy();
// kPageSize 2048, kSpareSize 64, kPagesPerBlock 64, kBlockCount 1024, kPageCount 65536
// TelemetryLoggable, type "Flash Memory": Connected, ECC Status
```

- Construction: SPI mode 0, wait while BUSY (a reset during a program or erase may corrupt data,
  8.2.1), Device Reset (FFh), 500 us (no command is accepted during tRST), JEDEC ID (9Fh with one
  dummy byte: EFh AAh 21h), then SR-1 (Axh) = 00h and SR-2 (Bxh) = ECC-E | BUF (18h), read back.
  After power-up and after a reset the chip sets BP3-BP0 and TB, which protect the whole array
  (7.1.1), so SR-1 must be cleared. BUF = 1 (buffer read mode) is the power-up default of the
  xxIG part on this board; the driver also sets it for an xxIT part. `IsConnected()` is true when
  the ID matched and both registers read back as written. The Write Status Register instruction
  is sent without a Write Enable: 8.2.5 lists the instructions that need WEL (program, erase, bad
  block management), and the status register write is not among them. When `IsConnected()` is
  false, `ReadPage`, `ProgramPage` and `EraseBlock` return false and `IsBadBlock` returns true
  without touching the chip (a missing chip reads as zeros, which are not data).
- `ReadPage`: Page Data Read (13h, 8 dummy clocks, PA[15:8], PA[7:0]), wait for BUSY (tRD 60 us
  maximum with ECC), Fast Read from the buffer (0Bh, column, one dummy byte), then SR-3 (Cxh) for
  the ECC result. column + size may reach 2112 (the spare area is columns 2048 to 2111). It
  returns false for an uncorrectable page (ECC-1 = 1) as well as for range and transfer errors;
  the data is still copied.
- `ProgramPage`: Write Enable (checked), Load Program Data (02h, which sets the rest of the
  buffer to FFh), Program Execute (10h), wait for BUSY (tPP 700 us maximum), check P-FAIL. With the
  ECC on, the chip writes the ECC bytes of the spare area itself and overwrites what was loaded
  there (8.2.11). Figure 2 splits each 16-byte spare group into bytes 0-1 (bad-block marker), 2-3
  (user data II, no ECC), 4-7 (user data I, ECC protected), 8-D (ECC of the group's 512-byte
  sector) and E-F (ECC of bytes 4-D); so only bytes 2 to 7 of each group are free for the user.
  Keep spare byte 0 and main byte 0 of a good block's first page at FFh (the bad-block marker).
- `EraseBlock`: Write Enable, Block Erase (D8h with the block's first page), wait for BUSY (tBE
  10 ms maximum, 30 ms timeout), check E-FAIL. This call blocks for a few milliseconds.
- `IsBadBlock`: the factory marks a bad block with a byte other than FFh at byte 0 of the main
  array and byte 0 of the spare area of its first page (10.2). The driver switches the ECC off
  (SR-2 = BUF only) for this read, so that the ECC engine cannot alter the marker bytes, and
  switches it back on afterwards. Spare bytes 0 and 1 have no ECC protection anyway (figure 2);
  main byte 0 does. The datasheet's flow chart (figure 32) does not state the ECC setting, so
  turning it off is a driver choice. Out-of-range blocks and failed reads count as bad. An erase
  destroys the factory marker, so scan every block before the first erase and keep the list.
- NAND rules the driver does not enforce: program pages of a block in ascending order (8.2.13),
  at most 4 partial programs per page (NoP, 9.6), erase before reprogramming, skip bad blocks.
  Block 0 is guaranteed valid at shipment; at least 1004 of the 1024 blocks are valid (10.1).
  There is no flash translation layer, wear levelling or file system; the on-chip bad-block
  look-up table (A1h, A5h) and the continuous read mode are not used.
- No thread. Every chip access holds one mutex and blocks until the chip is done (up to the
  10 ms of an erase). `IsConnected`, `GetEccStatus` and the telemetry read atomics and never wait.

## 4. `prosdk::platform::MountSdCard`

```cpp
bool MountSdCard(int spiBus, int csGpio, std::string_view mountPoint = "/sd",
                 int hz = kSdCardDefaultClockRate);   // 20 MHz
```

Usage, as in `examples/BoardozaSdLog`:

```cpp
prosdk::platform::SetSPIPins(0, /*sck=*/12, /*mosi=*/11, /*miso=*/13);
if (prosdk::platform::MountSdCard(0, /*csGpio=*/10, "/sd")) {
  prosdk::platform::SetLogDirectory("/sd/logs");   // before DataLogManager starts
}
prosdk::DataLogManager::Start();
```

- ESP32: `detail::RetainSpiBus(bus)` starts the bus if needed (or joins it) and counts the card
  as a bus user; the chip select is claimed in the HAL pin table (`ClaimPin`, owner "SD card"), so
  DIO or another SPI device cannot take it; the MISO pull-up is enabled; then
  `esp_vfs_fat_sdspi_mount` with `SDSPI_HOST_DEFAULT()` on `detail::SpiHostOf(bus)`, the given
  chip select and `max_freq_khz = hz / 1000`. Mount options: never format
  (`format_if_mount_failed = false`), at most 5 open files. On failure every claim is released
  and one console line names the ESP-IDF error. On success one console line gives the card size.
- `hz` is 400 kHz (`SDMMC_FREQ_PROBING`) to 40 MHz (`SDMMC_FREQ_HIGHSPEED`); the default is
  `SDMMC_FREQ_DEFAULT`, 20 MHz (`sd_protocol_types.h`). ESP-IDF identifies the card at the probing
  rate first.
- File systems: FAT16 and FAT32 with long file names (`CONFIG_FATFS_LFN_STACK`, 255 characters).
  exFAT is off in this build (`FF_FS_EXFAT 0` in `ffconf.h`), so a card over 32 GB must be
  reformatted as FAT32. `CONFIG_FATFS_VOLUME_COUNT` is 2: at most two FAT volumes at once.
- The mount lasts until the program ends; there is no unmount call, and the bus and the chip
  select stay held. Mounting the same mount point twice fails.
- Arguments are checked: bus 0 or 1, chip select 0 to 48, a mount point that starts with `/` and
  does not end with one, and the clock range.
- Files are reached through the C and C++ standard library under the mount point.
  `DataLogManager` writes `.wpilog` files there after `SetLogDirectory("/sd/logs")` and creates
  that directory. Its old-log cleanup does not run on the card: `GetStorageSpace()` only knows the
  LittleFS partition (the ESP32 C library has no `statvfs`), so free space is unknown and nothing
  is deleted. Remove old logs by hand.
- Host: no SD simulation. `MountSdCard` checks its arguments, prints a warning and returns false.
- The SD protocol, FAT and the VFS are ESP-IDF's (`sdmmc`, `fatfs`, `esp_driver_sdspi`, linked
  from the Arduino-ESP32 core); prosdk only adds the glue.

## 5. Sharing the bus

The two flash boards and the SD card can share one bus with different chip selects; F1 serializes
transfers per device and ESP-IDF arbitrates between the prosdk devices and the SD card's own
device. ESP-IDF documents that an SD card on a shared bus should be initialized (put into SPI
mode) before the other devices start talking, and that its chip select should idle high until
then (**not re-checked here**): call `MountSdCard` before constructing the flash drivers on the
same bus. The SD card counts as one of the bus's users (at most 6 on SPI2, 3 on SPI3, F1).
Clocked LEDs (a device without chip select) need a bus of their own, so with them only one bus is
left for storage.

## 6. Verified and unverified

Verified against the datasheets (sections cited in the source files):
- W25Q40CL: SPI modes 0 and 3; JEDEC ID EF 40 13; instruction codes 06h, 05h, 02h, 20h, D8h, C7h,
  0Bh, ABh, 9Fh; BUSY (S0) and WEL (S1); Fast Read dummy byte; page wrap; factory default of the
  protection bits (0); tRES1 3 us, tPP 0.8 ms, tSE 300 ms, tBE2 1 s, tCE 4 s maximums.
- W25N01GV: page layout and bad-block marker bytes (figure 2, 10.2); SR-1/SR-2/SR-3 bit positions
  (figures 4a to 4c, read from the rendered pages); register addresses Axh/Bxh/Cxh; power-up
  protection of the whole array; BUF default per part; JEDEC ID EF AA 21; instruction codes FFh,
  9Fh, 0Fh, 1Fh, 06h, 13h, 0Bh, 02h, 10h, D8h with their byte layouts (8.1.3); tRST, tRD2, tPP,
  tBE, NoP 4; at least 1004 valid blocks.
- ESP-IDF: `esp_vfs_fat_sdspi_mount` and its configuration structures, the SDMMC frequency
  constants, the FAT options above, and that `libfatfs.a`, `libesp_driver_sdspi.a` and
  `libsdmmc.a` are in the Arduino-ESP32 link line.

Checked by compilation only: every source and header under `src/` (syntax check for the host and
for the ESP32-S3 target), the two tests (host), and the example (host only, as a .cpp copy; the
ESP32 build of the example runs in the integration step).
The host tests (`[w25qflash]`, `[w25nflash]`) run the drivers against command-level models of the
two chips; they were written but not yet run in this package (the build runs in the integration
step). Nothing has run on hardware.

Unverified (needs hardware or the board schematics' nets):
- Which nets the 10 kOhm resistors on the W25Q40 and W25N01GV boards connect to.
- The TXB0106 host-side reference on the W25N01GV board, and the highest clock through it.
- Reliable clocks over flying leads (both flash defaults are 10 MHz; the SD default is 20 MHz).
- Whether the MISO internal pull-up is enough for the MicroSD board.
- SD card mount, speed and bus sharing with the flash boards; the ESP-IDF bus-sharing advice
  above; that `esp_vfs_fat_sdspi_mount` drives the chip select as a GPIO (ESP-IDF source
  behaviour, not checked here).
- NAND bad blocks and ECC results on a real chip.
- W25Q parts other than the W25Q40CL.

## 7. Resource use

- W25QFlash, W25NFlash: one `prosdk::SPI` device (one chip select on the bus, one of 6 on SPI2 or
  3 on SPI3), one mutex. No thread, timer, RMT, LEDC or PCNT unit. Each driver uses a few bytes of
  stack per call; reads and programs use the caller's buffer.
- MountSdCard: one bus user and the chip-select pin, the ESP-IDF SD SPI device and FATFS volume
  (one of 2), and FATFS's per-file buffers (`CONFIG_FATFS_PER_FILE_CACHE`, up to 5 open files).
  Long file names use the stack of the calling task (`CONFIG_FATFS_LFN_STACK`).
- Example `BoardozaSdLog`: SPI2 on GPIO 10 to 13.

## 8. Tests

- `W25QFlashTest.cpp`: identification (frames ABh then 9Fh, size, SPI mode and clock); page
  splitting (two Page Program frames at the right addresses, a write enable per page, no bytes
  outside the range, Fast Read frame); erases that do not wait (busy reads, Read and erase refused
  while busy, `WaitReady`, sector/block/chip ranges, timeout); a missing chip.
- `W25NFlashTest.cpp`: reset, unprotect and ID; program, read (including columns and the spare
  area) and erase with their frames; ECC results (corrected, uncorrectable), P-FAIL and E-FAIL of a
  protected array, a timeout; bad-block markers in the main and spare bytes read with the ECC
  off and the ECC restored; a missing chip (every access refused).

## 9. Shared edits (integration)

- design/drivers.md: a link to this file in the Boardoza section; the "Tests" list gains
  W25QFlash and W25NFlash.
- design/API-DIFFERENCES.md section 7: three rows. `prosdk::W25QFlash` (hand-written, no WPILib
  counterpart; `(SPI::Port, cs)`), `prosdk::W25NFlash` (same), `prosdk::platform::MountSdCard`
  (prosdk platform addition; ESP-IDF FAT over SPI; host stub returns false).
- design/hal-esp32.md: an "SD card" note in the SPI section: the card joins an F1 bus through
  `RetainSpiBus`, claims its chip select (`ClaimPin`, VENDOR), enables the MISO pull-up, and stays
  mounted until the program ends; mount it before other devices on the same bus talk; it counts
  as one bus user.
- tools/upstream/examples.json ("handwritten") and design/examples.md: `BoardozaSdLog`.
