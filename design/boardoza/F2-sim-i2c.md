# F2: sim I2C addressing and NACK injection (gap G2)

Platform package for the Boardoza I2C drivers (P1-P6). It changes the host sim only; nothing new
runs on the robot. Boards covered: none directly; every Boardoza I2C board test uses it.

## Why

Before F2 the WPILib sim I2C (`host/hal/sim/mockdata/I2CData.cpp`) dropped the device address
before it called the read and write callbacks, and the sim `HAL_TransactionI2C`, `HAL_WriteI2C`
and `HAL_ReadI2C` always returned 0. A test could not tell two devices on one port apart
(LSM303AGR accel 0x19 + mag 0x1E, BMI088 0x19 + 0x69, an I2C switch plus a device behind it) and
could not simulate a missing chip (`IsConnected() == false`).

## Sim HAL additions (prosdk only, not in WPILib)

Added by `tools/upstream/patches/hal/0004-I2C-sim-device-address-and-result.patch` and
regenerated with `tools/upstream/wpilib.py --module hal` (`wpilib.py --check` passes; the file
list in `generated.txt` is unchanged). Declared in `src/prosdk/hal/simulation/I2CData.h`:

```c
typedef int32_t (*HALSIM_I2CResultCallback)(void* param, int32_t address);
int32_t HALSIM_GetI2CLastAddress(int32_t index);
void HALSIM_SetI2CResultCallback(int32_t index, HALSIM_I2CResultCallback callback, void* param);
```

- `HALSIM_GetI2CLastAddress`: the 7-bit address of the transfer in progress on the port, valid
  inside a read, write or result callback. Outside one it is the last address used; -1 before
  the first transfer and after `HALSIM_ResetI2CData`.
- `HALSIM_SetI2CResultCallback`: one callback per port (a second call replaces it; NULL clears
  it). It is called once at the start of every `HAL_TransactionI2C`, `HAL_WriteI2C` and
  `HAL_ReadI2C` on the port, and the transfer returns its result. A result < 0 (an address NACK)
  skips the write and read callbacks, so `prosdk::I2C` reports an abort (`true`) and a read
  buffer is left untouched. With no callback every transfer returns 0, as in WPILib.
- The sim now serializes the transfers of one port (a recursive mutex in `I2CData`), so the last
  address cannot change under a callback when a driver thread and the test thread both use the
  port. The real backend serializes per bus too (Arduino's I2C HAL lock).
- `HAL_TransactionI2C` now splits its phases like the ESP32 backend (`esp32/I2C.cpp`): with
  nothing to receive it is a write only (`AddressOnly()` gives one empty write and no read), with
  nothing to send it is a read only (no empty write), otherwise a write then a read. WPILib's sim
  called both callbacks every time, also with a size of 0; the existing tests (FakeI2CDevice,
  GoBildaPinpoint, the I2CCommunication snippet) never make a transaction with a size of 0.

On the robot these two functions are stubs in `src/prosdk/platform/hal/esp32/SimStubs.cpp`
(`HALSIM_GetI2CLastAddress` returns -1; the setter does nothing), like every other `HALSIM_*`
function there. `design/hal-esp32-functions.txt` was regenerated with
`tools/build/hal_functions.py` (695 -> 697 functions), so `hal_link_test.py` checks that both are
defined in `libprosdk.a`.

## Test helper: `test/prosdk/drivers/FakeI2CBus.hpp`

`prosdk::test::FakeI2CBus` (header only, tests only): several register-file devices on one sim
port, keyed by address. `FakeI2CDevice.hpp` is unchanged and still answers every address on
port 0.

```cpp
test::FakeI2CBus bus;                       // port 0; FakeI2CBus bus{1} for port 1
auto& accel = bus.Add(0x19);                // 8-bit pointer, 8-bit registers
auto& mlx = bus.Add(0x33, 16, 16);          // 16-bit pointer, 16-bit big-endian words
auto& dac = bus.Add(0x58, 8, 16, true);     // 16-bit words sent low byte first
accel.regs[0x0F] = 0x33;                    // WHO_AM_I
accel.nack = true;                          // the chip stops answering
bus.Remove(0x19);                           // or disappears
```

`Device` fields: `regs` (2^pointerBits registers; 8-bit registers use the low byte), `pointer`,
`writes` (every write transfer with its pointer bytes, empty ones included), `reads` (read
transfers with at least one byte), `nack`, and the hooks `onWrite` / `onRead`.

- A write sets the pointer from its first 1 or 2 bytes (a 16-bit pointer is MSB first) and
  stores the rest from the pointer on. A write shorter than the pointer changes nothing. In word
  mode a trailing odd byte is dropped.
- A read returns registers from the pointer on. Reads and writes both auto-increment the pointer
  (one register per byte, or per word in word mode) and wrap at the end of the register file.
- An address with no device, or a device with `nack` set, NACKs every transfer.
- `onWrite` / `onRead`, when set, replace the register file for command-style chips (MS5611 ADC
  commands, the HIH8000 measurement request, the CaliPile general call); writes are still
  logged. The general-call address 0x00 is a device like any other (`bus.Add(0x00)`).
- Limits: one `FakeI2CBus` per port at a time (it owns the port's result callback), and not
  together with `FakeI2CDevice` on the same port. The callbacks lock the public `mutex`; a test
  that reads or changes a device while a driver thread runs holds it, and must not call the
  driver while holding it. A NACK is per transfer: the sim cannot NACK in the middle of the data
  or return a short read.

Tests: `test/prosdk/drivers/FakeI2CBusTest.cpp` (raw HAL address and result, two devices plus
NACK, pointer auto-increment and wrap, 16-bit pointers and words in both byte orders, hooks and
the general call). Run on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): 71 assertions in 5 test cases pass.

## Build: vendored C sources in drivers

`tools/build/modules/drivers.cmake` now globs `src/prosdk/drivers/*.c` as well as `*.cpp`, so
vendored C sources (ST VL53L7CX ULD `vl53l7cx_api.c`, P4; Bosch `bme68x.c` if P1 vendors it) go
into the host and target archives with their upstream names. The project already enables C
(`project(prosdk LANGUAGES C CXX)`). The Arduino library build compiles `src/**.c` on its own.

## Hardware items (G9): documented, no code

`src/prosdk/platform/hal/esp32/I2C.cpp` is not changed. Read from the Arduino-ESP32 3.3.10 source
(`cores/esp32/esp32-hal-i2c-ng.c`, the path used with its bundled ESP-IDF 5.5); none of this is
verified on hardware.

- **Zero-length write** (`I2C::AddressOnly()`, the HIH8131 measurement request):
  `HAL_TransactionI2C` with no data calls `i2cWrite(..., size 0)`, and Arduino-ESP32 3.3.10
  already turns a size-0 write into `i2c_master_probe` (address with the write bit, then stop;
  ESP-IDF `i2c_master.h`). So the fallback the plan named is what the core already does.
  Unverified: that the HIH8131 starts a measurement on it, and the probe's timing.
- **General call, address 0x00** (CaliPile reload 0x00/0x04; blocking for both TPiS boards):
  the HAL has no address check, and Arduino rejects only addresses >= 128, so 0x00 reaches
  `i2c_master_bus_add_device`. Whether ESP-IDF 5.5 accepts device address 0 and sends the
  general call is unverified (its source is not in the Arduino package). If it fails on
  hardware, F2 owns the fix in `esp32/I2C.cpp`.
- **Return codes on the robot**: -1 for an address NACK, a data NACK, a short read, a bus error
  or the 50 ms timeout (`kTimeoutMs`). The sim only produces what the result callback returns.

## Resource use

None on the robot (two empty stub functions). Host sim: one recursive mutex, one atomic address
and one callback pointer per I2C port.

## Files

- `tools/upstream/patches/hal/0004-I2C-sim-device-address-and-result.patch` (new)
- generated by `wpilib.py`: `host/hal/sim/I2C.cpp`, `host/hal/sim/mockdata/I2CData.cpp`,
  `host/hal/sim/mockdata/I2CDataInternal.hpp`, `src/prosdk/hal/simulation/I2CData.h`
- `src/prosdk/platform/hal/esp32/SimStubs.cpp` (two stubs); `design/hal-esp32-functions.txt`
  (regenerated by `tools/build/hal_functions.py`)
- `test/prosdk/drivers/FakeI2CBus.hpp`, `FakeI2CBusTest.cpp` (new)
- `tools/build/modules/drivers.cmake` (the `*.c` glob)

Shared edit for integration: a design/API-DIFFERENCES.md section 3 note listing the two
prosdk-only `HALSIM_*` I2C functions.
