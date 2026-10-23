# Old SDK drivers (Probot Lib 0.2.6) kept for a later port

The old SDK code was removed from branch `prosdk-2027`. It is still on branch `stable`
(commit `ea2ea48`; branch `main` pointed at the same commit and was deleted on 2026-10-23). Read any file with:

```sh
git show ea2ea48:<path>
```

A later wave ports these drivers into `src/prosdk/drivers/` on top of the prosdk HAL and the
WPILib-style `MotorController` / gyro / encoder APIs. Assessment and known bugs:
`design/reports/toolchain.md` section 5.

## Motor controllers

| Path at `ea2ea48` | Notes |
|---|---|
| `src/probot/devices/motors/bts7960b_motor_controller.hpp` | BTS7960 (IBT-2) H-bridge, 20 kHz LEDC. Make the wiring mode configurable. |
| `src/probot/devices/motors/boardoza_vnh5019_motor_controller.hpp` | VNH5019 H-bridge (INA/INB/PWM, optional ENA/ENB), brake mode. Drop its built-in PID. |
| `src/probot/devices/motors/imotor_controller.hpp` | Old interface (replaced by WPILib `MotorController`). Reference only. |
| `src/probot/devices/motors/motor_controller_group.hpp` | Replaced by WPILib follower/composition. Reference only. |

## Sensors

| Path at `ea2ea48` | Notes |
|---|---|
| `src/probot/devices/sensors/imu/mpu6050.hpp` | MPU6050 raw Wire driver (gyro bias calibration, complementary filter). Becomes a WPILib-style gyro. |
| `src/probot/devices/sensors/imu/imu.hpp` | IMU interface used by the MPU6050 driver. |
| `src/probot/devices/sensors/encoder.hpp` | `IEncoder` interface only. A new PCNT encoder replaces it. |

## ProHub (PCA9685 + UART encoder co-processor)

| Path at `ea2ea48` | Notes |
|---|---|
| `src/probot/prohub/prohub_board.hpp` | Board constants (I2C pins, PCA9685 addresses, encoder UART). |
| `src/probot/prohub/prohub_config.hpp` | Configuration constants. |
| `src/probot/prohub/encoder_frame.hpp` | 108-byte encoder frame layout (must stay wire-compatible with the Pico firmware). |
| `src/probot/prohub/uart_encoder.hpp` | UART encoder reader task, CRC counter. |
| `src/probot/prohub/pca_motor.hpp` | PCA9685 motor/servo output. Bug: I2C inside `portENTER_CRITICAL`; use a mutex. |
| `src/probot/prohub/double_buffer.hpp` | Tearing SPSC buffer; rewrite as seqlock or triple buffer. |
| `src/probot/prohub/inner_loop.hpp` | 3 ms on-board PID inner loop task (concept only). |
| `src/probot/prohub/prohub.hpp` | `ProHub.motor(n)` facade (concept only). |

## Other pieces that may be useful as reference

| Path at `ea2ea48` | Notes |
|---|---|
| `src/probot/devices/leds/builtin.hpp` | Built-in RGB LED helper. |
| `src/probot/test/test_motor.hpp`, `test_plant.hpp`, `test_encoder.hpp`, `null_encoder.hpp` | Test doubles; replaced by WPILib sim classes. |
| `examples/` | Old Arduino sketches (IBT2MotorDemo, JoystickTest, MotorOpenLoopDemo, command_based/*). |
