# BMI270 IMU Driver Design Notes

This driver implements the `mc_experimental::ImuDevice` contract for Bosch Sensortec's BMI270 over I2C. The implementation follows the existing `ImuTask` usage model: initialize once, optionally wake from a data-ready interrupt, and publish one accel/gyro sample per read.

## Design choices

- The driver uses the Bosch-provided BMI270 configuration blob because the device does not produce normal accel/gyro data until that 8 kB initialization payload has been loaded after POR or soft reset.
- The implementation keeps the runtime path simple and deterministic. It reads accel, gyro, and temperature registers directly instead of enabling FIFO support because `ImuTask` already models a single-sample producer.
- The chosen full-scale ranges are `+-2 g` for accelerometer and `+-250 dps` for gyroscope. This matches the existing MPU6050 reference behavior and gives the highest sensitivity for general motion estimation.
- The driver uses normal aliasing-free filtering for both sensors and leaves gyroscope low-noise mode disabled. That keeps current lower than BMI270 performance mode while preserving the normal-mode signal chain described in the Bosch datasheet.
- Advanced power save is disabled during configuration and left disabled in operation. This avoids the BMI270's 450 us inter-write restriction from becoming a latent runtime hazard in recovery paths.

## Interface mapping

- `init()`: calls `reset()`, `check_identity()`, and `configure()` in sequence. This matches the task's recovery behavior, which simply retries `init()` on failures.
- `reset()`: writes `0xB6` to `CMD`, which is the BMI270 soft-reset command, then waits long enough for the post-reset register access window.
- `check_identity()`: reads `CHIP_ID` and expects `0x24`.
- `configure()`: disables advanced power save, uploads the Bosch init blob through `INIT_DATA`, checks `INTERNAL_STATUS`, programs ranges and ODR, optionally enables `INT1` data-ready signaling, then enables accel, gyro, and temperature.
- `has_data_ready_irq()`: returns the configuration flag so `ImuTask` can choose between interrupt-driven wakeups and polling.
- `data_ready_pin()`: exposes the configured GPIO pin number for `ImuTask` ISR attachment.
- `sample_rate_hz()`: returns the requested ODR so board code can inspect the effective target rate.
- `read_sample()`: reads shadow-safe LSB-first accel and gyro registers, converts them to SI units, reads temperature, timestamps the sample with the host clock, and sets `valid`.

## Timing and device constraints

- Bosch specifies a minimum `450 us` wait after POR or soft reset before register access and the same order of magnitude write-settle timing around configuration transitions. The driver uses `1 ms` delays because the HAL only exposes millisecond sleeps and this stays safely above the device minimum.
- The configuration blob is uploaded in `32-byte` chunks. BMI270 allows chunked writes as long as `INIT_ADDR_0/1` is advanced in units of `bytes / 2` between bursts.
- The selected ODR table only accepts rates supported by both accelerometer and gyroscope in the simple direct-read mode: `25, 50, 100, 200, 400, 800, 1600 Hz`.
- Temperature is only valid when the gyro path is active. The driver enables the temperature sensor together with gyro and returns `NaN` if the device reports the invalid temperature code.

## Sources

- Bosch Sensortec BMI270 Datasheet, BST-BMI270-DS000, revision 1.5.
- Bosch Sensortec BMI270 SensorAPI `bmi270.c` and `bmi2_defs.h` for the official initialization blob and register constants.
