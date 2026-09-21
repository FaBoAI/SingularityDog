# ICM-20948 diagnostic acquisition

`singularitydog_hw/imu.py` uses Python's standard library and Linux `I2C_RDWR`.
No installation or network access is needed at runtime. Construction is inert;
`start()` or context entry opens the supplied adapter. The default is
`/dev/i2c-7`, address `0x68`. The caller must ensure exclusive sensor access.

```python
from singularitydog_hw.imu import ICM20948

with ICM20948('/dev/i2c-7', address=0x68) as device:
    metadata = device.configuration
    original = device.original_registers
    sample = device.read_sample()  # None until a new ready event exists.
# Modified registers have been restored or RestoreError has been raised.
restoration = device.restore_status
```

The caller schedules polling at 100 Hz; the driver does not create a thread.
Each returned dictionary contains `sequence`, `raw_accel`, `raw_gyro`,
`accel_m_s2`, `gyro_rad_s`, `frame='sensor'`, `monotonic_ns`, `wall_time_ns`,
`read_started_monotonic_ns`, `read_finished_monotonic_ns`, and readiness flags.
Vector order is X, Y, Z in the chip's output registers. This is neither a body
frame nor a claim that chip axes match the breakout's silkscreen. The candidate
mounting rotation is never applied. Accelerometer values include gravity;
calibration, gravity removal, quaternion estimation, and magnetometer setup
are outside this driver.

Register choices were checked against TDK's **DS-000189**: identity `0xEA`;
bank selector `0x7F`; bank 0 power `0x06/0x07`, low-power `0x05`, ready enable
`0x11`, ready status `0x1A`; bank 2 divider/configuration `0x00/0x01` and
`0x10/0x11/0x14`. Configuration `0x21` selects ±250 dps / ±2 g, enabled DLPF
at 23.9 Hz. Conversion uses 131 LSB/(degree/s), 16384 LSB/g, standard gravity
9.80665 m/s², and radians per degree. The fourteen-byte bank 0 burst begins at
`0x2D` and decodes seven signed big-endian words: acceleration, angular velocity,
then die temperature. Temperature is `raw / 333.87 + 21` degrees C, or null if
the retained power configuration disables it. It is not ambient or motor
temperature. See the [TDK revision 1.5 datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf),
sections 3, 8, 10 and 14.

Both dividers are 10. Tables 16/18 specify a nominal 1125/11 ≈ 102.27 Hz;
section 10.1 instead writes the gyro base as 1.1 kHz. Metadata preserves this
datasheet discrepancy. Host polling is 100 Hz, and neither rate is a measured
guarantee. The 50 ms wake delay and 100 ms post-configuration settling delay
are conservative implementation choices; the datasheet gives 35 ms typical
gyro startup. Setup readiness is discarded before capture begins.

Acquisition consumes read-to-clear readiness, reads the latest values, then
clears any ready event that raced that burst. `data_ready_during_read=True`
records that race. This prevents the raced event from counting the same image
again on the next call; it may discard a newer update. Identical numbers with
a fresh ready event are retained. No FIFO or sensor sample counter is used,
so missed conversions cannot be counted exactly. Timestamps describe host
reads, not conversion times: monotonic time is the transaction midpoint,
wall time is collected immediately afterwards. Separate gyro/accel timing
and synchronization are not established by this acquisition mode.

Startup snapshots configuration before changing it, clears sleep/LP_EN,
briefly disables measurement while writing settings, enables measurement,
and checks all requested register values. It rejects active DMP/FIFO/auxiliary
master or self-test states instead of trying to reconstruct their internal
state. No device reset is issued. The ready enable can toggle the INT1 pin;
no host GPIO is configured or required.

`restore()` and context exit restore every changed configuration register and
the original bank. Power/sleep is restored last. Read or setup failures also
attempt restoration. A failed write is considered potentially applied;
restoration continues across individual failures, raises `RestoreError`, and
retains failed register keys for an explicit retry before `close()`. The
`restore_status` field reports `not_needed`, `pending`, `restored`, or `failed`.
Pending interrupt history and sensor conversion phase cannot be restored.
SIGKILL, power loss, or an unavailable bus can prevent successful restoration.

Offline verification:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=runtime python3 -m unittest discover -s runtime/tests -p test_imu.py -v
```

The 19 fake-bus/ABI tests cover signed decoding/SI units, temperature, read-only trim auditing, explicit bank readback,
configuration readback, readiness consumption and its race, setup/read faults,
partial restoration and retry after sleep, unsupported active engines,
self-test rejection, and the combined Linux ioctl ABI. They do not establish
electrical communication, physical axes, measured sample rate, or real-device
restoration; those require a deployment receipt.
