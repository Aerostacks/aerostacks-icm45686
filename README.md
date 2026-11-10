# aerostacks-icm45686

Polling I2C driver for the TDK InvenSense ICM-45686 6-axis IMU (accelerometer and
gyroscope). It identifies the chip, soft-resets it, configures range, rate and
filters with readback verification, reads fresh samples by polling the data-ready
flag, and powers the sensor down.

## Install

```
pip install aerostacks-icm45686
```

## Usage

```python
from smbus2 import SMBus

from aerostacks_icm45686 import I2cRegisterBus, Icm45686, Icm45686Config

with SMBus(1) as bus:
    imu = Icm45686(I2cRegisterBus(bus, 0x68), Icm45686Config(rate_hz=100))
    imu.initialize()
    try:
        sample = imu.read_sample()
        print(sample.ax_m_s2, sample.gz_rad_s, sample.temperature_c)
    finally:
        imu.stop()
```

Any object with `read(register, length) -> bytes` and `write(register, data)`
works as a transport (`RegisterBus`), which is how the driver is tested without
hardware. The caller owns the bus and must serialize access to the sensor.

## Defaults

100 Hz output, +/-4 g, +/-125 dps, low-noise mode, FIR on, UI low-pass filter at
ODR/4. Acceleration includes gravity. Option tables (ranges, output rates, filter
divisors) live in `aerostacks_icm45686.constants`.

## Polling

Samples are read by register polling, not the FIFO. `Icm45686Config.max_rate_hz`
(default 200 Hz) rejects output rates that polling cannot sustain; raise it only
if your bus is fast enough. The driver sleeps until `poll_lead_s` before the next
sample is due, then polls status every `poll_interval_s`. Both are constructor
arguments (defaults 0.5 ms and 1 ms) and are worth tuning for your bus: on a
bit-banged `i2c-gpio` bus a register read costs about 1 ms, and a lead/interval of
0.5 ms / 0.2 ms needed ~1.4 status reads per sample with no lost samples.

## Soft reset may NACK

On some buses, notably bit-banged `i2c-gpio`, the chip resets before it
acknowledges the soft-reset write, so the write reports a NACK even though the
reset took effect. The driver tolerates this: an error on the reset write is
never treated as success on its own, and initialization only continues once a
freshly cleared-then-set reset-done flag is observed. It logs a warning when this
happens; that is expected on such buses. If the flag never appears,
`initialize()` raises `TimeoutError`.

## Timestamps

`ImuSample.time_ns` and `monotonic_ns` mark when the host finished reading the
sample. They are not hardware sampling times, do not compensate filter lag, and
are not synchronized with other sensors. Samples carrying the vendor's invalid
marker are skipped and counted in `Icm45686.invalid_samples`.

## License

MIT
