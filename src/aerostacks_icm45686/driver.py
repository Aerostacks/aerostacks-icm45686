"""ICM-45686 driver: identify, configure, poll fresh samples, power down.

Defaults: 100 Hz, +/-4 g, +/-125 dps, low-noise mode, FIR on, UI LPF ODR/4.
Acceleration includes gravity. Timestamps mark host read completion; they are
not hardware sample timestamps or synchronization with other sensors.
The caller owns the bus and must serialize access to this sensor.
"""

from __future__ import annotations

import logging
import math
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass

from . import constants as c
from .transport import RegisterBus

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Icm45686Config:
    accel_range_g: int = 4
    gyro_range_dps: float = 125
    rate_hz: float = 100
    filter_divisor: int = 4
    # Ceiling on rate_hz for register polling; see constants.DEFAULT_MAX_RATE_HZ.
    max_rate_hz: float = c.DEFAULT_MAX_RATE_HZ

    def __post_init__(self) -> None:
        for name, choices in (
            ("accel_range_g", c.ACCEL_RANGES),
            ("gyro_range_dps", c.GYRO_RANGES),
            ("rate_hz", c.ODR_CODES),
            ("filter_divisor", c.FILTER_DIVISORS),
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or value not in choices:
                raise ValueError(f"Unsupported ICM-45686 {name}: {value}")
        if self.rate_hz > self.max_rate_hz:
            raise ValueError(
                f"ICM-45686 rate_hz {self.rate_hz} exceeds max_rate_hz "
                f"{self.max_rate_hz}; register polling would lose samples"
            )

    @property
    def cutoff_hz(self) -> float:
        """Nominal UI LPF cutoff; with FIR on, the real bandwidth also depends on it."""
        return self.rate_hz / self.filter_divisor


@dataclass(frozen=True)
class ImuSample:
    """Host read-completion times and sensor-axis measurements, including gravity.

    Timestamps are not hardware sampling times and do not compensate filter lag.
    """

    time_ns: int
    monotonic_ns: int
    ax_m_s2: float
    ay_m_s2: float
    az_m_s2: float
    gx_rad_s: float
    gy_rad_s: float
    gz_rad_s: float
    temperature_c: float
    accel_clipped: bool
    gyro_clipped: bool


class Icm45686:
    """Identify, initialize, read fresh samples, and power down one ICM-45686."""

    def __init__(
        self,
        bus: RegisterBus,
        config: Icm45686Config | None = None,
        *,
        poll_lead_s: float = c.DEFAULT_POLL_LEAD_S,
        poll_interval_s: float = c.DEFAULT_POLL_INTERVAL_S,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        time_ns: Callable[[], int] = time.time_ns,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.bus = bus
        self.config = config or Icm45686Config()
        self.poll_lead_s = poll_lead_s
        self.poll_interval_s = poll_interval_s
        self._sleep = sleep
        self._monotonic = monotonic
        self._time_ns = time_ns
        self._monotonic_ns = monotonic_ns
        self._powered = False
        self._initialized = False
        self._last_ready: float | None = None
        self.invalid_samples = 0

    def _read(self, register: int, length: int = 1) -> bytes:
        data = self.bus.read(register, length)
        if len(data) != length:
            raise OSError(f"Short IMU read at 0x{register:02X}: {len(data)}/{length}")
        return data

    def _configure(self, register: int, mask: int, value: int) -> None:
        """Preserve reserved fields and verify each configured field by readback."""
        current = self._read(register)[0]
        updated = (current & ~mask) | (value & mask)
        self.bus.write(register, bytes([updated]))
        actual = self._read(register)[0]
        if actual & mask != value & mask:
            raise OSError(f"IMU configuration readback failed at 0x{register:04X}")

    def _read_indirect(self, address: int) -> int:
        # TDK transport: STOP triggers prefetch, with >=4 us guard times.
        self._sleep(0.00001)
        self.bus.write(c.IREG_ADDR, address.to_bytes(2, "big"))
        self._sleep(0.00001)
        return self._read(c.IREG_DATA)[0]

    def _configure_indirect(self, address: int, mask: int, value: int) -> None:
        current = self._read_indirect(address)
        updated = (current & ~mask) | (value & mask)
        self._sleep(0.00001)
        # One burst avoids prefetch between address and the data being written.
        self.bus.write(c.IREG_ADDR, address.to_bytes(2, "big") + bytes([updated]))
        self._sleep(0.00001)
        if self._read_indirect(address) & mask != value & mask:
            raise OSError(f"IMU configuration readback failed at 0x{address:04X}")

    def initialize(self) -> None:
        self._sleep(0.003)
        try:
            identity = self._read(c.WHO_AM_I)[0]
        except OSError:
            # TDK AN-000364: retry a first I2C transaction NACK once.
            identity = self._read(c.WHO_AM_I)[0]
        if identity != c.DEVICE_ID:
            raise OSError(f"WHO_AM_I is 0x{identity:02X}; expected 0xE9")
        logger.info("ICM-45686 identified: WHO_AM_I=0x%02X", identity)
        # Clear any stale reset-done flag so only this reset can satisfy the check.
        self._read(c.STATUS_REGISTER)
        reset_error = None
        try:
            self.bus.write(c.REG_MISC2, bytes([c.SOFT_RESET]))
        except OSError as error:
            # Observed on a bit-banged i2c-gpio bus: every reset takes effect before
            # the write is acknowledged. Only a fresh reset-done flag below
            # establishes success; an error alone never counts as a reset.
            reset_error = error
        self._sleep(0.001)
        deadline = self._monotonic() + 0.1
        while not self._read(c.STATUS_REGISTER)[0] & c.RESET_DONE:
            if self._monotonic() >= deadline:
                raise TimeoutError("ICM-45686 reset did not complete") from reset_error
            self._sleep(0.001)
        if reset_error is not None:
            logger.warning(
                "Reset write reported %s; fresh reset completion verified", reset_error
            )
        logger.info("ICM-45686 software reset verified")
        # After reset: sensors off, little-endian data (SREG_CTRL), default
        # interface and drive settings, which this driver never changes.
        rate = c.ODR_CODES[self.config.rate_hz]
        self._configure(
            c.ACCEL_CONFIG0,
            0x7F,
            (c.ACCEL_RANGES[self.config.accel_range_g] << 4) | rate,
        )
        self._configure(
            c.GYRO_CONFIG0,
            0xFF,
            (c.GYRO_RANGES[self.config.gyro_range_dps] << 4) | rate,
        )
        self._configure_indirect(c.ACCEL_SRC_CTRL, 0x03, 0x01)
        self._configure_indirect(c.GYRO_SRC_CTRL, 0x60, 0x20)
        bandwidth = c.FILTER_DIVISORS[self.config.filter_divisor]
        self._configure_indirect(c.ACCEL_UI_LPF, 0x07, bandwidth)
        self._configure_indirect(c.GYRO_UI_LPF, 0x07, bandwidth)
        # The DRDY status bit is only latched when its interrupt source is enabled.
        self._configure(c.INT1_CONFIG0, c.READY_MASK, c.READY_MASK)
        current = self._read(c.PWR_MGMT0)[0]
        try:
            self.bus.write(c.PWR_MGMT0, bytes([(current & ~0x0F) | c.LOW_NOISE_MODES]))
        finally:
            # Power down in stop() even if only the acknowledgement was lost.
            self._powered = True
        self._sleep(0.070 + 1 / self.config.rate_hz)
        if self._read(c.PWR_MGMT0)[0] & 0x0F != c.LOW_NOISE_MODES:
            raise OSError("IMU power configuration readback failed")
        self._read(c.STATUS_REGISTER)
        self._last_ready = None
        self._initialized = True
        logger.info(
            "IMU ready: %g Hz, +/- %g g, +/- %g dps, UI LPF nominal ODR/%d (~%g Hz), "
            "FIR on",
            self.config.rate_hz,
            self.config.accel_range_g,
            self.config.gyro_range_dps,
            self.config.filter_divisor,
            self.config.cutoff_hz,
        )

    def _wait_ready(self, deadline: float) -> float:
        """Sleep until shortly before the next sample is due, then poll DRDY.

        Returns the time the successful status read was issued. DRDY was set
        no later than about that moment, so the schedule can only drift early
        (costing an extra poll), never late enough to miss a sample. Anchoring
        on read completion instead drifted by the status-read duration, which
        is ~1 ms on a bit-banged i2c-gpio bus. Polling every millisecond from
        the previous read cost ~70% of a core there.
        """
        if self._last_ready is not None:
            wake = self._last_ready + 1 / self.config.rate_hz - self.poll_lead_s
            pause = min(wake, deadline) - self._monotonic()
            if pause > 0:
                self._sleep(pause)
        while True:
            issued = self._monotonic()
            if self._read(c.STATUS_REGISTER)[0] & c.READY_MASK:
                return issued
            if self._monotonic() >= deadline:
                raise TimeoutError("No fresh IMU data before timeout")
            self._sleep(self.poll_interval_s)

    def read_sample(self, timeout_s: float = 0.5) -> ImuSample:
        """Wait for the next valid sample, then burst-read six axes and temperature.

        Samples carrying the vendor's invalid marker are skipped and counted in
        ``invalid_samples``.
        """
        if not self._initialized:
            raise RuntimeError("Initialize the IMU before reading samples")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        deadline = self._monotonic() + timeout_s
        while True:
            self._last_ready = self._wait_ready(deadline)
            data = self._read(c.DATA_REGISTER, 14)
            received_ns = self._time_ns()
            received_monotonic_ns = self._monotonic_ns()
            values = struct.unpack("<7h", data)
            if c.INVALID_VALUE not in values:
                break
            self.invalid_samples += 1
            logger.debug("Skipped invalid IMU sample: %s", values)
        ax, ay, az, gx, gy, gz, temperature = values
        accel_scale = self.config.accel_range_g * c.STANDARD_GRAVITY / 32768
        gyro_scale = math.radians(self.config.gyro_range_dps) / 32768
        return ImuSample(
            time_ns=received_ns,
            monotonic_ns=received_monotonic_ns,
            ax_m_s2=ax * accel_scale,
            ay_m_s2=ay * accel_scale,
            az_m_s2=az * accel_scale,
            gx_rad_s=gx * gyro_scale,
            gy_rad_s=gy * gyro_scale,
            gz_rad_s=gz * gyro_scale,
            temperature_c=25 + temperature / c.TEMP_SENSITIVITY,
            # -32768 is the invalid marker, so the negative rail is -32767.
            accel_clipped=any(abs(value) >= 32767 for value in (ax, ay, az)),
            gyro_clipped=any(abs(value) >= 32767 for value in (gx, gy, gz)),
        )

    def stop(self) -> None:
        self._initialized = False
        if self._powered:
            self._configure(c.PWR_MGMT0, 0x0F, 0)
            self._powered = False
