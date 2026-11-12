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
from .fifo import FifoBatch, FifoConfig, parse_packets
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
        fifo: FifoConfig | None = None,
        poll_lead_s: float = c.DEFAULT_POLL_LEAD_S,
        poll_interval_s: float = c.DEFAULT_POLL_INTERVAL_S,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        time_ns: Callable[[], int] = time.time_ns,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.bus = bus
        self.config = config or Icm45686Config()
        self.fifo_config = fifo or FifoConfig()
        self._fifo_active = False
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

    def _identify_and_reset(self) -> None:
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

    def _configure_signal_path(self, rate_hz: float) -> None:
        # After reset: sensors off, little-endian data (SREG_CTRL), default
        # interface and drive settings, which this driver never changes.
        rate = c.ODR_CODES[rate_hz]
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

    def initialize(self) -> None:
        self._identify_and_reset()
        self._configure_signal_path(self.config.rate_hz)
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

    def initialize_fifo(self) -> None:
        """Reset, then configure hi-res FIFO streaming (see ``read_fifo``).

        Ranges and filters come from ``Icm45686Config``; the output rate, FIFO
        depth, mode and timestamp resolution come from ``FifoConfig`` (so the
        polling rate ceiling does not apply). Section numbers are DS-000577.
        """
        fifo = self.fifo_config
        rate = fifo.rate_hz
        self._identify_and_reset()
        # 20.50 SREG_CTRL.SREG_DATA_ENDIAN_SEL=1: big-endian FIFO data and
        # FIFO_COUNT, matching the section 6.1 packet figure.
        self._configure_indirect(c.SREG_CTRL, c.SREG_BIG_ENDIAN, c.SREG_BIG_ENDIAN)
        # 17.40/17.41: all APEX algorithms and the eDMP off (needed for 8 KB).
        self._configure(c.EDMP_APEX_EN0, c.APEX_EN0_MASK, 0)
        self._configure(c.EDMP_APEX_EN1, c.APEX_EN1_MASK, 0)
        # 17.32/17.28: FIFO interface off and Bypass while reconfiguring (depth
        # can only change in Bypass).
        self._configure(c.FIFO_CONFIG3, c.FIFO_CONFIG3_MASK, 0)
        self._configure(c.FIFO_CONFIG0, c.FIFO_MODE_MASK, c.FIFO_MODES["bypass"] << 6)
        # 17.5/17.6 ODR + ranges, section 4 filters (sensors are still off).
        self._configure_signal_path(rate)
        # 17.39: no FIFO decimation.
        self._configure(c.ODR_DECIMATE_CONFIG, 0xFF, 0)
        # 17.28: FIFO depth (2 KB = 000111, 8 KB = 011111).
        self._configure(
            c.FIFO_CONFIG0, c.FIFO_DEPTH_MASK, c.FIFO_DEPTH_CODES[fifo.fifo_depth_bytes]
        )
        # 17.34: timestamp resolution, no delta mode (WOM bits untouched).
        self._configure(
            c.TMST_WOM_CONFIG,
            c.TMST_WOM_CONFIG_MASK,
            c.TMST_RESOL_16US if fifo.tmst_resol_us == 16 else 0,
        )
        # 20.47: timestamp counter on. 17.33: insert it into FIFO frames, and
        # no compression (frames stay 20 bytes).
        self._configure_indirect(c.SMC_CONTROL_0, c.SMC_TMST_EN, c.SMC_TMST_EN)
        self._configure(
            c.FIFO_CONFIG4, c.FIFO_CONFIG4_MASK, c.FIFO_CONFIG4_TMST_FSYNC_EN
        )
        # 17.32: hi-res + gyro + accel frames (ES0/ES1 stay off).
        enables = (
            c.FIFO_CONFIG3_HIRES_EN | c.FIFO_CONFIG3_GYRO_EN | c.FIFO_CONFIG3_ACCEL_EN
        )
        self._configure(c.FIFO_CONFIG3, enables, enables)
        # 17.31: flush. FIFO_FLUSH is a command bit, so no readback check.
        flush = self._read(c.FIFO_CONFIG2)[0] | c.FIFO_FLUSH
        self.bus.write(c.FIFO_CONFIG2, bytes([flush]))
        # 17.28: enable the FIFO, then (per 17.32) the sensor-register interface.
        self._configure(c.FIFO_CONFIG0, c.FIFO_MODE_MASK, c.FIFO_MODES[fifo.mode] << 6)
        self._configure(c.FIFO_CONFIG3, c.FIFO_CONFIG3_IF_EN, c.FIFO_CONFIG3_IF_EN)
        self._fifo_active = True
        # Sensors on last so no sample precedes the FIFO setup.
        current = self._read(c.PWR_MGMT0)[0]
        try:
            self.bus.write(c.PWR_MGMT0, bytes([(current & ~0x0F) | c.LOW_NOISE_MODES]))
        finally:
            self._powered = True
        self._sleep(0.070 + 1 / rate)
        if self._read(c.PWR_MGMT0)[0] & 0x0F != c.LOW_NOISE_MODES:
            raise OSError("IMU power configuration readback failed")
        self._initialized = True
        logger.info(
            "IMU FIFO ready: %g Hz, %d B %s, %d us timestamps",
            rate,
            fifo.fifo_depth_bytes,
            fifo.mode,
            fifo.tmst_resol_us,
        )

    def _read_long(self, register: int, length: int) -> bytes:
        burst = getattr(self.bus, "read_burst", None)
        data = burst(register, length) if burst else self.bus.read(register, length)
        if len(data) != length:
            raise OSError(f"Short IMU read at 0x{register:02X}: {len(data)}/{length}")
        return data

    def read_fifo(self) -> FifoBatch:
        """Read FIFO_COUNT, then drain that many bytes with one burst read.

        Returns parsed hi-res samples; ``read_completed_monotonic_ns`` is taken
        right after the data read, for ``ClockMapper`` anchors.
        """
        if not (self._initialized and self._fifo_active):
            raise RuntimeError("Call initialize_fifo() before read_fifo()")
        fifo = self.fifo_config
        count = int.from_bytes(self._read(c.FIFO_COUNT_0, 2), "big")  # 17.18/17.19
        packets = count // c.FIFO_PACKET_BYTES if fifo.count_in_bytes else count
        packets = min(packets, fifo.fifo_depth_bytes // c.FIFO_PACKET_BYTES)
        nbytes = packets * c.FIFO_PACKET_BYTES
        data = self._read_long(c.FIFO_DATA, nbytes) if nbytes else b""
        completed = self._monotonic_ns()
        samples, malformed = parse_packets(data)
        if malformed:
            logger.warning("Skipped %d malformed FIFO packets", malformed)
        return FifoBatch(
            samples=samples,
            read_completed_monotonic_ns=completed,
            fifo_count_bytes=nbytes,
            was_full=nbytes + c.FIFO_PACKET_BYTES > fifo.fifo_depth_bytes,
            malformed_packets=malformed,
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
        try:
            if self._fifo_active:
                # Reverse of the enable order (17.32): interface off, then Bypass.
                self._fifo_active = False
                self._configure(c.FIFO_CONFIG3, c.FIFO_CONFIG3_IF_EN, 0)
                self._configure(c.FIFO_CONFIG0, c.FIFO_MODE_MASK, 0)
        finally:
            if self._powered:
                self._configure(c.PWR_MGMT0, 0x0F, 0)
                self._powered = False
