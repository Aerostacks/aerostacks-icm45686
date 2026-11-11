"""FIFO support: config, hi-res packet parsing, timestamp unwrap, clock mapping.

Everything here is pure (no bus access) so it can be tested without hardware.
Datasheet references are to TDK DS-000577 rev 1.0 (ICM-45686).

20-byte high-resolution packet (section 6.1, big-endian figure). Offsets:

    0      header (section 6.2)
    1-2    Ax  16 high bits, H then L
    3-4    Ay
    5-6    Az
    7-8    Gx
    9-10   Gy
    11-12  Gz
    13-14  temperature, 16-bit signed, H then L
    15-16  timestamp, 16-bit unsigned, H then L
    17     Ax LSB nibble (bits 7:4) | Gx LSB nibble (bits 3:0)
    18     Ay / Gy nibbles
    19     Az / Gz nibbles

Each 20-bit axis value is (signed16 << 4) | nibble. The accelerometer is 19-bit,
so its nibble's bit 0 is always 0. The figure labels two columns per trailing
byte ("Ax LSB   Gx LSB"); accel in the upper nibble is the natural reading but
is not spelled out and should be confirmed on hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import constants as c


@dataclass(frozen=True)
class FifoConfig:
    """FIFO mode settings. ``rate_hz`` is the sensor ODR (and FIFO rate)."""

    rate_hz: float = 100
    fifo_depth_bytes: int = 8192  # 8192 requires all APEX features off
    mode: str = "stream"  # stream: overwrite oldest when full
    tmst_resol_us: int = 1  # 1 or 16
    # FIFO_COUNT_0/1 unit. Section 17.18 says packets; section 6 says bytes.
    count_in_bytes: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.rate_hz, bool) or self.rate_hz not in c.ODR_CODES:
            raise ValueError(f"Unsupported ICM-45686 rate_hz: {self.rate_hz}")
        if self.rate_hz < c.FIFO_MIN_RATE_HZ:
            raise ValueError(
                f"FIFO rate_hz must be >= {c.FIFO_MIN_RATE_HZ}: the 16-bit "
                "timestamp wraps every 65.5 ms and must be < 1 wrap per packet"
            )
        if self.fifo_depth_bytes not in c.FIFO_DEPTH_CODES:
            raise ValueError(
                f"fifo_depth_bytes must be one of {sorted(c.FIFO_DEPTH_CODES)}"
            )
        if self.mode not in ("stream", "stop_on_full"):
            raise ValueError("mode must be 'stream' or 'stop_on_full'")
        if self.tmst_resol_us not in (1, 16):
            raise ValueError("tmst_resol_us must be 1 or 16")

    @property
    def period_us(self) -> float:
        return 1e6 / self.rate_hz


@dataclass(frozen=True)
class FifoSample:
    """One hi-res FIFO packet. ``timestamp_raw`` is the 16-bit chip counter."""

    timestamp_raw: int
    ax_m_s2: float
    ay_m_s2: float
    az_m_s2: float
    gx_rad_s: float
    gy_rad_s: float
    gz_rad_s: float
    temperature_c: float
    accel_clipped: bool
    gyro_clipped: bool
    header: int = 0


@dataclass(frozen=True)
class FifoBatch:
    samples: list[FifoSample] = field(default_factory=list)
    read_completed_monotonic_ns: int = 0
    fifo_count_bytes: int = 0
    was_full: bool = False  # no room for another packet: stream mode overwrote
    malformed_packets: int = 0


def _signed20(high16: int, nibble: int) -> int:
    value = (high16 << 4) | (nibble & 0x0F)
    return value - (1 << 20) if value & (1 << 19) else value


def parse_packets(
    data: bytes, *, big_endian: bool = True
) -> tuple[list[FifoSample], int]:
    """Parse concatenated 20-byte hi-res packets; return (samples, malformed).

    Empty-FIFO headers (0x80/0xFF) are skipped silently. Any other header that is
    not a hi-res+timestamp header, and a trailing partial packet, count as
    malformed. Packets are frame-aligned, so a bad packet is skipped whole.
    Little-endian swaps each 16-bit field (layout per section 15, inferred).
    """
    order = "big" if big_endian else "little"
    size = c.FIFO_PACKET_BYTES
    samples: list[FifoSample] = []
    malformed = 0
    for start in range(0, len(data), size):
        packet = data[start : start + size]
        if len(packet) < size:
            malformed += 1
            break
        header = packet[0]
        if header in c.FIFO_EMPTY_HEADERS:
            continue
        if header & c.FIFO_HEADER_MASK != c.FIFO_HEADER_HIRES:
            malformed += 1
            continue
        words = [
            int.from_bytes(packet[i : i + 2], order, signed=False)
            for i in range(1, 17, 2)
        ]  # ax ay az gx gy gz temp timestamp
        raw = []
        for axis in range(6):
            nibbles = packet[17 + axis % 3]
            nibble = nibbles >> 4 if axis < 3 else nibbles & 0x0F
            raw.append(_signed20(words[axis], nibble))
        temp = words[6] - 65536 if words[6] & 0x8000 else words[6]
        accel_scale = c.STANDARD_GRAVITY / c.HIRES_ACCEL_LSB_PER_G
        gyro_scale = math.radians(1) / c.HIRES_GYRO_LSB_PER_DPS
        accel_limit = c.HIRES_ACCEL_FULL_SCALE_RAW
        gyro_limit = c.HIRES_FULL_SCALE_RAW
        samples.append(
            FifoSample(
                timestamp_raw=words[7],
                ax_m_s2=raw[0] * accel_scale,
                ay_m_s2=raw[1] * accel_scale,
                az_m_s2=raw[2] * accel_scale,
                gx_rad_s=raw[3] * gyro_scale,
                gy_rad_s=raw[4] * gyro_scale,
                gz_rad_s=raw[5] * gyro_scale,
                temperature_c=25 + temp / c.FIFO_TEMP_SENSITIVITY,
                accel_clipped=any(abs(v) >= accel_limit for v in raw[:3]),
                gyro_clipped=any(abs(v) >= gyro_limit for v in raw[3:]),
                header=header,
            )
        )
    return samples, malformed


class TimestampUnwrapper:
    """Unwrap the chip's 16-bit timestamp into a monotonic 64-bit time.

    ASSUMPTION: consecutive packets are less than one wrap (65536 ticks, 65.5 ms
    at 1 us resolution) apart. Longer gaps alias silently, so FIFO rates are
    limited to >= 50 Hz. A delta of 0 is treated as 0, not a full wrap.
    ``unit_us`` is the timestamp tick (1 or 16 us).
    """

    def __init__(self, unit_us: int = 1) -> None:
        self.unit_us = unit_us
        self._last_raw: int | None = None
        self._chip_time_us = 0
        self.last_delta_us: int | None = None

    def update(self, raw: int) -> int:
        """Feed one raw timestamp; return chip_time_us (starts at its raw value)."""
        if not 0 <= raw < c.TMST_WRAP:
            raise ValueError("raw timestamp must be 16-bit")
        if self._last_raw is None:
            self._chip_time_us = raw * self.unit_us
            self.last_delta_us = None
        else:
            delta = ((raw - self._last_raw) % c.TMST_WRAP) * self.unit_us
            self._chip_time_us += delta
            self.last_delta_us = delta
        self._last_raw = raw
        return self._chip_time_us

    @property
    def chip_time_us(self) -> int:
        return self._chip_time_us

    def detect_gaps(self, expected_period_us: float) -> int:
        """Samples missing before the last update (0 if delta <= 1.5 periods).

        Stream-mode overwrite drops whole packets, so a missing run shows up as
        a larger delta between neighbours.
        """
        if self.last_delta_us is None or expected_period_us <= 0:
            return 0
        if self.last_delta_us <= 1.5 * expected_period_us:
            return 0
        return max(0, round(self.last_delta_us / expected_period_us) - 1)


class ClockMapper:
    """Map chip time to host monotonic time with a sliding least-squares fit.

    Anchor: (chip_time_us of the last packet in a batch, host monotonic_ns at
    read completion). Fit: host_ns = a + b * chip_ns over the last ``window``
    anchors. With fewer than 2 anchors b = 1 (offset only). The anchor latency
    (sample age at read time) is a constant absorbed into ``a``.
    """

    def __init__(self, window: int = 60) -> None:
        if window < 2:
            raise ValueError("window must be >= 2")
        self.window = window
        self._anchors: list[tuple[int, int]] = []
        self._x0 = 0  # chip ns reference
        self._y0 = 0  # host ns reference
        self._slope = 1.0
        self._intercept = 0.0  # host_ns - _y0 at chip_ns == _x0

    def add_anchor(self, chip_time_us: int, host_monotonic_ns: int) -> None:
        chip_ns = chip_time_us * 1000
        if self._anchors and chip_ns <= self._anchors[-1][0]:
            raise ValueError("anchor chip_time_us must increase")
        self._anchors.append((chip_ns, host_monotonic_ns))
        del self._anchors[: -self.window]
        self._refit()

    def _refit(self) -> None:
        self._x0, self._y0 = self._anchors[0]
        xs = [x - self._x0 for x, _ in self._anchors]
        ys = [y - self._y0 for _, y in self._anchors]
        n = len(xs)
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        sxx = sum((x - mean_x) ** 2 for x in xs)
        if n < 2 or sxx == 0:
            self._slope = 1.0
        else:
            sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys, strict=True))
            self._slope = sxy / sxx
        self._intercept = mean_y - self._slope * mean_x

    @property
    def slope(self) -> float:
        return self._slope

    @property
    def anchor_count(self) -> int:
        return len(self._anchors)

    def map(self, chip_time_us: float) -> int:
        """Host monotonic_ns for ``chip_time_us`` (raises before any anchor)."""
        if not self._anchors:
            raise RuntimeError("ClockMapper has no anchors")
        x = chip_time_us * 1000 - self._x0
        return round(self._y0 + self._intercept + self._slope * x)
