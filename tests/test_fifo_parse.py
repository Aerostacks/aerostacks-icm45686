import math

import pytest
from conftest import build_packet

from aerostacks_icm45686 import constants as c
from aerostacks_icm45686 import parse_packets

G = c.STANDARD_GRAVITY


def test_basic_values_and_scaling():
    pkt = build_packet(
        accel=(16384, -16384, 8192),
        gyro=(131072, -131072, 0),
        temp=128 * 5,
        timestamp=0x1234,
    )
    (s,), bad = parse_packets(pkt)
    assert bad == 0
    assert s.timestamp_raw == 0x1234
    assert s.ax_m_s2 == pytest.approx(G)
    assert s.ay_m_s2 == pytest.approx(-G)
    assert s.az_m_s2 == pytest.approx(G / 2)
    assert s.gx_rad_s == pytest.approx(math.radians(131072 / (524288 / 4000)))
    assert s.gy_rad_s == pytest.approx(-s.gx_rad_s)
    assert s.temperature_c == pytest.approx(30.0)
    assert not s.accel_clipped
    assert not s.gyro_clipped


def test_nibble_packing_and_sign_extension():
    # Low nibbles live in bytes 17-19, accel in the high nibble.
    pkt = build_packet(accel=(0x12346 & ~1, 0, 0), gyro=(0x0000B, 0, 0))
    assert pkt[17] == 0x6B & 0xF0 | 0x0B
    (s,), _ = parse_packets(pkt)
    assert s.ax_m_s2 == pytest.approx((0x12346 & ~1) / 16384 * G)
    # -1 in 20 bits: all bits set, including every nibble.
    (n,), _ = parse_packets(build_packet(accel=(-2, 0, 0), gyro=(-1, -17, 3)))
    assert n.ax_m_s2 == pytest.approx(-2 / 16384 * G)
    assert n.gx_rad_s == pytest.approx(math.radians(-1 / (524288 / 4000)))
    assert n.gy_rad_s == pytest.approx(math.radians(-17 / (524288 / 4000)))
    assert n.gz_rad_s == pytest.approx(math.radians(3 / (524288 / 4000)))


def test_negative_temperature():
    (s,), _ = parse_packets(build_packet(temp=-256))
    assert s.temperature_c == pytest.approx(23.0)


@pytest.mark.parametrize("raw", [524287, -524288, -524287])
def test_clipping_extremes(raw):
    (s,), _ = parse_packets(build_packet(accel=(0, raw & ~1, 0)))
    assert s.accel_clipped and not s.gyro_clipped
    (s,), _ = parse_packets(build_packet(gyro=(0, 0, raw)))
    assert s.gyro_clipped and not s.accel_clipped


def test_just_below_clip():
    (s,), _ = parse_packets(build_packet(accel=(524284, 0, 0), gyro=(524286, 0, 0)))
    assert not s.accel_clipped and not s.gyro_clipped


def test_header_flags_accepted():
    # FSYNC tag and both ODR-change bits are legal on a hi-res frame.
    (s,), bad = parse_packets(build_packet(header=0x7F))
    assert bad == 0 and s.header == 0x7F


@pytest.mark.parametrize("header", [0x00, 0xF8, 0x68, 0x58, 0x70, 0x38])
def test_malformed_headers_counted_and_skipped(header):
    good = build_packet(timestamp=7)
    data = good + build_packet(header=header) + build_packet(timestamp=9)
    samples, bad = parse_packets(data)
    assert [s.timestamp_raw for s in samples] == [7, 9]
    assert bad == 1


@pytest.mark.parametrize("header", [0x80, 0xFF])
def test_empty_pattern_is_silent(header):
    samples, bad = parse_packets(build_packet(header=header) + build_packet())
    assert len(samples) == 1 and bad == 0


def test_truncated_tail_counts_once():
    samples, bad = parse_packets(build_packet() + build_packet()[:13])
    assert len(samples) == 1 and bad == 1


def test_little_endian_variant():
    pkt = bytearray(build_packet(accel=(16384, 0, 0), timestamp=0x0102, temp=0x0304))
    for i in range(1, 17, 2):
        pkt[i], pkt[i + 1] = pkt[i + 1], pkt[i]
    (s,), bad = parse_packets(bytes(pkt), big_endian=False)
    assert bad == 0
    assert s.timestamp_raw == 0x0102
    assert s.ax_m_s2 == pytest.approx(G)
    assert s.temperature_c == pytest.approx(25 + 0x0304 / 128)
