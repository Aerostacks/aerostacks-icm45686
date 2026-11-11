import random

import pytest

from aerostacks_icm45686 import ClockMapper, TimestampUnwrapper


def test_unwrapper_monotonic_across_wraps():
    u = TimestampUnwrapper()
    truth = 60000
    out = [u.update(truth % 65536)]
    for _ in range(50):
        truth += 10_000
        out.append(u.update(truth % 65536))
    assert out == [60000 + 10_000 * i for i in range(51)]


def test_unwrapper_sixteen_us_units():
    u = TimestampUnwrapper(unit_us=16)
    u.update(65535)
    assert u.update(1) == 65535 * 16 + 2 * 16


def test_unwrapper_rejects_non_16_bit():
    with pytest.raises(ValueError):
        TimestampUnwrapper().update(65536)


def test_gap_detection():
    u = TimestampUnwrapper()
    t = 65000
    u.update(t % 65536)
    assert u.detect_gaps(10_000) == 0
    t += 10_000
    u.update(t % 65536)
    assert u.detect_gaps(10_000) == 0
    t += 15_000  # exactly 1.5x: not a gap
    u.update(t % 65536)
    assert u.detect_gaps(10_000) == 0
    t += 15_001
    u.update(t % 65536)
    assert u.detect_gaps(10_000) == 1  # round(1.5) rounds to even -> 2 - 1
    t += 30_000  # two missing, across a wrap
    u.update(t % 65536)
    assert u.detect_gaps(10_000) == 2
    t += 10_000
    u.update(t % 65536)
    assert u.detect_gaps(10_000) == 0


def test_gap_detection_jitter_tolerant():
    u = TimestampUnwrapper()
    u.update(0)
    u.update(10_300)
    assert u.detect_gaps(10_000) == 0
    u.update(10_300 + 20_400)  # one missing, 2% slow clock
    assert u.detect_gaps(10_000) == 1


def test_mapper_offset_only_then_fit():
    m = ClockMapper()
    with pytest.raises(RuntimeError):
        m.map(0)
    m.add_anchor(1_000_000, 5_000_000_000)
    assert m.slope == 1.0
    assert m.map(1_000_000) == 5_000_000_000
    assert m.map(1_010_000) == 5_000_000_000 + 10_000_000
    m.add_anchor(3_000_000, 5_000_000_000 + 2_000_000_000)
    assert m.anchor_count == 2


def test_mapper_tracks_50ppm_drift_with_jitter():
    rng = random.Random(1)
    drift = 1 + 50e-6  # host clock runs 50 ppm fast vs chip
    m = ClockMapper(window=60)
    base = 123_456_789_000_000
    errors = []
    for k in range(1, 400):
        chip_us = k * 2_000_000 + 777
        host = base + round(chip_us * 1000 * drift) + rng.randint(0, 2_000_000)
        m.add_anchor(chip_us, host)
        if k > 70:
            probe = chip_us + 500_000  # mid-interval point, in the future
            truth = base + round(probe * 1000 * drift)
            errors.append(m.map(probe) - truth)
    assert m.slope == pytest.approx(drift, abs=20e-6)
    # Constant jitter bias (mean 1 ms) is expected; dispersion must be small.
    mean = sum(errors) / len(errors)
    assert 0.5e6 < mean < 1.5e6
    assert max(abs(e - mean) for e in errors) < 600_000


def test_mapper_window_slides():
    m = ClockMapper(window=3)
    for k in range(10):
        m.add_anchor(k * 1000, k * 1_000_000)
    assert m.anchor_count == 3


def test_mapper_rejects_non_increasing_anchor():
    m = ClockMapper()
    m.add_anchor(10, 10)
    with pytest.raises(ValueError):
        m.add_anchor(10, 20)
