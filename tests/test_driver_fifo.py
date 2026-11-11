import pytest
from conftest import FakeChip, build_packet

from aerostacks_icm45686 import FifoConfig, Icm45686, Icm45686Config
from aerostacks_icm45686 import constants as c


def make(chip, **fifo_kw):
    clock = iter(range(1_000, 10**9, 1_000))
    t = [0.0]

    def mono():
        t[0] += 0.0005
        return t[0]

    imu = Icm45686(
        chip,
        Icm45686Config(),
        fifo=FifoConfig(**fifo_kw),
        sleep=lambda s: None,
        monotonic=mono,
        monotonic_ns=lambda: next(clock),
    )
    return imu


EXPECTED = [
    # (kind, register, value) in order; IREG writes use ("ireg", addr, value).
    ("write", c.REG_MISC2, 0x02),
    ("ireg", c.SREG_CTRL, 0x02),
    ("write", c.EDMP_APEX_EN0, 0x00),
    ("write", c.EDMP_APEX_EN1, 0x00),
    ("write", c.FIFO_CONFIG3, 0x00),
    ("write", c.FIFO_CONFIG0, 0x00),
    ("write", c.ACCEL_CONFIG0, (c.ACCEL_RANGES[4] << 4) | 9),
    ("write", c.GYRO_CONFIG0, (c.GYRO_RANGES[125] << 4) | 9),
    ("ireg", c.ACCEL_SRC_CTRL, 0x01),
    ("ireg", c.GYRO_SRC_CTRL, 0x20),
    ("ireg", c.ACCEL_UI_LPF, 1),
    ("ireg", c.GYRO_UI_LPF, 1),
    ("write", c.ODR_DECIMATE_CONFIG, 0x00),
    ("write", c.FIFO_CONFIG0, 0b011111),
    ("write", c.TMST_WOM_CONFIG, 0x00),
    ("ireg", c.SMC_CONTROL_0, 0x01),
    ("write", c.FIFO_CONFIG4, 0x02),
    ("write", c.FIFO_CONFIG3, 0x0E),
    ("write", c.FIFO_CONFIG2, 0xA0),
    ("write", c.FIFO_CONFIG0, 0b01 << 6 | 0b011111),
    ("write", c.FIFO_CONFIG3, 0x0F),
    ("write", c.PWR_MGMT0, 0x0F),
]


def test_initialize_fifo_write_sequence():
    chip = FakeChip()
    make(chip).initialize_fifo()
    assert chip.log == EXPECTED


def test_initialize_fifo_tolerates_reset_nack():
    chip = FakeChip(reset_nack=True)
    make(chip).initialize_fifo()
    assert chip.regs[c.FIFO_CONFIG0] == 0b01011111
    assert chip.regs[c.FIFO_CONFIG3] == 0x0F


def test_2k_depth_16us_and_stop_on_full():
    chip = FakeChip()
    imu = make(chip, fifo_depth_bytes=2048, tmst_resol_us=16, mode="stop_on_full")
    imu.initialize_fifo()
    assert chip.regs[c.FIFO_CONFIG0] == (0b10 << 6) | 0b000111
    assert chip.regs[c.TMST_WOM_CONFIG] == 0x20


def test_stop_disables_fifo_before_power_down():
    chip = FakeChip()
    imu = make(chip)
    imu.initialize_fifo()
    chip.log.clear()
    imu.stop()
    assert chip.log == [
        ("write", c.FIFO_CONFIG3, 0x0E),
        ("write", c.FIFO_CONFIG0, 0b011111),
        ("write", c.PWR_MGMT0, 0x00),
    ]


def test_read_fifo_single_burst_and_parse():
    chip = FakeChip()
    imu = make(chip)
    imu.initialize_fifo()
    for i in range(200):
        chip.fifo += build_packet(accel=(16384, 0, 0), timestamp=i * 10_000 % 65536)
    batch = imu.read_fifo()
    assert chip.burst_reads == [4000]
    assert len(batch.samples) == 200
    assert batch.fifo_count_bytes == 4000
    assert not batch.was_full
    assert batch.malformed_packets == 0
    assert batch.read_completed_monotonic_ns > 0
    assert not chip.fifo


def test_read_fifo_empty_and_full_and_malformed():
    chip = FakeChip()
    imu = make(chip)
    imu.initialize_fifo()
    batch = imu.read_fifo()
    assert batch.samples == [] and batch.fifo_count_bytes == 0
    assert chip.burst_reads == []
    for _ in range(409):
        chip.fifo += build_packet()
    chip.fifo[20] = 0x00
    batch = imu.read_fifo()
    assert batch.was_full
    assert batch.malformed_packets == 1 and len(batch.samples) == 408


def test_read_fifo_count_in_bytes_mode():
    chip = FakeChip()
    imu = make(chip, count_in_bytes=True)
    imu.initialize_fifo()
    chip.fifo += build_packet() * 3

    orig = chip.read

    def read(register, length):
        if register == c.FIFO_COUNT_0:
            return len(chip.fifo).to_bytes(2, "big")
        return orig(register, length)

    chip.read = read
    assert len(imu.read_fifo().samples) == 3


def test_read_fifo_requires_init():
    with pytest.raises(RuntimeError):
        make(FakeChip()).read_fifo()


def test_polling_initialize_unchanged_and_fifo_rates():
    chip = FakeChip()
    imu = Icm45686(chip, sleep=lambda s: None)
    imu.initialize()
    assert chip.regs[c.FIFO_CONFIG0] == 0  # polling path leaves FIFO alone
    with pytest.raises(ValueError):
        Icm45686Config(rate_hz=400)
    assert FifoConfig(rate_hz=1600).rate_hz == 1600
    with pytest.raises(ValueError):
        FifoConfig(rate_hz=25)
    with pytest.raises(ValueError):
        FifoConfig(fifo_depth_bytes=4096)
