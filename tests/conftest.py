"""Test helpers: synthetic packets and a fake register bus."""

from __future__ import annotations

import pytest

from aerostacks_icm45686 import constants as c


def build_packet(
    accel=(0, 0, 0),
    gyro=(0, 0, 0),
    temp=0,
    timestamp=0,
    header=0x78,
) -> bytes:
    """Big-endian 20-byte hi-res packet per datasheet section 6.1."""
    out = bytearray([header])
    nibbles = [0, 0, 0]
    raws = [*accel, *gyro]
    words = []
    for index, raw in enumerate(raws):
        value = raw & 0xFFFFF
        words.append(value >> 4)
        nibble = value & 0x0F
        if index < 3:
            nibbles[index] |= nibble << 4
        else:
            nibbles[index - 3] |= nibble
    for word in words:
        out += word.to_bytes(2, "big")
    out += (temp & 0xFFFF).to_bytes(2, "big")
    out += (timestamp & 0xFFFF).to_bytes(2, "big")
    out += bytes(nibbles)
    assert len(out) == 20
    return bytes(out)


class FakeChip:
    """Register-level ICM-45686 stand-in recording every bus transaction."""

    def __init__(self, *, reset_nack: bool = False) -> None:
        self.regs = dict.fromkeys(range(256), 0)
        self.regs[c.WHO_AM_I] = c.DEVICE_ID
        self.regs[c.FIFO_CONFIG2] = 0x20
        self.ireg = {}
        self.ireg_addr = 0
        self.log: list[tuple] = []
        self.fifo = bytearray()
        self.reset_nack = reset_nack
        self.burst_reads: list[int] = []

    def read(self, register: int, length: int) -> bytes:
        if register == c.FIFO_COUNT_0:
            return len(self.fifo_packets()).to_bytes(2, "big")[:length]
        if register == c.FIFO_DATA:
            self.burst_reads.append(length)
            data = bytes(self.fifo[:length])
            del self.fifo[:length]
            return data
        if register == c.IREG_DATA:
            return bytes([self.ireg.get(self.ireg_addr, 0)])
        return bytes(self.regs[register + i] for i in range(length))

    def fifo_packets(self) -> list[bytes]:
        return [bytes(self.fifo[i : i + 20]) for i in range(0, len(self.fifo), 20)]

    def write(self, register: int, data: bytes) -> None:
        if register == c.REG_MISC2:
            self.log.append(("write", register, data[0]))
            self.regs[c.STATUS_REGISTER] = c.RESET_DONE
            if self.reset_nack:
                raise OSError(121, "NACK")
            return
        if register == c.IREG_ADDR:
            self.ireg_addr = int.from_bytes(data[:2], "big")
            if len(data) == 3:
                self.ireg[self.ireg_addr] = data[2]
                self.log.append(("ireg", self.ireg_addr, data[2]))
            return
        self.log.append(("write", register, data[0]))
        value = data[0]
        if register == c.FIFO_CONFIG2:
            value &= ~c.FIFO_FLUSH
        self.regs[register] = value

    def writes(self):
        return [entry[1:] for entry in self.log]


@pytest.fixture
def chip():
    return FakeChip()
