"""Register transports."""

from __future__ import annotations

from typing import Any, Protocol

from smbus2 import i2c_msg

from .constants import I2C_MAX_READ_BYTES


class RegisterBus(Protocol):
    """One device's register transport; implementations own bus addressing."""

    def read(self, register: int, length: int) -> bytes: ...

    def write(self, register: int, data: bytes) -> None: ...


class BurstRegisterBus(RegisterBus, Protocol):
    """A transport that can also read long runs of bytes from one register."""

    def read_burst(self, register: int, length: int) -> bytes: ...


class I2cRegisterBus:
    """Linux I2C transport using combined transactions via ``smbus2``.

    Avoids SMBus byte-order assumptions. ``bus`` is an open ``smbus2.SMBus``.
    """

    def __init__(self, bus: Any, address: int) -> None:
        self.bus = bus
        self.address = address

    def read(self, register: int, length: int) -> bytes:
        selector = i2c_msg.write(self.address, [register])
        result = i2c_msg.read(self.address, length)
        try:
            self.bus.i2c_rdwr(selector, result)
        except OSError as error:
            raise OSError(
                error.errno,
                f"I2C read at addr 0x{self.address:02X}, register 0x{register:02X}: "
                f"{error}",
            ) from error
        return bytes(result)

    def read_burst(
        self, register: int, length: int, max_chunk: int = I2C_MAX_READ_BYTES
    ) -> bytes:
        """Read ``length`` bytes from one register, e.g. the FIFO data port.

        Uses ``i2c_rdwr`` (not SMBus block reads, which stop at 32 bytes). Reads
        are split into ``max_chunk``-byte transactions only when ``length``
        exceeds it; Linux i2c-dev rejects messages over 8192 bytes. Each chunk
        re-sends the register address, which is correct for a FIFO port because
        the FIFO read pointer, not the I2C address, advances.
        """
        if length < 0 or max_chunk <= 0:
            raise ValueError("length must be >= 0 and max_chunk positive")
        chunks = []
        remaining = length
        while remaining > 0:
            step = min(remaining, max_chunk)
            chunks.append(self.read(register, step))
            remaining -= step
        return b"".join(chunks)

    def write(self, register: int, data: bytes) -> None:
        try:
            self.bus.i2c_rdwr(i2c_msg.write(self.address, bytes([register]) + data))
        except OSError as error:
            raise OSError(
                error.errno,
                f"I2C write at addr 0x{self.address:02X}, register 0x{register:02X}: "
                f"{error}",
            ) from error
