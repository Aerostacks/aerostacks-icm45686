"""Register transports."""

from __future__ import annotations

from typing import Any, Protocol

from smbus2 import i2c_msg


class RegisterBus(Protocol):
    """One device's register transport; implementations own bus addressing."""

    def read(self, register: int, length: int) -> bytes: ...

    def write(self, register: int, data: bytes) -> None: ...


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

    def write(self, register: int, data: bytes) -> None:
        try:
            self.bus.i2c_rdwr(i2c_msg.write(self.address, bytes([register]) + data))
        except OSError as error:
            raise OSError(
                error.errno,
                f"I2C write at addr 0x{self.address:02X}, register 0x{register:02X}: "
                f"{error}",
            ) from error
