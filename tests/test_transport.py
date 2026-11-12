import pytest

from aerostacks_icm45686 import I2cRegisterBus


class StubBus:
    def __init__(self):
        self.calls = []
        self.next = 0

    def i2c_rdwr(self, selector, result):
        n = result.len
        self.calls.append(n)
        result.data = bytes(i % 256 for i in range(self.next, self.next + n))
        self.next += n


def test_read_burst_chunks_only_when_needed(monkeypatch):
    import aerostacks_icm45686.transport as t

    class Msg:
        def __init__(self, length=0):
            self.len = length
            self.data = b""

        def __bytes__(self):
            return self.data

    monkeypatch.setattr(t.i2c_msg, "write", lambda a, d: Msg())
    monkeypatch.setattr(t.i2c_msg, "read", lambda a, n: Msg(n))
    stub = StubBus()
    bus = I2cRegisterBus(stub, 0x68)
    assert len(bus.read_burst(0x14, 4000)) == 4000
    assert stub.calls == [4000]
    stub.calls.clear()
    assert len(bus.read_burst(0x14, 100, max_chunk=40)) == 100
    assert stub.calls == [40, 40, 20]
    with pytest.raises(ValueError):
        bus.read_burst(0x14, -1)
