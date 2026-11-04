#!/usr/bin/env python3

#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Sample-count time base: the RX tick tracker, and RX frames stamped with a sample index."""

from migen import *
from migen.sim import passive

from litex.gen.sim import run_simulation

from litex.soc.interconnect import stream

from litex_m2sdr.gateware.sample_time import RXTickAttach, RXTickTracker, rx_tick_layout
from litex_m2sdr.gateware.header      import RXHeaderInserter

INC            = 1          # 2R2T: one PHY word is one sample period.
RX_FRAME_WORDS = 8
RX_SYNC        = 0x5aa5_5aa5_5aa5_5aa5

# Tick tracker ------------------------------------------------------------------------------------

def test_tracker_reports_the_tick_of_the_word_it_offers():
    dut  = RXTickTracker()
    seen = []

    def producer():
        yield dut.inc.eq(INC)
        yield dut.exact.eq(1)
        tick = 5000
        for i in range(30):
            if i == 17:
                tick += 3 * INC                         # three words dropped upstream
            yield dut.sink.data.eq(0x1000 + i)
            yield dut.sink.tick.eq(tick)
            yield dut.sink.valid.eq(1)
            yield
            while not (yield dut.sink.ready):
                yield
            yield dut.sink.valid.eq(0)
            yield
            tick += INC
        for _ in range(20):
            yield
        seen.append(("now", (yield dut.now)))

    @passive
    def consumer():
        n = 0
        while True:
            yield dut.source.ready.eq(1 if (n % 3) == 0 else 0)
            yield
            if (yield dut.source.valid) and (yield dut.source.ready):
                seen.append(((yield dut.source.data), (yield dut.tick)))
            n += 1

    run_simulation(dut, [producer(), consumer()])
    words = [x for x in seen if x[0] != "now"]
    assert [d for d, _ in words] == [0x1000 + i for i in range(30)]
    assert [t for _, t in words] == [5000 + INC*i + (3*INC if i >= 17 else 0) for i in range(30)]
    assert seen[-1] == ("now", 5000 + INC*29 + 3*INC + INC)

# RX chain: attach -> pipeline register -> tracker -> header inserter ------------------------------

class RXChain(Module):
    """As in the AD9361 core: the tick is attached to each word, rides the pipeline register, is
    stripped by the tracker, and is what the header inserter stamps the frame with."""
    def __init__(self):
        self.submodules.attach   = attach   = RXTickAttach(64)
        self.submodules.buffer   = buffer   = stream.Buffer(rx_tick_layout(64))
        self.submodules.tracker  = tracker  = RXTickTracker()
        self.submodules.inserter = inserter = RXHeaderInserter(data_width=64, with_csr=False)
        self.sink = attach.sink
        self.tick = attach.tick
        self.comb += [
            attach.source.connect(buffer.sink),
            buffer.source.connect(tracker.sink),
            tracker.source.connect(inserter.sink),
            tracker.inc.eq(INC),
            tracker.exact.eq(1),
            inserter.timestamp.eq(tracker.tick),
            inserter.stamp_on_payload.eq(1),         # stamping with a per-word index
            inserter.header.eq(RX_SYNC),
            inserter.enable.eq(1),
            inserter.header_enable.eq(1),
            inserter.frame_cycles.eq(RX_FRAME_WORDS),
        ]

def _rx_chain(drops=(), words=160, period=3):
    dut = RXChain()
    out = []

    def producer():
        tick = 9000
        for i in range(words):
            if i in drops:
                tick += INC * drops[i]                 # samples lost upstream of the attach
            yield dut.tick.eq(tick)
            yield dut.sink.data.eq(tick)               # payload == its own index, easy to check
            yield dut.sink.valid.eq(1)
            yield
            while not (yield dut.sink.ready):
                yield
            yield dut.sink.valid.eq(0)
            for _ in range(period):
                yield
            tick += INC
        for _ in range(40):
            yield

    @passive
    def consumer():
        while True:
            yield dut.inserter.source.ready.eq(1)
            yield
            if (yield dut.inserter.source.valid) and (yield dut.inserter.source.ready):
                out.append(((yield dut.inserter.source.data), (yield dut.inserter.source.first)))

    run_simulation(dut, [producer(), consumer()])

    # Split the stream into frames: sync word, timestamp word, then RX_FRAME_WORDS payload words.
    frames, i = [], 0
    while i + 1 < len(out):
        if out[i][0] != RX_SYNC:
            i += 1
            continue
        stamp   = out[i+1][0]
        payload = [d for d, _ in out[i+2:i+2+RX_FRAME_WORDS]]
        if len(payload) == RX_FRAME_WORDS:
            frames.append((stamp, payload))
        i += 2 + RX_FRAME_WORDS
    return frames

def test_rx_frame_stamp_is_the_tick_of_its_first_payload_word():
    frames = _rx_chain()
    assert len(frames) >= 8, len(frames)
    for stamp, payload in frames:
        assert stamp == payload[0], (stamp, payload[0])
        assert payload == [payload[0] + INC*k for k in range(RX_FRAME_WORDS)]

def test_rx_stamps_stay_exact_across_dropped_samples():
    frames = _rx_chain(drops={13: 5, 40: 1, 77: 200, 95: 3})
    assert len(frames) >= 8, len(frames)
    gaps = 0
    for stamp, payload in frames:
        assert stamp == payload[0], (stamp, payload)   # still the first payload word's index
        gaps += sum(1 for a, b in zip(payload, payload[1:]) if b - a != INC)
    assert gaps >= 2, gaps                             # the drops are visible inside frames

if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"  PASS {name}")
