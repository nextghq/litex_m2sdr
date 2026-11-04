#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Sample-count time base.

Time is the index of a sample ("tick"), counted in the RFIC clock domain by the AD9361 core: the
counter advances by the number of sample periods in one PHY word (1 in 2R2T, 2 in 1R1T) on every RX
word strobe, whether or not the word is consumed. It is the only clock the ADC and DAC really share,
so stamping RX frames and releasing TX frames against it is exact by construction -- no nanosecond
conversion and no second oscillator involved (what a USRP calls ticks).

* ``RXTickTracker`` (sys): each RX word crosses the clock-domain FIFO and the RX pipeline register
  together with its tick. The tracker strips it and exposes, for the word currently offered to the
  consumer (the RX header inserter), the tick it was sampled at. A frame header stamped with that value is the index of the
  frame's first sample even if samples were dropped upstream.


"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from litepcie.common import dma_layout

# Layouts ------------------------------------------------------------------------------------------

def rx_tick_layout(data_width=64):
    return [("data", data_width), ("tick", 64)]


# RX Tick Tracker (sys) ----------------------------------------------------------------------------

class RXTickTracker(LiteXModule):
    """Strips the tick off the RX words and exposes the tick of the word currently offered downstream.

    It sits right after the RX pipeline register (which carries data and tick together) and in front
    of the bit-mode stage. In the 12-bit/SC16 format that stage is a wire, so the word offered to the
    final consumer (the RX header inserter) is the word offered here and ``tick`` is its sample index.
    In the repacking formats (8-bit, BFP8) words are not 1:1 and ``tick`` falls back to ``now``.
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(rx_tick_layout())  # i (from the RX pipeline register)
        self.source = source = stream.Endpoint(dma_layout(64))    # o (to bit-mode / consumer)

        self.inc   = Signal(2)  # i: sample periods per word.
        self.exact = Signal()   # i: the consumer sees these words 1:1 (12-bit/SC16 mode).

        self.tick  = Signal(64) # o: tick of the word offered to the consumer.
        self.now   = Signal(64) # o: tick following the newest word taken (sys view of now).

        # # #

        self.comb += [
            sink.connect(source, omit={"tick"}),
            If(self.exact & sink.valid,
                self.tick.eq(sink.tick)
            ).Else(
                self.tick.eq(self.now)
            ),
        ]
        self.sync += If(sink.valid & sink.ready, self.now.eq(sink.tick + self.inc))


# RX Tick Attach (rfic) ----------------------------------------------------------------------------

class RXTickAttach(LiteXModule):
    """Attaches the current sample tick to each RX word, in the rfic domain.

    Placed immediately before the clock-domain crossing, i.e. AFTER any rfic-domain repacking (the
    oversampling 64->128 gather), so the tick carried with a word is the sample index of that word's
    FIRST sample whatever the word width.
    """
    def __init__(self, data_width=64):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))      # i (from the packer/gather)
        self.source = source = stream.Endpoint(rx_tick_layout(data_width))  # o (to the CDC)
        self.tick   = Signal(64)                                            # i (rfic): counter value.

        # # #

        self.comb += [
            sink.connect(source, omit={"tick"}),
            source.tick.eq(self.tick),
        ]
