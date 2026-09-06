# CAN and CAN FD

A working lesson in three parts.

**Part I** teaches the protocol on its own terms. No project in it — just how a
CAN bus works, why it was built that way, and the arithmetic you have to be
able to redo. Worked examples use whatever numbers make the point clearest.

**Part II** comes back to this repository and maps the firmware onto Part I at
the level of "which function does what, and why it is shaped like that". It is
deliberately not a line-by-line audit.

**Part III** is exercises with answers.

Read Part I in order the first time. Part II assumes it.


# Part I — How CAN works

## 1. The bus

### 1.1 One differential pair, no clock, no master

A CAN bus is two wires, CANH and CANL, shared by every node. There is no clock
line, no chip select, no master, and no arbiter. Every node is a peer, every
node hears everything, and the wire itself resolves who gets to talk.

A bit is carried as a voltage *difference* between the two wires, not as either
wire against ground. Two states exist:

- **Recessive**, differential near 0 V, logical **1**. Nobody is driving.
- **Dominant**, differential around 2 V, logical **0**. Somebody is driving.

Receivers threshold on the difference. That buys three things a single-ended
line does not have:

- Noise couples into CANH and CANL almost equally, moving both wires and
  leaving the difference alone. Near switching motor currents this is not a
  nicety.
- There is no shared ground reference to get wrong. CAN receivers tolerate a
  wide common-mode range, typically -12 V to +12 V, so two nodes' grounds can
  sit volts apart and the link still works.
- Emissions largely cancel, because the two currents are equal and opposite.

Twist the pair. The cancellation depends on both wires seeing the same field,
which is what twisting enforces.

### 1.2 Dominant and recessive: the wired-AND

This is the single most important idea in CAN, and everything else falls out of
it. The bus is a **wired-AND**. If any node drives dominant, the bus is
dominant. Recessive happens only when *every* node has released.

```
  node A   1   1   0   1
  node B   1   0   1   1
  node C   1   1   1   0
  ------------------------
  bus      1   0   0   0
```

The asymmetry is deliberate: dominant is actively driven, recessive is merely
released. A node cannot force the bus recessive, only dominant.

Two consequences carry the entire protocol.

**Every transmitter also receives.** While transmitting, a node samples the bus
on every bit and compares it against what it meant to send. If it sent
recessive and read dominant, someone else overrode it. If it sent dominant and
read recessive, something is broken.

**Losing costs nothing.** During the identifier field, a node that sent
recessive and read dominant simply stops transmitting and starts receiving the
frame that beat it. It corrupted nothing. The winner never noticed. The loser
still gets the message, and can retry when the bus goes idle. No slot was
wasted and no central node decided anything.

That is why CAN needs no master and no scheduler, and why it stays
deterministic under load: the highest-priority message pending anywhere on the
bus is always the one being sent.

### 1.3 What a transceiver does

Between the controller's two logic pins and the differential pair sits a
transceiver. It does four things, none of which an MCU pin can:

1. **Single-ended to differential, one direction each.** TXD in, CANH/CANL out;
   CANH/CANL in, RXD out.
2. **Open-drain behaviour on the bus.** For recessive, the output drivers are
   *off* and the bus floats to its idle level, set by the termination and the
   receivers' bias. For dominant, it actively drives CANH high and CANL low.
   This is what makes the wired-AND of 1.2 physically real.
3. **Level translation and protection.** 5 V bus levels from a 3.3 V MCU, plus
   ESD, short-circuit and thermal protection on the bus pins.
4. **Bit-rate-grade timing.** A transceiver is specified for a maximum **loop
   delay** — TXD edge in to RXD edge out, through the bus and back. Section 3.3
   turns that number into a constraint you must satisfy.

Two practical notes that cost people days.

Most transceivers have a **standby / silent pin** (`S`, `STB`, or similar) that
must be asserted to the active level — usually pulled **low** — for the part to
transmit. Left floating, many parts come up listen-only: the controller
transmits, nothing reaches the wire, and software sees no error it recognises.
A missing pulldown resistor is one of the two most common reasons a first CAN
board does not work.

If the MCU is 3.3 V, you want a transceiver with a separate `VIO` supply pin
(on the common TJA1051 family this is the `/3` suffix) so RXD comes back at
3.3 V levels rather than 5 V.

### 1.4 Why an MCU pin is not a bus

The controller's TX and RX pins carry the same bits, but as ordinary
push-pull CMOS logic referenced to local ground:

- A push-pull output drives *both* levels hard. A CAN bus needs one level driven
  and one released, or the wired-AND cannot work at all.
- A GPIO cannot source the current a terminated bus draws. Two 120 ohm
  terminators in parallel is 60 ohm, and a 2 V dominant differential across
  that is about 33 mA — outside what a typical MCU pin should be asked to do.
- There is no fault protection and no common-mode range. A shorted bus takes
  the pin, and probably the port, with it.

One configuration detail worth knowing now: it is worth putting a **pull-up on
the RX pin**. If the transceiver is absent or unpowered, RX floats, and a
floating input is sampled as noise — the controller will report stuff and form
errors for signals that were never on any wire. A pull-up makes an absent
transceiver read as a permanently idle, permanently recessive bus, which is a
much more honest thing to debug. Leave TX without a pull; the transceiver has
its own TXD pull-up, so that an MCU in reset, whose pins revert to floating
inputs, cannot hold the bus dominant.


## 2. Arbitration

### 2.1 How priority falls out of the wired-AND

Arbitration happens over the identifier field, most significant bit first. For
a standard frame that is 11 bits, ID10 down to ID0.

Every node that wants to transmit starts at the same time — when the bus goes
idle — and sends its identifier. On each bit, a node that sends recessive and
reads dominant has just been outbid, so it stops and receives instead.

The result: **the numerically lowest identifier wins**, and it wins at the most
significant bit where the identifiers differ. Low ID means more dominant bits
early means higher priority.

Three nodes with identifiers 0x100, 0x200 and 0x207 starting together:

```
                  ID10 ID9 ID8 ID7 ID6 ID5 ID4 ID3 ID2 ID1 ID0
  0x100              0   0   1   0   0   0   0   0   0   0   0
  0x200              0   1   0   0   0   0   0   0   0   0   0
  0x207              0   1   0   0   0   0   0   0   1   1   1
  ----------------------------------------------------------------
  bus                0   0   1   0   0   0   0   0   0   0   0
                         ^
                         0x200 and 0x207 send recessive and read
                         dominant. Both lose here, on the same bit.
```

0x100 wins on ID9 and the other two back off, having transmitted nothing but a
start bit and one identifier bit. When the bus next goes idle they both start
again, and now:

```
                  ID10 ID9 ID8 ID7 ID6 ID5 ID4 ID3 ID2 ID1 ID0
  0x200              0   1   0   0   0   0   0   0   0   0   0
  0x207              0   1   0   0   0   0   0   0   1   1   1
                                                     ^
                                                     0x207 loses at ID2
```

A set of replies serialises itself in identifier order, deterministically, with
no collisions and no retries. Nothing coordinated that.

### 2.2 Designing an identifier scheme

Identifiers are a priority space, so lay them out on purpose.

**Put control above telemetry.** If a controller sends commands and devices
send feedback, commands should have the lower identifiers. Otherwise a burst of
feedback frames sits in front of every command and the control loop slips.

**Leave the top of the space free for safety traffic.** An emergency stop that
cannot preempt a routine command is not an emergency stop. Reserving the lowest
block for faults and e-stop costs nothing up front and cannot be retrofitted
cheaply.

```
  0x000 - 0x0FF   emergency stop, faults        (beats everything)
  0x100 - 0x1FF   commands
  0x200 - 0x2FF   feedback / telemetry
  0x300 - 0x3FF   configuration
```

**One broadcast usually beats many unicasts.** One frame carrying eight values
costs a single arbitration and a single set of frame overhead. Eight small
frames pay that overhead eight times, and can easily cost twice the wire time
for the same payload. The trade is that one corrupted broadcast misses all
recipients for a cycle — but CAN retransmits automatically on error, so if bus
load is low this is usually the right call.

Two hard rules:

- The seven most significant bits of a standard identifier must not all be
  recessive, which makes 0x7F0 to 0x7FF unusable.
- **Exactly one transmitter per identifier.** Two nodes sending different
  content under the same ID tie all the way through arbitration, neither
  realises it lost, and then they destroy each other at the first differing
  data bit — outside the arbitration field, so it counts as a bit error rather
  than a lost arbitration. Both retransmit and collide again.


## 3. Bit timing

### 3.1 Time quanta and the four segments

There is no clock line, so every node reconstructs bit boundaries from its own
oscillator plus the edges in the data. Getting every node to agree on where a
bit starts and when to sample it is what bit timing configuration is for.

Everything is built from one unit. The controller divides its kernel clock by a
prescaler to get a **time quantum**:

```
  t_q = prescaler / f_kernel
```

A bit is an integer number of time quanta, split into segments:

```
|<------------------- 1 bit time = N_tq time quanta ------------------>|
|      |                                                     |         |
| SYNC |   Prop_Seg        |   Phase_Seg1                     | Phase_Seg2
| 1 tq |                                                      |
+------+----------------------------------------------------+----------+
       ^                                                     ^
       an edge is expected here                         sample point
```

- **SYNC_SEG** is always exactly 1 tq. Edges are supposed to fall inside it.
- **Prop_Seg** absorbs signal propagation. It must be long enough that a bit
  driven by the far end of the bus has arrived and settled before sampling.
- **Phase_Seg1** and **Phase_Seg2** straddle the sample point and are the parts
  that get stretched or shortened during resynchronisation.
- **The sample point** is the boundary between Phase_Seg1 and Phase_Seg2. Every
  node reads the bit value at this instant.

Most controller APIs, including ST's, do not expose Prop_Seg separately.
`TimeSeg1` is `Prop_Seg + Phase_Seg1` lumped together and `TimeSeg2` is
Phase_Seg2. So:

```
  N_tq         = 1 + TimeSeg1 + TimeSeg2
  bit rate     = f_kernel / (prescaler * N_tq)
  sample point = (1 + TimeSeg1) / N_tq
```

Registers typically store these values minus one and add the one back in
hardware. Give the driver the true segment lengths, not the register values.

### 3.2 Deriving a configuration

Four steps. Take a 40 MHz kernel clock and a target of 500 kbit/s at an 80%
sample point.

**Step 1 — the bit rate equation has one unknown product.**

```
  prescaler * N_tq = f_kernel / bit_rate = 40e6 / 500e3 = 80
```

**Step 2 — factor it, and discard the unusable.** Both segments have a minimum
of 2 tq in most controllers, so N_tq below about 5 cannot express a sensible
sample point.

```
  prescaler 1,  N_tq 80   t_q = 25 ns    usable, finest resolution
  prescaler 2,  N_tq 40   t_q = 50 ns    usable
  prescaler 4,  N_tq 20   t_q = 100 ns   usable
  prescaler 8,  N_tq 10   t_q = 200 ns   usable, coarse
  prescaler 16, N_tq 5    t_q = 400 ns   best sample point is 3/5 = 60%
```

**Step 3 — split N_tq for the sample point.** You want
`1 + TimeSeg1 = SP * N_tq`. Taking N_tq = 20:

```
  1 + TimeSeg1 = 0.80 * 20 = 16   ->   TimeSeg1 = 15, TimeSeg2 = 4
  SJW = TimeSeg2 = 4
```

**Step 4 — check the division came out exact.**

```
  40e6 / (4 * 20) = 500 000 bit/s     exact
  (1 + 15) / 20   = 0.80              exact
```

If it does not come out exact, the bit rate is wrong and no amount of software
will tell you. Which brings us to the trap worth internalising now: **the same
register values with a different kernel clock give a different bit rate,
silently.** If a configuration derived for 50 MHz is left in place when the
clock becomes 75 MHz, the bus runs at 1.5x the intended rate and nothing in
software reports it. Always confirm the kernel clock is what you think it is.

### 3.3 The sample point pays for propagation delay

Prop_Seg must cover a full round trip: the local transceiver drives the bus,
the signal crosses the cable, the far transceiver converts it to logic, that
node responds dominant, and its dominant bit gets all the way back and settles
before the local sample point.

```
  Prop_Seg  >=  2 x (transceiver loop delay + cable delay)
```

Copper is about 5 ns per metre. Suppose a bit is 15 tq of 66.67 ns, with
TimeSeg1 = 12 and TimeSeg2 = 2. Keeping `Phase_Seg1 >= Phase_Seg2` so that
section 4's `min(PhSeg1, PhSeg2)` stays equal to Phase_Seg2:

```
  Prop_Seg <= 12 - 2 = 10 tq = 667 ns
  transceiver loop + cable <= 333 ns
  over 2 m of cable (10 ns): loop delay budget = 323 ns
```

Now move the sample point earlier to 80%, TimeSeg1 = 11 and TimeSeg2 = 3:

```
  Prop_Seg <= 11 - 3 = 8 tq = 533 ns
  transceiver loop + cable <= 267 ns
  over 2 m: loop delay budget = 257 ns
```

A transceiver specified at 250 ns worst case fits the first with 73 ns to
spare and the second with 7 ns. Seven nanoseconds is not margin, it is luck.

So a later sample point buys propagation budget. Section 4 is where the bill
arrives.

### 3.4 Synchronisation and the jump width

Every receiver runs its own bit-time counter off its own oscillator. Two
mechanisms keep those counters aligned:

- **Hard synchronisation** happens once per frame, on the falling edge that
  starts it. The receiver restarts its counter so that edge lands in SYNC_SEG.
- **Resynchronisation** happens on every subsequent recessive-to-dominant edge.
  An edge arriving *late* (receiver running fast) lengthens Phase_Seg1; an edge
  arriving *early* (receiver running slow) shortens Phase_Seg2. Either way the
  sample point is pushed back toward the middle of the incoming bit.

The **synchronisation jump width (SJW)** caps how much a single edge is allowed
to move things. Two hard rules:

```
  SJW <= Phase_Seg1        and        SJW <= Phase_Seg2
```

Phase_Seg2 is almost always the smaller, so `SJW = TimeSeg2` is the standard
choice, and the one to use unless you have a reason not to.

**Bit stuffing** guarantees the edges exist. After five consecutive bits of the
same value the transmitter inserts one of the opposite value, which receivers
remove. So there is a resynchronisation opportunity at least every ten bits.
That "ten bits" is the constant in the next section's first formula; the
"thirteen bits" in the second is the worst-case span with no usable edge at
all, during an error flag.


## 4. Oscillator tolerance

### 4.1 The two conditions

Two nodes must stay aligned for a whole frame using resynchronisation alone.
ISO 11898-1 turns that into two constraints. `df` is the fractional frequency
error of one node relative to nominal; the factor of two is there because two
nodes can be off in opposite directions.

```
  (1)  df  <=  SJW / (2 * 10 * N_tq)

  (2)  df  <=  min(PhSeg1, PhSeg2) / (2 * (13 * N_tq - PhSeg2))
```

Condition (1) is the resynchronisation budget: over the ten bits between
guaranteed edges, the drift two nodes accumulate must be recoverable by one SJW
correction.

Condition (2) is the no-edge case: during an error flag there can be thirteen
bit times with no usable resynchronisation, and the drift over that span must
not push the sample point out of the bit.

Your tolerance is the smaller of the two. **Condition (2) is almost always the
binding one.**

### 4.2 Tolerance follows the sample point

Here is the part worth remembering. With `SJW = Phase_Seg2`, condition (2)
reduces to

```
  df = (1 - SP) / (2 * (13 - (1 - SP)))
```

which depends **only on the sample point** — not on how many time quanta you
used. Sweeping one 15 tq budget:

| TimeSeg1 | TimeSeg2 | SJW | sample point | max df |
|---|---|---|---|---|
| 13 | 1 | 1 | 93.3% | 0.258% |
| 12 | 2 | 2 | 86.7% | 0.518% |
| 11 | 3 | 3 | 80.0% | 0.781% |
| 10 | 4 | 4 | 73.3% | 1.047% |
| 9 | 5 | 5 | 66.7% | 1.316% |

Worked, for the 86.7% row:

```
  (1)  2 / (2 * 10 * 15)              = 2 / 300  = 0.667%
  (2)  min(12,2) / (2 * (13*15 - 2))  = 2 / 386  = 0.518%     binds
```

**Adding time quanta does not buy tolerance.** An 80% sample point gives
0.781% whether you build it from 15, 25 or 75 tq. What more time quanta buy is
*resolution* — at 15 tq the sample point moves in 6.7% steps, at 25 tq in 4.0%
steps, at 75 tq in 1.3% steps. That matters when a device vendor specifies a
sample point you need to land near.

This is the whole trade-off in one line: **later sample point buys propagation
delay and costs oscillator tolerance; earlier does the reverse.** 75% to 87.5%
is the usual band; automotive tends to 80%.

Note also that nodes do **not** have to agree on sample point. They must agree
on the bit *rate*. Each node independently picks its own prescaler, tq count
and sample point, and independently has to satisfy its own tolerance and
propagation constraints.

### 4.3 Crystal or RC: an engineering decision

Put two candidate clock sources against a budget of, say, 0.5%.

**Internal RC oscillator.** A typical MCU's internal RC is factory trimmed to
roughly +/- 1% at room temperature, drifting to several percent across the full
operating temperature range. At room temperature, before anything gets warm,
the *specified* accuracy is already about twice the budget.

**Crystal.** A commodity crystal is +/- 30 to +/- 50 ppm, which is 0.003% to
0.005%. Add load-capacitor error and ageing and call it 100 ppm. That is fifty
times inside the budget.

The gap is not marginal, it is two orders of magnitude. This is why every CAN
reference tells you to use a crystal, and it is worth understanding *how* an RC
oscillator fails, because the failure is maddening rather than obvious: at 25 C
on the bench it will often work. It starts failing when the board warms up, or
when one node sits near a heat source and another does not, and it presents as
random CRC and form errors rather than anything you can point at.

Three consequences:

1. The budget applies to **every node on the bus**, not just yours. One
   RC-clocked node corrupts frames for everybody, including frames it has no
   part in, because its error flags are dominant bits on a shared wire.
2. Budget the whole stack — initial tolerance, temperature drift, ageing, load
   capacitance mismatch — not just the datasheet's initial accuracy figure.
3. If you are tight, moving the sample point earlier buys tolerance back at the
   cost of propagation margin (3.3). Whether that trade is available depends on
   your transceiver and cable length.


## 5. CAN FD

### 5.1 What FD changes

Three things:

1. **Payload up to 64 bytes** instead of 8.
2. **Bit rate switching (BRS)**, where the data phase runs faster than the
   arbitration phase.
3. **Stronger CRC** (17 bits for payloads up to 16 bytes, 21 above) plus a
   stuff-count field, so long frames are protected properly.

In the frame itself: the RTR bit is gone (FD has no remote frames), a reserved
bit becomes FDF to mark the frame as FD, and ESI appears to advertise whether
the transmitter is error-passive.

### 5.2 The compatibility trap

An FD controller will happily **receive** classic frames. Mixed traffic in that
direction is fine.

The reverse is not. A classic-only controller that sees an FD frame reads the
FDF bit as a frame-format violation, declares a form error, and transmits an
active error flag — six dominant bits, on the shared wire, in the middle of
your frame. It destroys that frame for every node, drives its own receive error
counter up and the transmitter's too via the retransmission.

**One classic-only node makes an FD bus unusable.** Before putting FD traffic
on a shared bus, confirm every node is FD-*capable*, not merely FD-*tolerant*
(which only means it stays quiet). This is a datasheet check, and it is the
most expensive thing to discover late.

### 5.3 DLC is a lookup table

The data length code on the wire is four bits. Four bits is sixteen values, and
CAN FD needs seventeen useful lengths. It cannot have them, so above 8 bytes
the encoding becomes a table with gaps:

| DLC | bytes | | DLC | bytes |
|---|---|---|---|---|
| 0-8 | 0-8 | | 12 | 24 |
| 9 | 12 | | 13 | 32 |
| 10 | 16 | | 14 | 48 |
| 11 | 20 | | 15 | 64 |

Two consequences.

**There is no 36-byte frame.** Above 32 the next size is 48; between 8 and 12
there is nothing. Design payloads to land on 0-8, 12, 16, 20, 24, 32, 48 or 64,
or pay for padding on the wire.

**A received length comes back as the encoded constant, not a byte count.** If
you copy a fixed number of bytes out of the receive buffer without decoding it,
a short frame leaves you reading stale bytes from the previous message. Decode
it:

```c
static uint8_t dlc_to_bytes(uint32_t dlc)
{
  static const uint8_t tbl[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  return tbl[dlc & 0x0F];
}
```

This is also why drivers give you named constants. A typo like
`DataLength = 32` would be truncated into the 4-bit field and quietly send a
64-byte frame; `FDCAN_DLC_BYTES_32` cannot.

### 5.4 Bit rate switching

With BRS on, a frame runs at the nominal bit rate through arbitration and the
acknowledge slot, then switches to a faster rate for the payload and CRC. It
has to work that way: arbitration requires every node to see every bit within
one bit time, which caps the arbitration rate, whereas the data phase has only
one transmitter and can go faster.

BRS is not free:

- It needs its own set of data-phase timing parameters. If those are simply a
  copy of the nominal timing, enabling BRS changes nothing at all.
- Above roughly 1 Mbit/s in the data phase, the transceiver loop delay no
  longer fits inside a data bit and you must configure **transmitter delay
  compensation**.
- You inherit a second pair of oscillator tolerance conditions for the data
  phase, and the GPIO slew rate on the TX pin starts to matter.

If bus load is low, BRS solves a problem you do not have. Turn it on when the
bandwidth arithmetic says you need it, not before.


## 6. Inside the controller

### 6.1 Acceptance filtering

Frames are filtered in hardware before they ever reach software. A **mask
filter** accepts when

```
  (received_id & mask) == (filter_id & mask)
```

The mask says which bits must match; zero bits are wildcards. So filter 0x200
with mask 0x700 pins only the top three bits of an 11-bit ID:

```
  0x200 = 0b010 0000 0000
  0x700 = 0b111 0000 0000     only the top three bits must match

  accepted:  0b010 xxxx xxxx  ->  0x200 through 0x2FF   (256 identifiers)
```

If you only meant to accept eight of those, tighten the mask (0x7F8 pins all
but the low three bits, giving exactly 0x200 to 0x207) or use a **range
filter**, which takes a low and a high identifier and is far easier to read six
months later.

A **global filter** decides what happens to frames matching no filter at all —
accept into a FIFO, or reject in hardware. Reject is usually right: anything
you did not ask for costs FIFO slots and CPU to discard in software.

Most controllers also offer routing to a **dedicated receive buffer** rather
than a FIFO. One filter per device, each pointing at its own buffer, gives each
device a private mailbox: no queue, so a talkative node cannot displace a quiet
one, the contents are always that node's most recent frame, and a per-buffer
"new data" flag gives you a liveness indicator for free.

### 6.2 FIFOs, and what a full one does

A receive FIFO defaults to **blocking mode**, and its behaviour when full is
worth knowing exactly:

- The incoming frame is filtered, accepted, and then **dropped**.
- A "message lost" flag is set, and an interrupt fires *if you enabled it*.
- **The node still acknowledges the frame.** Acceptance filtering and ACK
  generation happen in the protocol engine, before the FIFO write. The sender
  sees a successful transmission and moves on.

So a full FIFO is silent data loss that looks lossless from both ends. Worse,
in blocking mode what you lose is the *newest* frame while the FIFO keeps the
oldest — for control feedback, exactly backwards.

Overwrite mode discards the oldest instead, but introduces a subtlety: a read
that races an overwrite can return an inconsistent element, so there is a
prescribed read sequence to follow. The better answer is usually to not fill
the FIFO, and to *notice* if you ever do — enable the message-lost interrupt
rather than assuming.

### 6.3 Message RAM

Modern CAN controllers give you a single pool of RAM and let you carve it up:
filters, receive FIFOs, receive buffers, transmit event FIFO, transmit buffers,
transmit FIFO. You declare how many of each and how large the data field is,
and everything is allocated from a base offset upward in a fixed order.

Two things to watch:

- **A frame longer than the configured element size is silently truncated, not
  rejected.** Sizing elements down to save RAM is a real trade.
- **If the pool is shared between two CAN instances, the second one's base
  offset is yours to compute.** Drivers generally will not do it for you, and
  will happily let two peripherals overwrite each other's buffers — which
  presents as random corruption in both.


## 7. Errors and fault confinement

### 7.1 The five errors

Every node detects all five, not just the two involved in a transfer:

| error | condition |
|---|---|
| **bit** | transmitted one level, sampled the other (outside arbitration and ACK) |
| **stuff** | six consecutive bits of the same value where stuffing forbids it |
| **form** | a fixed-form field held the wrong value (delimiters, EOF, FDF) |
| **CRC** | computed CRC differs from the received one |
| **ACK** | transmitter sampled recessive in the acknowledge slot |

The two bit-error directions are the most useful diagnostic split you have:
sent dominant, read recessive means *my transmitter is not reaching the bus*;
sent recessive, read dominant means *something else is driving the bus*.

A node detecting any of these transmits an **error flag**: six dominant bits.
That deliberately violates bit stuffing, so every other node sees a stuff error
and joins in, and the frame is destroyed globally. The transmitter then
retries. Error handling is a bus-wide property, not a per-link one.

### 7.2 Error-active, error-passive, bus-off

Two hardware counters: transmit error count (TEC) and receive error count
(REC).

```
  TEC <= 127 and REC <= 127   ERROR ACTIVE
                              Sends 6 dominant bits as an error flag. Full
                              participant; destroys bad frames for everyone,
                              which is the point.

  TEC > 127 or REC > 127      ERROR PASSIVE
                              Sends 6 *recessive* bits as an error flag, which
                              changes nothing on the wire. Still transmits and
                              receives, but must wait an extra 8 recessive bits
                              before starting a frame, so it yields to healthy
                              nodes.

  TEC > 255                   BUS OFF
                              Stops entirely. Drives nothing, acknowledges
                              nothing.
```

Roughly: a transmit error is +8 on TEC, a receive error +1 on REC, and a
success is -1. The 8-to-1 asymmetry is deliberate — the node causing the
trouble climbs eight times faster than the nodes merely observing it, so a
faulty node removes itself rather than taking the bus down with it.

**One exception matters more than it looks.** An error-passive transmitter that
detects an ACK error does not increment TEC. A node transmitting into a bus
where nothing answers will therefore climb to 128, go error-passive, and then
*freeze there* — retransmitting forever, never reaching bus-off. Section 14
walks through why that is the single most confusing first-bring-up symptom.

### 7.3 Recovery

Recovery from bus-off requires 128 occurrences of 11 consecutive recessive bits
on the receive input. At 1 Mbit/s on an idle bus that is at least 1.4 ms.

On many modern controllers — the STM32 FDCAN among them — this does **not**
happen automatically. The peripheral sets its init bit and stops, and stays
stopped until software clears it. Older peripherals had an automatic bus-off
management bit; do not assume it is there.

Two rules for a recovery handler:

- **Do not recover in a tight loop.** If the fault is permanent you will
  thrash. Rate limit, count, and after a threshold stop trying and fail safe.
- **Decide what the application does when the bus dies**, and decide it before
  it happens rather than while it is happening.


## 8. The physical bus

### 8.1 Termination

Exactly **two** 120 ohm resistors, one across CANH-CANL at each physical *end*
of the trunk. Not at every node, not in the middle.

```
   [node]      [node]  [node]  ...  [node]        [node]
      |          |       |            |             |
120R==+==========+=======+============+=============+==120R
      |          |       |            |             |
                 ^ stubs must be short, < 30 cm at 1 Mbit/s
```

The two in parallel present 60 ohm to whichever transceiver is driving — the
load a 2 V dominant differential is specified into.

Termination does two jobs and people usually remember only the first. It
absorbs reflections. It also provides the resistive path that pulls the pair
back to zero differential when every driver releases for a recessive bit. That
second job sets how fast recessive recovery is, and it is the one that bites
first, because the dominant-to-recessive edge is *passive*.

| mistake | CANH-CANL, power off | symptom |
|---|---|---|
| none fitted | open | slow recessive recovery, ringing, random stuff and form errors that worsen under load |
| one only | 120 ohm | reflections off the open end; often works on a short bench harness and fails in the real machine |
| three or more | 40 ohm or less | transceiver cannot pull the differential to its minimum; the far end fails first |
| fitted at middle nodes | 60 ohm, looks correct | sections beyond the terminators are unterminated stubs and ring |

The three-terminator case is the one to actually worry about, because most
transceiver breakout boards ship with a 120 ohm resistor already fitted. Nine
boards is nine resistors in parallel, 13 ohm, and nothing works at all. Check
every board and remove all but two.

**How to check, in order.** Power off, measure CANH to CANL: you want **60
ohm**. 120 means one terminator, 40 means three, open means none, near zero
means a short. Then CANH to ground and CANL to ground: both should be megohms.
Power on and idle, both wires should sit near 2.5 V with under 0.5 V between
them.

### 8.2 Topology

Daisy-chain, never star. Keep the pair twisted right up to each connector. At
1 Mbit/s the theoretical length limit is around 40 m, so on anything
desk-sized length is not the problem — **stub length and star topology are**.


# Part II — How the firmware uses it

Part I was the protocol. This part maps this repository's firmware onto it,
at the level of what each piece is for and why it is shaped that way.

## 9. The shape of the driver

There is no CAN library here. The Arduino core does not wrap the H7's FDCAN
peripheral, so `src/main.cpp` talks to ST's HAL directly, with the
initialisation code originally generated by STM32CubeMX and then hand-edited.

Four pieces, all in `src/main.cpp`:

| piece | job | Part I section |
|---|---|---|
| `HAL_FDCAN_MspInit()` | clocks and pins: turns on the FDCAN kernel clock, configures the PLL that feeds it, muxes the TX/RX pins | 1.3, 1.4, 3.1 |
| `MX_FDCAN1_Init()` | the peripheral: bit timing, frame format, message RAM sizing, filters, then start | 3, 5, 6 |
| `CAN_SendActions()` | builds one transmit header and queues one frame | 2, 5.3 |
| `CAN_Poll()` | drains the receive FIFO and unpacks frames into the observation arrays | 6.1, 6.2 |

`HAL_FDCAN_MspInit` is worth a note: you never call it. `HAL_FDCAN_Init()`
calls it internally when the handle is in its reset state. That is why the
clock and pin setup lives in a function that appears to have no caller.

The whole driver is polled. `Model()` sends, waits, then drains — there are no
CAN interrupts enabled, which is the root of several items in section 16.

## 10. Where the bit rate comes from

This is the chain worth understanding, because a break anywhere in it changes
the bit rate silently (3.2).

```
  HSE crystal  ->  PLL1  ->  SYSCLK          (SystemClock_Config)
       |
       +-------->  PLL2  ->  fdcan_ker_ck    (HAL_FDCAN_MspInit)
                              |
                              +-> prescaler -> t_q -> bit rate (MX_FDCAN1_Init)
```

The subtlety: **PLL1 and PLL2 share one input source**. On the H7,
`RCC_PLLCKSELR.PLLSRC` feeds all three PLLs; only the per-PLL `DIVM` dividers
differ. So PLL2's input is whatever `SystemClock_Config()` chose for PLL1 —
which means changing the system clock source silently changes the CAN kernel
clock, and therefore the bit rate.

That is exactly what happened when this project moved from the internal RC to
the crystal. The kernel clock changed, and the bit timing had to be re-derived
to keep the same bit rate. Both `HAL_FDCAN_MspInit()`'s PLL2 values and
`MX_FDCAN1_Init()`'s segment values had to change together.

`SystemClock_Config()` here is declared `extern "C"` deliberately. The Arduino
core provides a `WEAK` default; a strong C-linkage definition overrides it. In
a `.cpp` file without `extern "C"` the name would be mangled, the override
would silently not happen, and the board would run on the core's clock tree
while the CAN timing assumed otherwise — which is the silent-bit-rate failure
of 3.2, arrived at from an unexpected direction.

**The one thing to verify on hardware rather than infer.** Everything in
sections 3 and 4 depends on the kernel clock actually being what the
arithmetic assumes:

```c
static void ReportClocks(void)
{
  Serial_.print("SYSCLK       = "); Serial_.println(HAL_RCC_GetSysClockFreq());
  Serial_.print("HCLK         = "); Serial_.println(HAL_RCC_GetHCLKFreq());
  Serial_.print("fdcan_ker_ck = ");
  Serial_.println(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN));
  Serial_.print("PLL src (0=HSI 1=CSI 2=HSE) = ");
  Serial_.println((RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC)
                  >> RCC_PLLCKSELR_PLLSRC_Pos);
}
```

Call it after `MX_FDCAN1_Init()` — PLL2 does not exist until
`HAL_FDCAN_MspInit()` has run.

## 11. The message scheme

Two identifiers, following 2.2:

```
  0x100          mainboard -> all ESCs, broadcast
                 one FD frame, 32 bytes = 8 floats of joint commands

  0x200 + n      ESC n -> mainboard, n = 0..7
                 8 bytes = joint position float, then joint velocity float
```

Commands sit below feedback, so a command always wins arbitration against a
backlog of replies — the right way round (2.2). The eight replies then
serialise themselves in ESC order with no coordination, which is arbitration
doing exactly what section 2.1 describes.

The broadcast is the right shape too: one 32-byte frame carries all eight joint
targets for one arbitration and one set of overhead, where eight separate small
frames would cost roughly twice the wire time for the same payload.

`0x000` to `0x0FF` is currently unused and sits *above* the commands in
priority. That is where an emergency stop belongs.

## 12. The transmit path

`CAN_SendActions()` fills an `FDCAN_TxHeaderTypeDef` and queues it. The fields
that matter, against Part I:

```c
tx.IdType        = FDCAN_STANDARD_ID;    // 11-bit, section 2.1
tx.FDFormat      = FDCAN_FD_CAN;         // FD frame, section 5.1
tx.BitRateSwitch = FDCAN_BRS_OFF;        // one bit rate throughout, section 5.4
tx.DataLength    = FDCAN_DLC_BYTES_32;   // DLC 13, not 32, section 5.3
```

BRS is off and should stay off: the data-phase timing in `MX_FDCAN1_Init()` is
a copy of the nominal timing, so turning it on would switch from 1 Mbit/s to
1 Mbit/s while adding a transmitter-delay-compensation problem (5.4).

One field is a hardcoded lie worth knowing about:
`tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE` claims the node is error-active.
Hardware overrides it, so it does no harm — but do not read that line as a
statement of fact about the node's state (7.2).

`CAN_SendActions()` returns a bool that is currently discarded. That bool is
the only in-band signal that the transmit FIFO has wedged, which is precisely
the symptom of the failure in section 14.

## 13. The receive path

`CAN_Poll()` drains the FIFO into `joint_pos[]` and `joint_vels[]`, indexing by
`identifier - 0x200`.

The filter is a mask filter, `FilterID1 = 0x200`, `FilterID2 = 0x700`. By 6.1
that accepts `0x200` through `0x2FF` — 256 identifiers for eight ESCs. The
`idx < 8` guard in software discards the rest, so nothing breaks, but 248
identifiers are being accepted into the FIFO only to be thrown away. A mask of
`0x7F8`, or a range filter of 0x200 to 0x207, expresses the actual intent.

The global filter rejects non-matching frames and remote frames in hardware,
which is right (6.1). Note that the node's own 0x100 transmissions match no
filter — correct, and moot anyway, since a node does not receive its own frames
in normal mode.

Two gaps against Part I:

**The received length is ignored.** `CAN_Poll()` copies 8 bytes
unconditionally without decoding `rx.DataLength` (5.3). If an ESC ever sends a
shorter frame, stale bytes from the previous message get fed to the network as
a joint position.

**There is no staleness tracking.** If an ESC drops off the bus entirely, its
entry in `joint_pos[]` keeps its last value forever and the policy walks on a
leg that stopped reporting. A silent ESC is currently indistinguishable from a
stationary one. Per-ESC last-seen timestamps and a staleness mask fix this.

There is also a timing question. The control loop does:

```c
CAN_SendActions(actions);
vTaskDelay(pdMS_TO_TICKS(2));
CAN_Poll();
```

A command plus eight replies needs roughly 1.2 ms of wire time. But
`vTaskDelay(2)` at a 1 kHz tick unblocks on the *second tick boundary after the
call*, and the call lands at an arbitrary point inside the current tick — so
the real delay is anywhere between 1.0 and 2.0 ms. At the short end you poll
while the last ESCs are still transmitting. Their frames are not lost; they sit
in the FIFO and get read on the *next* cycle, 20 ms later. So some of the time,
two joints feed the network data that is one full control period old, and
nothing reports it.

Widening the window is the cheap fix. Waiting for the expected count with a
deadline is better. Best is to stop polling: enable the receive interrupt,
drain in the callback, and notify the control task when all eight have arrived.

## 14. Bringing a bus up safely

Section 7.2 mentioned an exception that freezes TEC at 128. Here is why it
matters, because it produces the most convincing wrong-looking-right you will
meet.

**Going straight to normal mode on a bus with no other node**, with a working
transceiver:

1. The frame transmits cleanly. Bit monitoring passes — every dominant bit
   comes back through the transceiver's own receiver.
2. At the acknowledge slot the transmitter expects some other node to drive
   dominant. There are no other nodes. **ACK error**, `TEC += 8`.
3. With automatic retransmission enabled, it retries. After sixteen attempts,
   about 5 ms in, `TEC = 128` and the node goes error-passive.
4. The exception from 7.2 now applies: an error-passive transmitter detecting
   an ACK error does not increment TEC. **The counter freezes at 128.**
5. So it never reaches bus-off. It retransmits the same frame forever, at
   roughly 3000 attempts per second, at 100% bus utilisation.

What you observe: nothing. No error printed, the control loop keeps its period,
and on a scope the TX pin is a continuous stream of well-formed frames. The
only symptom is `TEC = 128, ErrorPassive = 1, LastErrorCode = ack`, frozen —
visible only if you are reading the protocol status.

**With no transceiver, or one whose standby pin is floating**, it is different
and much faster: the TX pin drives dominant, nothing converts it, the RX pin
never goes dominant, and the transmitter sees a **bit error** on the first bit.
The ACK exception does not apply, TEC climbs freely, and the node is bus-off in
well under a millisecond — where, per 7.3, it stays until software intervenes.
Same visible symptoms: none.

**The bring-up ladder.** Each rung isolates one thing, and each takes about ten
minutes:

| mode | drives the pin | needs an ACK | proves |
|---|---|---|---|
| internal loopback | no | no | message RAM, filters, DLC, your framing code |
| **external loopback** | **yes** | no | the pin, transceiver, harness, termination |
| bus monitoring | no | no | your bit timing matches a real transmitter |
| normal | yes | yes | everything |

**External loopback is the rung most people skip and the one that matters
most.** The controller transmits real frames out of the pin, through the
transceiver, onto the real wire where you can scope them — but feeds its
receiver internally and ignores acknowledge errors. A live, correctly-timed
waveform to measure, with no second node and no error counters climbing.

**Bus monitoring** is the other one. The node never drives a dominant bit, so
it cannot ACK and cannot disturb anything. Power one ESC, let it transmit, and
see whether frames land in your FIFO. If they do, your bit timing and filter
are both proven. If they do not and the ESC is definitely sending, your bit
rate does not match.

## 15. Instrumentation

The theme of sections 10, 13 and 14 is that CAN failures are silent in
software. Two functions turn most of this document into printed lines.

`ReportClocks()` from section 10, once at startup. If the kernel clock is not
what the bit timing assumes, nothing else matters.

A protocol-status dump, once a second:

```c
static void CAN_ReportStatus(void)
{
  FDCAN_ProtocolStatusTypeDef ps;
  FDCAN_ErrorCountersTypeDef  ec;
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
  HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
  static const char *lec[8] = {"none","stuff","form","ack","bit1","bit0","crc","nochange"};

  Serial_.print("TEC=");      Serial_.print(ec.TxErrorCnt);
  Serial_.print(" REC=");     Serial_.print(ec.RxErrorCnt);
  Serial_.print(" lec=");     Serial_.print(lec[ps.LastErrorCode & 7]);
  Serial_.print(" passive="); Serial_.print(ps.ErrorPassive);
  Serial_.print(" busoff=");  Serial_.println(ps.BusOff);
}
```

Which you read with this table — most of the debugging you will ever do:

| what you see | what it means |
|---|---|
| `TEC=0 REC=0 lec=none` | nothing transmitted, or everything worked |
| `TEC=128 passive=1 lec=ack`, frozen | transmitting, nobody acknowledging. Section 14, case one |
| `TEC` climbing to 256, `lec=bit0` | dominant bits not coming back. Transceiver disabled, unpowered, or absent |
| `lec=bit1` while idle | something else holding the bus dominant. Shorted pair, or a stuck node |
| `lec=stuff` / `form`, intermittent | bit rate mismatch, bad termination, or a clock out of tolerance (4.3) |
| `lec=crc` | corruption on the wire. Reflections, noise, missing twist |
| `busoff=1` | 32+ consecutive failed transmissions. Recover explicitly (7.3) |

Beyond that, the ordered debugging sequence is: print the clocks, print the
status, verify the timing registers read back what you set, run external
loopback, run bus monitoring against one live node. Then reach for a
multimeter (60 ohm across the pair, power off), then a scope (measure one bit
width — it must be exactly the period your bit rate implies), then bisect by
disconnecting nodes.

A second MCU in bus-monitoring mode with an accept-all filter, dumping frames
over serial, is a free CAN analyser. Bus monitoring never drives a dominant bit
and never acknowledges, so it cannot disturb a working bus.

## 16. Open items

Roughly in the order they will hurt.

**Before driving a real bus**

1. Confirm `fdcan_ker_ck` on hardware (section 10). The bit timing assumes it.
2. Add the protocol status dump (15). Without it, every failure in 14 is
   invisible.
3. Put a pull-up on the RX pin (1.4). A floating pin generates errors that
   never happened.
4. Confirm every ESC is CAN FD *capable*, not merely FD-tolerant (5.2).
5. Walk the ladder in 14 rather than going straight to normal mode.

**Correctness**

6. Check the return value of `CAN_SendActions()` (12).
7. Decode `rx.DataLength` before copying (13, 5.3).
8. Add per-ESC last-seen timestamps and a staleness mask (13).
9. Widen the receive window, or move to interrupt-driven receive (13).
10. Add a bus-off handler, and decide what the gait does when the motor bus
    dies (7.3).

**Tidying**

11. Tighten the filter to a range of 0x200 to 0x207, or mask 0x7F8 (13).
12. Choose the sample point deliberately rather than inheriting it (4.2).
13. Reserve 0x000-0x0FF for emergency stop and faults (11).
14. Keep `PINOUT.md` in step with the code. Two sources of truth that disagree
    are worse than one that is wrong, because you will trust whichever you read
    last.

**Not now, but know why**

15. Leave BRS off until the bandwidth arithmetic says otherwise (5.4, 12).
16. If the FIFO ever gives ordering or starvation trouble, the redesign is one
    dedicated receive buffer per ESC (6.1).


# Part III — Exercises

Work them before reading the answers.

**E1.** A controller has a 75 MHz kernel clock. Configure it for 500 kbit/s at
an 80% sample point. Give prescaler, TimeSeg1, TimeSeg2 and SJW.

**E2.** For the configuration you just derived, compute the maximum oscillator
tolerance from both ISO conditions and say which one binds.

**E3.** You want to send 20 bytes in one CAN FD frame. Which DLC? Now you want
36 bytes. What do you do?

**E4.** A mask filter is set to `FilterID1 = 0x180`, `FilterID2 = 0x780`. Which
identifiers reach the FIFO, and how many are there?

**E5.** Three nodes start transmitting in the same bit time with identifiers
0x100, 0x080 and 0x101. Who wins, and at which identifier bit does each loser
drop out? Then: what happens if two nodes transmit identifier 0x100 with
*different* payloads in the same bit time?

**E6.** A second CAN instance needs 4 standard filters, an 8-element receive
FIFO of 16-byte elements, and a 4-element transmit FIFO of 16-byte elements. A
16-byte element is 2 header words plus 4 data words. The first instance already
occupies words 0 to 576 of a 2560-word pool. How many words does the second
need, what must its base offset be, and does it fit?

**E7.** A 20 ms control cycle carries one 32-byte FD command frame plus eight
8-byte replies. An FD frame with 32 bytes and no BRS is 316 bit times including
intermission; an 8-byte one is 120. At 1 Mbit/s, what is the bus load? Now
change the replies to 32-byte FD frames — what is the new load, and what does
it do to a 2 ms receive window?

**E8.** A device datasheet specifies an 87.5% sample point. Must yours match?
Can you hit 87.5% exactly at 1 Mbit/s from a 75 MHz kernel clock?

**E9.** Name the fault for each status readout:

```
  (a) TEC=248 REC=0  lec=bit0  passive=1 busoff=0   and climbing fast
  (b) TEC=128 REC=0  lec=ack   passive=1 busoff=0   and not changing
  (c) TEC=0   REC=37 lec=stuff passive=0 busoff=0   REC creeping up
  (d) TEC=0   REC=0  lec=bit1  passive=0 busoff=0   bus should be idle
```

---

## Answers

**E1.** `prescaler * N_tq = 75e6 / 500e3 = 150`. Factor it: 2x75, 6x25 and
10x15 are all usable. Take `prescaler = 6, N_tq = 25`. Then
`1 + TimeSeg1 = 0.80 * 25 = 20`, so `TimeSeg1 = 19`, `TimeSeg2 = 5`, `SJW = 5`.
Check: `75e6 / (6 * 25) = 500 000` exactly, sample point `20/25 = 80.0%`.

**E2.**

```
  (1)  SJW / (2 * 10 * N_tq)                        = 5 / 500 = 1.000%
  (2)  min(PhSeg1,PhSeg2) / (2*(13*N_tq - PhSeg2))  = 5 / (2*(325-5)) = 0.781%
```

Condition (2) binds, at +/- 0.781% — the same as every other 80% configuration,
which is the point section 4.2 makes. Tolerance follows the sample point, not
the time quantum count.

**E3.** 20 bytes is DLC 11. 36 bytes does not exist: the encoding goes 32, 48,
64 with nothing between. Send 48 and pad 12 bytes you pay for on the wire,
split into two frames, or redesign the payload down to 32.

**E4.** Accept when `(id & 0x780) == (0x180 & 0x780) = 0x180`. The mask pins
ID10 down to ID7 and leaves ID6 to ID0 free, so the accepted set is **0x180 to
0x1FF**, 128 identifiers.

**E5.**

```
                ID10 ID9 ID8 ID7 ID6 ID5 ID4 ID3 ID2 ID1 ID0
  0x100            0   0   1   0   0   0   0   0   0   0   0
  0x080            0   0   0   1   0   0   0   0   0   0   0
  0x101            0   0   1   0   0   0   0   0   0   0   1
  ------------------------------------------------------------
  bus              0   0   0   1   0   0   0   0   0   0   0
                           ^
```

0x080 wins at ID8, where it sends dominant and the other two send recessive —
so both lose on the same bit. Later, when 0x100 and 0x101 retry together, they
tie all the way to ID0, where 0x100 sends dominant and wins.

Two nodes sending 0x100 with different payloads is the failure case from 2.2.
They tie through the entire identifier and neither knows it lost. At the first
data bit where the payloads differ, one transmits recessive and samples
dominant *outside* the arbitration field — a bit error, not a lost arbitration.
Both frames are destroyed, both counters climb, and both retransmit and collide
again. One identifier, one transmitter, always.

**E6.** `4*1 + 8*6 + 4*6 = 4 + 48 + 24 = 76 words`. The first instance occupies
words 0 to 576, so the base offset must be **577**. Total 653 of 2560 words,
about 26% — it fits comfortably. Forget the offset and both peripherals write
from word 0 and corrupt each other, presenting as random garbage in both (6.3).

**E7.** `316 + 8*120 = 1276 bit times = 1276 us`, so
`1276 / 20000 =` **6.4%**. With 32-byte replies it becomes `9 * 316 = 2844 us`,
or **14.2%**. Still fine for the bus, but not for a 2 ms window: you now need
at least 2.85 ms of wire time before the last reply lands, so you would be
polling before even half the replies had arrived. Widen the window or move to
interrupt-driven receive (13).

**E8.** No — nodes must agree on the bit *rate*, not the sample point. Each
node independently picks its own prescaler, tq count and sample point, and
independently satisfies its own tolerance and propagation constraints (4.2).

You cannot hit 87.5% exactly anyway. 87.5% is 7/8, so `1 + TimeSeg1` must be
`7/8 * N_tq`, which needs N_tq divisible by 8 — but `prescaler * N_tq = 75` and
75 has no factor of 8. The closest at 15 tq is 86.67%; at 25 tq it is 88.0%.
Aim for the neighbourhood and satisfy the two constraints; the exact figure is
not sacred.

**E9.**

**(a)** `lec=bit0` with TEC climbing fast: the node sends dominant and samples
recessive, so its dominant bits are not reaching the bus. Transceiver
unpowered, standby pin not asserted, TXD not connected, or no transceiver
fitted. It will be bus-off within a millisecond — the second case in section 14.

**(b)** `lec=ack` with TEC frozen at exactly 128: frames are getting out
correctly and nobody is acknowledging. No other functioning node on the bus, or
every other node is in bus-monitoring mode or bus-off. The freeze is the
error-passive ACK exception from 7.2, which is why this node retransmits
forever and never goes bus-off — the first case in section 14.

**(c)** TEC zero and REC creeping with `lec=stuff`: you are *receiving* frames
that are being corrupted. You are not the transmitter, so you are not at fault,
but something on the bus is — a node whose bit rate does not match, a node
whose clock is outside tolerance (4.3), reflections from bad termination (8.1),
or a classic-only controller error-flagging FD frames (5.2). Scope the pair.

**(d)** `lec=bit1` on an idle bus: the node sent recessive and sampled dominant
when nothing should be driving. Something is holding the bus dominant — CANH
shorted to CANL, a shorted transceiver, or a node stuck transmitting. Power
down and measure resistance across the pair (8.1).


# Reference

Checked against:

- ISO 11898-1 error-confinement and bit-timing rules, as summarised in TI
  SPRAC35, *Calculator for CAN Bit Timing Parameters*.
- ST AN5348, *Introduction to FDCAN peripherals for STM32 MCUs*, for message
  RAM element sizes and receive FIFO blocking versus overwrite behaviour.
- `stm32h7xx_hal_fdcan.h`, for the DLC constants, the protocol status and error
  counter structures, and the operating modes.
- ST's note *How to recover from bus-off state with FDCAN on STM32 MCUs*, for
  the absence of automatic bus-off recovery and the `CCCR.INIT` procedure.
- NXP TJA1051 data sheet, for the 250 ns worst-case loop delay used in the
  propagation budget in 3.3.

Confirm transceiver numbers against whatever part you actually fit, and
oscillator accuracy figures against your own datasheet revision. Everything
else follows from arithmetic you can redo.
