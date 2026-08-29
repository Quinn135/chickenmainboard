# CAN and CAN FD on the mainboard

A working lesson. Every section is tied to a line in `src/main.cpp`, a wire on
the board, or a number you have to compute. Read it in order the first time.

Contents:

0. What your code actually says
1. The wire
2. Arbitration
3. Bit timing
4. Oscillator tolerance
5. CAN FD
6. Filters, FIFOs and message RAM
7. Errors
8. Four questions
9. Exercises
10. The change list


## 0. What your code actually says

Before anything else: three of the numbers you gave me do not match
`src/main.cpp`. The lesson uses the code, not the description, because the code
is what the silicon will run.

| | You said | `src/main.cpp` |
|---|---|---|
| `Init.Mode` | `FDCAN_MODE_INTERNAL_LOOPBACK` | `FDCAN_MODE_NORMAL` |
| `Init.NominalTimeSeg1` | 7 | 12 |
| FDCAN kernel clock | 50 MHz | 75 MHz, if HSE is 25 MHz |
| tq per bit | 10 | 15 |
| Sample point | 80.0% | 86.7% |

`PINOUT.md` agrees with your description, so it is stale too.

The kernel clock is the one you must confirm on hardware rather than take from
me. Here is the reasoning and then the measurement.

`HAL_FDCAN_MspInit()` sets PLL2 to `M=2, N=12, Q=2, FRACN=0`. On the H7 the PLL
input source is shared: `RCC_PLLCKSELR.PLLSRC` feeds PLL1, PLL2 and PLL3 alike.
Only the `DIVMx` dividers are per-PLL. So PLL2's input is whatever
`SystemClock_Config()` selected for PLL1.

`src/main.cpp` defines `extern "C" void SystemClock_Config(void)` and selects
`RCC_PLLSOURCE_HSE`. The Arduino core's own `SystemClock_Config()` in
`generic_clock.c` is declared `WEAK`, and a strong definition in a directly
linked object file wins over a weak one in a library archive. `setup()` then
calls it explicitly on its first line as well. So HSE is the PLL source, and
`PINOUT.md`'s claim that the board runs on HSI is out of date.

With HSE at 25 MHz:

```
  PLL2 input  = 25 MHz / M2  = 12.5 MHz     (RCC_PLL2VCIRANGE_3 is 8-16 MHz, ok)
  PLL2 VCO    = 12.5 x N12   = 150 MHz      (RCC_PLL2VCOMEDIUM is 150-420 MHz, ok,
                                             and exactly on the lower edge)
  fdcan_ker_ck = 150 MHz / Q2 = 75 MHz
```

25 MHz is the only HSE value that makes those three constraints work at once.
At 8 MHz the PLL2 input would be 4 MHz, below the declared VCI range, and the
VCO would be 48 MHz, below the declared VCO range. The same 25 MHz produces a
legal PLL1 too: 12.5 MHz in, VCO 800 MHz, P2 gives SYSCLK 400 MHz, AHB /2 gives
HCLK 200 MHz, which is what `FLASH_LATENCY_2` at `VOS1` is rated for.

Do not trust that chain. Measure it. Put this in `setup()` right after
`Serial_.begin()` and read the number off the console:

```c
static void print_kv(const char *k, uint32_t v)
{
  Serial_.print(k);
  Serial_.print(" = ");
  Serial_.println(v);
}

static void ReportClocks(void)
{
  print_kv("SYSCLK          ", HAL_RCC_GetSysClockFreq());
  print_kv("HCLK            ", HAL_RCC_GetHCLKFreq());
  print_kv("fdcan_ker_ck    ", HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN));
  print_kv("PLL source (0=HSI 1=CSI 2=HSE)",
           (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC) >> RCC_PLLCKSELR_PLLSRC_Pos);
}
```

Call it after `MX_FDCAN1_Init()`, because PLL2 does not exist until
`HAL_FDCAN_MspInit()` has run. Everything in sections 3 and 4 depends on that
one number. If it prints 50000000 rather than 75000000, your bus is running at
667 kbit/s and not 1 Mbit/s, and none of your ESCs will talk to you.

There is a second thing worth knowing before you go further. Your board does
not exist yet as a bus. `Init.Mode` is already `FDCAN_MODE_NORMAL`, so the very
next time you flash, the peripheral will start driving PH13 for real. Section 8
walks through exactly what happens.


## 1. The wire

### 1.1 Why a differential pair

A CAN bus carries one bit at a time as a voltage *difference* between two
wires, CANH and CANL. There is no clock line and no separate data line. Two
states exist:

```
  V
 3.5 |                    +---------------+
     |                    |     CANH      |
 2.5 |--------------------+               +-----------------
     |                    |     CANL      |
 1.5 |                    +---------------+
     +----------------------------------------------------> t

        recessive              dominant           recessive
        Vdiff ~ 0 V            Vdiff ~ 2 V        Vdiff ~ 0 V
        logical 1              logical 0          logical 1
```

Receivers threshold on the *difference*, not on either wire against ground.
That buys three things a single-ended line does not have:

- Noise from a motor coil couples into CANH and CANL almost equally. It moves
  both wires and leaves the difference alone. On a walking robot with eight
  ESCs switching current, this is not a nicety.
- There is no shared ground reference to get wrong. CAN receivers tolerate a
  wide common-mode range, typically -12 V to +12 V, so the mainboard ground and
  an ESC ground can sit volts apart and the link still works.
- Emissions from the pair largely cancel, because the two currents are equal
  and opposite.

Twist the pair. The cancellation depends on the two wires seeing the same
field, which is what twisting enforces.

### 1.2 What the transceiver actually does

Four functions, all of which the MCU pin cannot do:

1. **Single-ended to differential, one direction each.** TXD in, CANH/CANL out.
   CANH/CANL in, RXD out. Your PH13 is TXD and PI9 is RXD.
2. **Open-drain behaviour on the bus.** For recessive, the transceiver's output
   drivers are *off*. The bus floats to its own idle level, set by the
   termination resistors and the receivers' bias. For dominant, it actively
   drives CANH high and CANL low. This asymmetry is the whole trick. See 1.4.
3. **Level translation and protection.** 5 V bus levels from a 3.3 V MCU;
   bus-pin ESD and short-circuit protection; thermal shutdown.
4. **Bit-rate-grade timing.** A CAN FD transceiver is specified for the loop
   delay you are allowed to have, which section 3 turns into a number you must
   satisfy.

For your board you want a 3.3 V-IO part, which for the TJA1051 family means the
`/3` suffix: `TJA1051T/3`. It has a `VIO` pin you tie to 3V3 while `VCC` is 5 V,
so RXD comes back at 3.3 V levels. Its loop delay is specified 40 ns min, 250 ns
max, and it is rated for CAN FD data phases up to 5 Mbit/s. Hold onto 250 ns;
section 3 spends it.

Also check the `S` / `STB` pin. On a TJA1051 it must be pulled **low** for
normal mode. Left floating or high, the transmitter is disabled and the part
only listens. That single unpopulated pulldown resistor is one of the two most
common reasons a first CAN board does not work, and it produces the *bad*
failure mode in section 8.1.

### 1.3 Why PI9 and PH13 are not the bus

They carry the same bits, but as ordinary 3.3 V push-pull CMOS logic referenced
to the mainboard ground. Specifically:

- PH13 is push-pull. It drives *both* levels hard. A CAN bus needs one level
  driven and one released, or wired-AND cannot work (1.4).
- A GPIO cannot source the current a terminated bus draws. Two 120 ohm
  resistors in parallel is 60 ohm; a dominant bit pulls roughly 2 V across it,
  about 33 mA of differential current. That is outside what an H7 pin should be
  asked to do, and the pin has no common-mode range at all.
- No fault protection. A shorted bus would take the pin, and probably the port,
  with it.

Note the polarity in `HAL_FDCAN_MspInit()`. Both pins are `GPIO_MODE_AF_PP`
with `GPIO_AF9_FDCAN1`, which is correct, but:

```c
GPIO_InitStruct.Pin = GPIO_PIN_9;      // PI9, FDCAN1_RX
GPIO_InitStruct.Pull = GPIO_NOPULL;    // <-- change this
```

With no transceiver fitted, or with one that is unpowered, PI9 floats. The
FDCAN receiver samples a floating pin as noise, and will report stuff and form
errors for signals that were never on any wire. Set `GPIO_PULLUP` on the RX pin
so an absent transceiver reads as a permanently recessive, permanently idle
bus. Leave TX as `GPIO_NOPULL`; the transceiver has its own TXD pull-up so that
a resetting MCU, whose pins revert to floating inputs, cannot hold the bus
dominant.

`GPIO_SPEED_FREQ_LOW` is fine at 1 Mbit/s, where a bit is 1000 ns and you need
edges in the tens of nanoseconds. Revisit it if you ever enable BRS.

### 1.4 Dominant, recessive, and why there is no master

The bus is a wired-AND. If any node drives dominant, the bus is dominant.
Recessive only happens when *every* node has released.

```
  node A   1   1   0   1
  node B   1   0   1   1
  node C   1   1   1   0
  ------------------------
  bus      1   0   0   0
```

Two consequences, and they carry the entire protocol:

**Every transmitter also receives.** While a node transmits, it samples the bus
at the sample point of every bit and compares it against what it meant to send.
If it sent recessive and read dominant, another node overrode it. If it sent
dominant and read recessive, something is broken (that is a bit error;
section 7).

**Losing is free.** During the identifier field, a node that sent recessive and
read dominant simply stops transmitting and switches to receiving the frame
that beat it. It did not corrupt anything. The winner never noticed. The loser
still gets the message. Nothing was retransmitted, no slot was wasted, and no
central node decided anything. That is why CAN needs no master and no
scheduler, and it is why the bus is deterministic under load: the highest
priority message pending anywhere on the bus is always the one being sent.


## 2. Arbitration

### 2.1 The mechanism, on your IDs

Arbitration happens over the 11 identifier bits, most significant first
(ID10 down to ID0). Lower numeric ID means more dominant bits early, means
higher priority.

Your three cases, with 0x100 = mainboard command, 0x200 = ESC 0 feedback,
0x207 = ESC 7 feedback:

```
                  ID10 ID9 ID8 ID7 ID6 ID5 ID4 ID3 ID2 ID1 ID0
  0x100 mainboard    0   0   1   0   0   0   0   0   0   0   0
  0x200 ESC 0        0   1   0   0   0   0   0   0   0   0   0
  0x207 ESC 7        0   1   0   0   0   0   0   0   1   1   1
  ----------------------------------------------------------------
  bus                0   0   1   0   0   0   0   0   0   0   0
                         ^
                         both ESCs send recessive here and read
                         dominant. They lose, stop, and receive.
```

If all three start in the same bit time, the mainboard wins on ID9 and both
ESCs back off after ten bit times, having transmitted nothing but a SOF and one
identifier bit. Once the command frame ends and the bus goes idle, ESC 0 and
ESC 7 both start again. Now:

```
                  ID10 ID9 ID8 ID7 ID6 ID5 ID4 ID3 ID2 ID1 ID0
  0x200 ESC 0        0   1   0   0   0   0   0   0   0   0   0
  0x207 ESC 7        0   1   0   0   0   0   0   0   1   1   1
                                                     ^
                                                     ESC 7 loses at ID2
```

The eight replies serialise themselves in ID order, deterministically, with no
collision and no retry. ESC 0 goes first, ESC 7 last. At 1 Mbit/s with 8-byte
classic feedback frames, ESC 7 starts its frame about 7 x 111 = 777 us after
ESC 0 started. Remember that number for section 8.

### 2.2 Was your numbering a good choice

Mostly yes. Three observations.

**Commands above feedback is correct.** Command is 0x100, feedback is 0x2xx, so
commands preempt feedback. Your control loop sends a command and then waits for
replies to it; the command must go out first or the whole cycle slips. If you
had numbered them the other way, a burst of eight feedback frames would sit in
front of every command.

**One broadcast beats eight unicasts.** Your 32-byte frame carrying all eight
joint targets costs one arbitration and one set of frame overhead: 316 bit
times on the wire, of which 256 are payload. Eight separate 4-byte classic
frames would cost 8 x 79 = 632 bit times to carry the same 256 bits. Twice the
wire time for the same data. The cost of broadcasting is that a single
corrupted command frame misses all eight joints for a cycle, but CAN
retransmits automatically on error and your bus load is 6%, so this is the
right trade.

**You have left the right space free.** 0x000 to 0x0FF is empty and sits above
your commands. Put a fault or emergency-stop broadcast there. An e-stop that
cannot preempt a joint command is not an e-stop:

```c
static const uint32_t CAN_ID_ESTOP   = 0x000;  // beats everything
static const uint32_t CAN_ID_FAULT   = 0x010;  // ESC reports a fault
static const uint32_t CAN_ID_CMD     = 0x100;  // you have this
static const uint32_t CAN_ID_FB_BASE = 0x200;  // you have this
static const uint32_t CAN_ID_CFG     = 0x300;  // per-ESC config, 0x300 + n
```

Two rules to respect while you are in there. The seven most significant bits of
a standard identifier must not all be recessive, which makes 0x7F0 to 0x7FF
unusable. And no two nodes may ever transmit different content under the same
identifier: two nodes starting the same ID in the same bit time will tie all
the way through arbitration and then destroy each other's frames at the first
differing data bit. Your scheme has one transmitter per ID, which is the rule.


## 3. Bit timing

### 3.1 Time quanta

Everything is built from one unit. The FDCAN peripheral divides its kernel
clock by `NominalPrescaler` to get a time quantum:

```
  t_q = NominalPrescaler / fdcan_ker_ck
```

A bit is an integer number of time quanta, split into three segments:

```
|<------------------- 1 bit time = N_tq time quanta ------------------>|
|      |                                                     |         |
| SYNC |   Prop_Seg        |   Phase_Seg1                     | Phase_Seg2
| 1 tq |                                                      |
+------+----------------------------------------------------+----------+
       ^                                                     ^
       a recessive-to-dominant edge is expected here     sample point
       (this is what synchronisation aligns)             the bit is read here
```

- **SYNC_SEG** is always exactly 1 tq. Edges are supposed to fall inside it.
- **Prop_Seg** exists to absorb signal propagation. It must be long enough that
  a bit driven by the far end of the bus has arrived, been converted back to
  logic, and settled, before we sample. Section 3.4 sizes it.
- **Phase_Seg1** and **Phase_Seg2** straddle the sample point and are the parts
  that get stretched and shortened during resynchronisation.
- **The sample point** is the boundary between Phase_Seg1 and Phase_Seg2. Every
  node on the bus reads the bit value at this instant.

The ST HAL does not expose Prop_Seg separately. `NominalTimeSeg1` is
`Prop_Seg + Phase_Seg1` lumped together, and `NominalTimeSeg2` is Phase_Seg2.
So:

```
  N_tq         = 1 + NominalTimeSeg1 + NominalTimeSeg2
  bit rate     = fdcan_ker_ck / (NominalPrescaler * N_tq)
  sample point = (1 + NominalTimeSeg1) / N_tq
```

Hardware stores `NTSEG1 - 1` and `NTSEG2 - 1` in `FDCAN_NBTP` and adds the one
back. You give the HAL the true segment lengths, not the register values.

### 3.2 The 50 MHz numbers you described

```
  fdcan_ker_ck   = 50 MHz
  NominalPrescaler = 5      -> t_q = 5 / 50e6 = 100 ns
  N_tq needed for 1 Mbit/s: bit time 1000 ns / 100 ns = 10 tq
  1 + Seg1 + Seg2 = 10      -> Seg1 + Seg2 = 9
  want 80% sample point:  (1 + Seg1) / 10 = 0.80  ->  Seg1 = 7, Seg2 = 2
  check: 50e6 / (5 * 10) = 1 000 000 bit/s exactly, SP = 8/10 = 80.0%
```

That is the derivation. Four steps: prescaler picks t_q, bit rate picks N_tq,
sample point splits N_tq, then check the division comes out exact.

### 3.3 The numbers actually in your code

```
  fdcan_ker_ck     = 75 MHz     (assuming HSE = 25 MHz, section 0)
  NominalPrescaler = 5          -> t_q = 5 / 75e6 = 66.67 ns
  NominalTimeSeg1  = 12
  NominalTimeSeg2  = 2
  N_tq = 1 + 12 + 2 = 15
  bit rate = 75e6 / (5 * 15) = 1 000 000 bit/s exactly
  sample point = 13 / 15 = 86.67%
  SJW = 2
```

So the bit rate is right and the sample point moved from 80% to 86.7%. Whoever
retimed this for 75 MHz kept `Seg2 = 2` and put all the extra tq into `Seg1`,
which is the lazy split. Section 3.5 and section 4 show what it costs and what
it buys, and it is not one-sided.

Note the trap: if the kernel clock is *not* 75 MHz and is still 50 MHz, the
same register values give `50e6 / (5 * 15) = 666.7 kbit/s`. Nothing in software
will tell you. This is exactly why section 0 makes you print the clock.

### 3.4 Sizing Prop_Seg, which is where the transceiver bill arrives

Prop_Seg must cover the round trip: your transceiver drives the bus, the signal
crosses the cable, the far transceiver converts it to logic, that node responds
dominant, and its dominant bit has to get all the way back and settle at your
receiver before your sample point.

```
  Prop_Seg  >=  2 x (transceiver loop delay + cable delay)
```

Copper is about 5 ns per metre. With `Seg1 = 12` and `Seg2 = 2` at 66.67 ns per
tq, Seg1 is 800 ns. Keep at least `Phase_Seg1 >= Phase_Seg2 = 2 tq` so that
`min(PhSeg1, PhSeg2)` in section 4 stays equal to Phase_Seg2, and:

```
  Prop_Seg <= 12 - 2 = 10 tq = 667 ns
  transceiver loop + cable <= 333 ns
  on a 2 m robot (10 ns of cable): loop delay budget = 323 ns
```

A TJA1051T/3 at its 250 ns worst case fits with 73 ns to spare. Now redo it for
an 80% sample point, `Seg1 = 11`, `Seg2 = 3`:

```
  Prop_Seg <= 11 - 3 = 8 tq = 533 ns
  transceiver loop + cable <= 267 ns
  on a 2 m robot: loop delay budget = 257 ns
```

The same transceiver fits with 7 ns to spare. That is not margin, that is luck.

So the 86.7% sample point in your code is defensible: it buys propagation
budget. It is not free, and section 4 is where the bill lands.

### 3.5 SJW and resynchronisation

There is no clock line. Every receiver runs its own bit-time counter off its
own oscillator, and the only thing keeping the counters aligned is the edges in
the data.

Two mechanisms:

- **Hard synchronisation** happens once per frame, on the falling edge of SOF.
  The receiver restarts its bit-time counter so that edge lands in SYNC_SEG.
- **Resynchronisation** happens on every subsequent recessive-to-dominant edge.
  If the edge arrives *late* (the receiver is running fast), Phase_Seg1 is
  lengthened. If it arrives *early* (the receiver is running slow), Phase_Seg2
  is shortened. Either way the sample point is pushed back toward the middle of
  the incoming bit.

`NominalSyncJumpWidth` is the cap on how much one edge is allowed to move
things, in tq. Your `SJW = 2` with `t_q = 66.67 ns` means each edge can correct
at most 133 ns of accumulated drift.

Two hard rules:

```
  SJW <= Phase_Seg2                       (you cannot shorten Seg2 below zero)
  SJW <= Phase_Seg1                       (same, the other way)
```

so in practice `SJW <= min(Phase_Seg1, Phase_Seg2)`, and since Phase_Seg2 is
almost always the smaller one, `SJW = Phase_Seg2` is the standard choice. Yours
satisfies this: `SJW = 2 = Seg2`.

Bit stuffing is what guarantees edges exist at all. After five consecutive bits
of the same value, the transmitter inserts one bit of the opposite value, which
receivers remove. So there is a resynchronisation opportunity at least every
ten bits. That "ten bits" is the constant that shows up in the next section's
first formula, and the "13 bits" in the second is the worst-case span with no
usable edge at all, during an error flag.


## 4. Oscillator tolerance

### 4.1 The two conditions

Two nodes must stay aligned for a whole frame using only resynchronisation.
That gives two constraints, both from ISO 11898-1. `df` is the fractional
frequency error of one node relative to nominal; the factor of 2 is there
because the two nodes can be off in opposite directions.

```
  (1)  df  <=  SJW / (2 * 10 * N_tq)

  (2)  df  <=  min(PhSeg1, PhSeg2) / (2 * (13 * N_tq - PhSeg2))
```

Condition (1) is the resynchronisation budget: over the ten bits between
guaranteed edges, the drift accumulated by two nodes must be recoverable by one
SJW correction.

Condition (2) is the no-edge case: during an error flag there can be 13 bit
times with no usable resynchronisation, and the drift over that span must not
push the sample point out of the bit.

Your tolerance is the smaller of the two. Condition (2) is almost always the
binding one.

### 4.2 Your numbers

Both configurations, with `PhSeg1` taken as the whole of `Seg1` (an upper
bound; carving out Prop_Seg only makes it worse, and it does not matter here
because `Seg2` is the smaller term either way):

| config | N_tq | SJW | Seg2 | SP | cond (1) | cond (2) | df max |
|---|---|---|---|---|---|---|---|
| 50 MHz, p5, 7/2, SJW 2 (described) | 10 | 2 | 2 | 80.0% | 1.000% | 0.781% | **0.781%** |
| 75 MHz, p5, 12/2, SJW 2 (your code) | 15 | 2 | 2 | 86.7% | 0.667% | 0.518% | **0.518%** |
| 75 MHz, p5, 11/3, SJW 3 | 15 | 3 | 3 | 80.0% | 1.000% | 0.781% | **0.781%** |

Worked, for the middle row:

```
  (1)  2 / (2 * 10 * 15)              = 2 / 300  = 0.00667 = 0.667%
  (2)  min(12,2) / (2 * (13*15 - 2))  = 2 / 386  = 0.00518 = 0.518%
  df = 0.518%  =  5181 ppm
```

Every node on the bus, including all eight ESCs, must hold its CAN clock within
+/- 0.518% of nominal, at every temperature the robot will see. That is the
number your current code demands.

Notice the pattern in the table: the tolerance is set by the *sample point*,
not by how many tq you use. With `SJW = Phase_Seg2`, condition (2) reduces to

```
  df = (1 - SP) / (2 * (13 - (1 - SP)))
```

which for SP = 80% is 0.2 / 25.6 = 0.781% regardless of N_tq. Adding time
quanta does not buy tolerance. It buys *resolution* to place the sample point
where you want it. Here is the same 15 tq budget swept:

| Seg1 | Seg2 | SJW | sample point | df max |
|---|---|---|---|---|
| 13 | 1 | 1 | 93.3% | 0.258% |
| 12 | 2 | 2 | 86.7% | 0.518% |
| 11 | 3 | 3 | 80.0% | 0.781% |
| 10 | 4 | 4 | 73.3% | 1.047% |
| 9 | 5 | 5 | 66.7% | 1.316% |

This is the whole trade-off, in one table. Later sample point buys propagation
delay (3.4) and costs oscillator tolerance. Earlier sample point does the
reverse. 75% to 87.5% is the usual band; automotive tends to 80%.

### 4.3 Why this makes RC versus crystal an engineering decision

Put the two candidate clock sources against a 0.518% budget.

**Internal RC (HSI).** The H743's HSI is factory trimmed to roughly +/- 1% at
room temperature, and drifts to a few percent across the full operating
temperature range. Check Table 37 in the datasheet revision you have, but the
conclusion does not depend on the third digit: at room temperature, before
anything gets warm, the *specified* accuracy is already about twice your
budget. A robot's electronics bay does not stay at room temperature.

**Crystal (HSE).** A commodity 25 MHz crystal is +/- 30 to +/- 50 ppm, which is
0.003% to 0.005%. Add load-capacitor error and ageing and call it 100 ppm,
0.01%. That is fifty times inside your budget.

The gap is not marginal, it is two orders of magnitude. This is the difference
between a bus that works and a bus that does not, and it is why every CAN
reference tells you to use a crystal. It is also worth understanding *why* an
RC oscillator fails in a specific, maddening way: at 25 C on the bench it will
often work. It starts failing when the board warms up, or when one ESC is near
a motor and another is not, and the failures look like random CRC and form
errors rather than anything you can point at.

Three practical consequences:

1. If `RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC` prints anything other than 2
   (HSE), stop and fix the clock tree before touching the bus. Section 0's
   snippet tells you.
2. The budget applies to *every* node. Ask what the ESCs clock their CAN
   controllers from. One RC-clocked ESC will corrupt frames for everybody,
   including frames it is not part of, because its error flags are dominant
   bits on a shared wire.
3. Your worst case is the sum of everything: crystal initial tolerance, plus
   temperature drift, plus ageing, plus load capacitance mismatch. Budget the
   whole stack against 0.518%, not just the datasheet's initial accuracy.

If you want the headroom back, move to `Seg1 = 11, Seg2 = 3, SJW = 3` and the
budget widens to 0.781% at the cost of the propagation margin in 3.4. With a
250 ns transceiver on a short robot harness that is a coin flip. With a faster
transceiver it is free.


## 5. CAN FD versus classic CAN

### 5.1 What FD changes

Three things, and you are using one of them.

1. **Payload up to 64 bytes** instead of 8. You use 32.
2. **Bit rate switching (BRS)**, where the data phase runs at a higher bit rate
   than the arbitration phase. You have this off.
3. **Stronger CRC** (17 bits for payloads up to 16 bytes, 21 bits above) and a
   stuff-count field, so long frames are protected properly.

Frame-level differences you will see in the register fields: the RTR bit is
gone (FD has no remote frames), the reserved bit becomes FDF to mark the frame
as FD, and ESI appears to advertise whether the transmitter is error-passive.

### 5.2 The compatibility trap

`FrameFormat = FDCAN_FRAME_FD_NO_BRS` means your controller *transmits* FD
frames. It will still receive classic frames happily, so if your ESCs reply in
classic 8-byte frames that works.

The reverse does not work. A classic-only CAN controller that sees your FD
frame reads the FDF bit as a violation of the frame format, declares a form
error, and transmits an active error flag: six dominant bits, on the shared
wire, in the middle of your command frame. It destroys the frame for all eight
ESCs, drives its own receive error counter up, and yours too via the
retransmission. One classic-only node makes an FD bus unusable.

So before you leave loopback: confirm all eight ESCs use CAN FD capable
controllers. Not FD-*tolerant*, which only means they stay quiet; actually
FD-capable. This is a datasheet check, not a code check, and it is the single
most expensive thing to discover late.

### 5.3 Why 32 bytes is `FDCAN_DLC_BYTES_32` and not 32

The DLC field on the wire is four bits. Four bits is sixteen values, and CAN FD
needs to express seventeen useful lengths. It cannot, so it does not: above 8
bytes the encoding becomes a lookup table with gaps.

| DLC | bytes | | DLC | bytes |
|---|---|---|---|---|
| 0-8 | 0-8 | | 12 | 24 |
| 9 | 12 | | 13 | **32** |
| 10 | 16 | | 14 | 48 |
| 11 | 20 | | 15 | 64 |

32 bytes is DLC 13. The HAL constant is literally that number:

```c
#define FDCAN_DLC_BYTES_32  ((uint32_t)0x0000000DU)
```

The constant exists so you never have to remember whether 32 is 12 or 13, and
so a typo like `tx.DataLength = 32` fails loudly rather than quietly sending a
64-byte frame (32 would be truncated into the 4-bit field). The practical
consequences:

- **There is no 36-byte frame.** If your command payload ever grows past 32
  bytes, the next size is 48. You pad 16 bytes and pay for them on the wire.
  Likewise nothing exists between 8 and 12. Design your payloads to land on
  0, 4, 8, 12, 16, 20, 24, 32, 48 or 64.
- **`rx.DataLength` comes back as the same encoded constant**, not a byte
  count. Your `CAN_Poll()` currently ignores it entirely and copies 8 bytes
  unconditionally. If an ESC ever sends a shorter frame you will read stale
  bytes out of `buf` and feed them to the network as a joint position. Decode
  it:

```c
static uint8_t dlc_to_bytes(uint32_t dlc)
{
  static const uint8_t tbl[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  return tbl[dlc & 0x0F];
}
```

### 5.4 BRS, and why yours is off

With BRS on, the frame runs at the nominal bit rate through arbitration and the
ACK, and switches to a faster data bit rate for the payload and CRC. It has to:
arbitration requires every node to see every bit within one bit time, which
caps the arbitration rate; the data phase has only one transmitter, so it can
go faster.

You should leave it off, for three reasons.

**You do not need the bandwidth.** Your cycle is a 32-byte command plus eight
8-byte replies, 1204 bits, about 1.2 ms of a 20 ms cycle. Section 6.4 does the
arithmetic. You are at 6% bus load. BRS solves a problem you do not have.

**Turning it on today would do nothing anyway.** Look at your own init:

```c
hfdcan1.Init.NominalPrescaler = 5;  hfdcan1.Init.NominalTimeSeg1 = 12;
hfdcan1.Init.DataPrescaler    = 5;  hfdcan1.Init.DataTimeSeg1    = 12;
```

The data timing is a copy of the nominal timing. Setting `BitRateSwitch =
FDCAN_BRS_ON` would switch from 1 Mbit/s to 1 Mbit/s.

**It costs you a second timing problem.** A faster data phase means a shorter
data bit, which the transceiver's loop delay no longer fits inside. Above
roughly 1 Mbit/s in the data phase you must configure transmitter delay
compensation (`HAL_FDCAN_ConfigTxDelayCompensation` plus
`HAL_FDCAN_EnableTxDelayCompensation`), you get a second pair of oscillator
tolerance conditions to satisfy for the data phase, and `GPIO_SPEED_FREQ_LOW`
on PH13 stops being adequate.

Your current TX header is right for what you are doing:

```c
tx.FDFormat      = FDCAN_FD_CAN;     // FD frame
tx.BitRateSwitch = FDCAN_BRS_OFF;    // one bit rate throughout
tx.DataLength    = FDCAN_DLC_BYTES_32;
```

One thing worth adding for later: `tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE`
is a hardcoded lie if the node ever goes error-passive. The controller
overrides it in hardware, so it does no harm, but do not read that line as a
statement of fact about the node's state.


## 6. Filters, FIFOs and the H7 message RAM

### 6.1 Message RAM is yours to lay out

Unlike the older bxCAN, the H7's FDCAN has no fixed buffer layout. There is a
single 10 Kbyte pool, 2560 32-bit words, shared by FDCAN1 and FDCAN2, and your
`Init` struct carves it up. Every element you ask for is allocated from
`MessageRAMOffset` upward, in this order: standard filters, extended filters,
Rx FIFO0, Rx FIFO1, Rx buffers, Tx event FIFO, Tx buffers, Tx FIFO/queue.

Element sizes, in 32-bit words:

| element | words | note |
|---|---|---|
| standard filter | 1 | 11-bit IDs |
| extended filter | 2 | 29-bit IDs |
| Rx FIFO element, 64-byte data | 18 | 2 header + 16 data |
| Tx FIFO element, 64-byte data | 18 | 2 header + 16 data |
| Tx event FIFO element | 2 | |

Your configuration:

```
  StdFiltersNbr        =  1  x  1 word  =    1 word
  ExtFiltersNbr        =  0                   0
  RxFifo0ElmtsNbr      = 24  x 18 words =  432 words   (1728 B)
  RxFifo1ElmtsNbr      =  0                   0
  RxBuffersNbr         =  0                   0
  TxEventsNbr          =  0                   0
  TxBuffersNbr         =  0                   0
  TxFifoQueueElmtsNbr  =  8  x 18 words =  144 words   ( 576 B)
  -------------------------------------------------------------
  total                                    577 words   (2308 B)  = 22.5% of pool
  free for FDCAN2                         1983 words   (7932 B)
```

Two things follow. First, you are nowhere near the limit, so the 24-deep FIFO
is not extravagant. Second, `MessageRAMOffset = 0` is correct only because
FDCAN1 is the only instance in use. If you ever bring up FDCAN2 you must set
its `MessageRAMOffset` to 577 by hand. The HAL does not do this for you and
will happily let two peripherals write over each other's buffers, which
produces a fault that looks like random corruption in both.

You are also paying 64 bytes per element for messages that are at most 32 bytes
long. Setting `RxFifo0ElmtSize = FDCAN_DATA_BYTES_8` would take Rx FIFO0 from
432 words to 96, but leave it as it is: your ESCs might someday send more, and
a frame longer than the element size is *silently truncated*, not rejected.

### 6.2 What your filter actually accepts

A standard mask filter accepts a frame when

```
  (received_id & FilterID2) == (FilterID1 & FilterID2)
```

Yours is `FilterID1 = 0x200`, `FilterID2 = 0x700`:

```
  0x200 = 0b010 0000 0000
  0x700 = 0b111 0000 0000     the mask: only the top three bits must match

  accepted:  0b010 xxxx xxxx  ->  0x200 through 0x2FF
```

256 identifiers. You use eight of them. The other 248 are a hole: any node that
one day transmits, say, 0x2A0 lands in your FIFO, gets through
`idx = rx.Identifier - 0x200` as `idx = 0xA0`, fails the `idx < 8` test, and is
discarded. Correct today, but it costs FIFO slots and CPU to reject traffic
that should never have been accepted.

Tighten it. Two ways, both one line:

```c
f.FilterType = FDCAN_FILTER_MASK;
f.FilterID1  = 0x200;
f.FilterID2  = 0x7F8;    // accept exactly 0x200..0x207
```

or, clearer to read six months from now:

```c
f.FilterType = FDCAN_FILTER_RANGE;
f.FilterID1  = 0x200;    // low
f.FilterID2  = 0x207;    // high, inclusive
```

`HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)` is right: anything that matches no
filter is dropped in hardware, and remote frames are dropped on both ID types.
Remote frames do not exist in CAN FD anyway, but a classic-CAN node could still
send one and it costs nothing to refuse it. Note that your own 0x100
transmissions are not accepted by any filter, which is correct and also moot: a
node does not receive its own frames in `FDCAN_MODE_NORMAL`.

There is a third routing target worth knowing about, because your topology is
the textbook case for it. `FDCAN_FILTER_TO_RXBUFFER` sends a matching frame to
a *dedicated* Rx buffer rather than to a FIFO. Configure eight filters, one per
ESC, each pointing at its own buffer index, and each ESC gets a private
mailbox. There is no queue, so a talkative ESC cannot displace a quiet one, the
contents are always that ESC's most recent frame, and the per-buffer
"new data" flag gives you a liveness indicator for free. It costs eight filters
and eight buffers instead of one filter and a FIFO. If the FIFO ever gives you
trouble, this is the redesign; check the exact signatures of
`HAL_FDCAN_IsRxBufferMessageAvailable` and the `FDCAN_RX_BUFFER0` locations in
your HAL version before you commit to it.

### 6.3 What happens when Rx FIFO0 fills

The FIFO defaults to **blocking mode**. When it is full:

- The incoming frame is filtered, accepted, and then *dropped*.
- `FDCAN_RXF0S.RF0L` (message lost) is set.
- `FDCAN_IT_RX_FIFO0_MESSAGE_LOST` fires, if you enabled it. You have not.
- **The node still acknowledges the frame.** Acceptance filtering and ACK
  generation happen in the protocol engine, before the FIFO write. The ESC that
  sent it sees a successful transmission and moves on.

So a full FIFO is silent, lossless-looking data loss. Worse, in blocking mode
what you lose is the *newest* frame while the FIFO keeps the oldest. For
control feedback that is exactly backwards.

`HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0,
FDCAN_RX_FIFO_OVERWRITE)` flips it so the oldest element is discarded instead.
Read the M_CAN overwrite-mode read sequence before you enable it, though; a
read that races an overwrite can return an inconsistent element, and there is a
prescribed get-index dance to avoid it. Given your headroom (below), the better
answer is to not fill the FIFO in the first place, and to *notice* if you ever
do:

```c
// call once, after HAL_FDCAN_Init and before HAL_FDCAN_Start
HAL_FDCAN_ActivateNotification(&hfdcan1,
    FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF |
    FDCAN_IT_ERROR_PASSIVE        | FDCAN_IT_ERROR_WARNING, 0);
```

### 6.4 Is polling once per 20 ms safe with eight nodes

Two separate questions: does the FIFO overflow, and is the data fresh. The
answers are no and no.

**Capacity.** Count the wire time. Frame field counts at 1 Mbit/s, excluding
stuff bits and including the 3-bit intermission:

| frame | fields | bits | time |
|---|---|---|---|
| your command: FD, no BRS, 32 bytes, CRC-21 | 22 + 256 + 4 + 21 + 10 | 313 (+3) | 316 us |
| ESC reply: classic, 8 bytes | 44 + 64 | 108 (+3) | 111 us |
| ESC reply: FD no BRS, 8 bytes, CRC-17 | 22 + 64 + 4 + 17 + 10 | 117 (+3) | 120 us |

One control cycle, with classic 8-byte replies:

```
  316 + 8 x 111 = 1204 bit times = 1204 us
  bus load = 1204 / 20000 = 6.0%          (call it 7.5% with worst-case stuffing)
```

Eight frames arrive per 20 ms into a 24-deep FIFO, so you have three cycles,
60 ms, of slack. The FIFO overflows only if the `Model` task stalls for longer
than that. Your inference time is the risk; time `entry()` and check. Capacity
is fine.

**Freshness is not fine.** Look at what the loop does:

```c
CAN_SendActions(actions);
vTaskDelay(pdMS_TO_TICKS(2));
CAN_Poll();
```

The bus needs 1204 us minimum. `vTaskDelay(pdMS_TO_TICKS(2))` at the default
1000 Hz FreeRTOS tick is `vTaskDelay(2)`, and `vTaskDelay(n)` unblocks on the
nth tick boundary *after* the call. Since the call lands at an arbitrary point
inside the current tick, the real delay is anywhere between 1.0 ms and 2.0 ms.

At the short end you poll after 1.0 ms, while ESC 6 and ESC 7 are still on the
wire. Their frames are not lost, they sit in the FIFO and get read on the next
poll, 20 ms later. So roughly half the time, two of your eight joints feed the
network a position and velocity that are one full control period old, and
nothing anywhere reports it. On a walking policy that is a real defect and it
will look like a gait problem, not a bus problem.

Three fixes, in increasing order of correctness.

*Cheapest.* Widen the window and drain again just before you build the
observation:

```c
CAN_SendActions(actions);
vTaskDelay(pdMS_TO_TICKS(4));   // >= 1204 us of bus time plus ESC latency
CAN_Poll();
```

*Better.* Wait for the count you expect, with a deadline, so the loop adapts:

```c
static uint32_t esc_last_seen_ms[8];

static uint32_t CAN_PollUntil(uint32_t want, uint32_t timeout_us)
{
  uint32_t t0 = micros(), got = 0;
  uint8_t  seen = 0;                       // bitmask of ESCs heard from
  while ((micros() - t0) < timeout_us && got < want)
  {
    FDCAN_RxHeaderTypeDef rx;
    uint8_t buf[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
    {
      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, buf) != HAL_OK)
        break;
      uint32_t idx = rx.Identifier - CAN_ID_FB_BASE;
      if (idx < 8 && dlc_to_bytes(rx.DataLength) >= 8)
      {
        memcpy(&joint_pos[idx],  &buf[0], 4);
        memcpy(&joint_vels[idx], &buf[4], 4);
        esc_last_seen_ms[idx] = millis();
        if (!(seen & (1u << idx))) { seen |= (1u << idx); got++; }
      }
    }
    if (got < want) taskYIELD();
  }
  return got;                              // caller decides what a shortfall means
}
```

*Best.* Stop polling. Enable `FDCAN_IT_RX_FIFO0_NEW_MESSAGE`, drain in
`HAL_FDCAN_RxFifo0Callback`, and have the ISR give a FreeRTOS notification to
the `Model` task once it has all eight. The control task then blocks on the
notification with a timeout instead of guessing at a delay.

Whichever you pick, add the staleness check. Right now, if an ESC drops off the
bus entirely, `joint_pos[i]` keeps its last value forever and the network walks
on a leg that is not reporting:

```c
static uint8_t CAN_StaleMask(uint32_t max_age_ms)
{
  uint8_t stale = 0;
  for (int i = 0; i < 8; i++)
    if (millis() - esc_last_seen_ms[i] > max_age_ms) stale |= (1u << i);
  return stale;
}
```

### 6.5 Two smaller things in `CAN_Poll`

`rx.DataLength` is ignored, so a short frame reads stale bytes out of `buf`.
Section 5.3 has the decode table; the guard is in `CAN_PollUntil` above.

`CAN_SendActions` returns a bool that `Model()` throws away:

```c
CAN_SendActions(actions);      // returns false when the Tx FIFO is full
```

That bool is your only indication that the bus has stopped working. Section 8.1
is about the case where it goes false forever and nothing else changes.


## 7. Errors

### 7.1 What the controller detects

Five error types, all detected by every node, not just the two involved:

| error | condition | `LastErrorCode` |
|---|---|---|
| bit | transmitted a level, sampled the other one (outside arbitration and ACK) | `FDCAN_PROTOCOL_ERROR_BIT0` / `_BIT1` |
| stuff | six consecutive bits of the same value where stuffing forbids it | `_STUFF` |
| form | a fixed-form field held the wrong value (delimiters, EOF, FDF) | `_FORM` |
| CRC | computed CRC differs from the received one | `_CRC` |
| ACK | transmitter sampled recessive in the ACK slot | `_ACK` |

`_BIT0` means it sent dominant and read recessive. `_BIT1` means it sent
recessive and read dominant. Those two distinguish "my transmitter is not
reaching the bus" from "something else is driving the bus", which is the single
most useful diagnostic split you have.

A node that detects any of these transmits an **error flag**: six dominant
bits. That deliberately violates bit stuffing, so every other node sees a stuff
error and joins in, and the frame is destroyed globally. The transmitter then
retries. This is why error handling is a bus-wide property, not a per-link one.

### 7.2 The three states

Two counters, both maintained in hardware. Transmit error counter (TEC) and
receive error counter (REC).

```
  TEC <= 127 and REC <= 127     ERROR ACTIVE
                                sends 6 dominant bits as an error flag.
                                Full participant. Destroys bad frames for
                                everyone, which is the point.

  TEC > 127 or REC > 127        ERROR PASSIVE
                                sends 6 *recessive* bits as an error flag,
                                which changes nothing on the wire. Still
                                transmits and receives, but must wait an extra
                                8 recessive bits (suspend transmission) before
                                starting a new frame, so it yields to healthy
                                nodes.

  TEC > 255                     BUS OFF
                                stops entirely. Drives nothing, ACKs nothing.
```

Rough counting rules: a transmit error is worth +8 on TEC, a receive error +1
on REC, and a successful transmit or receive is worth -1. The 8-to-1 asymmetry
is deliberate: the node causing the trouble climbs eight times faster than the
nodes merely observing it, so a single faulty node removes itself rather than
taking the bus down.

One exception matters enormously for section 8. **An error-passive transmitter
that detects an ACK error does not increment TEC.** That single rule is why the
first thing you are about to do produces the failure mode it does.

Bus-off recovery requires 128 occurrences of 11 consecutive recessive bits on
the RX input (ST's FDCAN documentation says 129 for this implementation). At
1 Mbit/s on an idle bus that is at least 1.4 ms. And on the H7 it does not start
until *software* clears `CCCR.INIT`. There is no `ABOM` bit here; that was
bxCAN. FDCAN never recovers on its own.

### 7.3 Reading it, in your HAL

```c
static void CAN_ReportStatus(void)
{
  FDCAN_ProtocolStatusTypeDef ps;
  FDCAN_ErrorCountersTypeDef  ec;
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
  HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

  static const char *lec[8] = {
    "none", "stuff", "form", "ack", "bit1", "bit0", "crc", "nochange"
  };

  Serial_.print("TEC=");        Serial_.print(ec.TxErrorCnt);
  Serial_.print(" REC=");       Serial_.print(ec.RxErrorCnt);
  Serial_.print(" lec=");       Serial_.print(lec[ps.LastErrorCode & 7]);
  Serial_.print(" warn=");      Serial_.print(ps.Warning);
  Serial_.print(" passive=");   Serial_.print(ps.ErrorPassive);
  Serial_.print(" busoff=");    Serial_.println(ps.BusOff);
}
```

`LastErrorCode` is sticky-until-read in the sense that it holds the last error
seen; `FDCAN_PROTOCOL_ERROR_NO_CHANGE` (7) means no new error since it was last
written. Call this once a second from a low-priority task and you have turned
every failure in this document into a printed line.

The interpretation table, which is most of the debugging you will ever do:

| what you see | what it means |
|---|---|
| `TEC=0 REC=0 lec=none` | nothing has been transmitted, or everything worked |
| `TEC=128 passive=1 lec=ack`, frozen | you are transmitting and nobody is acknowledging. Section 8.1 |
| `TEC` climbing to 256, `lec=bit0` | your dominant bits are not coming back. Transceiver disabled, unpowered, or not fitted |
| `lec=bit1` while idle | something else is holding the bus dominant. Shorted CANH/CANL, or a stuck node |
| `lec=stuff` or `form`, intermittent | bit rate mismatch, bad termination, or a clock out of tolerance |
| `lec=crc` | data corruption on the wire. Reflections, noise, missing twist |
| `busoff=1` | 32 or more consecutive failed transmissions. Recover explicitly, see below |

### 7.4 Recovering from bus-off

Two parts: notice, and restart.

```c
// in MX_FDCAN1_Init, before HAL_FDCAN_Start
HAL_FDCAN_ActivateNotification(&hfdcan1,
    FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING |
    FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
// FreeRTOS: any ISR that calls a FromISR API must be numerically at or above
// configMAX_SYSCALL_INTERRUPT_PRIORITY, i.e. a *lower* logical priority. 5 is
// safe with the STM32duino default; check yours before adding a notification.
HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

extern "C" void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }

volatile uint32_t can_busoff_count = 0;

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h,
                                              uint32_t ErrorStatusITs)
{
  if (ErrorStatusITs & FDCAN_IT_BUS_OFF)
  {
    can_busoff_count++;
    // ST's documented recovery: clear INIT so the controller starts counting
    // its 11-recessive-bit sequences. HAL_FDCAN_Start() will not do this,
    // because the handle is still in HAL_FDCAN_STATE_BUSY.
    CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}
```

`CLEAR_BIT` on `CCCR.INIT` is what ST's own note recommends and is the minimal
correct action. The pure-HAL alternative is `HAL_FDCAN_Stop()` followed by
`HAL_FDCAN_Start()`, which additionally aborts everything queued in the Tx
FIFO. That is usually what you want on a robot: after a bus fault, the eight
joint commands sitting in the queue are stale by hundreds of milliseconds and
you would rather send fresh ones.

Do not recover in a tight loop. If the fault is permanent you will thrash. Rate
limit, count, and after some threshold stop trying and fail safe:

```c
if (can_busoff_count > 10) { /* hold position, stop the gait, say so */ }
```

Which raises the question you have to answer in the policy, not the driver:
what does a walking robot do when its motor bus dies? Deciding that now is
cheaper than deciding it while the robot falls over.


## 8. Four questions

### 8.1 What happens the first time you run `FDCAN_MODE_NORMAL` on an empty bus

Two completely different things happen depending on whether your transceiver is
alive, and neither of them looks like a failure from software. Work out which
one you are in before you start guessing.

**Case A: transceiver powered, `STB` low, bus terminated, no other node.**

1. `HAL_FDCAN_AddMessageToTxFifoQ` queues the 32-byte command. The controller
   starts transmitting on PH13. The transceiver drives CANH/CANL.
2. Every dominant bit comes back through the transceiver's own receiver onto
   PI9 within its loop delay. Bit monitoring passes. Arbitration is won by
   default. The frame transmits cleanly all 313 bits.
3. At the ACK slot the transmitter sends recessive and expects to sample
   dominant, because in CAN *every* node that received the frame correctly
   drives the ACK bit, including nodes that had no interest in it. There are no
   other nodes. It samples recessive.
4. **ACK error.** `LastErrorCode` becomes `FDCAN_PROTOCOL_ERROR_ACK`. Being
   error-active, the node transmits an active error flag, six dominant bits,
   destroying its own frame. `TEC += 8`.
5. Error delimiter, intermission, and because `AutoRetransmission = ENABLE`,
   it retries the same frame. About 322 bit times per attempt, so about 322 us.
6. After 16 attempts, roughly 5 ms after you switched to normal mode,
   `TEC = 128`, which is greater than 127, and the node goes **error-passive**.
7. Here is the part that is not obvious. An error-passive transmitter that
   detects an ACK error and sees no dominant bit during its passive error flag
   **does not increment TEC**. The counter freezes at 128.
8. So the node never reaches bus-off. It retransmits the same frame, forever,
   at roughly 3000 attempts per second, at 100% bus utilisation, with
   `TEC = 128` and `busoff = 0`, for as long as the robot is powered.

What you observe, with the code exactly as it is today:

- Nothing on the serial console. `MX_FDCAN1_Init` printed no error and
  `Error_Handler()` was not called.
- `joint_pos[]` and `joint_vels[]` stay at their initialised zeros forever, so
  the network runs on a zero observation and produces whatever it produces.
- The eyes blink. The IMU works. The 50 Hz loop keeps its period.
- For the first 160 ms, `CAN_SendActions()` returns `true`: the head of the Tx
  FIFO is stuck retrying, but there are 8 slots and `Model` only adds one every
  20 ms. On the ninth call the FIFO is full, `HAL_FDCAN_AddMessageToTxFifoQ`
  returns `HAL_ERROR`, and `CAN_SendActions` returns `false` from then on.
  `Model()` discards that return value, so this changes nothing you can see.
- On a scope, PH13 is a continuous stream of well-formed frames. It looks like
  a working bus. It is the most convincing wrong-looking-right you will get.

The only symptom is `TEC = 128, ErrorPassive = 1, LastErrorCode = ack`, frozen,
which you can only see if you added `CAN_ReportStatus()` from 7.3. Add it
before you flash.

**Case B: transceiver absent, unpowered, or `STB` floating/high.**

Completely different, and much faster.

1. The controller drives PH13 dominant for SOF. Nothing converts it, so PI9
   never goes dominant.
2. The transmitter monitors the bus, sent dominant, sampled recessive. **Bit
   error**, `FDCAN_PROTOCOL_ERROR_BIT0`. The frame dies at bit 1. `TEC += 8`.
3. The ACK-error exception does not apply, because this is not an ACK error.
   TEC keeps climbing. It climbs faster still if the node also detects bit
   errors while transmitting its own error flag, which it will.
4. After 32 or fewer attempts, and each attempt is only about 18 bit times,
   `TEC > 255` and the node goes **bus-off**. Elapsed time: well under a
   millisecond.
5. Bus-off sets `CCCR.INIT` in hardware and the peripheral stops. On the H7
   there is no automatic recovery. It stays there.
6. The Tx FIFO fills within 160 ms and `CAN_SendActions` returns `false`
   forever. Again discarded.

Same visible symptoms as case A: none. Same fix: read the protocol status.

**What to do instead of flipping straight to normal mode.** There is a bring-up
ladder, and each rung isolates one thing:

| mode | drives PH13 | needs an ACK | proves |
|---|---|---|---|
| `FDCAN_MODE_INTERNAL_LOOPBACK` | no | no | message RAM, filters, DLC, your framing code |
| `FDCAN_MODE_EXTERNAL_LOOPBACK` | **yes** | no | the pin, the transceiver, the harness, termination |
| `FDCAN_MODE_BUS_MONITORING` | no | no | your bit timing matches a real transmitter |
| `FDCAN_MODE_NORMAL` | yes | yes | everything |

`FDCAN_MODE_EXTERNAL_LOOPBACK` is the rung you are missing and it is the
important one. The controller transmits real frames out of PH13 and through the
transceiver onto the real wire, where you can put a scope on them, but it feeds
its receiver internally and ignores acknowledge errors. So you get a live,
correctly-timed CAN waveform to measure against, with no second node required
and no error counters climbing.

`FDCAN_MODE_BUS_MONITORING` is the other one worth knowing. The node never
drives a dominant bit, so it cannot ACK and cannot disturb anything. Power one
ESC, let it transmit, and see whether frames land in your FIFO. If they do,
your bit timing and your filter are both right, and you have proved it without
risk. If they do not, and the ESC is definitely transmitting (scope it), your
bit rate does not match.

Run the ladder in order. Each rung takes ten minutes and eliminates a category.

### 8.2 Termination

**Where.** Exactly two 120 ohm resistors, one across CANH-CANL at each physical
*end* of the trunk. Not at every node. Not in the middle. A CAN bus is a
transmission line of roughly 120 ohm characteristic impedance and you terminate
both ends of it, the same as any other transmission line.

```
   [mainboard]   [ESC0]  [ESC1]  ...  [ESC6]        [ESC7]
        |          |       |            |             |
  120R==+==========+=======+============+=============+==120R
        |          |       |            |             |
                   ^ stubs must be short, < 30 cm at 1 Mbit/s
```

The two resistors in parallel present 60 ohm to whichever transceiver is
driving. That is the load the 2 V dominant differential is specified into.

Termination does two jobs, and people usually only remember the first. It
absorbs reflections. It also provides the resistive path that pulls the pair
back to zero differential when every driver releases for a recessive bit. That
second job sets how fast recessive recovery is, and it is the one that bites
first, because the dominant-to-recessive edge is passive.

**What goes wrong.**

| mistake | resistance CANH-CANL, power off | symptom |
|---|---|---|
| none fitted | open, megohms | slow recessive recovery, ringing, noise pickup on a high-impedance bus, random stuff and form errors that get worse when motors run |
| one only | 120 ohm | reflections off the open end; dominant level higher than spec into the lighter load; often works on a short bench harness and fails in the robot |
| three or more | 40 ohm or less | transceiver cannot pull the differential to the 1.5 V minimum; the far end of the bus fails first |
| fitted at middle nodes | 60 ohm, looks correct | the sections beyond the terminators are unterminated stubs and ring |

The three-terminator case is the one to actually worry about, because most CAN
transceiver breakout boards ship with a 120 ohm resistor already soldered on.
Eight ESC breakouts plus the mainboard is nine resistors in parallel, 13 ohm,
and nothing works at all. Check every board and desolder or unjumper all but
two.

**How to check, in order.**

1. Power the whole bus off. Multimeter on resistance, probes on CANH and CANL
   at any point on the trunk. You want **60 ohm**, give or take a couple of
   ohms. 120 means one, 40 means three, 30 means four, open means none, near
   zero means a short.
2. Same conditions, CANH to ground and CANL to ground. Both should be
   megohms. Tens of ohms means a short to ground somewhere.
3. Power on, bus idle. CANH and CANL should both sit near 2.5 V with respect to
   ground for a 5 V transceiver, and their difference should be under 0.5 V.
4. Scope, two channels on CANH and CANL, math channel set to A minus B. During
   traffic, dominant should be 1.5 to 3.0 V differential and recessive under
   0.5 V. Look specifically at the dominant-to-recessive edge. If it rings,
   overshoots, or takes a large fraction of a bit to settle, your termination
   is wrong or your stubs are too long. That edge is the sensitive one.

**Layout, while you are here.** Daisy-chain, never star. Keep the pair twisted
right up to each connector. At 1 Mbit/s the theoretical bus length limit is
around 40 m and a walking robot is a metre or two, so length is not your
problem; stub length and star topology are.

### 8.3 Worked exercise: retiming for a 75 MHz kernel clock

You have already made this change in `src/main.cpp`. Do the derivation from
scratch anyway, without looking, then compare against both the answer and your
own code.

**The problem.** `fdcan_ker_ck` is now 75 MHz instead of 50 MHz. Keep the bit
rate at exactly 1 Mbit/s. Choose `NominalPrescaler`, `NominalTimeSeg1`,
`NominalTimeSeg2` and `NominalSyncJumpWidth`. Aim for an 80% sample point.
Then say where the sample point actually lands and what that costs.

Constraints you have to respect: the HAL allows `NominalPrescaler` 1 to 512,
`NominalTimeSeg1` 2 to 256, `NominalTimeSeg2` 2 to 128, and
`NominalSyncJumpWidth` 1 to 128, with `SJW <= min(Seg1, Seg2)`.

Stop here and work it out.

---

**Answer.**

Step 1. The bit rate equation has only one unknown product:

```
  bit_rate = fdcan_ker_ck / (prescaler * N_tq)
  1e6      = 75e6 / (prescaler * N_tq)
  prescaler * N_tq = 75
```

Step 2. Factor 75. There are only six options, and most are unusable:

| prescaler | N_tq | t_q | usable |
|---|---|---|---|
| 1 | 75 | 13.33 ns | yes |
| 3 | 25 | 40.00 ns | yes |
| 5 | 15 | 66.67 ns | yes |
| 15 | 5 | 200.00 ns | technically, but Seg1 and Seg2 are both 2 minimum, so the best sample point is 3/5 = 60% |
| 25 | 3 | 333 ns | no, Seg1 + Seg2 would be 2 and each has a minimum of 2 |
| 75 | 1 | 1000 ns | no |

Step 3. Split each usable N_tq for 80%. The sample point is `(1 + Seg1) / N_tq`,
so you want `1 + Seg1 = 0.80 * N_tq`:

| prescaler | N_tq | Seg1 | Seg2 | SJW | sample point |
|---|---|---|---|---|---|
| 1 | 75 | 59 | 15 | 15 | 60/75 = **80.00%** |
| 3 | 25 | 19 | 5 | 5 | 20/25 = **80.00%** |
| 5 | 15 | 11 | 3 | 3 | 12/15 = **80.00%** |

All three are exact, because 75 is divisible by 5 and 80% is 4/5. That is luck,
and it is the reason 75 MHz is a pleasant clock for 1 Mbit/s. Verify one:

```
  75e6 / (3 * 25) = 75e6 / 75 = 1 000 000 bit/s      exact
  (1 + 19) / 25   = 0.80                             exact
```

Step 4. Compare with what is in your code:

```c
hfdcan1.Init.NominalPrescaler     = 5;
hfdcan1.Init.NominalTimeSeg1      = 12;    // -> 15 tq, sample point 13/15
hfdcan1.Init.NominalTimeSeg2      = 2;
hfdcan1.Init.NominalSyncJumpWidth = 2;
```

The bit rate is right, 1.000 Mbit/s exactly. The sample point is 86.67%, not
80%. Whoever retimed it kept `Seg2 = 2` from the 50 MHz configuration and put
all five extra time quanta into `Seg1`. That is a choice, not a mistake, but it
is a choice with two consequences.

**The tradeoff.** From section 4.2, a later sample point means a shorter
Phase_Seg2, and Phase_Seg2 is what the oscillator tolerance depends on:

| config | sample point | oscillator tolerance | transceiver loop budget, 2 m bus |
|---|---|---|---|
| p5, 12/2, SJW 2 (your code) | 86.67% | +/- 0.518% | 323 ns |
| p5, 11/3, SJW 3 | 80.00% | +/- 0.781% | 257 ns |
| p3, 19/5, SJW 5 | 80.00% | +/- 0.781% | 257 ns |
| p1, 59/15, SJW 15 | 80.00% | +/- 0.781% | 257 ns |

Moving the sample point later trades clock accuracy for propagation delay,
one for one. Your 86.67% buys 66 ns of extra transceiver budget and costs you a
third of your clock tolerance. With a TJA1051T/3's 250 ns worst-case loop
delay, 80% leaves 7 ns of margin and 86.67% leaves 73 ns, so the case for
keeping 86.67% is real, provided every node on the bus has a crystal. If any
ESC is RC-clocked, 0.518% is not survivable and you should be at 80% with a
faster transceiver instead.

**The other thing the table shows.** All three 80% rows have identical
tolerance. More time quanta does not buy accuracy. What it buys is
*resolution*: at 15 tq the sample point moves in 6.67% steps, at 25 tq in 4.0%
steps, at 75 tq in 1.33% steps. That matters when an ESC vendor specifies a
sample point you have to land near. If you want the flexibility for free, use
`prescaler = 3, Seg1 = 19, Seg2 = 5, SJW = 5`: exactly 1 Mbit/s, exactly 80%,
a 5 tq jump width, and 4% granularity if you need to move.

**The failure you avoided.** If the clock had gone to 75 MHz and the timing had
*not* been retimed, `75e6 / (5 * 10)` is 1.5 Mbit/s and nothing would talk. And
if the timing was retimed but the clock is still 50 MHz for some reason,
`50e6 / (5 * 15)` is 666.7 kbit/s and nothing would talk. Both are silent in
software. This is the third time this document has told you to print the kernel
clock.

### 8.4 Debugging a bus that does not work

Ordered by cost. Do not skip ahead; each step is cheaper than the one after it
and eliminates more.

**Free, software only**

1. **Print the clocks.** Section 0. `fdcan_ker_ck`, SYSCLK, and the PLL source.
   If the kernel clock is not what your bit timing assumes, stop here; nothing
   else matters. Five minutes.
2. **Print the protocol status and error counters, once a second.** Section
   7.3, and the interpretation table there. This one addition identifies most
   of the failures in this document by name. If you do exactly one thing on
   this list, do this one.
3. **Verify your bit timing arithmetic against the register.** Read
   `FDCAN1->NBTP` and decode it. `NBRP`, `NTSEG1` and `NTSEG2` are all stored
   minus one, so what you read back should be prescaler 4, seg1 11, seg2 1 for
   the values in your code. If they do not match what you set,
   `HAL_FDCAN_Init` rejected something.
4. **Run `FDCAN_MODE_EXTERNAL_LOOPBACK`.** Section 8.1. Real frames on the real
   wire, no ACK needed, no error counters climbing. Everything from step 6
   onward is easier to do in this mode.
5. **Run `FDCAN_MODE_BUS_MONITORING` with one ESC transmitting.** If frames
   arrive, your bit timing and filter are correct and the problem is on your
   transmit side. If they do not, and the ESC is definitely sending, your bit
   rate does not match theirs.

**Multimeter**

6. **Power off. 60 ohm across CANH-CANL.** Section 8.2. Then CANH to ground and
   CANL to ground: both megohms.
7. **Power on, bus idle. Transceiver pins.** `VCC` 5 V, `VIO` 3.3 V, `STB`/`S`
   at 0 V. CANH and CANL both near 2.5 V, difference under 0.5 V. A floating
   `STB` pin is the single most common first-board fault and this is where you
   catch it.
8. **Continuity end to end** on CANH and on CANL, and confirm they are not
   swapped at any connector. Swapped CANH/CANL is not fatal for a single link
   but it is fatal on a shared bus.

**Oscilloscope, one channel**

9. **Probe PH13 with the controller transmitting.** Are there bits at all? Then
   measure the width of one bit with the cursors. It must be 1.00 us. This is
   the direct measurement that steps 1 and 3 were inferring. If it reads
   1.50 us your kernel clock is 50 MHz, not 75.
10. **Probe PI9.** In external loopback with a live transceiver it should
    mirror PH13, delayed. Stuck high means the transceiver is not driving the
    bus: check `STB`, `VCC`, and the TXD connection. Stuck low means something
    is holding the bus dominant.

**Oscilloscope, two channels**

11. **PH13 and PI9 together. Measure the loop delay.** Cursor the delay from a
    PH13 falling edge to the corresponding PI9 falling edge. That is the number
    section 3.4 budgeted. If it exceeds the budget you computed, your sample
    point is too early for your transceiver and you have found a real cause of
    intermittent errors.
12. **CANH and CANL, math channel A minus B.** Dominant 1.5 to 3.0 V, recessive
    under 0.5 V. Study the dominant-to-recessive edge for ringing. Section 8.2.
13. **Trigger on SOF and count to the ACK slot.** For an FD base frame with a
    32-byte payload, counting SOF as bit 1, the fields run: 22 bits of header,
    256 of data, 4 of stuff count, 21 of CRC, 1 CRC delimiter, and then the ACK
    slot at bit 305. Trigger on the falling edge that starts the frame, set the
    horizontal delay to 300 us, and look at the differential in the window that
    follows. Stuff bits push it later, so search 304 to 380 us rather than
    expecting an exact position. If the bus goes dominant in the ACK slot while
    your own TX pin is recessive, someone acknowledged you. That is direct
    proof a second node is alive and agrees with your bit timing, and it is the
    single most informative measurement on this list.
14. **Scope an ESC's own transmission and measure its bit width.** If it is not
    1.00 us you have a bit rate mismatch, and now you know which side is wrong.

**Bisection**

15. **Disconnect all ESCs but one.** Then add them back one at a time. A single
    node with the wrong bit rate, no crystal, or a classic-only CAN controller
    (section 5.2) breaks the whole bus, and this finds it in three iterations.

**Build the analyser you do not own**

16. **Second MCU in `FDCAN_MODE_BUS_MONITORING`.** Any spare FDCAN-capable part,
    accept-all filter, dump identifier, DLC and payload bytes over serial. Bus
    monitoring never drives a dominant bit and never acknowledges, so it cannot
    disturb a bus that is working. This is a free CAN analyser and it is worth
    the afternoon; a real one costs money and this one you can leave soldered
    to the robot.


## 9. Exercises

Work them before reading the answers. They are all things you will actually
have to do.

**E1.** Your kernel clock is 75 MHz. You need 500 kbit/s for a slower auxiliary
bus, sample point 80%. Give prescaler, Seg1, Seg2, SJW.

**E2.** For the configuration you just derived, compute the maximum oscillator
tolerance from both ISO conditions, and say which one binds.

**E3.** You want to send 20 bytes in one frame. Which `FDCAN_DLC_BYTES_*`
constant? Now you want 36 bytes. What do you do?

**E4.** A colleague sets `FilterID1 = 0x180`, `FilterID2 = 0x780`,
`FilterType = FDCAN_FILTER_MASK`. Which identifiers reach the FIFO?

**E5.** Three nodes start transmitting in the same bit time, with identifiers
0x100, 0x080 and 0x101. Who wins, and at which identifier bit does each loser
drop out? Then: what happens if two nodes transmit identifier 0x100 with
different payloads in the same bit time?

**E6.** You add FDCAN2 with 4 standard filters, an 8-element Rx FIFO0 of
16-byte elements, and a 4-element Tx FIFO of 16-byte elements. How many words
does it need, what must you set `MessageRAMOffset` to, and does it fit?

**E7.** You change the ESC feedback to FD frames carrying 32 bytes each, still
at 50 Hz. What is the new bus load, and what does it do to the 2 ms window in
`Model()`?

**E8.** Your ESC datasheet specifies a sample point of 87.5%. Must yours match?
Can you hit 87.5% exactly at 1 Mbit/s from a 75 MHz kernel clock?

**E9.** Name the fault for each of these, printed by `CAN_ReportStatus()`:

```
  (a) TEC=248 REC=0 lec=bit0   warn=1 passive=1 busoff=0   and climbing fast
  (b) TEC=128 REC=0 lec=ack    warn=1 passive=1 busoff=0   and not changing
  (c) TEC=0   REC=37 lec=stuff warn=0 passive=0 busoff=0   REC creeping up
  (d) TEC=0   REC=0  lec=bit1  warn=0 passive=0 busoff=0   bus should be idle
```

---

### Answers

**E1.** `prescaler * N_tq = 75e6 / 500e3 = 150`. Factor it: 2 x 75, 6 x 25,
10 x 15 are all usable. Take `prescaler = 6, N_tq = 25`, then
`1 + Seg1 = 0.80 * 25 = 20`, so `Seg1 = 19`, `Seg2 = 5`, `SJW = 5`.
Check: `75e6 / (6 * 25) = 500 000` exactly, sample point `20/25 = 80.0%`.

**E2.**

```
  (1)  SJW / (2 * 10 * N_tq)                = 5 / (2*10*25)      = 5/500 = 1.000%
  (2)  min(PhSeg1,PhSeg2) / (2*(13*N_tq - PhSeg2)) = 5 / (2*(325-5)) = 5/640 = 0.781%
```

Condition (2) binds, at +/- 0.781%. Same as every other 80% configuration, which
is the point section 4.2 was making: tolerance follows the sample point, not
the time quantum count.

**E3.** 20 bytes is `FDCAN_DLC_BYTES_20`, which is DLC 11. 36 bytes does not
exist. The encoding goes 32, 48, 64 with nothing between, so you either send 48
and pad 12 bytes you will pay for on the wire, or split into two frames, or
redesign the payload down to 32. Design payloads to land on 0-8, 12, 16, 20,
24, 32, 48 or 64.

**E4.** Accept when `(id & 0x780) == (0x180 & 0x780) = 0x180`. The mask 0x780
pins ID10 to ID7 and leaves ID6 to ID0 free, so the accepted set is
**0x180 to 0x1FF**, 128 identifiers.

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

0x080 wins at ID8: it sends dominant while the other two send recessive, so
both of them lose there, on the same bit. Later, when 0x100 and 0x101 retry
together, they tie all the way to ID0, where 0x100 sends dominant and wins.
Lower identifier always wins, and it wins at the most significant bit where the
identifiers differ.

Two nodes sending 0x100 with different payloads is the failure case. They tie
through the entire identifier, through RRS, IDE, FDF and DLC if those match,
and neither knows it lost. At the first data bit where the payloads differ, one
node transmits recessive and samples dominant *outside* the arbitration field,
which is a bit error, not a lost arbitration. Both frames are destroyed, both
error counters climb, and both retransmit and collide again. One identifier,
one transmitter. Always.

**E6.** A 16-byte element is 2 header words plus 4 data words, 6 words. So
`4 * 1 + 8 * 6 + 4 * 6 = 4 + 48 + 24 = 76 words`. FDCAN1 occupies words 0 to
576, so FDCAN2 needs `MessageRAMOffset = 577`. Total 653 words of 2560, about
26%. It fits comfortably. If you forget the offset, both peripherals write from
word 0 and corrupt each other, which presents as random garbage in both.

**E7.** An FD frame with 32 bytes and no BRS is 316 bit times including
intermission. Nine of them per cycle:
`9 * 316 = 2844 bits = 2844 us`, so bus load goes from 6.0% to
**2844 / 20000 = 14.2%**. Still fine for the bus. Not fine for `Model()`: the
existing `vTaskDelay(pdMS_TO_TICKS(2))` gives 1 to 2 ms and you now need at
least 2.85 ms of bus time plus ESC latency. You would be polling before even
half the replies had arrived. Widen the window to 4 ms or move to the
notification-driven receive in section 6.4.

**E8.** No, sample points do not have to match. Nodes must agree on the bit
*rate*; each node independently picks its own prescaler, time quantum count and
sample point, and independently has to satisfy its own oscillator tolerance and
propagation constraints. A node with an 80% sample point and a node with an
87.5% sample point interoperate.

You cannot hit 87.5% exactly anyway. 87.5% is 7/8, so `1 + Seg1` must be
`7/8 * N_tq`, which needs `N_tq` divisible by 8. But `prescaler * N_tq = 75`
and 75 has no factor of 8. The closest you can get at 15 tq is 86.67%; at 25 tq
it is 88.0% (`Seg1 = 21, Seg2 = 3`); at 75 tq it is 86.67% or 88.0%. Aim for
the neighbourhood and satisfy the two constraints; the exact figure is not
sacred.

**E9.**

(a) `lec=bit0` and TEC climbing fast means the node sends dominant and samples
recessive: its dominant bits are not reaching the bus. Transceiver unpowered,
`STB`/`S` not pulled low, TXD not connected, or no transceiver fitted. It will
be bus-off within a millisecond. Case B in section 8.1.

(b) `lec=ack` with TEC frozen at exactly 128 means the frames are getting out
correctly and nobody is acknowledging. There is no other functioning node on
the bus, or every other node is in bus monitoring mode or bus-off. Case A in
section 8.1. TEC is frozen because of the error-passive ACK exception in
section 7.2; this node will retransmit forever and never go bus-off.

(c) TEC zero and REC creeping up with `lec=stuff` means you are *receiving*
frames that are being corrupted. You are not the transmitter, so you are not to
blame, but something on the bus is: a node whose bit rate does not match,
a node whose clock is outside the tolerance from section 4, reflections from
bad termination, or a classic-only controller error-flagging your FD frames
(section 5.2). Scope the differential pair.

(d) `lec=bit1` on an idle bus means the node sent recessive and sampled
dominant when nothing should be driving. Something is holding the bus dominant:
CANH shorted to CANL, a shorted transceiver, or a node stuck transmitting.
Power everything down and go to step 6 of the debug checklist.


## 10. The change list

Ordered by what will hurt most if you skip it. Nothing here is speculative;
each item is a line in `src/main.cpp` or `PINOUT.md`.

**Before you flash `FDCAN_MODE_NORMAL` again**

1. Add `ReportClocks()` from section 0 and confirm `fdcan_ker_ck` is 75000000
   and the PLL source is HSE. If it is not, your bus is not at 1 Mbit/s.
2. Add `CAN_ReportStatus()` from section 7.3 on a one-second timer. Without it
   every failure in section 8.1 is invisible.
3. Set `GPIO_PULLUP` on PI9 in `HAL_FDCAN_MspInit()`. A floating RX pin
   generates errors that never happened.
4. Confirm all eight ESCs use CAN FD capable controllers, not classic-only
   (section 5.2). This is a datasheet check and it is the most expensive thing
   to find out late.
5. Walk the bring-up ladder in section 8.1 rather than going straight to
   normal mode: internal loopback, external loopback, bus monitoring against
   one live ESC, normal with one ESC, normal with eight.

**Correctness**

6. Check the return value of `CAN_SendActions()` in `Model()`. It is your only
   in-band signal that the Tx FIFO has wedged.
7. Decode `rx.DataLength` in `CAN_Poll()` before copying 8 bytes
   (section 5.3). Today a short frame feeds stale buffer contents to the
   network as a joint position.
8. Add per-ESC `last_seen` timestamps and a staleness mask (section 6.4). A
   silent ESC currently looks identical to a stationary one.
9. Widen the receive window. `vTaskDelay(pdMS_TO_TICKS(2))` is 1 to 2 ms at the
   default tick rate and the bus needs at least 1204 us. Use 4 ms, or move to
   the notification-driven receive.
10. Add the bus-off handler from section 7.4. FDCAN never recovers by itself,
    and decide what the gait does when the motor bus dies.

**Tidying**

11. Tighten the filter to `FDCAN_FILTER_RANGE` 0x200 to 0x207, or mask 0x7F8
    (section 6.2). You currently accept 256 identifiers to use 8.
12. Decide the sample point deliberately. `Seg1 = 12, Seg2 = 2` gives 86.67%
    and 0.518% clock tolerance; `Seg1 = 11, Seg2 = 3, SJW = 3` gives 80.0% and
    0.781%; `prescaler = 3, Seg1 = 19, Seg2 = 5, SJW = 5` gives the same 80%
    with finer granularity to move later. Section 8.3 has the reasoning.
13. Reserve 0x000 to 0x0FF for emergency stop and fault reporting, above your
    commands (section 2.2).
14. Update `PINOUT.md`. It says internal loopback, 50 MHz, HSI and PLL2
    `M=4 N=9 FRACN=3072`; the code says normal mode, 75 MHz, HSE and PLL2
    `M=2 N=12 FRACN=0`. Two sources of truth that disagree is worse than one
    that is wrong, because you will trust whichever one you read last.
15. Note in `PINOUT.md` that PH0 is now load-bearing. `SystemClock_Config()`
    selects HSE, so the crystal is required for the board to boot at all: with
    no HSE, `HAL_RCC_OscConfig` times out and `Error_Handler()` hangs before
    `setup()` reaches `Serial_.begin()`. Confirm whether the design uses a
    crystal on PH0 and PH1 (`RCC_HSE_ON`, which is what the code says) or a
    clock input on PH0 alone (`RCC_HSE_BYPASS`), because those need different
    parts fitted.

**Not now, but know why**

16. Leave BRS off. Your bus load is 6% and your data timing is currently a copy
    of your nominal timing, so turning it on would change nothing except adding
    a transmitter-delay-compensation problem (section 5.4).
17. Leave `RxFifo0ElmtSize` at 64 bytes. It costs 1728 bytes of a 10 Kbyte pool
    you are not using, and a frame longer than the element size is silently
    truncated rather than rejected.
18. If the FIFO ever gives you ordering or starvation trouble, the redesign is
    eight dedicated Rx buffers, one per ESC, via `FDCAN_FILTER_TO_RXBUFFER`
    (section 6.2). Your topology is the case that feature exists for.


## Reference

Numbers in this document were derived from `src/main.cpp` as committed, and
checked against:

- ISO 11898-1 error-confinement and bit-timing rules, as summarised in TI
  SPRAC35, *Calculator for CAN Bit Timing Parameters*.
- ST AN5348, *Introduction to FDCAN peripherals for STM32 MCUs*, for message
  RAM element sizes and Rx FIFO blocking versus overwrite behaviour.
- `stm32h7xx_hal_fdcan.h` from the STM32H7 HAL driver, for the DLC constants,
  the protocol status and error counter structures, and the operating modes.
- ST's note *How to recover from bus-off state with FDCAN on STM32 MCUs*, for
  the fact that the H7 has no automatic bus-off recovery and the `CCCR.INIT`
  procedure.
- NXP TJA1051 data sheet, for the 250 ns worst-case loop delay used in the
  propagation budget.
- STM32H742xI/G, STM32H743xI/G data sheet, for HSI accuracy and the HSE range.

Confirm the transceiver number against whatever part you actually fit, and the
HSI figures against your datasheet revision. Everything else follows from
arithmetic you can redo.
