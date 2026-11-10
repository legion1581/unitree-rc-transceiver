# Unitree handheld RC ↔ robot radio link — protocol

This document describes the 2.4 GHz LoRa link between a Unitree handheld remote and a
robot's RF module. It is the reference for both sides implemented in this
project: transmitting as a remote and receiving one.

---

## 1. Parts and data path

```
 handheld remote                         robot
 ┌──────────────┐   LoRa 2.4 GHz    ┌────────────────┐  UART 115200  ┌──────────┐        ┌─────┐
 │ main MCU ──► │ ───────────────►  │ RF module      │ ────────────► │ body MCU │ ─────► │ SoC │
 │ RF module    │ ◄───────────────  │ (SX1280)       │ ◄──────────── │          │ ◄───── │     │
 └──────────────┘   short ACKs      └────────────────┘  frames        └──────────┘        └─────┘
```

- The robot's RF module is a **bridge**: it filters frames by address, forwards the
  payload to the body MCU, and sends short ACK frames back. It does not read sticks.
- Stick/button encoding and the application checksum are produced by the remote's main
  MCU and consumed above the RF module on the robot side.
- There is no application-layer cipher; payloads are carried in clear.

## 2. Identity: code → ID → air-id → channel

Each remote has a 6-character code printed on it. Everything else is derived from it.
A fictional code `AB12CD` is used as the worked example below.

### 2.1 Code → 10-digit ID

Read each character as base 36 (`0-9` = 0–9, `A-Z` = 10–35), call them `v0..v5`:

```
ID = "%02d%02d%02d%04d" % (v1 + 21, v2, v3, v4*36 + v5)      # v0 is ignored
```

Example: `AB12CD` → v = `[10, 11, 1, 2, 12, 13]` → `ID = 3201020445`.

### 2.2 ID → air-id (2-byte link address)

```
air_A = (ID % 1000) & 0xFF
air_B = ((ID / 10000) % 100 + 18 * ((ID / 1000000) % 100)) & 0xFF
```

With the ID written as digits `AA BB CC DDDD`, that is `air_A = (last 3 digits) mod 256`
and `air_B = (CC + 18·BB) mod 256`.

Example: `ID 3201020445` → `air_A = 445 mod 256 = 0xBD`, `air_B = (02 + 18·01) = 20 = 0x14`.

### 2.3 ID → channel

```
n    = ID % 100                 # last two digits
col  = n % 25
row  = n / 25                   # 0..3, the "home" row
freq = 2401.0 MHz + 0.9 MHz * CHAN[row*25 + col]
```

The link works in Hz (`CHAN[n]·900000 + 2401000000`). `CHAN[]` is the 100-entry channel
map in [§4](#4-channel-plan).

## 3. Physical layer

| parameter | value |
|-----------|-------|
| modem | SX1280, LoRa |
| spreading factor | SF5 |
| bandwidth | 406.25 kHz |
| coding rate | 4/6 **long interleaving** |
| preamble | 12 symbols |
| header | explicit (variable length) |
| payload CRC | on (hardware CRC) |
| IQ | standard |
| sync word | `0x1424` (SX1280 LoRa default) |

> **Reference-frequency note.** The SX1280 derives frequency from a 52 MHz reference and
> assumes it is exact. A transmitter whose crystal is off by a few tens of ppm will land
> outside the receiver's window (~100 kHz at this SF/BW). A matched remote/robot pair is
> fine; a third device (e.g. a dev board with a plain crystal) must correct for its own
> ppm error in software. See the project README's calibration section.

## 4. Channel plan

### 4.1 Search channels (0.9 MHz grid)

`CHAN[100]` holds the values 1..100 (with 62 appearing twice, at n=7 and n=57, and 12
not appearing). Frequencies in MHz, `freq = 2401.0 + 0.9·CHAN[row*25+col]`:

| col → | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 | 24 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| row 0 | 2424.4 | 2410.0 | 2418.1 | 2451.4 | 2414.5 | 2422.6 | 2448.7 | 2456.8 | 2419.9 | 2453.2 | 2416.3 | 2405.5 | 2436.1 | 2421.7 | 2402.8 | 2455.9 | 2419.0 | 2407.3 | 2437.9 | 2423.5 | 2404.6 | 2457.7 | 2420.8 | 2409.1 | 2439.7 |
| row 1 | 2446.9 | 2432.5 | 2463.1 | 2406.4 | 2459.5 | 2445.1 | 2426.2 | 2434.3 | 2464.9 | 2408.2 | 2461.3 | 2428.0 | 2458.6 | 2444.2 | 2447.8 | 2410.9 | 2464.0 | 2429.8 | 2460.4 | 2446.0 | 2449.6 | 2412.7 | 2465.8 | 2431.6 | 2462.2 |
| row 2 | 2401.9 | 2455.0 | 2440.6 | 2428.9 | 2437.0 | 2467.6 | 2403.7 | 2456.8 | 2442.4 | 2430.7 | 2438.8 | 2450.5 | 2413.6 | 2466.7 | 2425.3 | 2433.4 | 2441.5 | 2452.3 | 2415.4 | 2468.5 | 2427.1 | 2435.2 | 2443.3 | 2454.1 | 2417.2 |
| row 3 | 2469.4 | 2477.5 | 2485.6 | 2473.9 | 2482.0 | 2490.1 | 2471.2 | 2479.3 | 2487.4 | 2475.7 | 2483.8 | 2473.0 | 2481.1 | 2489.2 | 2470.3 | 2478.4 | 2486.5 | 2474.8 | 2482.9 | 2491.0 | 2472.1 | 2480.2 | 2488.3 | 2476.6 | 2484.7 |

- An ID's **home channel** is the cell at (`row = n/25`, `col = n%25`).
- An ID's **search set** is the whole column `col`: four channels, one per row. A
  searching remote and the robot both work through this column and rendezvous there.

Example — ID 3201020445 (n = 45, col 20): home 2449.6 MHz; search set 2404.6 / 2449.6 / 2427.1 / 2472.1.

### 4.2 Bonded channels (1 MHz grid)

```
freq = 2400.0 MHz + ch * 1.0 MHz,   ch ∈ 92..99   →  2492..2499 MHz
```

`ch` arrives in an arm command ([§5.3](#53-command-frames-ff-ff-ff-xx)); out-of-range `ch`
falls back to 0 (2400.0 MHz). This band sits above the entire search band (max 2491.0
MHz), so the two never overlap.

## 5. Frames on air

Byte 0 is the first payload byte; lengths exclude header and CRC.

### 5.1 Remote → robot: control frame (42 bytes)

Sent repeatedly by a powered-on remote.

| offset | size | field |
|--------|------|-------|
| 0 | 1 | `air_A` |
| 1 | 1 | `air_B` |
| 2 | 2 | magic `55 51` |
| 4 | 1 | buttons 1: bit0 R1, bit1 L1, bit2 Start, bit3 Select, bit4 R2, bit5 L2, bit6 F1, bit7 F2 |
| 5 | 1 | buttons 2: bit0 A, bit1 B, bit2 X, bit3 Y, bit4 Up, bit5 Right, bit6 Down, bit7 Left |
| 6 | 4 | LX, float32 LE |
| 10 | 4 | RX, float32 LE |
| 14 | 4 | RY, float32 LE |
| 18 | 4 | (reserved, 0) |
| 22 | 4 | LY, float32 LE |
| 26 | 14 | (reserved, 0) |
| 40 | 2 | checksum = `sum(bytes[4:40]) & 0xFFFF`, **big-endian** |

Idle frame (air-id `bd 14`, sticks centred report `-0.0` = `00 00 00 80`):

```
bd 14 55 51 00 00 00 00 00 80 00 00 00 80 00 00 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 80
```

The robot module strips bytes 0–1 and forwards the rest (`55 51 …`, 40 bytes).

### 5.2 Robot → remote: ACK / downlink

The robot module transmits only these short replies, each right after it accepts a frame:

| reply | bytes | sent after |
|-------|-------|-----------|
| unicast ACK | `air_A air_B 50` (3 B) | an addressed frame ([§5.4](#54-addressed-frames-unicast)) |
| data ACK | `FF FF air_A air_B` (4 B) | a bonded data frame addressed to this device |

A remote uses this downlink for its link / signal indicators (the RSSI of the ACK).

### 5.3 Command frames `FF FF FF xx`

| frame | len | effect on the robot module |
|-------|-----|----------------------------|
| `FF FF FF FE ch` | 5 | **arm, all**: go bonded on `2400 + ch` MHz |
| `FF FF FF FC A ch` | 6 | **arm, targeted**: as above only if `A == air_A` |
| `FF FF FF FD` | any | **disarm, all** |
| `FF FF FF FB A` | any | **disarm, targeted** (if `A == air_A`) |

### 5.4 Addressed frames (unicast)

Any frame that does **not** start with `FF FF` must start with `air_A air_B`. On a match
the module forwards bytes `[2:]` to the host and replies with the unicast ACK; otherwise
the frame is dropped. The 42-byte control frame ([§5.1](#51-remote--robot-control-frame-42-bytes))
is an addressed frame.

### 5.5 Bonded data frames

Format `FF FF A x payload…` (≥ 7 bytes), accepted while the module is tuned to the bonded
band or 2400.0 MHz. The payload (bytes `[4:]`) is forwarded to the host; if `A == air_A`
the module also replies with the data ACK.

## 6. Link behaviour

```
boot / disarm
     │  load ID, derive air-id + home channel, configure radio
     ▼
 ┌─────────┐  listen on the home channel; if no hit, step to the next row of the same
 │ SEARCH  │  column (home, home+1, … mod 4). A frame for a different air-id is skipped.
 └────┬────┘
      │ addressed frame with our air-id
      ▼
 ┌─────────┐  stay on channel, forward payloads, ACK each one
 │ LOCKED  │  link lost -> back to SEARCH
 └────┬────┘
      │ arm command
      ▼
 ┌─────────┐  retune to 2400+ch MHz; accept addressed + bonded data frames, ACK them
 │ BONDED  │  disarm -> re-init -> SEARCH
 └─────────┘
```

A searching robot (no remote bonded) accepts the **first** addressed frame carrying a
matching air-id — there is no multi-step handshake — and begins forwarding its payload to
the host. Each newly received payload is forwarded once; when no new frame arrives the
module re-sends the last payload a few times, then goes quiet.

## 7. Robot module — host UART

- 115200 baud; one message per UART burst; no framing bytes, no CR/LF needed.
- Command set is exactly two commands; anything else starting with `AT+` → `AT CMD ERR!`.

| command | effect | reply |
|---------|--------|-------|
| `AT+T` | read identity | `OK!  is Dog!\nID: <ID>\nFrequency: <Hz>\n` |
| `AT+N=<10 digits>` | set the ID (message must be 15 bytes), derive air-id + channel, persist the ID, re-init the radio, restart search | `OK!\nID: <ID>\nFrequency: <Hz>\n` |

`AT+N` errors: missing `=` → `AT CMD ERR!`; wrong length → `CODE LEN ERR!`; non-digit →
`CODE ERR!`. A message that is not `AT+…` is ignored; the module does not forward host data
to the air. Received radio payloads are written to the UART raw, with their length.

## 8. Go2 integration

On the Go2 the robot side publishes the decoded controller on the DDS topic
**`rt/wirelesscontroller`** (`unitree_go::msg::dds_::WirelessController_`: `lx, ly, rx, ry,
keys`). Subscribing to it is the simplest way to confirm a link is being received:

```bash
export CYCLONEDDS_URI=file:///unitree/etc/cyclonedds_noshm.xml
cyclonedds subscribe rt/wirelesscontroller
```

The topic publishes while a controller is active; transmitting makes the stick fields
track the input you send. The RF module is reached from the SoC through the body MCU,
which exposes its `AT+T` / `AT+N` console ([§7](#7-robot-module--host-uart)).

## 9. Emulating a remote

To make a searching robot accept emulated input: pick the target remote's ID, transmit the
42-byte control frame ([§5.1](#51-remote--robot-control-frame-42-bytes)) on that ID's home
channel at the physical-layer settings in [§3](#3-physical-layer), repeating at ~10 Hz.
The robot bonds to the first matching frame and acts on the sticks/buttons; its ACK comes
back as the downlink ([§5.2](#52-robot--remote-ack--downlink)). The robot must have **no**
real remote bonded (its own remote off), or it will stay with that remote. This is what the
`drive` mode in [`../src/main.cpp`](../src/main.cpp) does.

## 10. Security properties

- **Addressing is not authentication.** The air-id is a function of the printed code, so
  anyone who knows the code knows the address.
- There is no key, cipher or rolling counter. Integrity is the LoRa CRC plus the remote's
  16-bit additive checksum.
- Arm/disarm commands are unauthenticated; the "all" variants act on every module in range.

## 11. Not fully characterised

- The complete bonded-link data format and its hop timing (the 2492–2499 MHz band).
- The reserved frame bytes (18–21 and 26–39), always zero in captures so far.
- The robot module's exact rule for giving up a locked channel and resuming search.
