# Beat Analyzer

Real-time beat analysis and VU metering over JACK/PipeWire with OSC output.

## Overview

```
JACK Audio (bpm_1, vu_1..vu_12)
       │
       ├── BTrack (FFT + onset + tempo)  →  Synthclock  →  /beat iif
       ├── VU meter (RMS + peak)                        →  /vu/0../vu/11 ff
       │
       ├── OSC receive (port 7775)       →  /clockmode, /tap, /beat
       └── Pioneer DJ Link (50000-50002) →  master beat straight from the mixer
```

### Clock modes

| Mode | Source | What it does |
|-------|--------|--------------|
| 0 — a3motion | OSC port 7775 | Receives `/beat` and relays it to every target except motion |
| 1 — internal | BTrack + Synthclock | Detects the beat from audio itself, sends to everyone |
| 2 — pioneer | Pro DJ Link | Takes the master beat from the Pioneer network, sends to everyone |

Switch over OSC: `/clockmode i` (0, 1 or 2) to port 7775.

## Build

### Prerequisites

```bash
sudo apt-get install -y \
    build-essential cmake pkg-config \
    jackd2 libjack-jackd2-dev \
    libsamplerate0-dev
```
### Compiling

```bash
./build.sh            # or: mkdir build && cd build && cmake .. && make -j$(nproc)
```

Binary: `build/beat-analyzer`

### Tests

```bash
cd build && ctest
```

## Configuration

Everything is set through `.env` in the working directory (typically `build/.env`).
Template: `.env.example` in the project root.

```bash
cp .env.example build/.env
```

### The variables that matter most

```bash
# Audio
JACK_CLIENT_NAME=beat-analyzer
NUM_BPM_CHANNELS=1              # JACK ports: bpm_1 (beat detection)
NUM_VU_CHANNELS=12              # JACK ports: vu_1..vu_12 (peak meters)
BPM_MIN=60
BPM_MAX=140

# OSC targets (as many as you like, format: name=host:port)
OSC_HOST_radla=192.168.43.96:9000
OSC_HOST_mixer=192.168.43.55:7771
OSC_HOST_motion=192.168.43.54:7771

# Separate VU ports (optional -- /beat and /vu on different ports)
OSC_VU_radla=192.168.43.96:9001
OSC_VU_mixer=192.168.43.55:7772
OSC_VU_motion=192.168.43.54:7772

# OSC receive
OSC_PORT_A3MOTION=7775          # /beat, /clockmode, /tap

# Pioneer Pro DJ Link
PIONEER_DEVICE_NUM=7            # virtual CDJ number (default: 7)

# VU meter
VU_RMS_ATTACK=0.8               # 0.0-1.0
VU_RMS_RELEASE=0.2              # 0.0-1.0
VU_PEAK_FALLOFF=20.0            # dB/s
OSC_SEND_RATE=25                # Hz (1-100)

# Debug
DEBUG_BEAT_CONSOLE=1
DEBUG_BTRACK_CONSOLE=0
DEBUG_PIONEER_CONSOLE=0
DEBUG_VU_CONSOLE=0
LOG_LEVEL=1                     # 0=DEBUG 1=INFO 2=WARN 3=ERROR
```

Every variable with its default: see `.env.example`.

## OSC protocol

### Outgoing

| Address | Type | Carries |
|---------|-----|--------|
| `/beat` | `iif` | beat (1-4), bar, bpm |
| `/vu/0` .. `/vu/11` | `ff` | peak, rms (linear 0.0-1.0) |

VU is sent as an OSC bundle -- one UDP packet for all channels.

#### JACK port → OSC index

**The JACK ports are 1-based, the OSC addresses are 0-based.** So port `vu_N` sends on
`/vu/(N-1)` (`jack_client.cpp` registers `vu_(i+1)`, `beat_analyzer_app.cpp` emits `/vu/i`):

| JACK port | OSC address |
|---|---|
| `vu_1` | `/vu/0` |
| `vu_2` | `/vu/1` |
| … | … |
| `vu_12` | `/vu/11` |

Patch while thinking in OSC indices and everything lands one channel across -- and the mistake
does not announce itself, because every channel still carries a plausible level.

#### What the channels mean in the A³ system

The beat-analyzer itself attaches no meaning to a channel: it reports one VU value per JACK
input, in port order. What an index *means* comes from the patching and from nowhere else. In
the A³ setup that is:

| JACK port | OSC address | Signal | Used in A³ Motion for |
|---|---|---|---|
| `vu_1` .. `vu_4` | `/vu/0` .. `/vu/3` | mixer channels 1-4 | the corona around each channel's blob |
| `vu_5` | `/vu/4` | subwoofer | sphere glow |
| `vu_6` .. `vu_9` | `/vu/5` .. `/vu/8` | speakers 1-4 | speaker beams |
| `vu_10` .. `vu_12` | `/vu/9` .. `/vu/11` | – | currently unused, discarded by A³ Motion |

The unused channels can still carry a level if something is patched into them. That is not a
sign that anything reads them.

A change to the patching therefore changes what the Motion UI shows, immediately and without a
warning anywhere. Keep this table current when the wiring changes.

With `OSC_VU_*` configured, `/vu` goes to the separate port and `/beat` stays on the main one.
Without it, everything shares one port.

### Incoming (port 7775)

| Address | Type | Carries |
|---------|-----|--------|
| `/beat` | `iif` | beat (1-4), bar, bpm |
| `/clockmode` | `i` | 0=a3motion, 1=internal, 2=pioneer |
| `/tap` | `i` | beat number (sets the phase) |

### Pioneer Pro DJ Link (mode 2)

Listens directly on the Pioneer network (UDP ports 50000/50001/50002).
Registers as a virtual CDJ and accepts beats only from the **tempo master**.
No OSC involved -- plain UDP speaking Pro DJ Link.

## Architecture

```
src/
├── main.cpp                    entry point, signal handlers
├── app/
│   ├── beat_analyzer_app.cpp     configuration, init, lifecycle
│   └── beat_processing.cpp       JACK callback, BTrack, Synthclock, VU
├── audio/
│   ├── jack_client.cpp         JACK I/O (BPM + VU ports)
│   └── audio_buffer.cpp        circular buffer
├── analysis/
│   ├── vu_meter.cpp            RMS + peak (runs in the JACK callback)
│   ├── grid_calculator.cpp     grid analysis from real beats
│   ├── beat_detection.cpp      onset detection (tests only)
│   └── beat_tracker.cpp        beat tracking (tests only)
├── osc/
│   ├── osc_sender.cpp          lock-free ring buffer → UDP threads (/beat and /vu separately)
│   ├── osc_receiver.cpp        OSC receive (/beat, /clockmode, /tap)
│   ├── osc_messages.cpp        serialisation (raw binary, no liblo)
│   └── pioneer_receiver.cpp    Pro DJ Link (virtual CDJ, 3 sockets)
├── config/
│   └── config_loader.cpp       key-value parser
└── util/
    └── logging.cpp             log levels, console

external/
└── BTrack/                     git submodule (adamstark/BTrack)
```

### Threading

```
JACK realtime thread (~2.9 ms budget at 128 samples / 44.1 kHz)
  ├── VU meter:  12× processMono (cheap)
  └── BPM audio: memcpy into a lock-free SPSC ring buffer

Beat thread (2 kHz polling)
  ├── ring buffer → BTrack (FFT + onset + tempo)
  ├── Synthclock (phase accumulator)
  ├── drain the TAP queue
  └── send /beat

VU thread (25 Hz)
  └── send the /vu bundle

OSC senders (one thread per target)
  └── SPSC ring buffer → non-blocking sendto()

Pioneer thread (mode 2 only)
  ├── poll() on 3 sockets
  ├── keep-alive every 1.5 s (virtual CDJ)
  └── parse beat/status packets
```

BTrack does **not** run in the JACK callback -- too expensive (a 256-point FFT plus
transcendentals per hop). JACK copies 512 bytes per callback into the ring buffer and nothing
more.

### Dependencies

| Library | Purpose | How it is pulled in |
|---------|-------|------------|
| [BTrack](https://github.com/adamstark/BTrack) | beat detection | git submodule (`external/BTrack`) |
| JACK | audio I/O | system (`libjack-dev`) |
| libsamplerate | resampling inside BTrack | system (`libsamplerate0-dev`) |

No liblo -- OSC goes over raw UDP sockets.

## Deployment

### Systemd user service

```bash
cp beat-analyzer.service ~/.config/systemd/user/
systemctl --user enable beat-analyzer
systemctl --user start beat-analyzer
```

### JACK connections

```bash
jack_lsp                                              # list ports
jack_connect system:capture_1 beat-analyzer:bpm_1     # BPM input
jack_connect system:capture_1 beat-analyzer:vu_1      # VU inputs
```

### Low latency (PipeWire)

```bash
pw-metadata -n settings 0 clock.force-quantum 128     # 128 frames = ~2.9 ms
```

## License

MIT
