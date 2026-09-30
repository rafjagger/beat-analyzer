# Beat Analyzer

Real-time beat analysis and VU metering over JACK/PipeWire with OSC output.

## Overview

```
JACK Audio (bpm_1, vu_in1_pre..vu_free70)
       │
       ├── BTrack (FFT + onset + tempo)  →  Synthclock  →  /beat iif
       ├── VU meter (RMS + peak)                        →  /vu/1../vu/40 ff
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
NUM_VU_CHANNELS=40              # JACK ports: vu_in1_pre..vu_free70 (REAPER VU outs 31-70)
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
| `/vu/1` .. `/vu/40` | `ff` | peak, rms (linear 0.0-1.0) |

VU is sent as one OSC bundle per block of ten -- inputs, Main, Booth, stereo, four UDP packets
of 296 bytes for 40 channels -- so each block arrives as one consistent picture. A bundle and a slot in the sender's queue are 512 bytes; until 2026-09-30 one
bundle took every channel, and everything past the 17th was silently cut off.

#### JACK port → OSC index

The VU inputs are named after what REAPER sends them: its outputs **31–70**, one meter each, in
blocks of ten (the A³ Core manual has the whole channel map). **`/vu/N` is VU channel N, fed
from REAPER out 30 + N** -- the OSC counts from 1, like the map (until 2026-09-30 it counted from
0). The names and addresses live in `src/audio/vu_ports.cpp`.

| REAPER out | JACK port | OSC |
|---|---|---|
| 31–34 | `vu_in1_pre` … `vu_in4_pre` -- channel inputs, pre-fader, post-FX | `/vu/1` … `/vu/4` |
| 35–38 | `vu_in1_post` … `vu_in4_post` -- channel inputs, post-fader | `/vu/5` … `/vu/8` |
| 39–40 | `vu_free39`, `vu_free40` | `/vu/9`, `/vu/10` |
| 41 | `vu_main_sub` | `/vu/11` |
| 42–50 | `vu_main_top1` … `vu_main_top9` | `/vu/12` … `/vu/20` |
| 51 | `vu_booth_sub` | `/vu/21` |
| 52–60 | `vu_booth_top1` … `vu_booth_top9` | `/vu/22` … `/vu/30` |
| 61–62 | `vu_phones_L`, `vu_phones_R` | `/vu/31`, `/vu/32` |
| 63–64 | `vu_rec_L`, `vu_rec_R` | `/vu/33`, `/vu/34` |
| 65–66 | `vu_aux_L`, `vu_aux_R` | `/vu/35`, `/vu/36` |
| 67–70 | `vu_free67` … `vu_free70` | `/vu/37` … `/vu/40` |

`NUM_VU_CHANNELS=40` opens all of them. More than 40 (up to 64) are named `vu_41` and on;
the count is held to 64, the size of the meter arrays.

#### What reads them

The beat-analyzer attaches no meaning to a channel beyond its name: it reports one value per
input, in port order. Until 2026-09-30 A³ Motion read `/vu/0..3` as the channel inputs,
`/vu/4` as the subwoofer (sphere glow) and `/vu/5..8` as four speakers, and the A³ Mixer
`/vu/0..3` as its input meters and `/vu/4..11` as its eight output meters. **Under the map above
those positions mean something else** (`/vu/4` is channel 4 pre-fader, not the subwoofer); both
devices have to be moved to the new indices.

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
jack_connect REAPER:out31 beat-analyzer:vu_in1_pre   # VU inputs
```

### Low latency (PipeWire)

```bash
pw-metadata -n settings 0 clock.force-quantum 128     # 128 frames = ~2.9 ms
```

## License

MIT
