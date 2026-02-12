# Beat Analyzer

Echtzeit Beat-Analyse und VU-Metering über JACK/PipeWire mit OSC-Output.

## Überblick

```
JACK Audio (bpm_1, vu_1..vu_12)
       │
       ├── BTrack (FFT + Onset + Tempo)  →  Synthclock  →  /beat iif
       ├── VU-Meter (RMS + Peak)                        →  /vu/0../vu/11 ff
       │
       ├── OSC Empfang (Port 7775)       →  /clockmode, /tap, /beat
       └── Pioneer DJ Link (50000-50002) →  Master-Beat direkt vom Mixer
```

### Clock-Modi

| Modus | Quelle | Beschreibung |
|-------|--------|--------------|
| 0 — a3motion | OSC Port 7775 | `/beat` empfangen, an alle außer motion weiterleiten |
| 1 — intern | BTrack + Synthclock | Eigene Beat-Erkennung aus Audio, an alle senden |
| 2 — pioneer | Pro DJ Link | Master-Beat vom Pioneer-Netzwerk, an alle senden |

Umschalten via OSC: `/clockmode i` (0, 1 oder 2) an Port 7775.

## Build

### Voraussetzungen

```bash
sudo apt-get install -y \
    build-essential cmake pkg-config \
    jackd2 libjack-jackd2-dev \
    libsamplerate0-dev
```

### Kompilieren

```bash
./build.sh            # oder: mkdir build && cd build && cmake .. && make -j$(nproc)
```

Binary: `build/beat-analyzer`

### Tests

```bash
cd build && ctest
```

## Konfiguration

Alle Einstellungen über `.env` im Working Directory (typisch `build/.env`).
Vorlage: `.env.example` im Projekt-Root.

```bash
cp .env.example build/.env
```

### Wichtigste Variablen

```bash
# Audio
JACK_CLIENT_NAME=beat-analyzer
NUM_BPM_CHANNELS=1              # JACK Ports: bpm_1 (Beat-Erkennung)
NUM_VU_CHANNELS=12              # JACK Ports: vu_1..vu_12 (Peakmeter)
BPM_MIN=60
BPM_MAX=140

# OSC Ziele (beliebig viele, Format: Name=host:port)
OSC_HOST_radla=192.168.43.96:9000
OSC_HOST_mixer=192.168.43.55:7771
OSC_HOST_motion=192.168.43.54:7771

# OSC Empfang
OSC_PORT_A3MOTION=7775          # /beat, /clockmode, /tap

# Pioneer Pro DJ Link
PIONEER_DEVICE_NUM=7            # Virtual CDJ Nummer (Standard: 7)

# VU-Meter
VU_RMS_ATTACK=0.8               # 0.0-1.0
VU_RMS_RELEASE=0.2              # 0.0-1.0
VU_PEAK_FALLOFF=20.0            # dB/s
OSC_SEND_RATE=25                # Hz (1-100)

# Debug
DEBUG_BPM_CONSOLE=1
DEBUG_VU_CONSOLE=0
LOG_LEVEL=1                     # 0=DEBUG 1=INFO 2=WARN 3=ERROR
```

Alle Variablen mit Defaults: siehe `.env.example`.

## OSC Protokoll

### Ausgehend

| Adresse | Typ | Inhalt |
|---------|-----|--------|
| `/beat` | `iif` | beat (1-4), bar, bpm |
| `/vu/0` .. `/vu/11` | `ff` | peak, rms (linear 0.0-1.0) |

VU wird als OSC Bundle gesendet (1 UDP-Paket für alle Kanäle).

### Eingehend (Port 7775)

| Adresse | Typ | Inhalt |
|---------|-----|--------|
| `/beat` | `iif` | beat (1-4), bar, bpm |
| `/clockmode` | `i` | 0=a3motion, 1=intern, 2=pioneer |
| `/tap` | `i` | Beat-Nummer (setzt Phase) |

### Pioneer Pro DJ Link (Modus 2)

Lauscht direkt auf dem Pioneer-Netzwerk (UDP Ports 50000/50001/50002).
Registriert sich als Virtual CDJ und empfängt nur Beats vom **Tempo-Master**.
Kein OSC nötig — reines UDP nach Pro DJ Link Protokoll.

## Architektur

```
src/
├── main.cpp                    App-Klasse, JACK-Callback, Beat/VU-Threads
├── audio/
│   ├── jack_client.cpp         JACK I/O (BPM + VU Ports)
│   └── audio_buffer.cpp        Circular Buffer
├── analysis/
│   ├── vu_meter.cpp            RMS + Peak (läuft im JACK-Callback)
│   ├── grid_calculator.cpp     Grid-Analyse aus Realbeats
│   ├── beat_detection.cpp      Onset Detection (nur Tests)
│   └── beat_tracker.cpp        Beat Tracking (nur Tests)
├── osc/
│   ├── osc_sender.cpp          Lock-free Ringbuffer → UDP Threads
│   ├── osc_receiver.cpp        OSC Empfang (/beat, /clockmode, /tap)
│   ├── osc_messages.cpp        Serialisierung (raw binary, kein liblo)
│   └── pioneer_receiver.cpp    Pro DJ Link (Virtual CDJ, 3 Sockets)
├── config/
│   └── config_loader.cpp       Key-Value Parser
└── util/
    └── logging.cpp             Log-Levels, Konsole

external/
└── BTrack/                     Git Submodule (adamstark/BTrack)
```

### Threading

```
JACK Realtime Thread (~2.9ms Budget bei 128 Samples/44.1kHz)
  ├── VU-Meter:  12× processMono (leichtgewichtig)
  └── BPM Audio: memcpy in lock-free SPSC Ringbuffer

Beat-Thread (2kHz Polling)
  ├── Ringbuffer → BTrack (FFT + Onset + Tempo)
  ├── Synthclock (Phase-Akkumulator)
  ├── TAP-Queue verarbeiten
  └── /beat senden

VU-Thread (25Hz)
  └── /vu Bundle senden

OSC-Sender (1 Thread pro Ziel)
  └── SPSC Ringbuffer → non-blocking sendto()

Pioneer-Thread (falls Modus 2)
  ├── poll() auf 3 Sockets
  ├── Keep-Alive alle 1.5s (Virtual CDJ)
  └── Beat/Status Pakete parsen
```

BTrack läuft **nicht** im JACK-Callback — zu teuer (256-pt FFT + Transzendente pro Hop).
JACK kopiert nur 512 Bytes pro Callback in den Ringbuffer.

### Abhängigkeiten

| Library | Zweck | Einbindung |
|---------|-------|------------|
| [BTrack](https://github.com/adamstark/BTrack) | Beat Detection | Git Submodule (`external/BTrack`) |
| JACK | Audio I/O | System (`libjack-dev`) |
| libsamplerate | Resampling in BTrack | System (`libsamplerate0-dev`) |

Kein liblo — OSC über rohe UDP Sockets.

## Deployment

### Systemd User Service

```bash
cp beat-analyzer.service ~/.config/systemd/user/
systemctl --user enable beat-analyzer
systemctl --user start beat-analyzer
```

### JACK Verbindung

```bash
jack_lsp                                              # Ports anzeigen
jack_connect system:capture_1 beat-analyzer:bpm_1     # BPM-Eingang
jack_connect system:capture_1 beat-analyzer:vu_1      # VU-Eingänge
```

### Niedrige Latenz (PipeWire)

```bash
pw-metadata -n settings 0 clock.force-quantum 128     # 128 Frames = ~2.9ms
```

## Lizenz

MIT
