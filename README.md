# Beat Analyzer

**Eigenständige Echtzeit-Anwendung für Beat-Analyse mit OSC-Output**

## 🎯 Features

- **Audio Input**: JACK/PipeWire
- **Beat Detection**: Beat / Beatclock
- **VU-Meter**: RMS und Peak Level pro Stereo-Track
- **OSC Output**: Beat Clock + VU-Meter für 4 Tracks
- **Konfiguration**: Einfache `.env` Datei
- **Platform**: Debian Linux

## 📋 Voraussetzungen

```bash
# Debian/Ubuntu
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libjack-dev \
    liblo-dev
```

## 🔧 Build

```bash
cd beat-analyzer
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 🚀 Verwendung

```bash
# Mit Standard-Konfiguration
./beat-analyzer

# Mit eigener Konfiguration
./beat-analyzer /path/to/config.yaml

# Hilfe anzeigen
./beat-analyzer --help
```

### JACK-Verbindung

```bash
# Verfügbare Ports anzeigen
jack_lsp

# Audio-Quelle verbinden
jack_connect system:capture_1 beat-analyzer:input_0
jack_connect system:capture_2 beat-analyzer:input_1
```

## 📊 Architektur

```
┌─────────────────────────────────────────────┐
│              BEAT ANALYZER                   │
├─────────────────────────────────────────────┤
│                                              │
│   JACK Client (8 Kanäle)                    │
│           ↓                                  │
│   Onset Detector (Spectral Difference)      │
│           ↓                                  │
│   Tempo Tracker (Autocorrelation + Viterbi) │
│           ↓                                  │
│   Beat Tracker (Grid + Phase)               │
│           ↓                                  │
│   OSC Sender (Beat Clock)                   │
│           ↓                                  │
│   Pro Tools (UDP Port 9000)                 │
│                                              │
└─────────────────────────────────────────────┘
```

## 🔬 Beat Detection Algorithmus

Die Beat-Erkennung basiert auf bewährten DSP-Konzepten:

### 1. Onset Detection
- **FFT-basierte Spektralanalyse** (Cooley-Tukey)
- **Complex Spectral Difference** für Transientenerkennung
- **Hanning-Fenster** für saubere Spektren

### 2. Tempo Tracking
- **Autocorrelation** der Detection Function
- **Comb Filter Bank** für Periodizitätserkennung
- **Rayleigh-Gewichtung** für BPM-Präferenz

### 3. Beat Tracking
- **Viterbi Decoding** für optimalen Beat-Pfad
- **Gaussian Transition Matrix** für Tempo-Stabilität
- **Echtzeit-Extrapolation** für Beat Clock

## 📡 OSC Protokoll

### Beat Clock Messages

```
Address: /beatclock/1 - /beatclock/4
Arguments:
  [0] frame_pos     (int64)       Absolute Frame-Position
  [1] bpm           (float)       BPM
  [2] beat_number   (int 0-3)     Beat im Takt (4/4)
  [3] beat_strength (float 0-1)   Gewichtung
```

### VU-Meter Messages

```
Address: /rms/1 - /rms/4
Arguments:
  [0] level_db      (float)       RMS Level in dB (0 = max)

Address: /peak/1 - /peak/4  
Arguments:
  [0] level_db      (float)       Peak Level in dB (0 = max)
```

**Beispiel:**
```
/beatclock/1 44100 120.0 2 0.95
/rms/1 -12.5
/peak/1 -6.2
```

## ⚙️ Konfiguration

Kopiere `.env.example` nach `.env` und passe die Werte an:

```bash
cp .env.example .env
```

`.env`:

```bash
# OSC Ziel
OSC_HOST=127.0.0.1
OSC_PORT=9000

# JACK Client Name
JACK_CLIENT_NAME=beat-analyzer

# BPM Range
BPM_MIN=60
BPM_MAX=200

# Log Level (0=DEBUG, 1=INFO, 2=WARN, 3=ERROR)
LOG_LEVEL=1
```

### Niedrige Latenz (Buffer Size)

Für minimale Latenz die PipeWire Buffer-Size anpassen:

```bash
# Temporär (64 Frames = ~1.5ms @ 44.1kHz)
pw-metadata -n settings 0 clock.force-quantum 64

# Permanent in ~/.config/pipewire/pipewire.conf:
context.properties = {
    default.clock.quantum = 64
}
```

## 📁 Projektstruktur

```
beat-analyzer/
├── CMakeLists.txt          # Build-Konfiguration
├── build.sh                # Build-Script
├── config/
│   └── config.yaml         # Konfiguration
├── include/
│   ├── audio/              # Audio I/O
│   ├── analysis/           # Beat Detection
│   ├── osc/                # OSC Kommunikation
│   ├── config/             # Konfiguration
│   └── util/               # Utilities
├── src/                    # Implementierung
└── tests/                  # Unit Tests
```

## 🧪 Tests

```bash
cd build
make test
```

## 📈 Performance

- **Latenz**: ~1.5ms @ 44.1kHz (64 Samples)
- **CPU**: ~5-10% (single core)
- **RAM**: ~20MB

## 🔧 Technische Details

### Keine externen DSP-Abhängigkeiten

Der Beat-Analyzer enthält **eigene Implementierungen** für:

- **FFT** (Cooley-Tukey Radix-2)
- **Fenster-Funktionen** (Hanning, Hamming, Blackman)
- **Onset Detection** (Complex Spectral Difference)
- **Autocorrelation**
- **Comb Filter Bank**
- **Viterbi Decoding**
- **Butterworth Filter**

### Einzige externe Abhängigkeiten

- **JACK** - Audio I/O
- **liblo** - OSC Kommunikation (optional)
- **Standard C++17**

## 📝 Lizenz

MIT License

## 🙏 Credits

Die Algorithmus-**Konzepte** basieren auf wissenschaftlichen Arbeiten:
- Queen Mary University - Tempo Tracking Algorithmen
- Davies & Plumbley - "A Spectral Difference Approach"

Die **Implementierung** ist komplett eigenständig und neu geschrieben.
