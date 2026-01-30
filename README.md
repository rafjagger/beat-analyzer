# Beat Analyzer

**Eigenständige Echtzeit-Anwendung für Beat-Analyse mit OSC-Output**

## 🎯 Features

- **Audio Input**: JACK/PipeWire (4x Stereo = 8 Kanäle)
- **Beat Detection**: Eigene Implementierung (keine externen Abhängigkeiten)
- **OSC Output**: Beat Clock für 4 Tracks (Pro Tools kompatibel)
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

### Beat Clock Message

```
Address: /beatclock
Arguments:
  [0] track_id      (int 0-3)     Track-Nummer
  [1] frame_pos     (int64)       Absolute Frame-Position
  [2] bpm           (float)       BPM
  [3] beat_number   (int 0-3)     Beat im Takt (4/4)
  [4] beat_strength (float 0-1)   Gewichtung
```

**Beispiel:**
```
/beatclock 0 44100 120.0 2 0.95
```

## ⚙️ Konfiguration

`config/config.yaml`:

```yaml
[audio]
sample_rate: 44100
frames_per_buffer: 512
channels: 8

[jack]
client_name: beat-analyzer
auto_connect: true

[analysis]
beat_detection: true
bpm_range_min: 60
bpm_range_max: 200

[osc]
enabled: true
host: 127.0.0.1
port: 9000

[logging]
level: 1
console: true
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

- **Latenz**: ~23ms @ 44.1kHz (512 Samples)
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
