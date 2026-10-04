# Beat Analyzer

The beat clock and the level meters of the [A³ Audio](https://github.com/a3-audio/a3-system)
system. It runs on the A³ Core machine, listens on JACK and sends `/beat` and `/vu/N` over OSC.
Its beat comes from A³ Motion, from its own analysis of the music (BTrack), or from a
Pioneer Pro DJ Link tempo master.

**Documentation: [Beat Analyzer](https://a3-audio.github.io/a3-doc/user/beat-analyzer.html)**
(clock modes, meters, settings, troubleshooting). Its JACK inputs are on the
[Patchbay page](https://a3-audio.github.io/a3-doc/ressources/patchbay.html), its messages in the
[OSC reference](https://a3-audio.github.io/a3-doc/ressources/osc.html#osc-beat-analyzer).
Everything else: https://a3-audio.github.io/a3-doc/

## Build and test

Needs `build-essential`, `cmake`, `pkg-config`, `libjack-jackd2-dev` and `libsamplerate0-dev`.
BTrack is a submodule, patched at build time from `patches/`.

```bash
git submodule update --init
./build.sh            # Release; ./build.sh Debug for a debug build
cd build && ctest
```

The binary is `build/beat-analyzer`. On the A³ Core machine the a3-core package builds it and
installs `beat-analyzer.service`, see
[the user-side installer](https://a3-audio.github.io/a3-doc/configuration/core.html#core-user-install).

## Configuration

Settings are read from `build/.env`. Start from the template, which lists every key with its
default:

```bash
cp .env.example build/.env
```

OSC targets, ports and addresses are not written by hand: the a3-core package renders them into
a block at the end of `build/.env`.

## Design rule

BTrack never runs in the JACK callback. The callback copies the audio into a lock-free ring
buffer, and a separate thread does the beat tracking and sends `/beat`.

## License

MIT
