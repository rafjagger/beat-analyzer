# Beat Analyzer

The beat clock and the level meters of the [A³ Audio](https://github.com/a3-audio/a3-system)
system. It runs on the A³ Core machine, listens on JACK and sends `/beat` and `/vu/N` over OSC.
Its beat comes from A³ Motion, from its own analysis of the music (BTrack), or from
the tempo master of a Pro DJ Link network.

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

## Pro DJ Link

Pro DJ Link and rekordbox are trademarks of AlphaTheta Corporation; Pioneer DJ is a trademark
of Pioneer Corporation; CDJ is a product name of theirs. A³ is not affiliated with, endorsed or
certified by AlphaTheta or Pioneer. The beat-analyzer's Pro DJ Link support is an independent
implementation for interoperability, built from public documentation of the protocol:
Deep Symmetry's [DJ Link analysis](https://djl-analysis.deepsymmetry.org/) and
[prolink-connect](https://github.com/EvanPurkhiser/prolink-connect).

Using it on a network you do not run is at your own risk: ask the venue before joining their
DJ Link network. See [Trademarks and Pro DJ Link](https://a3-audio.github.io/a3-doc/ressources/trademarks.html).

## License

GPL-3.0-or-later. REUSE-compliant: the licenses are in `LICENSES/`, which file has
which is in `.reuse/dep5`. BTrack, linked in as the `external/BTrack` submodule, is
GPL-3.0-or-later as well.
