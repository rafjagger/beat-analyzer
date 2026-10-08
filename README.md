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

The binary is `build/beat-analyzer`.

## Debian package

`beat-analyzer` is its own package and needs nothing else of A³: any amd64 Debian with a JACK
server (PipeWire's included).

```bash
systemctl --user enable --now beat-analyzer   # the package enables nothing by itself
```

It is built from `packaging/` (`packaging/stage`, called by a3-system's `installer/package.py`,
version `03.0+N` from the newest `v*` tag). `git archive` carries no submodule, so the recipe
fetches BTrack at the commit in `packaging/btrack.lock` and checks every file it compiles against
`packaging/btrack.sha256`; `tests/packaging_test.sh` keeps both equal to the submodule. On an A³
Core the a3-core package adds a drop-in to the unit (ordering, CPU pinning) and its OSC targets.

## Configuration

Read in this order, a later file winning key by key:

1. `--config FILE`, else the first that exists of `./.env`, `../.env`,
   `~/.config/beat-analyzer/beat-analyzer.env`, `./.env.example` (a checkout runs from
   `build/.env` as before, even once the package has made the user's file; the package's unit
   runs in `/usr/share/beat-analyzer`, where there is no `.env`);
2. every `~/.config/beat-analyzer/conf.d/*.env`, sorted by name, for a checkout run too. A file
   there that cannot be read is logged and skipped; an unreadable `--config FILE` stops the start.

The package's unit makes `beat-analyzer.env` from the template (`.env.example`, every key with its
default) on the first start and never writes over it. In a checkout:

```bash
cp .env.example build/.env
```

Without an `OSC_HOST_<name>=host:port` the analyzer runs and sends nothing. On an A³ Core the
targets, ports and addresses are not written by hand: the a3-core package renders them into
`~/.config/beat-analyzer/conf.d/50-a3-osc.env`.

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
