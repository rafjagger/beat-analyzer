#!/usr/bin/env bash
# The Debian package's recipe (packaging/) and what it lays out. It runs from
# ctest with the source tree as $1; it builds nothing and installs nothing.
#
# The package stands alone (maintainer, 2026-10-08: "people want to use
# beat-analyzer without Core"): nothing in it may name A3's units or homes.
set -uo pipefail

SRC=$(cd "${1:?usage: packaging_test.sh <source dir>}" && pwd)
failures=0
pass() { echo "  ✓ $1"; }
fail() { echo "  ✗ $1" >&2; failures=$((failures + 1)); }
check() { local what=$1; shift; if "$@"; then pass "$what"; else fail "$what"; fi; }

SCRATCH=$(mktemp -d)
trap 'rm -rf "$SCRATCH"' EXIT

lock_value() { sed -n "s/^$1=//p" "$SRC/packaging/btrack.lock"; }

# --- BTrack: the pin and the manifest follow the submodule -------------------

if git -C "$SRC" rev-parse --git-dir >/dev/null 2>&1; then
    gitlink=$(git -C "$SRC" ls-tree HEAD external/BTrack | awk '{print $3}')
    check "packaging/btrack.lock pins the submodule's commit ($gitlink)" \
        test "$(lock_value commit)" = "$gitlink"
else
    pass "not a git checkout: the pin is checked where the submodule is"
fi

if [ -f "$SRC/external/BTrack/src/BTrack.cpp" ]; then
    check "packaging/btrack.sha256 matches the submodule's files" \
        bash -c "cd '$SRC/external/BTrack' && sha256sum --quiet -c '$SRC/packaging/btrack.sha256'"
    used=$(cd "$SRC/external/BTrack" && sort -u <<<"$(printf '%s\n' src/BTrack.cpp src/BTrack.h \
        src/OnsetDetectionFunction.cpp src/OnsetDetectionFunction.h src/CircularBuffer.h \
        libs/kiss_fft130/kiss_fft.c libs/kiss_fft130/kiss_fft.h libs/kiss_fft130/_kiss_fft_guts.h)")
    listed=$(awk '{print $2}' "$SRC/packaging/btrack.sha256" | sort -u)
    check "the manifest lists every BTrack file the build compiles" \
        test -z "$(comm -23 <(echo "$used") <(echo "$listed"))"
fi

# --- The unit: generic, seeded, no A3 ----------------------------------------

UNIT="$SRC/beat-analyzer.service"
check "the unit starts /usr/bin/beat-analyzer" grep -qx 'ExecStart=/usr/bin/beat-analyzer' "$UNIT"
check "the unit seeds the user's config first, and a failed seed does not stop it" \
    grep -qx 'ExecStartPre=-/usr/lib/beat-analyzer/beat-analyzer-seed %E/beat-analyzer' "$UNIT"
check "the unit names no A3 unit, user or checkout (a3-core adds its drop-in)" \
    bash -c "! grep -Ev '^#' '$UNIT' | grep -Eq 'a3-|/home/|CPUAffinity'"

# --- The seed: copies once, never over ---------------------------------------

SEED="$SRC/tools/beat-analyzer-seed"
printf 'BPM_MIN=60\n' > "$SCRATCH/example"
"$SEED" "$SCRATCH/cfg" "$SCRATCH/example" >/dev/null 2>&1
check "the seed creates <dir>/beat-analyzer.env from the example" \
    cmp -s "$SCRATCH/example" "$SCRATCH/cfg/beat-analyzer.env"
printf 'BPM_MIN=90\n' > "$SCRATCH/cfg/beat-analyzer.env"
"$SEED" "$SCRATCH/cfg" "$SCRATCH/example" >/dev/null 2>&1
check "the seed never writes over the user's file" \
    grep -qx 'BPM_MIN=90' "$SCRATCH/cfg/beat-analyzer.env"
check "the seed makes conf.d/ for other packages' files" test -d "$SCRATCH/cfg/conf.d"

# --- The control file --------------------------------------------------------

CONTROL="$SRC/packaging/DEBIAN/control"
check "control: Package beat-analyzer" grep -qx 'Package: beat-analyzer' "$CONTROL"
check "control: the libraries come from dpkg-shlibdeps" grep -q '^Depends:.*\${shlibs:Depends}' "$CONTROL"
check "control: no dependency on a3-core" bash -c "! grep -E '^(Depends|Pre-Depends):' '$CONTROL' | grep -q a3-core"
for script in postinst prerm postrm; do
    check "DEBIAN/$script is there and runs sh -n" sh -n "$SRC/packaging/DEBIAN/$script"
done
check "postinst enables nothing (an audio client is the user's choice)" \
    bash -c "! grep -Ev '^\s*#' '$SRC/packaging/DEBIAN/postinst' | grep -q 'enable'"

# --- packaging/stage files: the layout ---------------------------------------

mkdir -p "$SCRATCH/build" "$SCRATCH/btrack/libs/kiss_fft130"
printf '\177ELF' > "$SCRATCH/build/beat-analyzer"
printf 'license\n' > "$SCRATCH/btrack/LICENSE.txt"
printf 'copying\n' > "$SCRATCH/btrack/libs/kiss_fft130/COPYING"
echo "$SCRATCH/btrack" > "$SCRATCH/build/btrack-dir"
"$SRC/packaging/stage" files "$SRC" "$SCRATCH/build" "$SCRATCH/stage" >/dev/null 2>&1
for path in usr/bin/beat-analyzer usr/lib/beat-analyzer/beat-analyzer-seed; do
    check "stage files: $path, executable" test -x "$SCRATCH/stage/$path"
done
for path in usr/lib/systemd/user/beat-analyzer.service \
            usr/share/beat-analyzer/beat-analyzer.env.example \
            usr/share/doc/beat-analyzer/copyright \
            usr/share/doc/beat-analyzer/BTrack-LICENSE.txt \
            usr/share/doc/beat-analyzer/kiss_fft-COPYING; do
    check "stage files: $path" test -f "$SCRATCH/stage/$path"
done
check "stage files: nothing outside /usr" \
    test "$(cd "$SCRATCH/stage" 2>/dev/null && ls)" = "usr"

# The example the package ships carries no address: that would be a second
# truth on a Core, and a guess anywhere else.
check "the shipped example sets no OSC target" \
    bash -c "! grep -Eq '^(OSC_HOST_|OSC_VU_)' '$SRC/.env.example'"

[ "$failures" -eq 0 ] || { echo "$failures packaging check(s) failed" >&2; exit 1; }
