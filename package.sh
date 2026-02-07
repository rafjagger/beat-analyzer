#!/bin/bash

# Debian Packaging Script
# Erstellt .deb Package für Beat Analyzer

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION="1.0.0"
RELEASE="1"

echo "=== Building Debian Package ==="
echo "Version: $VERSION-$RELEASE"
echo ""

# Create debian package structure
mkdir -p beat-analyzer-$VERSION/DEBIAN
mkdir -p beat-analyzer-$VERSION/usr/bin
mkdir -p beat-analyzer-$VERSION/usr/lib
mkdir -p beat-analyzer-$VERSION/etc/beat-analyzer
mkdir -p beat-analyzer-$VERSION/var/log

# Copy binary
if [ ! -f "$SCRIPT_DIR/build/beat-analyzer" ]; then
    echo "ERROR: Binary not found at build/beat-analyzer"
    echo "Please build first: ./build.sh Release"
    exit 1
fi

cp "$SCRIPT_DIR/build/beat-analyzer" \
   beat-analyzer-$VERSION/usr/bin/

strip beat-analyzer-$VERSION/usr/bin/beat-analyzer

# Copy config
cp "$SCRIPT_DIR/config/config.yaml" \
   beat-analyzer-$VERSION/etc/beat-analyzer/

# Create DEBIAN/control
cat > beat-analyzer-$VERSION/DEBIAN/control << EOF
Package: beat-analyzer
Version: $VERSION-$RELEASE
Architecture: amd64
Maintainer: Beat Analyzer Development <dev@beat-analyzer.local>
Depends: libc6 (>= 2.29), libjack0
Description: Real-time beat analysis with OSC output
 Beat Analyzer ist eine Echtzeit-Anwendung für Beatanalyse mit OSC-Output.
 Sie nutzt die Queen Mary DSP Library aus Mixxx für hochpräzise Beat-Erkennung
 und synchronisiert mit Pro Tools über OSC (Open Sound Control).
Homepage: https://beat-analyzer.local
