#!/bin/bash

# Beat Analyzer Build Script for Debian
# Usage: ./build.sh [Release|Debug]

BUILD_TYPE=${1:-Release}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Beat Analyzer Build ==="
echo "Build Type: $BUILD_TYPE"
echo ""

# Check dependencies
echo "Checking dependencies..."
if ! command -v cmake &> /dev/null; then
    echo "ERROR: cmake not found"
    exit 1
fi

if ! pkg-config --exists jack; then
    echo "ERROR: JACK development files not found"
    echo "Install with: sudo apt-get install libjack-dev"
    exit 1
fi

# Create build directory
echo "Creating build directory..."
mkdir -p "$SCRIPT_DIR/build"
cd "$SCRIPT_DIR/build"

# A build directory configured for another checkout cannot be reused: CMake
# refuses it ("CMakeCache.txt directory ... is different"). That happened when
# the repository moved from ~/beat-analyzer to ~/a3-system/beat-analyzer.
# Only the cache is dropped; the binary stays until the build replaces it.
if [ -f CMakeCache.txt ]; then
    CACHED_SOURCE=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' CMakeCache.txt)
    if [ -n "$CACHED_SOURCE" ] && [ "$CACHED_SOURCE" != "$SCRIPT_DIR" ]; then
        echo "Build directory was configured for $CACHED_SOURCE — reconfiguring for $SCRIPT_DIR"
        rm -rf CMakeCache.txt CMakeFiles
    fi
fi

# Configure with CMake
echo "Configuring CMake..."
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
      -DBUILD_TESTS=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      ..

if [ $? -ne 0 ]; then
    echo "ERROR: CMake configuration failed"
    exit 1
fi

# Build
echo "Building..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "ERROR: Build failed"
    exit 1
fi

echo ""
echo "=== Build successful ==="
echo ""
echo "Next steps:"
echo "  1. Run tests: make test"
echo "  2. Install: sudo make install"
echo "  3. Run: beat-analyzer [config_file]"
