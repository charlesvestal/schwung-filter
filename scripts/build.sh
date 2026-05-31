#!/usr/bin/env bash
# Build FILTER module for Move Anything (ARM64)
#
# Automatically uses Docker for cross-compilation if needed.
# Set CROSS_PREFIX to skip Docker (e.g., for native ARM builds).
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="move-anything-builder"

# Check if we need Docker
if [ -z "$CROSS_PREFIX" ] && [ ! -f "/.dockerenv" ]; then
    echo "=== FILTER Module Build (via Docker) ==="
    echo ""

    # Build Docker image if needed
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo "Building Docker image (first time only)..."
        docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
        echo ""
    fi

    # Run build inside container
    echo "Running build..."
    docker run --rm \
        -v "$REPO_ROOT:/build" \
        -u "$(id -u):$(id -g)" \
        -w /build \
        "$IMAGE_NAME" \
        ./scripts/build.sh

    echo ""
    echo "=== Done ==="
    exit 0
fi

# === Actual build (runs in Docker or with cross-compiler) ===
CROSS_PREFIX="${CROSS_PREFIX:-aarch64-linux-gnu-}"

cd "$REPO_ROOT"

echo "=== Building FILTER Module ==="
echo "Cross prefix: $CROSS_PREFIX"

# Create build directories
mkdir -p build
mkdir -p dist/filter

# Compile DSP plugin (with aggressive optimizations for CM4)
echo "Compiling DSP plugin..."
${CROSS_PREFIX}gcc -Ofast -shared -fPIC \
    -march=armv8-a -mtune=cortex-a72 \
    -fomit-frame-pointer -fno-stack-protector \
    -DNDEBUG \
    src/dsp/filter.c src/dsp/svf_core.c src/dsp/smoother.c src/dsp/modulation.c \
    -o build/filter.so \
    -Isrc/dsp \
    -lm

# Copy files to dist (use cat to avoid ExtFS deallocation issues with Docker)
echo "Packaging..."
cat src/module.json > dist/filter/module.json
[ -f src/help.json ] && cat src/help.json > dist/filter/help.json
cat build/filter.so > dist/filter/filter.so
chmod +x dist/filter/filter.so

# Create tarball for release
cd dist
tar -czvf filter-module.tar.gz filter/
cd ..

echo ""
echo "=== Build Complete ==="
echo "Output: dist/filter/"
echo "Tarball: dist/filter-module.tar.gz"
echo ""
echo "To install on Move:"
echo "  ./scripts/install.sh"
