#!/bin/bash
# Install FILTER module to Move
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

if [ ! -d "dist/filter" ]; then
    echo "Error: dist/filter not found. Run ./scripts/build.sh first."
    exit 1
fi

echo "=== Installing FILTER Module ==="

# Deploy to Move - audio_fx subdirectory
echo "Copying module to Move..."
ssh ableton@move.local "mkdir -p /data/UserData/schwung/modules/audio_fx/filter"
scp -r dist/filter/* ableton@move.local:/data/UserData/schwung/modules/audio_fx/filter/

# Install chain presets if they exist
if [ -d "src/patches" ]; then
    echo "Installing chain presets..."
    ssh ableton@move.local "mkdir -p /data/UserData/schwung/patches"
    scp src/patches/*.json ableton@move.local:/data/UserData/schwung/patches/
fi

# Set permissions so Module Store can update later
echo "Setting permissions..."
ssh ableton@move.local "chmod -R a+rw /data/UserData/schwung/modules/audio_fx/filter"

echo ""
echo "=== Install Complete ==="
echo "Module installed to: /data/UserData/schwung/modules/audio_fx/filter/"
echo ""
echo "Restart Move Anything to load the new module."
