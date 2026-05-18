#!/bin/bash
# Build the ESP32-C6 esp-hosted slave firmware (network_adapter)
# Output: build/network_adapter.bin + bootloader + partition table
#
# Usage:
#   cd tools/slave_firmware
#   ./build.sh
#
# The built binaries can then be copied to ../sdio_flasher/target-firmware/

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_slave"
VERSION="${1:-2.12.3}"

echo "=== Building ESP-Hosted slave firmware v${VERSION} for ESP32-C6 ==="

# Create temp project
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

idf.py create-project-from-example "espressif/esp_hosted=${VERSION}:slave"
cd slave

# Configure for C6 with SDIO
idf.py set-target esp32c6
echo "CONFIG_ESP_SDIO_HOST_INTERFACE=y" >> sdkconfig.defaults

# ── ESP-NOW bridge ──────────────────────────────────────────────────────────
# Copy bridge sources into the generated slave project
cp "$SCRIPT_DIR/espnow_bridge.c" main/espnow_bridge.c
cp "$SCRIPT_DIR/espnow_bridge.h" main/espnow_bridge.h

# Patch main/CMakeLists.txt to include espnow_bridge.c in SRCS.
# WARNING: This sed pattern targets the idf_component_register(SRCS form produced
# by create-project-from-example.  If the generated template changes its
# CMakeLists.txt layout this substitution may silently fail.  Verify by
# inspecting main/CMakeLists.txt after patching.
sed -i 's/idf_component_register(SRCS/idf_component_register(SRCS "espnow_bridge.c"/' main/CMakeLists.txt

# Enable ESP-NOW custom-message support
echo "CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS=8" >> sdkconfig.defaults

# NOTE: espnow_bridge_init() must be called once at startup from the slave
# application.  The generated slave template exposes an init hook (often in
# app_main() inside main/app_main.c or similar).  Add the call there after
# the hosted slave stack is initialised, e.g.:
#
#   #include "espnow_bridge.h"
#   ...
#   espnow_bridge_init();
#
# The exact file and call-site depend on the esp-hosted version being built.
# ────────────────────────────────────────────────────────────────────────────

idf.py build

echo ""
echo "=== Build complete ==="
echo "Binaries:"
echo "  bootloader:      $BUILD_DIR/slave/build/bootloader/bootloader.bin"
echo "  partition-table:  $BUILD_DIR/slave/build/partition_table/partition-table.bin"
echo "  app:              $BUILD_DIR/slave/build/network_adapter.bin"
echo ""
echo "To copy to SDIO flasher:"
echo "  cp build/bootloader/bootloader.bin       $SCRIPT_DIR/../sdio_flasher/target-firmware/"
echo "  cp build/partition_table/partition-table.bin $SCRIPT_DIR/../sdio_flasher/target-firmware/"
echo "  cp build/network_adapter.bin             $SCRIPT_DIR/../sdio_flasher/target-firmware/app.bin"
