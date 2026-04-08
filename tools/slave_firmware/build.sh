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
