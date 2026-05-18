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
# The template uses set(COMPONENT_SRCS ...) + list(APPEND ...) style.
# We append our file unconditionally after the base SRCS list.
sed -i '/^set(COMPONENT_SRCS/,/)/ { /)/ a\list(APPEND COMPONENT_SRCS "espnow_bridge.c")
}' main/CMakeLists.txt

# Verify the patch took effect
if ! grep -q 'espnow_bridge.c' main/CMakeLists.txt; then
    echo "WARNING: CMakeLists.txt patch failed — adding fallback"
    echo 'list(APPEND COMPONENT_SRCS "espnow_bridge.c")' >> main/CMakeLists.txt
fi

# Enable peer data transfer + bump handler count + add esp_now dependency
cat >> sdkconfig.defaults << 'SDKEOF'
CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS=8
CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER=y
SDKEOF

# Auto-patch: call espnow_bridge_init() from the peer data example init hook.
# The slave template has example_peer_data_transfer.c which is compiled when
# CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER=y. We inject our init there.
# If that doesn't exist, we patch esp_hosted_coprocessor.c (the slave's main).
if [ -f main/example_peer_data_transfer.c ]; then
    # Add include at top (after the existing includes)
    sed -i '/#include.*slave_control/a #include "espnow_bridge.h"' main/example_peer_data_transfer.c
    # Add init call at end of the example init function
    sed -i '/ESP_LOGI.*peer_data_transfer.*ready\|return ESP_OK;/{
        /return ESP_OK;/i\    espnow_bridge_init();
        b
    }' main/example_peer_data_transfer.c
fi

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
