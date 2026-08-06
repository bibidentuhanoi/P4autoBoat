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

# Auto-patch: call espnow_bridge_init() from the co-processor's own init.
#
# DO NOT hang this off example_peer_data_transfer_init(). That function is
# compiled (CMakeLists gates it on CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER)
# but its only caller in esp_hosted_coprocessor.c is guarded by a DIFFERENT,
# never-defined symbol:
#
#     #ifdef CONFIG_EXAMPLE_PEER_DATA_TRANSFER      <-- never defined
#             example_peer_data_transfer_init();
#     #endif
#
# so the call is preprocessed away, nothing references the example, and
# --gc-sections then discards it *and* espnow_bridge.c along with it. The build
# still succeeds and produces a healthy-looking binary with no ESP-NOW in it.
# (That is exactly what shipped on 2026-05-18 and why the bridge never ran.)
#
# Instead inject the call directly after esp_hosted_coprocessor_init(), which is
# unconditionally executed, so the bridge is always reachable and survives GC.
COPRO=main/esp_hosted_coprocessor.c
if [ ! -f "$COPRO" ]; then
    echo "ERROR: $COPRO not found — slave layout changed, patch needs updating" >&2
    exit 1
fi

grep -q 'espnow_bridge.h' "$COPRO" || \
    sed -i '0,/^#include/s//#include "espnow_bridge.h"\n#include/' "$COPRO"

if ! grep -q 'espnow_bridge_init' "$COPRO"; then
    sed -i 's/^\([[:space:]]*\)esp_hosted_coprocessor_init();/\1esp_hosted_coprocessor_init();\n\1espnow_bridge_init();/' "$COPRO"
fi

if ! grep -q 'espnow_bridge_init' "$COPRO"; then
    echo "ERROR: failed to inject espnow_bridge_init() into $COPRO" >&2
    exit 1
fi

idf.py build

# ---------------------------------------------------------------------------
# Verify the bridge actually SURVIVED THE LINK.
#
# A clean `idf.py build` is NOT proof: espnow_bridge.c can compile into
# libmain.a and still be discarded by --gc-sections if nothing reaches it, and
# the resulting binary looks completely healthy. Check real symbols in the ELF.
# esp_now_init is the strongest signal — espnow_bridge.c calls it, so it can
# only be present if the bridge is genuinely linked in.
# ---------------------------------------------------------------------------
ELF="$BUILD_DIR/slave/build/network_adapter.elf"
if [ ! -f "$ELF" ]; then
    echo "ERROR: expected ELF not found at $ELF" >&2
    exit 1
fi

missing=0
for sym in espnow_bridge_init esp_now_init esp_now_send esp_now_add_peer; do
    if [ "$(nm "$ELF" | grep -c "\b${sym}\b")" -eq 0 ]; then
        echo "ERROR: symbol '$sym' missing from the linked firmware" >&2
        missing=1
    fi
done

if [ "$missing" -ne 0 ]; then
    echo "" >&2
    echo "The ESP-NOW bridge did NOT link into network_adapter.bin." >&2
    echo "DO NOT FLASH THIS BINARY — the C6 would run without ESP-NOW and the" >&2
    echo "P4 would fail with 'Failed to send ESPNOW_INIT'." >&2
    exit 1
fi

echo ""
echo "=== Bridge link verified: ESP-NOW symbols present in firmware ==="

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
