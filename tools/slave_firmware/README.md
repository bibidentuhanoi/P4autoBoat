# ESP32-C6 Slave Firmware Builder

Builds the esp-hosted `network_adapter` firmware for the ESP32-C6 co-processor.

## Usage

```bash
cd tools/slave_firmware
./build.sh           # builds the P4-paired release (v2.12.12)
./build.sh 2.12.12   # or specify a supported version
```

## Output

Binaries are in `build_slave/slave/build/`:
- `bootloader/bootloader.bin`
- `partition_table/partition-table.bin`
- `network_adapter.bin`

## Next Step

Copy binaries to the SDIO flasher and flash the C6:

```bash
cp build_slave/slave/build/bootloader/bootloader.bin       ../sdio_flasher/target-firmware/
cp build_slave/slave/build/partition_table/partition-table.bin ../sdio_flasher/target-firmware/
cp build_slave/slave/build/network_adapter.bin             ../sdio_flasher/target-firmware/app.bin
```

Then follow the instructions in `../sdio_flasher/README.md`.
