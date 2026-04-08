# ESP32-C6 SDIO Flasher

Flash the ESP32-C6 co-processor firmware from the ESP32-P4 host over the existing SDIO bus using [esp-serial-flasher](https://github.com/espressif/esp-serial-flasher).

This bypasses the esp-hosted OTA path (which requires protocol version compatibility) and writes directly to the C6's flash via the SDIO download mode — same as using an ESP-Prog, but no extra hardware needed.

## When to Use

- Upgrading the C6 esp-hosted slave firmware to a new version
- Recovering from a bricked C6 (bad OTA, corrupted flash)
- Changing the C6 partition table or bootloader (not possible via OTA)

## Hardware Setup

The ESP32-P4-Function-EV-Board has the SDIO bus and reset pin already wired:

| Signal   | ESP32-P4 GPIO | ESP32-C6 Pin |
|----------|---------------|--------------|
| SDIO_D0  | GPIO14        | D0           |
| SDIO_D1  | GPIO15        | D1           |
| SDIO_D2  | GPIO16        | D2           |
| SDIO_D3  | GPIO17        | D3           |
| SDIO_CLK | GPIO18        | CLK          |
| SDIO_CMD | GPIO19        | CMD          |
| EN       | GPIO54        | CHIP_PU      |

**Boot pin (C6 GPIO9) is NOT routed on this board.** You must bridge it to GND manually with a tweezer to put the C6 into download mode.

## Step-by-Step

### 1. Build the C6 slave firmware

```bash
cd /tmp
idf.py create-project-from-example "espressif/esp_hosted=2.12.3:slave"
cd slave
idf.py set-target esp32c6
echo "CONFIG_ESP_SDIO_HOST_INTERFACE=y" >> sdkconfig.defaults
idf.py build
```

### 2. Copy binaries to target-firmware/

```bash
cp build/bootloader/bootloader.bin       <project>/tools/sdio_flasher/target-firmware/
cp build/partition_table/partition-table.bin <project>/tools/sdio_flasher/target-firmware/
cp build/network_adapter.bin             <project>/tools/sdio_flasher/target-firmware/app.bin
```

### 3. Build the flasher

```bash
cd <project>/tools/sdio_flasher
idf.py set-target esp32p4
idf.py build
```

### 4. Flash the C6

1. **Hold C6 GPIO9 to GND** with a tweezer (locate the pad on the C6 module)
2. Flash the P4 with the flasher app:
   ```bash
   idf.py flash monitor
   ```
3. Wait for the log to show:
   ```
   Connected to target
   Loading bootloader...   Flash verified
   Loading partition table... Flash verified
   Loading app...          Flash verified
   Done!
   ```
4. **Release the tweezer**
5. Reset the board

### 5. Flash the boat firmware back

```bash
cd <project>
idf.py fullclean
idf.py set-target esp32p4
idf.py build flash monitor
```

> **Important:** Run `fullclean` after using the flasher — it uses a different sdkconfig/partition table that can contaminate the boat project's build cache.

## Pin Mapping (main/main.c)

If your board revision differs, update the `esp32_sdio_port_t` struct in `main/main.c`:

```c
.reset_pin   = GPIO_NUM_54,   // P4 → C6 CHIP_PU
.boot_pin    = GPIO_NUM_4,    // dummy (tweezer C6 GPIO9 to GND)
.sdio_d0_pin = GPIO_NUM_14,
.sdio_d1_pin = GPIO_NUM_15,
.sdio_d2_pin = GPIO_NUM_16,
.sdio_d3_pin = GPIO_NUM_17,
.sdio_clk_pin = GPIO_NUM_18,
.sdio_cmd_pin = GPIO_NUM_19,
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `SDIO initialization failed` | Check SDIO wiring, ensure no other app is using the SDIO bus |
| `Failed to connect` | C6 GPIO9 not held low — check tweezer contact |
| `flash_write failed` | Try reducing `SDMMC_FREQ_DEFAULT` to `SDMMC_FREQ_PROBING` in main.c |
| P4 boot loops after flashing | Run `idf.py fullclean` in boat project, then rebuild + flash |
