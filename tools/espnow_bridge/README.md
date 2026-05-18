# ESP-NOW ↔ USB CDC Bridge (ESP32-S3)

Bridges ESP-NOW radio to USB CDC serial for laptop communication.

## Build

```bash
cd tools/espnow_bridge
idf.py set-target esp32s3
idf.py build
idf.py flash
```

## Configuration

Edit `main/main.c`:
- `ESPNOW_CHANNEL` — must match P4 CONFIG_ESPNOW_CHANNEL
- Peer MAC — set to your C6's STA MAC for P2P, or leave broadcast

## Usage

After flashing, S3 appears as `/dev/ttyACM0`.
Run: `python visualize.py --serial /dev/ttyACM0`
