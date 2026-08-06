#!/usr/bin/env python3
"""
Verify the ESP32-S3 bridge's USB CDC self-test frames.

Purpose: prove the CDC TX-FIFO truncation fix works, in isolation — no P4, no
C6, no radio involved.  The S3 (built with USB_SELFTEST 1) emits synthetic
COBS-framed frames at real telemetry sizes through the same code path
production uses.  This script checks every frame arrives COMPLETE and intact.

Before the fix, tinyusb_cdcacm_write_queue()'s return value was discarded, so
anything larger than the free TX FIFO (512 B) was silently truncated.  Frames
of 1126 / 2299 / 4000 B would arrive short or fail to decode.

Usage:
    .venv/bin/python tools/usb_selftest_verify.py /dev/ttyACM0
    .venv/bin/python tools/usb_selftest_verify.py /dev/ttyACM0 --seconds 20

Exit code 0 = all frames intact, 1 = corruption/truncation detected.
"""
import argparse
import collections
import struct
import sys
import time

MAGIC = b"TST1"
HEADER_LEN = 12


def cobs_decode(data: bytes) -> bytes:
    """Decode one COBS frame (delimiter already stripped). b'' on malformed."""
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        idx += 1
        if code == 0:
            return b""
        num = code - 1
        if idx + num > len(data):
            return b""          # truncated mid-block
        out.extend(data[idx:idx + num])
        idx += num
        if code < 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)


def check_frame(decoded: bytes, stats, sizes):
    """Validate one decoded self-test frame. Returns a short verdict string."""
    if len(decoded) < HEADER_LEN:
        stats["runt"] += 1
        return f"RUNT ({len(decoded)} B)"

    if decoded[:4] != MAGIC:
        stats["bad_magic"] += 1
        return f"BAD MAGIC {decoded[:4]!r}"

    seq, declared = struct.unpack("<II", decoded[4:12])

    # THE truncation check: does the frame carry as many bytes as it claims?
    if len(decoded) != declared:
        stats["truncated"] += 1
        sizes[declared]["bad"] += 1
        return f"TRUNCATED seq={seq} got {len(decoded)} of {declared} B"

    # Body integrity (catches corruption that preserves length)
    for i in range(HEADER_LEN, declared):
        if decoded[i] != ((i * 7 + seq) & 0xFF):
            stats["corrupt"] += 1
            sizes[declared]["bad"] += 1
            return f"CORRUPT seq={seq} at byte {i}"

    stats["ok"] += 1
    sizes[declared]["ok"] += 1
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--seconds", type=float, default=15.0,
                    help="how long to listen (default: 15)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="ignored by USB CDC; pyserial requires a value")
    args = ap.parse_args()

    try:
        import serial as pyserial
    except ImportError:
        print("ERROR: pyserial not installed. Try: .venv/bin/pip install pyserial",
              file=sys.stderr)
        return 1

    try:
        ser = pyserial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as exc:                       # noqa: BLE001 - report any open failure
        print(f"ERROR: cannot open {args.port}: {exc}", file=sys.stderr)
        return 1

    stats = collections.Counter()
    sizes = collections.defaultdict(lambda: collections.Counter())
    buf = bytearray()
    deadline = time.time() + args.seconds

    print(f"Listening on {args.port} for {args.seconds:.0f}s ...")
    print("(S3 must be flashed with USB_SELFTEST 1)\n")

    with ser:
        while time.time() < deadline:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)

            while True:
                delim = buf.find(0x00)
                if delim < 0:
                    break
                frame, buf = bytes(buf[:delim]), bytearray(buf[delim + 1:])
                if not frame:
                    continue                      # resync delimiter, ignore
                decoded = cobs_decode(frame)
                if not decoded:
                    stats["cobs_fail"] += 1
                    print("  COBS DECODE FAILED "
                          f"({len(frame)} B encoded)")
                    continue
                verdict = check_frame(decoded, stats, sizes)
                if verdict:
                    print(f"  {verdict}")

    total = sum(stats.values())
    print("\n" + "=" * 58)
    print(f"{'RESULT':<22}{'count':>10}")
    print("-" * 58)
    for key in ("ok", "truncated", "corrupt", "cobs_fail", "bad_magic", "runt"):
        if stats[key]:
            print(f"{key:<22}{stats[key]:>10}")
    print("-" * 58)

    if sizes:
        print(f"\n{'frame size':<14}{'ok':>8}{'bad':>8}   (>512 B would fail pre-fix)")
        for size in sorted(sizes):
            s = sizes[size]
            flag = "  <-- exceeds old 512 B FIFO" if size > 512 else ""
            print(f"{size:<14}{s['ok']:>8}{s['bad']:>8}{flag}")

    print()
    if total == 0:
        print("NO FRAMES RECEIVED.")
        print("  - Is the S3 flashed with USB_SELFTEST 1?")
        print("  - Right port? (dmesg | tail  after plugging in)")
        print("  - The S3 only transmits once the port is open (DTR asserted).")
        return 1

    bad = total - stats["ok"]
    if bad:
        print(f"FAIL: {bad} of {total} frames were truncated or corrupt.")
        return 1

    print(f"PASS: all {stats['ok']} frames arrived complete and intact.")
    print("USB CDC TX path is sound — truncation fix verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
