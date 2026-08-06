#!/usr/bin/env bash
#
# gen_proto.sh — regenerate the Python protobuf binding from boat.proto.
#
# SCOPE: Python only. This script deliberately does NOT regenerate the nanopb
# C files (main/proto/boat.pb.c / .h). Those are committed, correct, and are
# compiled into the WORKING WiFi/WS path as well as ESP-NOW — regenerating them
# would put a working transport at risk for no benefit. The script asserts it
# left them untouched.
#
# Generation runs under the repo .venv — the SAME interpreter that runs
# visualize.py — so the protobuf gencode and runtime versions cannot drift
# apart (protobuf 4+ gencode embeds a runtime-version guard).
#
# Usage:  tools/gen_proto.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

VENV_PY="$REPO_ROOT/.venv/bin/python"
PROTO_SRC_DIR="main/proto"
PROTO_FILE="boat.proto"
OUT_DIR="proto"
DASHBOARD="main/dashboard.html"

# ---------------------------------------------------------------------------
# Preconditions
# ---------------------------------------------------------------------------
if [ ! -x "$VENV_PY" ]; then
    echo "ERROR: repo venv interpreter not found at $VENV_PY" >&2
    echo "       Create it, then: .venv/bin/pip install grpcio-tools" >&2
    exit 1
fi

if ! "$VENV_PY" -c 'import grpc_tools' 2>/dev/null; then
    echo "ERROR: grpcio-tools is not installed in the repo venv." >&2
    echo "       Fix with:" >&2
    echo "         .venv/bin/pip install grpcio-tools" >&2
    exit 1
fi

if [ ! -f "$PROTO_SRC_DIR/$PROTO_FILE" ]; then
    echo "ERROR: $PROTO_SRC_DIR/$PROTO_FILE not found" >&2
    exit 1
fi

# Record the committed C so we can prove we did not touch it.
C_BEFORE="$(git status --porcelain -- "$PROTO_SRC_DIR/boat.pb.c" "$PROTO_SRC_DIR/boat.pb.h" || true)"

# ---------------------------------------------------------------------------
# Generate
# ---------------------------------------------------------------------------
mkdir -p "$OUT_DIR"

"$VENV_PY" -m grpc_tools.protoc \
    -I "$PROTO_SRC_DIR" \
    --python_out="$OUT_DIR" \
    "$PROTO_FILE"

# Package marker so `from proto import boat_pb2` resolves from the repo root.
if [ ! -f "$OUT_DIR/__init__.py" ]; then
    : > "$OUT_DIR/__init__.py"
fi

# ---------------------------------------------------------------------------
# Verify output (no half-generated state)
# ---------------------------------------------------------------------------
if [ ! -f "$OUT_DIR/boat_pb2.py" ]; then
    echo "ERROR: generation reported success but $OUT_DIR/boat_pb2.py is missing" >&2
    exit 1
fi

# The real gate: it must IMPORT under the same interpreter visualize.py uses.
if ! "$VENV_PY" -c 'from proto import boat_pb2; boat_pb2.BoatMessage()' 2>/dev/null; then
    echo "ERROR: $OUT_DIR/boat_pb2.py generated but fails to import under .venv." >&2
    echo "       Usually a protobuf gencode/runtime major-version mismatch." >&2
    "$VENV_PY" -c 'from proto import boat_pb2' || true
    exit 1
fi

# ---------------------------------------------------------------------------
# Guard: the committed nanopb C must be untouched (protects the WiFi path)
# ---------------------------------------------------------------------------
C_AFTER="$(git status --porcelain -- "$PROTO_SRC_DIR/boat.pb.c" "$PROTO_SRC_DIR/boat.pb.h" || true)"
if [ "$C_BEFORE" != "$C_AFTER" ]; then
    echo "ERROR: this script modified main/proto/boat.pb.c/.h — it must not." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Drift detection for the ONE copy that cannot be generated:
# dashboard.html's hand-mirrored protobuf.js protoSchema.
# (A stale mirror silently drops unknown fields — the 2026-07 UI lockout.)
# ---------------------------------------------------------------------------
if [ -f "$DASHBOARD" ] && [ "$PROTO_SRC_DIR/$PROTO_FILE" -nt "$DASHBOARD" ]; then
    echo
    echo "*** WARNING: SCHEMA DRIFT RISK ***"
    echo "  $PROTO_SRC_DIR/$PROTO_FILE is NEWER than $DASHBOARD."
    echo "  dashboard.html's protoSchema is hand-mirrored and cannot be generated."
    echo "  If you changed boat.proto, mirror the change there in the SAME commit —"
    echo "  protobuf.js drops unknown fields silently (caused the 2026-07 UI lockout)."
    echo
fi

echo "OK: $OUT_DIR/boat_pb2.py generated and imports cleanly under .venv"
echo "    (nanopb C untouched; firmware build unaffected)"
