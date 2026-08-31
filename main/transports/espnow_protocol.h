#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Message type identifiers
 * -------------------------------------------------------------------------*/
typedef enum {
    MSG_SENSOR        = 0x02,
    MSG_MOTOR_CMD     = 0x03,
    MSG_ARM_CMD       = 0x04,
    MSG_DETECT_CMD    = 0x05,
    MSG_MOTOR_STATUS  = 0x06,
    MSG_STATUS        = 0x07,  /* SystemStatus -- see espnow_send_fn() */
    /* Field-mode-only compact telemetry -- see espnow_telemetry_t below.
     * Replaces MSG_SENSOR for the ESP-NOW link specifically; WS/bench mode
     * keeps sending full SensorSnapshot via MSG_SENSOR, unaffected. */
    MSG_FIELD_TELEMETRY = 0x08,
    MSG_ESPNOW_INIT   = 0x10,
    /* S3 ground station -> boat: "I am here". Broadcast periodically by the
     * USB bridge. The boat listens for it at boot to decide whether to run in
     * ESP-NOW field mode or fall back to WiFi. Header only, no payload. */
    MSG_GROUND_HELLO  = 0x12,
    /* S3 bridge -> laptop (USB only, never over the air): self-diagnostics so
     * the bridge is observable. Its console is unavailable once TinyUSB claims
     * the USB pins, so without this a silent bridge is indistinguishable from
     * a crashed one, or from a boat that never transmitted. */
    MSG_BRIDGE_STATUS = 0x13,
} espnow_msg_type_t;

/* Payload of MSG_BRIDGE_STATUS (packed, little-endian). */
typedef struct __attribute__((packed)) {
    uint32_t uptime_s;      /* bridge uptime */
    uint32_t espnow_pkts;   /* ESP-NOW packets received off-air */
    uint32_t espnow_bytes;  /* ...and their total payload bytes */
    uint32_t frames_out;    /* complete frames reassembled and sent to USB */
    uint32_t hello_sent;    /* MSG_GROUND_HELLO beacons broadcast */
    uint32_t reasm_drops;   /* frames dropped: overflow or length mismatch */
} espnow_bridge_status_t;

/* How often the S3 bridge broadcasts MSG_GROUND_HELLO. The boat's probe window
 * must comfortably exceed this so it cannot miss the gap between two beacons. */
#define ESPNOW_HELLO_INTERVAL_MS  500u

/* ---------------------------------------------------------------------------
 * Base packet header (4 bytes, packed)
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     /* espnow_msg_type_t */
    uint16_t payload_len;  /* byte length of payload following header */
    uint8_t  seq;          /* rolling sequence number */
} espnow_pkt_hdr_t;

#define ESPNOW_HDR_SIZE  (sizeof(espnow_pkt_hdr_t))  /* 4 */

/* ---------------------------------------------------------------------------
 * MSG_FIELD_TELEMETRY payload (packed, little-endian)
 *
 * Field-mode-only replacement for the full protobuf SensorSnapshot: IMU +
 * GPS, nothing else -- no ToF, no detections. ~42 bytes vs. up to 13270 for
 * the full boat_BoatMessage worst case: fits in ONE ESP-NOW packet (244B
 * limit, see ESPNOW_MAX_PAYLOAD below), so it never needs fragmentation.
 *
 * That matters concretely: a full snapshot needs ~55 fragments, each with a
 * mandatory 1ms inter-chunk delay on the C6 (espnow_bridge.c's
 * espnow_frag_send()) -- on the C6's single core, a backlog of those can
 * starve the esp_hosted RPC response the P4's sensor task is blocked
 * waiting on, which is what turned into esp_hosted_send_custom_data timing
 * out and the sensor task blowing its 50ms loop budget (see sensor_task.c's
 * "snapshot loop overrun" history). Zero fragmentation removes that failure
 * mode structurally, not just by making it less likely.
 *
 * Deliberately a hand-packed struct, not a new boat.proto message: adding a
 * message to boat.proto means regenerating main/proto/boat.pb.c/.h, which
 * tools/gen_proto.sh explicitly refuses to do ("would put a WORKING
 * transport at risk for no benefit"). This type never touches boat.proto or
 * its generated code, so it carries none of that risk to the WiFi/WS path
 * -- which keeps sending the full boat_SensorSnapshot exactly as before,
 * completely unaffected by anything here.
 *
 * Field names mirror boat.proto's IMUData/GpsFix 1:1 (values, not full
 * field sets -- altitude_m/fix_quality/utc_ms are omitted, matching what
 * the ESP-NOW-side tools actually consume) so nothing has to be re-derived
 * by a reader comparing the two. */
typedef struct __attribute__((packed)) {
    float    pitch;
    float    roll;
    float    heading;
    uint8_t  gps_valid;
    double   latitude;
    double   longitude;
    float    speed_mps;
    float    course_deg;
    uint8_t  satellites;
    float    hdop;
    /* Gyro-Z turn rate (deg/s), mag-independent -- the signal the bench
     * throttle-mismatch test reads. Appended last so the offsets of every
     * field above are unchanged; tools/espnow_drive.py's
     * FIELD_TELEMETRY_FMT must be extended in the same commit or the frame
     * is length-rejected (which is the intended loud failure, not silent
     * corruption). */
    float    yaw_rate;
} espnow_telemetry_t;

/* ---------------------------------------------------------------------------
 * Fragmentation header — prepended to each fragment payload
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint32_t total_len;    /* total reassembled payload length */
} espnow_frag_hdr_t;

/* ---------------------------------------------------------------------------
 * Frame-size constants
 * -------------------------------------------------------------------------*/
#define ESPNOW_MAX_PAYLOAD   244u
#define ESPNOW_FRAG_PAYLOAD  (ESPNOW_MAX_PAYLOAD - sizeof(espnow_frag_hdr_t))

/* ---------------------------------------------------------------------------
 * COBS (Consistent Overhead Byte Stuffing)
 *
 * Encodes/decodes so that 0x00 never appears in the payload, enabling it to
 * serve as a reliable frame delimiter.
 *
 * Output buffer sizing:
 *   encode: dst must hold at least (len + len/254 + 2) bytes
 *   decode: dst must hold at least len bytes
 * -------------------------------------------------------------------------*/

/**
 * cobs_encode - Encode src[0..len) into dst using COBS.
 * Returns the number of bytes written to dst (always >= 1).
 */
static inline size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t write_idx  = 0;
    size_t code_idx   = 0;   /* position where the next code byte will go */
    uint8_t code      = 1;   /* distance to the next zero (or block end) */

    /* Reserve space for the first code byte. */
    code_idx  = write_idx;
    write_idx = 1;

    for (size_t i = 0; i < len; i++) {
        if (src[i] != 0x00) {
            dst[write_idx++] = src[i];
            code++;

            /* A COBS block can reference at most 254 non-zero bytes.
             * If we hit that limit, close the block and start a new one. */
            if (code == 0xFF) {
                dst[code_idx] = code;
                code_idx  = write_idx;
                write_idx++;
                code = 1;
            }
        } else {
            /* Zero byte: write the current code and start a fresh block. */
            dst[code_idx] = code;
            code_idx  = write_idx;
            write_idx++;
            code = 1;
        }
    }

    /* Write the final code byte. */
    dst[code_idx] = code;

    return write_idx;
}

/**
 * cobs_decode - Decode COBS-encoded src[0..len) into dst.
 * Returns the number of bytes written to dst, or 0 on error (unexpected
 * zero byte inside the encoded stream).
 */
static inline size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read_idx  = 0;
    size_t write_idx = 0;

    while (read_idx < len) {
        uint8_t code = src[read_idx++];

        /* A zero code byte is invalid inside a COBS frame. */
        if (code == 0x00) {
            return 0;
        }

        /* Copy (code - 1) literal bytes. */
        uint8_t num_bytes = code - 1;
        if (read_idx + num_bytes > len) {
            return 0;  /* truncated frame */
        }
        memcpy(&dst[write_idx], &src[read_idx], num_bytes);
        write_idx += num_bytes;
        read_idx  += num_bytes;

        /* If code < 0xFF, the original byte at this position was 0x00;
         * restore it — unless we have reached the end of the data. */
        if (code < 0xFF && read_idx < len) {
            dst[write_idx++] = 0x00;
        }
    }

    return write_idx;
}
