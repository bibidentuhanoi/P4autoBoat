#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Message type identifiers
 * -------------------------------------------------------------------------*/
typedef enum {
    MSG_JPEG_CHUNK    = 0x01,
    MSG_SENSOR        = 0x02,
    MSG_MOTOR_CMD     = 0x03,
    MSG_ARM_CMD       = 0x04,
    MSG_DETECT_CMD    = 0x05,
    MSG_MOTOR_STATUS  = 0x06,
    MSG_ESPNOW_INIT   = 0x10,
    MSG_ESPNOW_CONFIG = 0x11,
} espnow_msg_type_t;

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
