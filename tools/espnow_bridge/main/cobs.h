#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

static inline size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t write_idx = 0, code_idx = 0;
    uint8_t code = 1;
    code_idx = write_idx; write_idx = 1;
    for (size_t i = 0; i < len; i++) {
        if (src[i] != 0x00) {
            dst[write_idx++] = src[i]; code++;
            if (code == 0xFF) {
                dst[code_idx] = code; code_idx = write_idx; write_idx++; code = 1;
            }
        } else {
            dst[code_idx] = code; code_idx = write_idx; write_idx++; code = 1;
        }
    }
    dst[code_idx] = code;
    return write_idx;
}

static inline size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read_idx = 0, write_idx = 0;
    while (read_idx < len) {
        uint8_t code = src[read_idx++];
        if (code == 0x00) return 0;
        uint8_t num = code - 1;
        if (read_idx + num > len) return 0;
        memcpy(&dst[write_idx], &src[read_idx], num);
        write_idx += num; read_idx += num;
        if (code < 0xFF && read_idx < len) dst[write_idx++] = 0x00;
    }
    return write_idx;
}
