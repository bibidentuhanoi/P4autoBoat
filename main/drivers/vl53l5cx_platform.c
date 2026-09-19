/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"

//Use the timeout from the config, the default timeout is -1
#if CONFIG_VL53L5CX_I2C_TIMEOUT == false
#define VL53L5CX_I2C_TIMEOUT (-1)
#else
#define VL53L5CX_I2C_TIMEOUT CONFIG_VL53L5CX_I2C_TIMEOUT_VALUE
#endif

/* The 8x8/four-target result block is about 1.4KB.  Sending that as one
 * 400kHz transaction owns the shared sensor bus for roughly 25-32ms, longer
 * than the IMU task's entire 20ms period.  Bound each transaction so the
 * ESP-IDF bus lock is released between chunks and the higher-priority IMU
 * task can run.  64 bytes is about 1.5ms of wire time at 400kHz. */
#define VL53L5CX_I2C_READ_CHUNK_SIZE 64U

//Define the reset scheme
#ifdef CONFIG_VL53L5CX_RESET_PIN_HIGH
#define VL53L5CX_RESET_LEVEL 1
#elif CONFIG_VL53L5CX_RESET_PIN_LOW
#define VL53L5CX_RESET_LEVEL 0
#endif

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {

    //Select the correct buffer
    i2c_master_transmit_multi_buffer_info_t i2c_buffers[2];

    //Convert the uint16 address to an array of uint8
    uint8_t i2c_address[] = {RegisterAdress >> 8, RegisterAdress & 0xFF};

    //Add the address first to the data format
    i2c_buffers[0].write_buffer = i2c_address;
    i2c_buffers[0].buffer_size = 2;

    //Add the content to the data format
    i2c_buffers[1].write_buffer = p_values;
    i2c_buffers[1].buffer_size = size;

    return i2c_master_multi_buffer_transmit(p_platform->handle, i2c_buffers, 2, VL53L5CX_I2C_TIMEOUT);
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {

    //Write a single byte
    return VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    uint32_t offset = 0;

    while (offset < size) {
        uint32_t remaining = size - offset;
        uint32_t chunk_size = remaining < VL53L5CX_I2C_READ_CHUNK_SIZE
                            ? remaining : VL53L5CX_I2C_READ_CHUNK_SIZE;
        uint16_t chunk_address = (uint16_t)(RegisterAdress + offset);
        uint8_t i2c_address[] = {
            (uint8_t)(chunk_address >> 8),
            (uint8_t)(chunk_address & 0xFF)
        };

        uint8_t status = (uint8_t)i2c_master_transmit_receive(
            p_platform->handle, i2c_address, sizeof(i2c_address),
            p_values + offset, chunk_size, VL53L5CX_I2C_TIMEOUT);
        if (status != 0U) {
            return status;
        }
        offset += chunk_size;
    }

    return 0U;
}

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {

    //Read a single byte
    return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform* p_platform)
{
    gpio_set_direction(p_platform->reset_gpio, GPIO_MODE_OUTPUT);

    gpio_set_level(p_platform->reset_gpio, VL53L5CX_RESET_LEVEL);
    VL53L5CX_WaitMs(p_platform, 100);

    gpio_set_level(p_platform->reset_gpio, !VL53L5CX_RESET_LEVEL);
    VL53L5CX_WaitMs(p_platform, 100);

    return ESP_OK;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i;
    uint8_t tmp[4] = {0};

    for (i = 0; i < size; i = i + 4) {

        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];

        memcpy(&(buffer[i]), tmp, 4);
    }
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
    vTaskDelay(TimeMs / portTICK_PERIOD_MS);

    return ESP_OK;
}
