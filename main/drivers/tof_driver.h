#ifndef TOF_DRIVER_H
#define TOF_DRIVER_H

#include "driver/i2c_master.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_xtalk.h"
#include "esp_err.h"

// Constants
#define VL53_DEFAULT_ADDR   0x29

// Structures
typedef struct {
    VL53L5CX_Configuration dev_a;
    VL53L5CX_Configuration dev_b;
} tof_devices_t;

esp_err_t tof_init(i2c_master_bus_handle_t bus_handle, tof_devices_t* devices);
esp_err_t tof_read_grid(VL53L5CX_Configuration* dev, VL53L5CX_ResultsData* results);

#endif // TOF_DRIVER_H
