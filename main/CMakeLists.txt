idf_component_register(SRCS "main.c" "file_system.c" "sensor_fusion.c" "calibration.c" "drivers/imu_driver.c" "drivers/tof_driver.c"
                    PRIV_REQUIRES spi_flash esp_timer nvs_flash driver
                    INCLUDE_DIRS "include" "drivers")
