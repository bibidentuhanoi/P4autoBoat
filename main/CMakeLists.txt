idf_component_register(
    SRCS
        "main.c"
        "file_system.c"
        "sensor_fusion.c"
        "calibration.c"
        "drivers/imu_driver.c"
        "drivers/tof_driver.c"
        "drivers/camera_driver.c"
        "wifi_manager.c"
        "camera_stream.c"
        "pipeline.c"
        "http_server.c"
        "sensor_task.c"
        "transports/ws_transport.c"
        "proto/boat.pb.c"
    PRIV_REQUIRES
        spi_flash esp_timer nvs_flash driver
        esp_video esp_wifi esp_wifi_remote esp_netif
        esp_http_server esp_driver_jpeg esp_driver_ppa
        livekit__nanopb
    INCLUDE_DIRS "include" "drivers" "." "proto"
    EMBED_TXTFILES "dashboard.html")
