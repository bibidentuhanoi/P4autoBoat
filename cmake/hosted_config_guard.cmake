# ---------------------------------------------------------------------------
# Configuration guard -- fails at CONFIGURE time, before anything compiles.
#
# A corrupted sdkconfig produces a binary that builds, links, boots and CANNOT
# TALK TO THE C6. Observed 2026-08-31: `idf.py fullclean` wiped the component
# state and left CONFIG_ESP_HOSTED_CP_TARGET collapsed to ESP32H2 (a chip with
# no WiFi at all), the board at NONE, the SDIO host interface gone and every
# WIFI_RMT_* symbol absent -- 182 settings in total. A later build SUCCEEDED
# against that wrong target, which is the dangerous case: nothing looks wrong
# until the boat is on the water with no link at all.
#
# sdkconfig.defaults pins these and a from-scratch regeneration honours them.
# This guard covers the other case: an sdkconfig that already exists and has
# silently drifted, which defaults cannot correct.
#
# Placed AFTER idf_component_register() because ESP-IDF does not populate the
# CONFIG_* CMake variables until a component has registered.
#
# Tests VALUES, not DEFINED: ESP-IDF emits every symbol, unset ones as an empty
# string -- `set(CONFIG_ESP_HOSTED_CP_TARGET_ESP32H2 "")` -- so if(DEFINED ...)
# is always true and would pass on any config at all.
set(REQUIRED_HOSTED_CONFIG
    CONFIG_SLAVE_IDF_TARGET_ESP32C6
    CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6
    CONFIG_ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD
    CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
    CONFIG_WIFI_RMT_STATIC_RX_BUFFER_NUM
    CONFIG_WIFI_RMT_DYNAMIC_RX_BUFFER_NUM
    CONFIG_WIFI_RMT_TX_BUFFER_TYPE
    CONFIG_WIFI_RMT_AMPDU_TX_ENABLED
    CONFIG_WIFI_RMT_TX_BA_WIN
    CONFIG_WIFI_RMT_AMPDU_RX_ENABLED
    CONFIG_WIFI_RMT_RX_BA_WIN)

# Symbols whose PRESENCE is the fault -- the wrong coprocessor being selected
# is exactly as bad as the right one being absent.
set(FORBIDDEN_HOSTED_CONFIG
    CONFIG_ESP_HOSTED_CP_TARGET_ESP32H2
    CONFIG_ESP_HOSTED_P4_DEV_BOARD_NONE)

set(HOSTED_CONFIG_FAULTS "")
foreach(sym IN LISTS REQUIRED_HOSTED_CONFIG)
    if(NOT DEFINED ${sym} OR "${${sym}}" STREQUAL "")
        list(APPEND HOSTED_CONFIG_FAULTS "${sym} is missing")
    endif()
endforeach()
foreach(sym IN LISTS FORBIDDEN_HOSTED_CONFIG)
    if(DEFINED ${sym} AND NOT "${${sym}}" STREQUAL "")
        list(APPEND HOSTED_CONFIG_FAULTS "${sym} is SET and must not be")
    endif()
endforeach()

if(HOSTED_CONFIG_FAULTS)
    string(REPLACE ";" "\n    " _faults "${HOSTED_CONFIG_FAULTS}")
    message(FATAL_ERROR
        "sdkconfig has lost the ESP32-C6 coprocessor configuration:\n\n"
        "    ${_faults}\n\n"
        "Building this gives a binary with NO LINK TO THE C6 -- no WiFi, no\n"
        "ESP-NOW, no dashboard, no python tool. It boots and looks healthy.\n\n"
        "Fix by REGENERATING, never by hand-editing sdkconfig:\n\n"
        "    rm sdkconfig && idf.py reconfigure\n\n"
        "sdkconfig.defaults pins every symbol above and is tracked in git.\n"
        "Re-apply any local menuconfig choices afterwards, then rebuild.")
endif()
