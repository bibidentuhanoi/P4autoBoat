# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BoatEspP4)

# The registry VL53L5CX driver performs its roughly 1.4 KB result read as one
# I2C transaction.  On the shared sensor bus that owns the driver lock for
# 25-32 ms and makes the 50 Hz IMU miss its period.  Keep the locked registry
# component, but replace exactly its platform source with our bounded-read
# implementation.  Failing if the upstream source layout changes prevents a
# silent return to the long transaction.
idf_component_get_property(_vl53l5cx_lib rjrp44__vl53l5cx COMPONENT_LIB)
get_target_property(_vl53l5cx_sources ${_vl53l5cx_lib} SOURCES)
set(_vl53l5cx_filtered_sources "")
set(_vl53l5cx_platform_source_count 0)
foreach(_source IN LISTS _vl53l5cx_sources)
    if(_source MATCHES "platform\\.c$")
        math(EXPR _vl53l5cx_platform_source_count
             "${_vl53l5cx_platform_source_count} + 1")
    else()
        list(APPEND _vl53l5cx_filtered_sources "${_source}")
    endif()
endforeach()
if(NOT _vl53l5cx_platform_source_count EQUAL 1)
    message(FATAL_ERROR
        "Expected exactly one VL53L5CX platform.c, found ${_vl53l5cx_platform_source_count}")
endif()
set_property(TARGET ${_vl53l5cx_lib} PROPERTY SOURCES
             "${_vl53l5cx_filtered_sources}")
target_sources(${_vl53l5cx_lib} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/main/drivers/vl53l5cx_platform.c")
