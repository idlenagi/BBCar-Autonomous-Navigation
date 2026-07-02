cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(MBED_APP_JSON_PATH mbed_app.json5)
include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(lv_F769 VERSION 1.0.0 LANGUAGES C CXX ASM)

include(mbed-os/tools/cmake/mbed_project_setup.cmake)

add_subdirectory(mbed-os)
add_subdirectory(lvgl)

# --- NEW: Network & MQTT Subdirectories ---
add_subdirectory(components/paho_mqtt/MQTTPacket)

# Build the Paho MQTT Static Library
add_library(paho_mqtt STATIC
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTConnectClient.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTConnectServer.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTDeserializePublish.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTFormat.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTPacket.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTSerializePublish.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTSubscribeClient.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTSubscribeServer.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTUnsubscribeClient.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTUnsubscribeServer.c
)
# ------------------------------------------

# Hardware BSP Source Files
FILE(GLOB_RECURSE HAL_Sources 
    Utilities/Components/ft6x06/ft6x06.c
    Utilities/Components/otm8009a/otm8009a.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery_ts.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery_lcd.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery_sdram.c
)

# Display Driver Source Files
FILE(GLOB_RECURSE LVGL_Sources CONFIGURE_DEPENDS 
     hal_stm_lvgl/tft.cpp
     hal_stm_lvgl/touchpad.cpp
)

# NOTE: eRPC sources and AccelerometerService_client.cpp have been completely removed!
add_executable(${PROJECT_NAME} 
    src/main.cpp
    ${HAL_Sources}
    ${LVGL_Sources}
)

add_definitions(
    -DSTM32 -DSTM32F7 -DSTM32F769xx -DSTM32F769I_DISCO -DDEBUG -DUSE_HAL_DRIVER -DDMA2_Stream0=0
)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/lvgl
    ${CMAKE_SOURCE_DIR}/Utilities/STM32F769I-Discovery
    ${CMAKE_SOURCE_DIR}/Utilities/Components/ft6x06/
    src
    # --- NEW: MQTT Includes ---
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTClient
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTClient/mbed
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket
    # --------------------------
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    m
    lvgl
    lvgl_demos
    mbed-os
    # --- NEW: Linked Network Libraries ---
    mbed-netsocket
    mbed-mbedtls
    paho_mqtt
    # -------------------------------------
)

mbed_set_post_build(${PROJECT_NAME})
