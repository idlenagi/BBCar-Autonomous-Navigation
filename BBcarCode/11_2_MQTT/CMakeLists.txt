#
# Mbed CE MQTT Project
#
cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

#### Initialize Mbed OS build system. ####
######################################################################################################
# Set path of mbed_app.json (necessary at all times)
set(MBED_APP_JSON_PATH mbed_app.json)

### Include Mbed toolchain setup file
include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

### Set up your project.
project(11_2_MQTT
        LANGUAGES C CXX ASM)

### Include Mbed project setup file
include(mbed_project_setup)

######################################################################################################
### Block of including project folders

# Add mbed-os subdirectory (necessary everytime)
add_subdirectory(mbed-os)

# Add subdirectories for components
add_subdirectory(components/wifi-ism43362)
add_subdirectory(components/paho_mqtt/MQTTPacket)

# Build Paho MQTT static library (Removed duplicate source files)
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

######################################################################################################

# Add executable
add_executable(${PROJECT_NAME} main.cpp)

### Link libraries block
if("MBED_CONF_TARGET_APPLICATION_PROFILE=bare-metal" IN_LIST MBED_CONFIG_DEFINITIONS)
    target_link_libraries(${PROJECT_NAME} 
    PRIVATE
        mbed-baremetal
    )
else()
    target_link_libraries(${PROJECT_NAME} 
    PRIVATE
        mbed-os
    )
endif()

# Base project and MQTT Includes
target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/components/paho_mqtt
        ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTClient
        ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTClient/mbed
        ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket
)

# Link Networking/MQTT libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        mbed-netsocket
        mbed-mbedtls
        paho_mqtt
        wifi-ism43362
)

# ==============================================================================
# LOCAL WORKSPACE FOLDERS (Added based on your file tree)
# ==============================================================================
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}/bbcar
    ${CMAKE_SOURCE_DIR}/Pixy2
    ${CMAKE_SOURCE_DIR}/pwmin
    ${CMAKE_SOURCE_DIR}/wifi_mqtt
)

file(GLOB_RECURSE USER_SOURCES 
    "bbcar/*.cpp" "bbcar/*.c"
    "Pixy2/*.cpp" "Pixy2/*.c"
    "pwmin/*.cpp" "pwmin/*.c"
    "wifi_mqtt/*.cpp" "wifi_mqtt/*.c"
)
target_sources(${PROJECT_NAME} PRIVATE ${USER_SOURCES})


# ==============================================================================
# BSP AND SENSOR FOLDERS
# ==============================================================================
# Add the specific subfolders containing the headers
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}/bsp-b-l475e-iot01/Drivers/BSP/B-L475E-IOT01
    ${CMAKE_SOURCE_DIR}/bsp-b-l475e-iot01/Drivers/BSP/Components
)

# Grab all the C/C++ source files recursively from the main bsp folder
file(GLOB_RECURSE BSP_SOURCES 
    "bsp-b-l475e-iot01/*.c" 
    "bsp-b-l475e-iot01/*.cpp"
)
target_sources(${PROJECT_NAME} PRIVATE ${BSP_SOURCES})

# ==============================================================================

# This generates build rules to generate bin/hex files
mbed_set_post_build(${PROJECT_NAME})
