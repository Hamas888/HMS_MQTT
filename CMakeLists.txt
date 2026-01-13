# HMS_MQTT/CMakeLists.txt

set(HMS_MQTT_VERSION 1.0.0)

# Check if we're building with Zephyr
if(DEFINED ZEPHYR_BASE)
    zephyr_library_sources(src/HMS_MQTT.cpp)
    zephyr_include_directories(include)

# Check if we're building with ESP-IDF
elseif(DEFINED ESP_PLATFORM OR DEFINED IDF_VER OR DEFINED ENV{IDF_PATH})
    idf_component_register(
        INCLUDE_DIRS "include"
        SRCS 
            "src/HMS_MQTT.cpp" 
            "src/HMS_MQTT_ESP_IDF.cpp"
        REQUIRES 
            "mqtt"
            "esp_eth"
            "esp_wifi"  
            "nvs_flash" 
            "esp_event" 
            "esp_netif"
    )

    # Force C++17 standard
    target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)

    # (optional) if you want strict enforcement instead of compiler default fallback
    set_target_properties(${COMPONENT_LIB} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    
# STM32 / generic CMake project
else()
    add_library(HMS_MQTT INTERFACE)
    target_include_directories(HMS_MQTT INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_features(HMS_MQTT INTERFACE cxx_std_17)
endif()