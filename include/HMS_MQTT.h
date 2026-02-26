/*
 ============================================================================================================================================
 * File:        HMS_MQTT.h
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Nov 4 2025
 * Brief:       This file package provides MQTT client for embedded & Desktop systems (Arduino, ESP-IDF, Zephyr, STM32 HAL).
 ============================================================================================================================================
 * License: 
 * MIT License
 * 
 * Copyright (c) 2025 Hamas Saeed
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do 
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * For any inquiries, contact Hamas Saeed at hamasaeed@gmail.com
 ============================================================================================================================================
 */

#ifndef HMS_MQTT_H
#define HMS_MQTT_H

/* Platform detection *//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(ARDUINO)
  #include <Arduino.h>
  #if defined(ESP32)
    #include <esp_eth.h>
    #include <esp_wifi.h>
    #include <mqtt_client.h>
    #include <freertos/task.h>
    #include <freertos/FreeRTOS.h>
    #define HMS_MQTT_ARDUINO_ESP32
  #elif defined(ESP8266)
    // Include ESP8266 MQTT libraries here
    #define HMS_MQTT_ARDUINO_ESP8266
  #endif
  #define HMS_MQTT_PLATFORM_ARDUINO
#elif defined(ESP_PLATFORM)
  #include <string>
  #include <stdio.h>
  #include <stdlib.h>
  #include <esp_mac.h>
  #include <esp_eth.h>
  #include <esp_wifi.h>
  #include <functional>
  #include <esp_netif.h>
  #include <lwip/inet.h>
  #include <esp_timer.h>
  #include <lwip/netdb.h>
  #include <mqtt_client.h>
  #include <lwip/sockets.h>
  #include <freertos/task.h>
  #include <esp_crt_bundle.h>
  #include <freertos/FreeRTOS.h>
  #define HMS_MQTT_PLATFORM_ESP_IDF
#elif defined(__ZEPHYR__)
  // Zephyr specific includes
  #define HMS_MQTT_PLATFORM_ZEPHYR
#elif defined(__STM32__)
  // STM32 HAL specific includes
  #define HMS_MQTT_PLATFORM_STM32_HAL
#elif defined(__linux__) || defined(_WIN32) || defined(__APPLE__)
  // Desktop specific includes
  #define HMS_MQTT_PLATFORM_DESKTOP
#endif // Platform detection


/* Control Knobs *///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef HMS_MQTT_DEBUG
  #define HMS_MQTT_DEBUG                              1                                                                                               // Set to 1 to enable debug logging
#endif

#ifndef HMS_MQTT_DEFAULT_PORT
  #define HMS_MQTT_DEFAULT_PORT                       1883                                                                                            // Default MQTT port
#endif

#ifndef HMS_MQTT_DEFAULT_TLS_PORT
  #define HMS_MQTT_DEFAULT_TLS_PORT                   8883                                                                                            // Default MQTT TLS port 
#endif

#ifndef HMS_MQTT_DEFAULT_KEEP_ALIVE
  #define HMS_MQTT_DEFAULT_KEEP_ALIVE                 60                                                                                              // Default keep-alive interval in seconds
#endif

#ifndef HMS_MQTT_MAX_RECONNECT_DELAY
  #define HMS_MQTT_MAX_RECONNECT_DELAY                20000                                                                                           // Maximum reconnect delay in milliseconds
#endif

#ifndef HMS_MQTT_DEFAULT_TASK_PRIORITY
  #define HMS_MQTT_DEFAULT_TASK_PRIORITY              5                                                                                               // Default MQTT task priority
#endif

#ifndef HMS_MQTT_DEFAULT_NETWORK_TIMEOUT
  #define HMS_MQTT_DEFAULT_NETWORK_TIMEOUT            10000                                                                                           // Default network timeout in milliseconds
#endif

#ifndef HMS_MQTT_DEFAULT_TASK_STACK_SIZE
  #define HMS_MQTT_DEFAULT_TASK_STACK_SIZE            8192                                                                                            // Increased for TLS Handshake
#endif

#ifndef HMS_MQTT_DEFAULT_NETWORK_CHECK_INTERVAL
  #define HMS_MQTT_DEFAULT_NETWORK_CHECK_INTERVAL     5000                                                                                            // Network connectivity check interval in milliseconds
#endif

#ifndef HMS_MQTT_NETWORK_CHECK_TIMEOUT
  #define HMS_MQTT_NETWORK_CHECK_TIMEOUT              3000                                                                                            // Timeout for network connectivity checks in milliseconds
#endif

#ifndef HMS_MQTT_DEFAULT_RECONNECT_TIMEOUT
  #define HMS_MQTT_DEFAULT_RECONNECT_TIMEOUT          5000                                                                                            // Default reconnect timeout in milliseconds
#endif

#ifndef HMS_MQTT_INTERNET_CHECK_HOST
  #define HMS_MQTT_INTERNET_CHECK_HOST                "8.8.8.8"                                                                                       // Default host to ping for internet connectivity check
#endif

#if HMS_MQTT_DEBUG
  #if __has_include("ChronoLog.h")
    #include "ChronoLog.h"
    #ifndef HMS_MQTT_LOG_LEVEL
      #define HMS_MQTT_LOG_LEVEL CHRONOLOG_LEVEL_DEBUG
      extern ChronoLogger *mqttLogger;
    #endif
  #else
    #error "HMS_MQTT_DEBUG is enabled but ChronoLog.h is missing. Please include the https://github.com/Hamas888/ChronoLog in your project."
  #endif
#endif

typedef enum {
  HMS_MQTT_OK,
  HMS_MQTT_ERR_TLS,
  HMS_MQTT_ERR_INIT,
  HMS_MQTT_ERR_MEMORY,
  HMS_MQTT_ERR_NETWORK,
  HMS_MQTT_ERR_TIMEOUT,
  HMS_MQTT_ERR_UNKNOWN,
  HMS_MQTT_ERR_CONNECT,
  HMS_MQTT_ERR_PUBLISH,
  HMS_MQTT_ERR_SUBSCRIBE,
  HMS_MQTT_ERR_DISCONNECT,
  HMS_MQTT_ERR_UNSUBSCRIBE,
  HMS_MQTT_ERR_INVALID_CONFIG,
  HMS_MQTT_ERR_NETWORK_TIMEOUT,
  HMS_MQTT_ERR_NO_INTERNET_ACCESS,
  HMS_MQTT_ERR_NETWORK_UNAVAILABLE,
} HMS_MQTT_Status;

typedef enum {
  HMS_MQTT_STATE_ERROR,
  HMS_MQTT_STATE_CONNECTED,
  HMS_MQTT_STATE_CONNECTING,
  HMS_MQTT_STATE_DISCONNECTED,
  HMS_MQTT_STATE_RECONNECTING,
  HMS_MQTT_STATE_NETWORK_UNAVAILABLE,
  HMS_MQTT_STATE_WAITING_FOR_NETWORK,
} HMS_MQTT_Connection_State;

typedef enum {
  HMS_MQTT_QOS_0 = 0,
  HMS_MQTT_QOS_1 = 1,
  HMS_MQTT_QOS_2 = 2,
} HMS_MQTT_QoS;

typedef enum {
  HMS_MQTT_NETWORK_MODE_WIFI,
  HMS_MQTT_NETWORK_MODE_ETHERNET,
  HMS_MQTT_NETWORK_MODE_GSM,
} HMS_MQTT_Network_Mode;

typedef enum {
  HMS_MQTT_EVENT_DATA,
  HMS_MQTT_EVENT_ERROR,
  HMS_MQTT_EVENT_CONNECTED,
  HMS_MQTT_EVENT_PUBLISHED,
  HMS_MQTT_EVENT_SUBSCRIBED,
  HMS_MQTT_EVENT_DISCONNECTED,
  HMS_MQTT_EVENT_UNSUBSCRIBED,
} HMS_MQTT_Event_Type;

typedef std::function<void(HMS_MQTT_Connection_State state)> HMS_MQTT_StateCallback;
typedef std::function<void(HMS_MQTT_Event_Type event, const char* info)> HMS_MQTT_EventCallback;
typedef std::function<void(const char* topic, const char* payload, size_t length)> HMS_MQTT_MessageCallback;

typedef struct {
  bool                        useTLS                          = false;
  bool                        lwtEnabled                      = false;
  bool                        pubRetain                       = false;
  bool                        subRetain                       = false;
  bool                        lwtRetain                       = false;
  bool                        autoReconnect                   = true;
  bool                        cleanSession                    = true;
  bool                        enableNetworkCheck              = true;                                                                                 // Enable automatic network connectivity checking
  bool                        enableInternetCheck             = true;                                                                                 // Enable internet connectivity checking (ping test)
  bool                        pauseOnNetworkLoss              = true;                                                                                 // Pause MQTT operations when network is lost
  uint16_t                    brokerPort                      = HMS_MQTT_DEFAULT_PORT;
  uint16_t                    keepAliveInterval               = HMS_MQTT_DEFAULT_KEEP_ALIVE;
  uint16_t                    reconnectTimeoutMs              = HMS_MQTT_DEFAULT_RECONNECT_TIMEOUT;
  uint16_t                    networkTimeoutMs                = HMS_MQTT_DEFAULT_NETWORK_TIMEOUT;
  uint16_t                    networkCheckIntervalMs          = HMS_MQTT_DEFAULT_NETWORK_CHECK_INTERVAL;                                              // How often to check network connectivity
  uint16_t                    networkCheckTimeoutMs           = HMS_MQTT_NETWORK_CHECK_TIMEOUT;                                                      // Timeout for network connectivity checks
  uint32_t                    taskStackSize                   = HMS_MQTT_DEFAULT_TASK_STACK_SIZE;
  uint8_t                     taskPriority                    = HMS_MQTT_DEFAULT_TASK_PRIORITY;
  std::string                 brokerEndpoint                  = "";
  std::string                 clientId                        = "";
  std::string                 username                        = "";
  std::string                 password                        = "";
  std::string                 pubTopic                        = "";
  std::string                 subTopic                        = "";
  std::string                 lwtTopic                        = "";
  std::string                 lwtMessage                      = "";
  std::string                 caCert                          = "";
  std::string                 privateKey                      = "";
  std::string                 clientCert                      = "";
  std::string                 internetCheckHost               = HMS_MQTT_INTERNET_CHECK_HOST;                                                        // Host to ping for internet connectivity check
  HMS_MQTT_QoS                lwtQoS                          = HMS_MQTT_QOS_0;
  HMS_MQTT_QoS                pubQoS                          = HMS_MQTT_QOS_0;
  HMS_MQTT_QoS                subQoS                          = HMS_MQTT_QOS_0;
  HMS_MQTT_MessageCallback    messageCallback                 = nullptr;
  HMS_MQTT_EventCallback      eventCallback                   = nullptr;
  HMS_MQTT_StateCallback      stateCallback                   = nullptr;
} HMS_MQTT_Client_Config;

class HMS_MQTT {
  public:
    HMS_MQTT(HMS_MQTT_Network_Mode networkMode = HMS_MQTT_NETWORK_MODE_WIFI);
    ~HMS_MQTT();

    HMS_MQTT_Status stop();
    HMS_MQTT_Status start();
    HMS_MQTT_Status restart();
    HMS_MQTT_Status sendMessage(std::string payload);
    HMS_MQTT_Status configure(HMS_MQTT_Client_Config& config);

    void setEventCallback(HMS_MQTT_EventCallback callback)      { config.eventCallback = callback;                                }
    void setStateCallback(HMS_MQTT_StateCallback callback)      { config.stateCallback = callback;                                }
    void setMessageCallback(HMS_MQTT_MessageCallback callback)  { config.messageCallback = callback;                              }

    // Network connectivity methods
    HMS_MQTT_Status checkNetworkConnectivity();
    HMS_MQTT_Status waitForNetwork(uint32_t timeoutMs = 30000);

    bool isConnected() const                                    { return connectionState == HMS_MQTT_STATE_CONNECTED;             }
    uint32_t getUptime() const                                  { return startTime == 0 ? 0 : (systemMillis() / 1000) - startTime;}
    bool hasInternetAccess() const                              { return internetAvailable;                                       }
    bool isNetworkConnected() const                             { return networkAvailable;                                        }    
    uint32_t getReconnectCount() const                          { return reconnectCount;                                          }
    uint32_t getMessagesReceived() const                        { return messagesReceived;                                        }
    uint32_t getMessagesPublished() const                       { return messagesPublished;                                       }
    HMS_MQTT_Connection_State getConnectionState() const        { return connectionState;                                         }

  private:
    bool                        initialized;
    bool                        taskRunning;
    bool                        networkAvailable;                                                                                                      // Current network availability status
    bool                        internetAvailable;                                                                                                     // Current internet availability status
    uint32_t                    reconnectCount;
    uint32_t                    messagesPublished;
    uint32_t                    messagesReceived;
    uint32_t                    startTime;
    uint32_t                    lastNetworkCheck;                                                                                                      // Last time network was checked
    std::string                 brokerUri;
    HMS_MQTT_Network_Mode       networkMode;
    HMS_MQTT_Connection_State   connectionState;
    HMS_MQTT_Client_Config      config;

    HMS_MQTT_Status stopMqttTask();
    HMS_MQTT_Status startMqttTask();
    HMS_MQTT_Status setupMqttConfig();
    HMS_MQTT_Status initializePlatform();
    HMS_MQTT_Status deinitializePlatform();

    // Network connectivity methods
    HMS_MQTT_Status checkNetworkInterface();
    HMS_MQTT_Status checkInternetConnectivity();
    HMS_MQTT_Status performNetworkCheck();
    bool shouldCheckNetwork() const;

    void mqttTaskLoop();
    void handleMqttEvent(esp_mqtt_event_handle_t event);
    void updateConnectionState(HMS_MQTT_Connection_State newState);
    void notifyEvent(HMS_MQTT_Event_Type event, const char* info = nullptr);

    HMS_MQTT_Status connect();
    HMS_MQTT_Status disconnect();

    void mqttDelay(uint32_t ms);
    uint32_t systemMillis() const;
    std::string generateClientId();

    HMS_MQTT_Status unsubscribe(const char* topic);
    HMS_MQTT_Status subscribe(const char* topic, HMS_MQTT_QoS qos = HMS_MQTT_QOS_0);
    HMS_MQTT_Status publish(const char* topic, const char* payload, HMS_MQTT_QoS qos = HMS_MQTT_QOS_0, bool retain = false);
    HMS_MQTT_Status publish(const char* topic, const uint8_t* payload, size_t length, HMS_MQTT_QoS qos = HMS_MQTT_QOS_0, bool retain = false);

    #if defined(HMS_MQTT_PLATFORM_ESP_IDF)
      esp_mqtt_client_handle_t    mqttClient;
      esp_mqtt_client_config_t    mqttConfig;
      TaskHandle_t                mqttTaskHandle;
    
      static void mqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
    #endif
};

#endif // HMS_MQTT_H