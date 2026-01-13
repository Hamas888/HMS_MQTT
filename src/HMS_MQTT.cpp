/*
 ============================================================================================================================================
 * File:        HMS_MQTT.cpp
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Nov 4 2025
 * Brief:       This file provides cross-platform MQTT client implementation for embedded & Desktop systems.
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

#include "HMS_MQTT.h"

// #ifdef HMS_MQTT_PLATFORM_ESP_IDF
// #include "esp_log.h"
// #include "esp_system.h"
// #include "esp_event.h"

// #include "esp_mac.h"
// #include "freertos/semphr.h"
// #include "freertos/timers.h"
// #include <cstring>
// #include <algorithm>
// #include <inttypes.h>

#if HMS_MQTT_DEBUG
    ChronoLogger *mqttLogger = new ChronoLogger("HMS_MQTT", HMS_MQTT_LOG_LEVEL);
#endif


HMS_MQTT::HMS_MQTT(HMS_MQTT_Network_Mode networkMode) : 
    initialized(false),
    taskRunning(false),
    networkAvailable(false),
    internetAvailable(false),
    reconnectCount(0),
    messagesPublished(0),
    messagesReceived(0),
    startTime(0),
    lastNetworkCheck(0),
    brokerUri(""),
    networkMode(networkMode), 
    connectionState(HMS_MQTT_STATE_DISCONNECTED) {

    #if defined(HMS_MQTT_PLATFORM_ESP_IDF)
        mqttClient = nullptr;
        mqttTaskHandle = nullptr;
        memset(&mqttConfig, 0, sizeof(mqttConfig));
    #endif
    
    #if HMS_MQTT_DEBUG
        mqttLogger->debug(
            "HMS_MQTT instance created with network mode: %s", 
            (networkMode == HMS_MQTT_NETWORK_MODE_WIFI) ? "WiFi" : "Ethernet"
        );
    #endif
}

HMS_MQTT::~HMS_MQTT() {
    stop();
    
    #if HMS_MQTT_DEBUG
        mqttLogger->debug("HMS_MQTT instance destroyed");
    #endif
}

HMS_MQTT_Status HMS_MQTT::stop() {
    if (!taskRunning) {
        return HMS_MQTT_OK;
    }
    
    updateConnectionState(HMS_MQTT_STATE_DISCONNECTED);
    
    #if defined(HMS_MQTT_PLATFORM_ESP_IDF)
    if (mqttClient) {
        esp_mqtt_client_stop(mqttClient);
    }
    #endif

    HMS_MQTT_Status status = stopMqttTask();
    if (status != HMS_MQTT_OK) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Failed to stop MQTT task");
        #endif
    }
    
    deinitializePlatform();
    taskRunning = false;
    
    #if HMS_MQTT_DEBUG
        mqttLogger->info("MQTT client stopped");
    #endif
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::start() {
    if (!initialized) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("MQTT client not configured");
        #endif
        return HMS_MQTT_ERR_INIT;
    }
    
    if (taskRunning) {
        #if HMS_MQTT_DEBUG
            mqttLogger->warn("MQTT client already running");
        #endif
        return HMS_MQTT_OK;
    }
    
    // Check network connectivity before starting
    if (config.enableNetworkCheck) {
        HMS_MQTT_Status networkStatus = performNetworkCheck();
        if (networkStatus != HMS_MQTT_OK) {
            #if HMS_MQTT_DEBUG
                mqttLogger->error("Network not available, cannot start MQTT client");
            #endif
            updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
            return networkStatus;
        }
        
        if (!networkAvailable) {
            #if HMS_MQTT_DEBUG
                mqttLogger->error("Network interface not connected");
            #endif
            updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
            return HMS_MQTT_ERR_NETWORK_UNAVAILABLE;
        }
        
        if (config.enableInternetCheck && !internetAvailable) {
            #if HMS_MQTT_DEBUG
                mqttLogger->error("No internet access available");
            #endif
            updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
            return HMS_MQTT_ERR_NO_INTERNET_ACCESS;
        }
    }
    
    HMS_MQTT_Status status = initializePlatform();
    if (status != HMS_MQTT_OK) {
        return status;
    }
    
    taskRunning = true;
    status = startMqttTask();
    if (status != HMS_MQTT_OK) {
        taskRunning = false;
        deinitializePlatform();
        return status;
    }
    
    startTime = systemMillis() / 1000;
    
    #if HMS_MQTT_DEBUG
        mqttLogger->info("MQTT client started successfully");
    #endif
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::restart() {
    HMS_MQTT_Status status = stop();
    if (status != HMS_MQTT_OK) {
        return status;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    return start();
}

void HMS_MQTT::mqttDelay(uint32_t ms) {
    #if defined(HMS_MQTT_PLATFORM_ESP_IDF)
        vTaskDelay(pdMS_TO_TICKS(ms));
    #else
        // Implement for other platforms as needed
    #endif
}

uint32_t HMS_MQTT::systemMillis() const {
    #if defined(HMS_MQTT_PLATFORM_ESP_IDF)
        return esp_timer_get_time() / 1000;
    #else
        // Implement for other platforms as needed
        return 0;
    #endif
}

bool HMS_MQTT::shouldCheckNetwork() const {
    if (!config.enableNetworkCheck) {
        return false;
    }
    
    uint32_t currentTime = systemMillis();
    return (currentTime - lastNetworkCheck) >= config.networkCheckIntervalMs;
}

HMS_MQTT_Status HMS_MQTT::performNetworkCheck() {
    HMS_MQTT_Status status = checkNetworkInterface();
    if (status != HMS_MQTT_OK) {
        networkAvailable = false;
        internetAvailable = false;
        return status;
    }
    
    if (config.enableInternetCheck) {
        status = checkInternetConnectivity();
        if (status != HMS_MQTT_OK) {
            internetAvailable = false;
            return status;
        }
    } else {
        internetAvailable = true; // Assume internet is available if check is disabled
    }
    
    lastNetworkCheck = systemMillis();
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::checkNetworkConnectivity() {
    if (!config.enableNetworkCheck) {
        networkAvailable = true;
        internetAvailable = true;
        return HMS_MQTT_OK;
    }
    
    return performNetworkCheck();
}

HMS_MQTT_Status HMS_MQTT::sendMessage(std::string payload) {
    if (!isConnected()) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Cannot send message: MQTT client is not connected");
        #endif
        return HMS_MQTT_ERR_CONNECT;
    }
    
    // Check network connectivity before sending
    if (config.enableNetworkCheck && shouldCheckNetwork()) {
        HMS_MQTT_Status networkStatus = performNetworkCheck();
        if (networkStatus != HMS_MQTT_OK || !networkAvailable) {
            #if HMS_MQTT_DEBUG
                mqttLogger->error("Network unavailable, cannot send message");
            #endif
            if (config.pauseOnNetworkLoss) {
                updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
            }
            return HMS_MQTT_ERR_NETWORK_UNAVAILABLE;
        }
        
        if (config.enableInternetCheck && !internetAvailable) {
            #if HMS_MQTT_DEBUG
                mqttLogger->error("No internet access, cannot send message");
            #endif
            if (config.pauseOnNetworkLoss) {
                updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
            }
            return HMS_MQTT_ERR_NO_INTERNET_ACCESS;
        }
    }

    HMS_MQTT_Status status = publish(config.pubTopic.c_str(), payload.c_str(), config.pubQoS, config.pubRetain);
    if (status == HMS_MQTT_OK) {
        // Don't increment here since publish() already increments the counter
        #if HMS_MQTT_DEBUG
        mqttLogger->debug("Message published to topic '%s': %s", config.pubTopic.c_str(), payload.c_str());
        #endif
    } else {
        #if HMS_MQTT_DEBUG
        mqttLogger->error("Failed to publish message to topic '%s'", config.pubTopic.c_str());
        #endif
    }
    return status;
}

HMS_MQTT_Status HMS_MQTT::waitForNetwork(uint32_t timeoutMs) {
    if (!config.enableNetworkCheck) {
        return HMS_MQTT_OK;
    }
    
    uint32_t startTime = systemMillis();
    
    while ((systemMillis() - startTime) < timeoutMs) {
        HMS_MQTT_Status status = performNetworkCheck();
        if (status == HMS_MQTT_OK && networkAvailable) {
            if (!config.enableInternetCheck || internetAvailable) {
                return HMS_MQTT_OK;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
    
    return networkAvailable ? HMS_MQTT_ERR_NO_INTERNET_ACCESS : HMS_MQTT_ERR_NETWORK_UNAVAILABLE;
}

void HMS_MQTT::handleMqttEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            #if HMS_MQTT_DEBUG
                mqttLogger->info("MQTT client connected");
            #endif
            reconnectCount = 0;
            updateConnectionState(HMS_MQTT_STATE_CONNECTED);
            notifyEvent(HMS_MQTT_EVENT_CONNECTED, "Connected to broker");
            
            if (!config.subTopic.empty()) {
                subscribe(config.subTopic.c_str(), config.subQoS);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            #if HMS_MQTT_DEBUG
                mqttLogger->info("MQTT client disconnected");
            #endif
            updateConnectionState(HMS_MQTT_STATE_DISCONNECTED);
            notifyEvent(HMS_MQTT_EVENT_DISCONNECTED, "Disconnected from broker");
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            #if HMS_MQTT_DEBUG
            mqttLogger->debug("MQTT subscribed, msg_id=%d", event->msg_id);
            #endif
            notifyEvent(HMS_MQTT_EVENT_SUBSCRIBED, "Subscription successful");
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            #if HMS_MQTT_DEBUG
            mqttLogger->debug("MQTT unsubscribed, msg_id=%d", event->msg_id);
            #endif
            notifyEvent(HMS_MQTT_EVENT_UNSUBSCRIBED, "Unsubscription successful");
            break;
            
        case MQTT_EVENT_PUBLISHED:
            #if HMS_MQTT_DEBUG
            mqttLogger->debug("MQTT published, msg_id=%d", event->msg_id);
            #endif
            messagesPublished++;
            notifyEvent(HMS_MQTT_EVENT_PUBLISHED, "Message published");
            break;
            
        case MQTT_EVENT_DATA:
            #if HMS_MQTT_DEBUG
            mqttLogger->debug("MQTT data received, topic=%.*s, payload length=%d", event->topic_len, event->topic, event->data_len);
            #endif
            messagesReceived++;

            if (config.messageCallback) {
                std::string topic(event->topic, event->topic_len);
                std::string payload(static_cast<char*>(event->data), event->data_len);

                config.messageCallback(topic.c_str(), payload.c_str(), event->data_len);
            }
            
            notifyEvent(HMS_MQTT_EVENT_DATA, "Message received");
            break;
            
        case MQTT_EVENT_ERROR:
            #if HMS_MQTT_DEBUG
                mqttLogger->error("MQTT error occurred");
            #endif
            updateConnectionState(HMS_MQTT_STATE_ERROR);
            notifyEvent(HMS_MQTT_EVENT_ERROR, "MQTT error occurred");
            break;
            
        default:
            #if HMS_MQTT_DEBUG
                if (event->event_id > 10) {
                    mqttLogger->warn("Unknown MQTT event: %d", event->event_id);
                }
            #endif
            break;
    }
}

HMS_MQTT_Status HMS_MQTT::configure(HMS_MQTT_Client_Config& config) {
    if (taskRunning) {
        #if HMS_MQTT_DEBUG
        mqttLogger->error("Cannot configure while MQTT client is running");
        #endif
        return HMS_MQTT_ERR_INIT;
    }
    
    if (config.brokerEndpoint.empty()) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Broker endpoint cannot be empty");
        #endif
        return HMS_MQTT_ERR_INVALID_CONFIG;
    }

    if(config.pubTopic.empty() || config.subTopic.empty() || (config.lwtEnabled && config.lwtTopic.empty())) {
        #if HMS_MQTT_DEBUG
            mqttLogger->warn("Publish or Subscribe %s topic is empty", config.lwtEnabled ? " or LWT " : "");
        #endif
        return HMS_MQTT_ERR_INVALID_CONFIG;
    }

    if(config.useTLS) {
        if (config.caCert.empty() || config.clientCert.empty() || config.privateKey.empty()) {
            #if HMS_MQTT_DEBUG
                mqttLogger->warn("TLS is enabled but one or more certificates are empty");
            #endif
            return HMS_MQTT_ERR_INVALID_CONFIG;
        }
    }
    
    if (config.clientId.empty()) {
        #if HMS_MQTT_DEBUG
            mqttLogger->warn("Client ID is empty auto-generating one");
        #endif
        config.clientId = generateClientId();
    }
    
    this->config = config;
    
    if (config.brokerPort == 0) {
        config.brokerPort = config.useTLS ? HMS_MQTT_DEFAULT_TLS_PORT : HMS_MQTT_DEFAULT_PORT;
    }
    
    HMS_MQTT_Status status = setupMqttConfig();
    if (status != HMS_MQTT_OK) {
        return status;
    }
    
    initialized = true;
    
    #if HMS_MQTT_DEBUG
    mqttLogger->info("MQTT client configured successfully");
    mqttLogger->info(
        "Broker: %s:%d, ClientID: %s, TLS: %s", 
        config.brokerEndpoint.c_str(), config.brokerPort, 
        config.clientId.c_str(), config.useTLS ? "Yes" : "No"
    );
    #endif
    
    return HMS_MQTT_OK;
}

void HMS_MQTT::notifyEvent(HMS_MQTT_Event_Type event, const char* info) {
    #if HMS_MQTT_DEBUG
        vTaskDelay(pdMS_TO_TICKS(5)); 
    #endif

    if (config.eventCallback) {
        config.eventCallback(event, info);
    }
}

void HMS_MQTT::updateConnectionState(HMS_MQTT_Connection_State newState) {
    if (connectionState != newState) {
        #if HMS_MQTT_DEBUG
            HMS_MQTT_Connection_State oldState = connectionState;
        #endif
        connectionState = newState;
        
        #if HMS_MQTT_DEBUG
            const char* stateNames[] = {"ERROR", "CONNECTED", "CONNECTING", "DISCONNECTED", "RECONNECTING", "NETWORK_UNAVAILABLE", "WAITING_FOR_NETWORK"};
            mqttLogger->debug("Connection state changed: %s -> %s", stateNames[oldState], stateNames[newState]);
            vTaskDelay(pdMS_TO_TICKS(5));
        #endif
        
        if (config.stateCallback) {
            config.stateCallback(newState);
        }
    }
}