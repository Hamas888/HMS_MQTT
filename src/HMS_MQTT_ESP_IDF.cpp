#include "HMS_MQTT.h"

#if defined(HMS_MQTT_PLATFORM_ESP_IDF)

#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"


void HMS_MQTT::mqttTaskLoop() {
    #if HMS_MQTT_DEBUG
    mqttLogger->info("MQTT task started");
    #endif
    
    // Initial network check
    if (config.enableNetworkCheck) {
        HMS_MQTT_Status networkStatus = performNetworkCheck();
        if (networkStatus != HMS_MQTT_OK || !networkAvailable) {
            #if HMS_MQTT_DEBUG
                mqttLogger->warn("Network not available at startup, waiting...");
            #endif
            updateConnectionState(HMS_MQTT_STATE_WAITING_FOR_NETWORK);
        }
    }
    
    // Start the MQTT client only if network is available
    if (!config.enableNetworkCheck || (networkAvailable && (!config.enableInternetCheck || internetAvailable))) {
        esp_err_t err = esp_mqtt_client_start(mqttClient);
        if (err != ESP_OK) {
            #if HMS_MQTT_DEBUG
                mqttLogger->error("Failed to start MQTT client: %s", esp_err_to_name(err));
            #endif
            updateConnectionState(HMS_MQTT_STATE_ERROR);
            vTaskDelete(nullptr);
            return;
        }
        updateConnectionState(HMS_MQTT_STATE_CONNECTING);
    }
    
    // Task main loop
    while (taskRunning) {
        // Check network connectivity periodically
        if (config.enableNetworkCheck && shouldCheckNetwork()) {
            HMS_MQTT_Status networkStatus = performNetworkCheck();
            
            if (networkStatus != HMS_MQTT_OK || !networkAvailable) {
                if (config.pauseOnNetworkLoss && connectionState != HMS_MQTT_STATE_NETWORK_UNAVAILABLE && connectionState != HMS_MQTT_STATE_WAITING_FOR_NETWORK) {
                    #if HMS_MQTT_DEBUG
                        mqttLogger->warn("Network lost, pausing MQTT operations");
                    #endif
                    updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
                    esp_mqtt_client_stop(mqttClient);
                }
            } else if (config.enableInternetCheck && !internetAvailable) {
                if (config.pauseOnNetworkLoss && connectionState != HMS_MQTT_STATE_NETWORK_UNAVAILABLE && connectionState != HMS_MQTT_STATE_WAITING_FOR_NETWORK) {
                    #if HMS_MQTT_DEBUG
                        mqttLogger->warn("Internet access lost, pausing MQTT operations");
                    #endif
                    updateConnectionState(HMS_MQTT_STATE_NETWORK_UNAVAILABLE);
                    esp_mqtt_client_stop(mqttClient);
                }
            } else {
                // Network is available, resume MQTT operations if paused
                if (connectionState == HMS_MQTT_STATE_NETWORK_UNAVAILABLE || connectionState == HMS_MQTT_STATE_WAITING_FOR_NETWORK) {
                    #if HMS_MQTT_DEBUG
                        mqttLogger->info("Network restored, resuming MQTT operations");
                    #endif
                    esp_err_t err = esp_mqtt_client_start(mqttClient);
                    if (err == ESP_OK) {
                        updateConnectionState(HMS_MQTT_STATE_CONNECTING);
                    } else {
                        #if HMS_MQTT_DEBUG
                            mqttLogger->error("Failed to restart MQTT client: %s", esp_err_to_name(err));
                        #endif
                        updateConnectionState(HMS_MQTT_STATE_ERROR);
                    }
                }
            }
        }
        
        // Handle reconnection logic
        if (config.autoReconnect && connectionState == HMS_MQTT_STATE_DISCONNECTED) {
            // Only attempt reconnection if network is available
            if (!config.enableNetworkCheck || (networkAvailable && (!config.enableInternetCheck || internetAvailable))) {
                updateConnectionState(HMS_MQTT_STATE_RECONNECTING);
                
                // Exponential backoff for reconnection
                uint32_t shift_count = (reconnectCount > 6u) ? 6u : reconnectCount;
                uint32_t backoff_delay = config.reconnectTimeoutMs * (1u << shift_count);
                uint32_t delay = (backoff_delay > HMS_MQTT_MAX_RECONNECT_DELAY) ? HMS_MQTT_MAX_RECONNECT_DELAY : backoff_delay;
                
                #if HMS_MQTT_DEBUG
                    mqttLogger->info("Attempting reconnection in %" PRIu32 " ms (attempt %" PRIu32 ")", delay, reconnectCount + 1);
                #endif
                
                // Interruptible delay
                uint32_t elapsed = 0;
                while (elapsed < delay && taskRunning) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    elapsed += 100;
                }

                if (taskRunning) {
                    esp_err_t err = esp_mqtt_client_reconnect(mqttClient);
                    if (err != ESP_OK) {
                        #if HMS_MQTT_DEBUG
                            mqttLogger->error("Reconnection failed: %s", esp_err_to_name(err));
                        #endif
                        reconnectCount++;
                    }
                }
            } else {
                // Network not available, wait for network
                updateConnectionState(HMS_MQTT_STATE_WAITING_FOR_NETWORK);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms for faster shutdown
    }
    
    #if HMS_MQTT_DEBUG
        mqttLogger->info("MQTT task ended");
    #endif
    
    mqttTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

HMS_MQTT_Status HMS_MQTT::connect() {
    if (!initialized) {
        return HMS_MQTT_ERR_INIT;
    }
    
    if (connectionState == HMS_MQTT_STATE_CONNECTED) {
        return HMS_MQTT_OK;
    }
    
    updateConnectionState(HMS_MQTT_STATE_CONNECTING);
    
    esp_err_t err = esp_mqtt_client_reconnect(mqttClient);
    if (err != ESP_OK) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Failed to connect: %s", esp_err_to_name(err));
        #endif
        updateConnectionState(HMS_MQTT_STATE_ERROR);
        return HMS_MQTT_ERR_CONNECT;
    }
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::disconnect() {
    if (connectionState == HMS_MQTT_STATE_DISCONNECTED) {
        return HMS_MQTT_OK;
    }
    
    esp_err_t err = esp_mqtt_client_disconnect(mqttClient);
    if (err != ESP_OK) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Failed to disconnect: %s", esp_err_to_name(err));
        #endif
        return HMS_MQTT_ERR_DISCONNECT;
    }
    
    return HMS_MQTT_OK;
}

std::string HMS_MQTT::generateClientId() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "HMS_MQTT_%02X%02X%02X", mac[3], mac[4], mac[5]);
    return clientId;
}

HMS_MQTT_Status HMS_MQTT::stopMqttTask() {
    if (mqttTaskHandle != nullptr) {
        taskRunning = false;
        // Wait for task to exit gracefully
        int timeout = 100; // 1000ms
        while (mqttTaskHandle != nullptr && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (mqttTaskHandle != nullptr) {
            #if HMS_MQTT_DEBUG
            mqttLogger->warn("Forcing MQTT task deletion");
            #endif
            vTaskDelete(mqttTaskHandle);
            mqttTaskHandle = nullptr;
        }
    }
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::startMqttTask() {
    if (mqttTaskHandle != nullptr) {
        return HMS_MQTT_OK;
    }
    
    BaseType_t result = xTaskCreate(
        [](void* param) {
            HMS_MQTT* mqttInstance = static_cast<HMS_MQTT*>(param);
            mqttInstance->mqttTaskLoop();
            vTaskDelete(nullptr);
        },
        "HMS_MQTT_Task",
        config.taskStackSize,
        this,
        config.taskPriority,
        &mqttTaskHandle
    );
    
    if (result != pdPASS) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Failed to create MQTT task");
        #endif
        return HMS_MQTT_ERR_INIT;
    }
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::setupMqttConfig() {
    memset(&mqttConfig, 0, sizeof(mqttConfig));
    
    if (config.brokerEndpoint.find("://") != std::string::npos) {                       // Basic configuration - handle URI formatting
        brokerUri = config.brokerEndpoint;                                              // Already has protocol scheme
    } else {                                                                            // Add protocol scheme
        if (config.useTLS) {
            brokerUri = "mqtts://" + config.brokerEndpoint + ":" + std::to_string(config.brokerPort);
        } else {
            brokerUri = "mqtt://" + config.brokerEndpoint + ":" + std::to_string(config.brokerPort);
        }
    }
    
    mqttConfig.broker.address.uri = brokerUri.c_str();
    
    #if HMS_MQTT_DEBUG
        mqttLogger->debug("Formatted MQTT URI: %s", brokerUri.c_str());
    #endif
    
    mqttConfig.session.keepalive                 = config.keepAliveInterval;
    mqttConfig.session.disable_clean_session     = !config.cleanSession;
    mqttConfig.network.timeout_ms                = config.networkTimeoutMs;
    mqttConfig.network.reconnect_timeout_ms      = config.reconnectTimeoutMs;
    mqttConfig.network.disable_auto_reconnect    = !config.autoReconnect;


    mqttConfig.credentials.client_id = config.clientId.c_str();                          // Client credentials
    if (!config.username.empty()) {
        mqttConfig.credentials.username = config.username.c_str();
    }
    if (!config.password.empty()) {
        mqttConfig.credentials.authentication.password = config.password.c_str();
    }
    
    if (!config.lwtTopic.empty() && !config.lwtMessage.empty()) {                        // Last Will and Testament
        mqttConfig.session.last_will.topic      = config.lwtTopic.c_str();
        mqttConfig.session.last_will.msg        = config.lwtMessage.c_str();
        mqttConfig.session.last_will.msg_len    = config.lwtMessage.length();
        mqttConfig.session.last_will.qos        = config.lwtQoS;
        mqttConfig.session.last_will.retain     = config.lwtRetain;
    }

    if (config.useTLS) {                                                                 // TLS Configuration
        if (!config.caCert.empty()) {
            mqttConfig.broker.verification.certificate = config.caCert.c_str();
        }
        if (!config.clientCert.empty()) {
            mqttConfig.credentials.authentication.certificate = config.clientCert.c_str();
        }
        if (!config.privateKey.empty()) {
            mqttConfig.credentials.authentication.key = config.privateKey.c_str();
        }
        mqttConfig.broker.verification.skip_cert_common_name_check = false;
    }
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::initializePlatform() {
    if (mqttClient != nullptr) {
        esp_mqtt_client_destroy(mqttClient);
        mqttClient = nullptr;
    }
    
    mqttClient = esp_mqtt_client_init(&mqttConfig);
    if (mqttClient == nullptr) {
        #if HMS_MQTT_DEBUG
        mqttLogger->error("Failed to initialize MQTT client");
        #endif
        return HMS_MQTT_ERR_INIT;
    }
    
    esp_err_t err = esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_ANY, mqttEventHandler, this);
    if (err != ESP_OK) {
        #if HMS_MQTT_DEBUG
        mqttLogger->error("Failed to register MQTT event handler: %s", esp_err_to_name(err));
        #endif
        esp_mqtt_client_destroy(mqttClient);
        mqttClient = nullptr;
        return HMS_MQTT_ERR_INIT;
    }
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::deinitializePlatform() {
    if (mqttClient != nullptr) {
        esp_mqtt_client_destroy(mqttClient);
        mqttClient = nullptr;
    }
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::checkNetworkInterface() {
    if (networkMode == HMS_MQTT_NETWORK_MODE_WIFI) {
        // Check WiFi connectivity
        wifi_ap_record_t ap_info;
        esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
        if (err == ESP_OK) {
            // Check if we have an IP address
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif) {
                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                    #if HMS_MQTT_DEBUG
                        if (!networkAvailable) {
                            mqttLogger->debug("WiFi connected and has IP address: " IPSTR, IP2STR(&ip_info.ip));
                        }
                    #endif
                    networkAvailable = true;
                    return HMS_MQTT_OK;
                }
            }
        }
        
        #if HMS_MQTT_DEBUG
            if (networkAvailable) {
                mqttLogger->debug("WiFi not connected or no IP address");
            }
        #endif
        networkAvailable = false;
        return HMS_MQTT_ERR_NETWORK_UNAVAILABLE;
        
    } else if (networkMode == HMS_MQTT_NETWORK_MODE_ETHERNET) {
        // Check Ethernet connectivity
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("ETH_DEF");
        if (!netif) {
            // Fallback: find any ethernet netif
            netif = esp_netif_next_unsafe(NULL);
            while (netif) {
                const char* ifkey = esp_netif_get_ifkey(netif);
                if (ifkey && strstr(ifkey, "ETH")) {
                    break;
                }
                netif = esp_netif_next_unsafe(netif);
            }
        }
        
        if (netif) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                #if HMS_MQTT_DEBUG
                    if (!networkAvailable) {
                        mqttLogger->debug("Ethernet connected and has IP address: " IPSTR, IP2STR(&ip_info.ip));
                    }
                #endif
                networkAvailable = true;
                return HMS_MQTT_OK;
            }
        }
        
        #if HMS_MQTT_DEBUG
            if (networkAvailable) {
                mqttLogger->debug("Ethernet not connected or no IP address");
            }
        #endif
        networkAvailable = false;
        return HMS_MQTT_ERR_NETWORK_UNAVAILABLE;
    }
    
    return HMS_MQTT_ERR_NETWORK;
}

HMS_MQTT_Status HMS_MQTT::checkInternetConnectivity() {
    if (!networkAvailable) {
        internetAvailable = false;
        return HMS_MQTT_ERR_NETWORK_UNAVAILABLE;
    }
    
    struct addrinfo hints;
    struct addrinfo *result;
    
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    
    int ret = getaddrinfo("google.com", NULL, &hints, &result);
    
    if (ret == 0) {
        #if HMS_MQTT_DEBUG
            if (!internetAvailable) {
                mqttLogger->debug("Internet connectivity confirmed via DNS resolution");
            }
        #endif
        internetAvailable = true;
        freeaddrinfo(result);
        return HMS_MQTT_OK;
    } else {
        #if HMS_MQTT_DEBUG
            if (internetAvailable) {
                mqttLogger->debug("No internet connectivity - DNS resolution failed");
            }
        #endif
        internetAvailable = false;
        return HMS_MQTT_ERR_NO_INTERNET_ACCESS;
    }
}

HMS_MQTT_Status HMS_MQTT::unsubscribe(const char* topic) {
    if (!topic) {
        return HMS_MQTT_ERR_UNSUBSCRIBE;
    }
    
    if (!isConnected()) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Cannot unsubscribe: not connected");
        #endif
        return HMS_MQTT_ERR_CONNECT;
    }
    
    int msg_id = esp_mqtt_client_unsubscribe(mqttClient, topic);
    
    if (msg_id == -1) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Failed to unsubscribe from topic: %s", topic);
        #endif
        return HMS_MQTT_ERR_UNSUBSCRIBE;
    }
    
    #if HMS_MQTT_DEBUG
        mqttLogger->debug("Unsubscribed from topic: %s, msg_id: %d", topic, msg_id);
    #endif
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::subscribe(const char* topic, HMS_MQTT_QoS qos) {
    if (!topic) {
        return HMS_MQTT_ERR_SUBSCRIBE;
    }
    
    if (!isConnected()) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Cannot subscribe: not connected");
        #endif
        return HMS_MQTT_ERR_CONNECT;
    }

    int msg_id = esp_mqtt_client_subscribe(mqttClient, topic, qos);
    
    if (msg_id == -1) {
        #if HMS_MQTT_DEBUG
        mqttLogger->error("Failed to subscribe to topic: %s", topic);
        #endif
        return HMS_MQTT_ERR_SUBSCRIBE;
    }
    
    #if HMS_MQTT_DEBUG
        mqttLogger->info("Subscribed to topic: %s, msg_id: %d", topic, msg_id);
    #endif
    
    return HMS_MQTT_OK;
}

HMS_MQTT_Status HMS_MQTT::publish(const char* topic, const char* payload, HMS_MQTT_QoS qos, bool retain) {
    if (!topic || !payload) {
        return HMS_MQTT_ERR_PUBLISH;
    }
    
    return publish(topic, reinterpret_cast<const uint8_t*>(payload), strlen(payload), qos, retain);
}

void HMS_MQTT::mqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    HMS_MQTT* instance = static_cast<HMS_MQTT*>(handler_args);
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    
    instance->handleMqttEvent(event);
}

HMS_MQTT_Status HMS_MQTT::publish(const char* topic, const uint8_t* payload, size_t length, HMS_MQTT_QoS qos, bool retain) {
    if (!topic || !payload || length == 0) {
        return HMS_MQTT_ERR_PUBLISH;
    }
    
    if (!isConnected()) {
        #if HMS_MQTT_DEBUG
            mqttLogger->debug("Cannot publish: not connected");
        #endif
        return HMS_MQTT_ERR_CONNECT;
    }
    
    int msg_id = esp_mqtt_client_publish(
        mqttClient, topic, reinterpret_cast<const char*>(payload), length, qos, retain
    );
    
    if (msg_id == -1) {
        #if HMS_MQTT_DEBUG
            mqttLogger->error("Failed to publish to topic: %s", topic);
        #endif
        return HMS_MQTT_ERR_PUBLISH;
    }
    
    #if HMS_MQTT_DEBUG
        mqttLogger->debug("Published to topic: %s, msg_id: %d", topic, msg_id);
    #endif
    
    return HMS_MQTT_OK;
}

#endif // HMS_MQTT_PLATFORM_ESP_IDF 