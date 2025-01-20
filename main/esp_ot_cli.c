/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "nvs_flash.h"
#include "openthread/cli.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"

#include "driver/gpio.h"
#include "openthread/udp.h"
#include "openthread/ip6.h"
#include "openthread/thread.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

#define TAG "ot_esp_cli"

static otUdpSocket sHeartbeatSocket;

// UDP receive callback
static void udp_receive_callback(void *aContext, otMessage *aMessage,
                               const otMessageInfo *aMessageInfo)
{
    if (aMessage == NULL || aMessageInfo == NULL) {
        ESP_LOGE(TAG, "Invalid UDP message received");
        return;
    }

    // Log message details
    uint16_t offset = otMessageGetOffset(aMessage);
    uint16_t length = otMessageGetLength(aMessage);
    ESP_LOGI(TAG, "Received UDP message - Offset: %d, Length: %d", offset, length);

    // Log sender information
    char sender_addr[50];
    otIp6AddressToString(&aMessageInfo->mPeerAddr, sender_addr, sizeof(sender_addr));
    ESP_LOGI(TAG, "Message from: %s:%d", sender_addr, aMessageInfo->mPeerPort);

    if (length <= offset) {
        ESP_LOGE(TAG, "Invalid message length");
        return;
    }

    char buf[128] = {0};  // Ensure null termination
    uint16_t read_len = length - offset;
    
    if (read_len >= sizeof(buf)) {
        ESP_LOGW(TAG, "Received message too long (%d bytes), truncating to %d", read_len, sizeof(buf) - 1);
        read_len = sizeof(buf) - 1;  // Leave room for null terminator
    }

    // Read the message in chunks
    uint16_t bytes_read = 0;
    uint16_t chunk_size = 16; // Read in smaller chunks
    
    while (bytes_read < read_len) {
        uint16_t to_read = (read_len - bytes_read) < chunk_size ? 
                          (read_len - bytes_read) : chunk_size;
                          
        otError error = otMessageRead(aMessage, 
                                    offset + bytes_read, 
                                    buf + bytes_read, 
                                    to_read);
                                    
        if (error != OT_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to read chunk at offset %d: %s (%d)", 
                     bytes_read, otThreadErrorToString(error), error);
            return;
        }
        
        bytes_read += to_read;
    }

    buf[bytes_read] = '\0';  // Ensure null termination
    ESP_LOGI(TAG, "Received UDP message: %s", buf);
}

static void init_udp_socket(otInstance *instance)
{
    if (instance == NULL) {
        ESP_LOGE(TAG, "Invalid OpenThread instance");
        return;
    }

    otError err;
    otSockAddr bindAddr;
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.mPort = 1234;

    err = otUdpOpen(instance, &sHeartbeatSocket, udp_receive_callback, NULL);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open UDP socket: %d", err);
        return;
    }

    err = otUdpBind(instance, &sHeartbeatSocket, &bindAddr, OT_NETIF_UNSPECIFIED);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to bind UDP socket: %d", err);
        otUdpClose(instance, &sHeartbeatSocket);
        return;
    }
    
    ESP_LOGI(TAG, "UDP socket initialized successfully");
}

static void send_heartbeat(otInstance *instance)
{
    if (instance == NULL) {
        ESP_LOGE(TAG, "Invalid OpenThread instance");
        return;
    }

    otError err;
    otMessage *message = NULL;
    otMessageInfo messageInfo;
    
    // Add detailed network state debugging
    bool ip6_enabled = otIp6IsEnabled(instance);
    otDeviceRole device_role = otThreadGetDeviceRole(instance);
    const char* role_str;
    
    switch (device_role) {
        case OT_DEVICE_ROLE_DISABLED: role_str = "DISABLED"; break;
        case OT_DEVICE_ROLE_DETACHED: role_str = "DETACHED"; break;
        case OT_DEVICE_ROLE_CHILD: role_str = "CHILD"; break;
        case OT_DEVICE_ROLE_ROUTER: role_str = "ROUTER"; break;
        case OT_DEVICE_ROLE_LEADER: role_str = "LEADER"; break;
        default: role_str = "UNKNOWN"; break;
    }
    
    // Get ML-EID (Mesh-Local EID) address
    const otNetifAddress *ml_eid = otIp6GetUnicastAddresses(instance);
    char ml_eid_str[50] = "unknown";
    while (ml_eid != NULL) {
        if (ml_eid->mAddressOrigin == OT_ADDRESS_ORIGIN_THREAD) {
            otIp6AddressToString(&ml_eid->mAddress, ml_eid_str, sizeof(ml_eid_str));
            break;
        }
        ml_eid = ml_eid->mNext;
    }
    
    ESP_LOGI(TAG, "Thread Status - IPv6: %s, Role: %s, RLOC16: 0x%04x, ML-EID: %s", 
             ip6_enabled ? "ENABLED" : "DISABLED",
             role_str,
             otThreadGetRloc16(instance),
             ml_eid_str);
             
    // Check if Thread interface is up
    if (!ip6_enabled || device_role == OT_DEVICE_ROLE_DISABLED) {
        ESP_LOGW(TAG, "Thread mesh not ready - skipping heartbeat");
        return;
    }

    // Create a new message
    message = otUdpNewMessage(instance, NULL);
    if (!message) {
        ESP_LOGE(TAG, "Failed to allocate message");
        return;
    }

    // Our heartbeat payload - include our ML-EID so others can reach us directly
    char payload[128];
    snprintf(payload, sizeof(payload), "heartbeat from %04x (ML-EID: %s)", 
             otThreadGetRloc16(instance), ml_eid_str);
    ESP_LOGI(TAG, "Sending: %s", payload);
    
    err = otMessageAppend(message, payload, strlen(payload));
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to append heartbeat payload: %d", err);
        otMessageFree(message);
        return;
    }

    memset(&messageInfo, 0, sizeof(messageInfo));
    // Use link-local all-nodes multicast (ff02::1) instead of realm-local
    // This ensures messages stay within direct radio range
    err = otIp6AddressFromString("ff02::1", &messageInfo.mPeerAddr);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to parse destination address: %s", otThreadErrorToString(err));
        otMessageFree(message);
        return;
    }
    messageInfo.mPeerPort = 1234;

    err = otUdpSend(instance, &sHeartbeatSocket, message, &messageInfo);
    if (err != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to send UDP heartbeat: %s (code %d)", 
                 otThreadErrorToString(err), err);
        otMessageFree(message);
    } else {
        ESP_LOGI(TAG, "Link-local heartbeat sent: %s", payload);
    }
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 9),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,        // typical internal pull-up for a BOOT button
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static void heartbeat_task(void *arg)
{
    otInstance *instance = esp_openthread_get_instance();
    TickType_t last_press_time = 0;
    const TickType_t debounce_interval = pdMS_TO_TICKS(500);

    while (1) {
        // Debounce button press
        if (gpio_get_level(9) == 0 && 
            (xTaskGetTickCount() - last_press_time) > debounce_interval) {
            last_press_time = xTaskGetTickCount();
            // Acquire the OpenThread lock, waiting forever if needed
            if (esp_openthread_lock_acquire(portMAX_DELAY)) {
                send_heartbeat(instance);
                esp_openthread_lock_release();
            }

            // Send once per second
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // Not pressed
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}

static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    // The OpenThread log level directly matches ESP log level
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
    // Initialize the OpenThread cli
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif

    esp_netif_t *openthread_netif;
    // Initialize the esp_netif bindings
    openthread_netif = init_openthread_netif(&config);
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

    // after esp_openthread_init(&config) but before launch_mainloop
    button_init();
    init_udp_socket(esp_openthread_get_instance());

    // Start the heartbeat task (slightly lower priority is usually fine)
    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 3, NULL);

    // Run the main loop
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif
#if CONFIG_OPENTHREAD_AUTO_START
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif
    esp_openthread_launch_mainloop();

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Used eventfds:
    // * netif
    // * ot task queue
    // * radio driver
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    xTaskCreate(ot_task_worker, "ot_cli_main", 10240, xTaskGetCurrentTaskHandle(), 5, NULL);
}
