#################### START OF FILE: main\wifi_manager.cpp ####################
#include "wifi_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs.h"
#include "cJSON.h"

static const char *TAG = "WIFI_MGR";
static bool s_reconnect = true;

// Wi-Fi Event Handler to automatically connect and reconnect when the station is ready or dropped
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_config_t conf;
        esp_wifi_get_config(WIFI_IF_STA, &conf);
        if (strlen((char*)conf.sta.ssid) > 0) {
            ESP_LOGI(TAG, "Station started, connecting to %s...", conf.sta.ssid);
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_reconnect) {
            wifi_config_t conf;
            esp_wifi_get_config(WIFI_IF_STA, &conf);
            if (strlen((char*)conf.sta.ssid) > 0) {
                ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying...");
                esp_wifi_connect();
            }
        } else {
            ESP_LOGI(TAG, "Disconnected. Auto-reconnect disabled temporarily.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_manager_init() {
    // Create underlying default Wi-Fi network interfaces
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    // Configure both station and AP Mode config profiles
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "ESPRobot_Config");
    ap_config.ap.ssid_len = strlen("ESPRobot_Config");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN; 

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // Auto-Connect Station if configurations exist
    nvs_handle_t my_handle;
    bool has_sta_config = false;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        char ssid[33] = {0}; char pass[65] = {0};
        size_t s_len = sizeof(ssid); size_t p_len = sizeof(pass);
        if (nvs_get_str(my_handle, "wifi_ssid", ssid, &s_len) == ESP_OK &&
            nvs_get_str(my_handle, "wifi_pass", pass, &p_len) == ESP_OK) {
            
            ESP_LOGI(TAG, "Loaded saved network: %s", ssid);
            wifi_config_t sta_config = {};
            strcpy((char*)sta_config.sta.ssid, ssid);
            strcpy((char*)sta_config.sta.password, pass);
            
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
            has_sta_config = true;
        }
        nvs_close(my_handle);
    }
    
    if (!has_sta_config) {
        ESP_LOGW(TAG, "No Wi-Fi credentials found. AP Mode only.");
    }

    // Start Wi-Fi (this will trigger WIFI_EVENT_STA_START and auto-connect cleanly)
    ESP_ERROR_CHECK(esp_wifi_start());
}

char* wifi_scan_networks_json() {
    // Disable auto-reconnect temporarily to prevent ESP_ERR_WIFI_BUSY locks
    s_reconnect = false;
    
    // Disconnect from active background STA connects to avoid lockup errors
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_wifi_scan_stop();
    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true;
    
    cJSON *root = cJSON_CreateArray();
    esp_err_t scan_err = esp_wifi_scan_start(&scan_config, true);
    
    if (scan_err == ESP_OK) {
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        
        if (ap_count > 0) {
            // Cap count at a safe maximum value to prevent stack overflows
            if (ap_count > 30) {
                ap_count = 30;
            }
            wifi_ap_record_t *ap_info = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
            if (ap_info != NULL) {
                if (esp_wifi_scan_get_ap_records(&ap_count, ap_info) == ESP_OK) {
                    for(int i = 0; i < ap_count; i++) {
                        if (strlen((char*)ap_info[i].ssid) > 0) {
                            cJSON_AddItemToArray(root, cJSON_CreateString((char*)ap_info[i].ssid));
                        }
                    }
                }
                free(ap_info);
            }
        }
    } else {
        ESP_LOGE(TAG, "Wi-Fi Scan failed to start: %s", esp_err_to_name(scan_err));
    }
    
    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); 

    // Re-enable auto-reconnect and connect if we have a config
    s_reconnect = true;
    wifi_config_t conf;
    if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
        if (strlen((char*)conf.sta.ssid) > 0) {
            ESP_LOGI(TAG, "Scan complete. Reconnecting to %s...", conf.sta.ssid);
            esp_wifi_connect();
        }
    }

    return json_str;
}

void wifi_save_credentials(const char* ssid, const char* pass) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_str(my_handle, "wifi_ssid", ssid);
        nvs_set_str(my_handle, "wifi_pass", pass);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Saved SSID: %s", ssid);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS to save credentials!");
    }
}