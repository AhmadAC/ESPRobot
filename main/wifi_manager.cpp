
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

// Background event handler to automatically reconnect if the router drops the connection
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying in 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 second delay prevents spamming the AP
        esp_wifi_connect();
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

    // Register basic event handlers for drops and IP assignment
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    // Configure AP Mode profile
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "ESPRobot_Config");
    ap_config.ap.ssid_len = strlen("ESPRobot_Config");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN; 

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
    // Explicitly mirror the old working code: Start Wi-Fi FIRST before reading NVS
    ESP_ERROR_CHECK(esp_wifi_start());

    // Auto-Connect Station sequentially if configurations exist
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        char ssid[33] = {0}; 
        char pass[65] = {0};
        size_t s_len = sizeof(ssid); 
        size_t p_len = sizeof(pass);
        
        if (nvs_get_str(my_handle, "wifi_ssid", ssid, &s_len) == ESP_OK &&
            nvs_get_str(my_handle, "wifi_pass", pass, &p_len) == ESP_OK) {
            
            ESP_LOGI(TAG, "Connecting to saved network: %s", ssid);
            wifi_config_t sta_config = {};
            
            // Safely copy string payload over to hardware configurations
            strncpy((char*)sta_config.sta.ssid, ssid, 32);
            strncpy((char*)sta_config.sta.password, pass, 64);
            
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "No Wi-Fi credentials found. AP Mode only.");
        }
        nvs_close(my_handle);
    }
}

char* wifi_scan_networks_json() {
    // Explicitly mirror the old working code: Never drop the active connection to scan
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
        ESP_LOGE(TAG, "Wi-Fi Scan failed: %s", esp_err_to_name(scan_err));
    }
    
    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); 
    
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