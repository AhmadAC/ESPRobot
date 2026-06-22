#include "wifi_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "cJSON.h"

static const char *TAG = "WIFI_MGR";

void wifi_manager_init() {
    // Create underlying default Wi-Fi network interfaces
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Configure both station and AP Mode config profiles
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "ESPRobot_Config");
    ap_config.ap.ssid_len = strlen("ESPRobot_Config");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN; 

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Auto-Connect Station if configurations exist
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        char ssid[32]; char pass[64];
        size_t s_len = sizeof(ssid); size_t p_len = sizeof(pass);
        if (nvs_get_str(my_handle, "wifi_ssid", ssid, &s_len) == ESP_OK &&
            nvs_get_str(my_handle, "wifi_pass", pass, &p_len) == ESP_OK) {
            
            ESP_LOGI(TAG, "Connecting to saved network: %s", ssid);
            wifi_config_t sta_config = {};
            strcpy((char*)sta_config.sta.ssid, ssid);
            strcpy((char*)sta_config.sta.password, pass);
            
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "No Wi-Fi credentials found. AP Mode only.");
        }
        nvs_close(my_handle);
    }
}

char* wifi_scan_networks_json() {
    esp_wifi_scan_stop();
    wifi_scan_config_t scan_config = {};
    esp_wifi_scan_start(&scan_config, true);
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    wifi_ap_record_t *ap_info = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
    esp_wifi_scan_get_ap_records(&ap_count, ap_info);

    cJSON *root = cJSON_CreateArray();
    for(int i = 0; i < ap_count; i++) {
        cJSON_AddItemToArray(root, cJSON_CreateString((char*)ap_info[i].ssid));
    }
    char* json_str = cJSON_PrintUnformatted(root);
    
    free(ap_info); 
    cJSON_Delete(root); 
    return json_str;
}

void wifi_save_credentials(const char* ssid, const char* pass) {
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_set_str(my_handle, "wifi_ssid", ssid);
    nvs_set_str(my_handle, "wifi_pass", pass);
    nvs_commit(my_handle);
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Saved SSID: %s", ssid);
}