#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

// Refactored Component Headers
#include "wifi_manager.h"
#include "sensor_monitor.h"
#include "servo_controller.h"
#include "web_server.h"

extern "C" void app_main(void) {
    // 1. Initialize Non-Volatile Storage for saves
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. Setup Base Network Event Loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // 3. Boot Modular Components
    wifi_manager_init();
    sensor_monitor_init();
    servo_controller_init();
    web_server_init();

    ESP_LOGI("MAIN", "ESPRobot Boot Sequence Complete!");
}