#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

// Modular Component Headers
#include "wifi_manager.h"
#include "sensor_monitor.h"
#include "servo_controller.h"
#include "web_server.h"

static const char *TAG = "MAIN";

// FreeRTOS task to monitor standard input for REPL control characters
static void console_read_task(void *pvParameter) {
    ESP_LOGI("REPL", "Native REPL Console Listener Task started.");
    
    int fd = fileno(stdin);
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    uint8_t buf[64];
    while (1) {
        int len = read(fd, buf, sizeof(buf));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (buf[i] == 0x03) { // Ctrl+C (Interrupt)
                    ESP_LOGW("REPL", "[Sent Ctrl+C - Interrupt]");
                    servo_set_action("stop");
                } else if (buf[i] == 0x04) { // Ctrl+D (Soft Reboot)
                    ESP_LOGW("REPL", "[Sent Ctrl+D - Soft Reboot]");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main(void) {
    // 1. Initialize Non-Volatile Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. Setup Base Network Event Loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. Boot Modular Components
    wifi_manager_init();
    sensor_monitor_init();
    servo_controller_init();
    web_server_init();

    // 4. Start standard input keyboard listener for Ctrl+C and Ctrl+D
    xTaskCreate(console_read_task, "console_read_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ESPRobot Boot Sequence Complete!");
}