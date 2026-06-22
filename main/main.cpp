#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

// Refactored Component Headers
#include "wifi_manager.h"
#include "sensor_monitor.h"
#include "servo_controller.h"
#include "web_server.h"

static const char *TAG = "MAIN";

// FreeRTOS Task to monitor standard input for REPL control characters
static void console_repl_task(void *pvParameter) {
    // Set standard input to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI("REPL", "REPL Console Listener initialized. Ctrl+C to Stop, Ctrl+D to Soft Reboot.");

    while (1) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == 3) { // Ctrl+C (Interrupt/ETX)
                ESP_LOGW("REPL", "[Sent Ctrl+C - Interrupt]");
                servo_set_action("stop");
            } else if (c == 4) { // Ctrl+D (Soft Reboot/EOT)
                ESP_LOGW("REPL", "[Sent Ctrl+D - Soft Reboot]");
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

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

    // 3. Boot Modular Components
    wifi_manager_init();
    sensor_monitor_init();
    servo_controller_init();
    web_server_init();

    // 4. Start Console/REPL Input Listener Task
    xTaskCreate(console_repl_task, "console_repl_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ESPRobot Boot Sequence Complete!");
}