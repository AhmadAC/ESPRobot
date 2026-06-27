#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

// Modular Component Headers
#include "wifi_manager.h"
#include "sensor_monitor.h"
#include "servo_controller.h"
#include "web_server.h"
#include "ble_manager.h"

static const char *TAG = "MAIN";

// FreeRTOS task to monitor standard input for REPL control characters
static void console_read_task(void *pvParameter) {
    ESP_LOGI("REPL", "Native REPL Console Listener Task started.");
    
    int fd = fileno(stdin);
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    uint8_t buf[64];
    char cmd[64];
    int cmd_idx = 0;
    
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
                } else if (buf[i] == '\r' || buf[i] == '\n') {
                    if (cmd_idx > 0) {
                        cmd[cmd_idx] = '\0';
                        if (strcmp(cmd, "reset") == 0) {
                            ESP_LOGW("REPL", "Command 'reset' received. Factory Resetting NVS...");
                            nvs_handle_t h;
                            if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
                                nvs_erase_all(h);
                                nvs_commit(h);
                                nvs_close(h);
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            esp_restart();
                        } else if (strcmp(cmd, "bt") == 0) {
                            ESP_LOGW("REPL", "Command 'bt' received. Switching to Bluetooth Mode...");
                            nvs_handle_t h;
                            if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
                                nvs_set_str(h, "boot_mode", "bt");
                                nvs_commit(h);
                                nvs_close(h);
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            esp_restart();
                        } else if (strcmp(cmd, "wifi") == 0) {
                            ESP_LOGW("REPL", "Command 'wifi' received. Switching to Wi-Fi Mode...");
                            nvs_handle_t h;
                            if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
                                nvs_set_str(h, "boot_mode", "wifi");
                                nvs_commit(h);
                                nvs_close(h);
                            }
                            vTaskDelay(pdMS_TO_TICKS(500));
                            esp_restart();
                        }
                        cmd_idx = 0;
                    }
                } else {
                    if (cmd_idx < sizeof(cmd) - 1) {
                        cmd[cmd_idx++] = (char)buf[i];
                    }
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

    // 3. Detect Saved Boot Mode
    nvs_handle_t my_handle;
    char boot_mode[16] = "wifi";
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        size_t len = sizeof(boot_mode);
        nvs_get_str(my_handle, "boot_mode", boot_mode, &len);
        nvs_close(my_handle);
    }

    // 4. Boot Modular Components
    sensor_monitor_init();
    servo_controller_init();
    
    if (strcmp(boot_mode, "bt") == 0) {
        ESP_LOGI(TAG, "Booting in BLUETOOTH Mode. (Wi-Fi Disabled)");
        ble_manager_init();
    } else {
        ESP_LOGI(TAG, "Booting in WI-FI Mode.");
        wifi_manager_init();
        web_server_init();
    }

    // 5. Start standard input keyboard listener for REPL commands
    xTaskCreate(console_read_task, "console_read_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ESPRobot Boot Sequence Complete!");
}