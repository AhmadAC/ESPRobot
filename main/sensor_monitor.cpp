#include "sensor_monitor.h"
#include "servo_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "SENSOR";

#define ULTRASONIC_TRIG_PIN GPIO_NUM_4
#define ULTRASONIC_ECHO_PIN GPIO_NUM_5

static bool sensor_enabled = false; 
static bool safety_lock_engaged = false;
static int32_t distance_threshold = 20; 
static float current_distance = -1.0f; 

// Dynamic response buffers
static char tripped_action[16] = "stop";
static char cleared_action[16] = "stand";

static void load_sensor_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_i32(my_handle, "sens_thresh", &distance_threshold);
        
        size_t len = sizeof(tripped_action);
        nvs_get_str(my_handle, "act_trip", tripped_action, &len);
        
        len = sizeof(cleared_action);
        nvs_get_str(my_handle, "act_clear", cleared_action, &len);
        
        nvs_close(my_handle);
    }
}

static float read_ultrasonic_distance() {
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    int64_t start_time = esp_timer_get_time();
    int64_t timeout = 15000; // ~2.5 meters max range
    
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_time > timeout) return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > timeout) return -1.0f;
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t duration = echo_end - echo_start;
    return (float)duration / 58.0f;
}

static void ultrasonic_safety_task(void *pvParameter) {
    bool last_lock_state = false;

    while (1) {
        if (sensor_enabled) {
            float dist = read_ultrasonic_distance();
            current_distance = dist;

            if (dist > 0 && dist < distance_threshold) {
                safety_lock_engaged = true;
            } else {
                safety_lock_engaged = false;
            }
        } else {
            safety_lock_engaged = false;
            current_distance = -1.0f;
        }

        // State Machine State Change Hook
        if (safety_lock_engaged != last_lock_state) {
            if (safety_lock_engaged) {
                ESP_LOGW(TAG, "Safety Lock ENGAGED! Obstacle at %.1f cm. Running action: %s", current_distance, tripped_action);
                servo_set_action_bypass(tripped_action);
            } else {
                ESP_LOGI(TAG, "Safety Lock RELEASED. Resuming control. Running action: %s", cleared_action);
                servo_set_action_bypass(cleared_action);
            }
            last_lock_state = safety_lock_engaged;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sensor_monitor_init() {
    load_sensor_nvs();

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRASONIC_TRIG_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRASONIC_ECHO_PIN);
    gpio_config(&io_conf);
    
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    // Pinned strictly to Core 0 to prevent interference with Core 1 operations (Web server / Wi-Fi)
    xTaskCreatePinnedToCore(ultrasonic_safety_task, "ultrasonic_task", 4096, NULL, 5, NULL, 0);
}

bool sensor_is_enabled() { return sensor_enabled; }
void sensor_set_enabled(bool enabled) { sensor_enabled = enabled; }
int32_t sensor_get_threshold() { return distance_threshold; }

void sensor_set_threshold(int32_t threshold) { 
    distance_threshold = threshold; 
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, "sens_thresh", distance_threshold);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

float sensor_get_distance() { return current_distance; }
bool sensor_is_safety_locked() { return safety_lock_engaged; }

const char* sensor_get_tripped_action() { return tripped_action; }
const char* sensor_get_cleared_action() { return cleared_action; }

void sensor_set_actions(const char* tripped, const char* cleared) {
    strncpy(tripped_action, tripped, sizeof(tripped_action) - 1);
    tripped_action[sizeof(tripped_action) - 1] = '\0';
    
    strncpy(cleared_action, cleared, sizeof(cleared_action) - 1);
    cleared_action[sizeof(cleared_action) - 1] = '\0';

    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_str(my_handle, "act_trip", tripped_action);
        nvs_set_str(my_handle, "act_clear", cleared_action);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}