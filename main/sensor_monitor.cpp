#include "sensor_monitor.h"
#include "servo_controller.h"
#include "audio_player.h"
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
static int32_t reaction_delay = 0;
static float current_distance = -1.0f; 

// Dynamic response buffers
static char tripped_action[16] = "stop";
static char cleared_action[16] = "stand";
static char tripped_audio[16] = "none";
static char cleared_audio[16] = "none";

static void load_sensor_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_i32(my_handle, "sens_thresh", &distance_threshold);
        nvs_get_i32(my_handle, "sens_react", &reaction_delay);
        
        size_t len = sizeof(tripped_action);
        nvs_get_str(my_handle, "act_trip", tripped_action, &len);
        
        len = sizeof(cleared_action);
        nvs_get_str(my_handle, "act_clear", cleared_action, &len);
        
        len = sizeof(tripped_audio);
        nvs_get_str(my_handle, "aud_trip", tripped_audio, &len);
        
        len = sizeof(cleared_audio);
        nvs_get_str(my_handle, "aud_clear", cleared_audio, &len);
        
        nvs_close(my_handle);
    }
}

static float read_ultrasonic_distance() {
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    esp_rom_delay_us(4);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    esp_rom_delay_us(15); 
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    int64_t start_time = esp_timer_get_time();
    int64_t start_timeout = 40000; 
    
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_time > start_timeout) return -1.0f; 
    }

    int64_t echo_start = esp_timer_get_time();
    int64_t echo_timeout = 40000; 
    
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > echo_timeout) return -1.0f; 
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t duration = echo_end - echo_start;
    float distance = (float)duration / 58.0f;
    
    if (distance > 400.0f || distance < 2.0f) return -1.0f;
    
    return distance;
}

static void ultrasonic_safety_task(void *pvParameter) {
    bool last_lock_state = false;
    int64_t last_detection_time = 0;

    while (1) {
        if (sensor_enabled) {
            float dist = read_ultrasonic_distance();
            current_distance = dist;

            if (dist > 0 && dist < distance_threshold) {
                safety_lock_engaged = true;
                last_detection_time = esp_timer_get_time();
            } else {
                if (safety_lock_engaged && (esp_timer_get_time() - last_detection_time > 1000000)) {
                    safety_lock_engaged = false;
                }
            }
        } else {
            safety_lock_engaged = false;
            current_distance = -1.0f;
        }

        // State Machine State Change Hook
        if (safety_lock_engaged != last_lock_state) {
            if (reaction_delay > 0) {
                vTaskDelay(pdMS_TO_TICKS(reaction_delay));
            }
            if (safety_lock_engaged) {
                ESP_LOGW(TAG, "Lock ENGAGED! Running Action: %s | Audio: %s", tripped_action, tripped_audio);
                if (strcmp(tripped_audio, "none") != 0) audio_play(tripped_audio);
                servo_set_action_bypass(tripped_action);
            } else {
                ESP_LOGI(TAG, "Lock RELEASED. Running Action: %s | Audio: %s", cleared_action, cleared_audio);
                if (strcmp(cleared_audio, "none") != 0) audio_play(cleared_audio);
                servo_set_action_bypass(cleared_action);
            }
            last_lock_state = safety_lock_engaged;
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void sensor_monitor_init() {
    load_sensor_nvs();

    gpio_reset_pin(ULTRASONIC_TRIG_PIN);
    gpio_set_direction(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    gpio_reset_pin(ULTRASONIC_ECHO_PIN);
    gpio_set_direction(ULTRASONIC_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ULTRASONIC_ECHO_PIN, GPIO_FLOATING);

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

int32_t sensor_get_reaction_time() { return reaction_delay; }

void sensor_set_reaction_time(int32_t time_ms) {
    reaction_delay = time_ms;
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, "sens_react", reaction_delay);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

float sensor_get_distance() { return current_distance; }
bool sensor_is_safety_locked() { return safety_lock_engaged; }

const char* sensor_get_tripped_action() { return tripped_action; }
const char* sensor_get_cleared_action() { return cleared_action; }
const char* sensor_get_tripped_audio() { return tripped_audio; }
const char* sensor_get_cleared_audio() { return cleared_audio; }

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

void sensor_set_audio_actions(const char* tripped, const char* cleared) {
    strncpy(tripped_audio, tripped, sizeof(tripped_audio) - 1);
    tripped_audio[sizeof(tripped_audio) - 1] = '\0';
    
    strncpy(cleared_audio, cleared, sizeof(cleared_audio) - 1);
    cleared_audio[sizeof(cleared_audio) - 1] = '\0';

    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_str(my_handle, "aud_trip", tripped_audio);
        nvs_set_str(my_handle, "aud_clear", cleared_audio);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}