#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "COBOT";

// --- Pin Definitions ---
#define SERVO_LOW_LEFT_PIN   GPIO_NUM_12
#define SERVO_HIGH_RIGHT_PIN GPIO_NUM_10
#define SERVO_HIGH_LEFT_PIN  GPIO_NUM_11
#define SERVO_LOW_RIGHT_PIN  GPIO_NUM_9

#define TRIG_PIN             GPIO_NUM_4
#define ECHO_PIN             GPIO_NUM_5

// --- LEDC Channels for Servos ---
#define SERVO_LOW_LEFT_CH    LEDC_CHANNEL_0
#define SERVO_HIGH_RIGHT_CH  LEDC_CHANNEL_1
#define SERVO_HIGH_LEFT_CH   LEDC_CHANNEL_2
#define SERVO_LOW_RIGHT_CH   LEDC_CHANNEL_3

// --- Variables ---
int pos = 0;
int pos2 = 0;
int vel = 5; // delay between movements
int algorithm_selected = 6; // select algorithms
int64_t start_time_ms = 0;

// --- Helper Functions ---
long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Wrapper to handle small ms delays properly in FreeRTOS
void delay_ms(uint32_t ms) {
    if (ms >= portTICK_PERIOD_MS) {
        vTaskDelay(ms / portTICK_PERIOD_MS);
    } else {
        esp_rom_delay_us(ms * 1000);
    }
}

// --- Servo Configuration ---
void servo_init(gpio_num_t pin, ledc_channel_t channel) {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_13_BIT; // 8192 max
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 50;  // 50Hz for servos
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num       = pin;
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = channel;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);
}

void servo_write(ledc_channel_t channel, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // 50Hz -> 20ms period. Duty mapping: 0 deg = ~0.5ms (2.5%), 180 deg = ~2.5ms (12.5%)
    // 2.5% of 8192 = 204.  12.5% of 8192 = 1024.
    uint32_t duty = 204 + ((1024 - 204) * angle) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// --- Distance Sensor ---
float readDistance() {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    // Wait for Echo HIGH (Timeout after 50ms)
    int64_t timeout_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - timeout_start > 50000) return 0;
    }

    // Measure Echo duration
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > 50000) break;
    }
    
    int64_t duration = esp_timer_get_time() - echo_start;
    float distance = duration * 0.034 / 2.0;

    ESP_LOGI(TAG, "Distance: %.2f cm", distance);
    return distance;
}

// --- Robot Movements ---
void resetAllServoPos() {
    servo_write(SERVO_LOW_LEFT_CH, 90);
    servo_write(SERVO_HIGH_RIGHT_CH, 90);
    servo_write(SERVO_HIGH_LEFT_CH, 90);
    servo_write(SERVO_LOW_RIGHT_CH, 90);
}

void hello() {
    servo_write(SERVO_LOW_RIGHT_CH, 90);
    servo_write(SERVO_LOW_LEFT_CH, 90);
    servo_write(SERVO_HIGH_RIGHT_CH, 110);
    servo_write(SERVO_HIGH_LEFT_CH, 0);
    delay_ms(300);
    servo_write(SERVO_HIGH_LEFT_CH, 40);
    delay_ms(300);
}

void littleJump() {
    servo_write(SERVO_LOW_RIGHT_CH, 180);
    servo_write(SERVO_LOW_LEFT_CH, 0);
    servo_write(SERVO_HIGH_RIGHT_CH, 0);
    servo_write(SERVO_HIGH_LEFT_CH, 180);
    delay_ms(1000);

    servo_write(SERVO_LOW_RIGHT_CH, 90);
    servo_write(SERVO_LOW_LEFT_CH, 90);
    servo_write(SERVO_HIGH_RIGHT_CH, 90);
    servo_write(SERVO_HIGH_LEFT_CH, 90);
    delay_ms(100);
}

void bow() {
    servo_write(SERVO_LOW_RIGHT_CH, 90);
    servo_write(SERVO_LOW_LEFT_CH, 90);
    servo_write(SERVO_HIGH_RIGHT_CH, 90);
    servo_write(SERVO_HIGH_LEFT_CH, 90);
    delay_ms(1000);

    servo_write(SERVO_HIGH_RIGHT_CH, 0);
    servo_write(SERVO_HIGH_LEFT_CH, 180);
    delay_ms(2000);

    servo_write(SERVO_HIGH_RIGHT_CH, 90);
    servo_write(SERVO_HIGH_LEFT_CH, 90);
    delay_ms(4000);
}

void alternateWalk() {
    for (pos = 0; pos <= 180; pos += 1) {
        pos2 = map_val(pos, 0, 180, 180, 0);
        servo_write(SERVO_LOW_RIGHT_CH, pos2);
        servo_write(SERVO_HIGH_RIGHT_CH, pos);
        servo_write(SERVO_LOW_LEFT_CH, pos2);
        servo_write(SERVO_HIGH_LEFT_CH, pos);
        delay_ms(vel);
    }
    for (pos = 180; pos >= 0; pos -= 1) {
        pos2 = map_val(pos, 180, 0, 0, 180);
        servo_write(SERVO_LOW_RIGHT_CH, pos);
        servo_write(SERVO_HIGH_RIGHT_CH, pos2);
        servo_write(SERVO_LOW_LEFT_CH, pos);
        servo_write(SERVO_HIGH_LEFT_CH, pos2);
        delay_ms(vel);
    }
}

void highAndLowSynchroWalk() {
    for (pos = 180; pos >= 0; pos -= 1) {
        pos2 = map_val(pos, 180, 0, 0, 180);
        servo_write(SERVO_LOW_RIGHT_CH, pos2);
        servo_write(SERVO_LOW_LEFT_CH, pos);
        delay_ms(vel);
    }
    delay_ms(500);
    for (pos = 0; pos <= 180; pos += 1) {
        pos2 = map_val(pos, 0, 180, 180, 0);
        servo_write(SERVO_LOW_RIGHT_CH, pos2);
        servo_write(SERVO_LOW_LEFT_CH, pos);
        servo_write(SERVO_HIGH_RIGHT_CH, pos);
        servo_write(SERVO_HIGH_LEFT_CH, pos2);
        delay_ms(vel);
    }
    for (pos = 180; pos >= 0; pos -= 1) {
        pos2 = map_val(pos, 180, 0, 0, 180);
        servo_write(SERVO_HIGH_RIGHT_CH, pos);
        servo_write(SERVO_HIGH_LEFT_CH, pos2);
        delay_ms(vel);
    }
}

// --- Main Application ---
extern "C" void app_main() {
    ESP_LOGI(TAG, "Initializing...");

    // Setup Ultrasonic Sensor Pins
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    // Setup Servo Pins
    servo_init(SERVO_LOW_LEFT_PIN, SERVO_LOW_LEFT_CH);
    servo_init(SERVO_HIGH_RIGHT_PIN, SERVO_HIGH_RIGHT_CH);
    servo_init(SERVO_HIGH_LEFT_PIN, SERVO_HIGH_LEFT_CH);
    servo_init(SERVO_LOW_RIGHT_PIN, SERVO_LOW_RIGHT_CH);

    ESP_LOGI(TAG, "Moving all servo to standard position ...");
    resetAllServoPos();
    
    // Wait 1 sec then start algorithm
    delay_ms(1000);
    ESP_LOGI(TAG, "done");
    
    ESP_LOGI(TAG, "Start Timer ...");
    start_time_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Starting loop algorithm");

    while (1) {
        if (algorithm_selected == 0) {
            resetAllServoPos();
        } else if (algorithm_selected == 1) {
            alternateWalk();
        } else if (algorithm_selected == 2) {
            highAndLowSynchroWalk();
        } else if (algorithm_selected == 3) {
            littleJump();
        } else if (algorithm_selected == 4) {
            bow();
        } else if (algorithm_selected == 5) {
            hello();
        } else if (algorithm_selected == 6) {
            int64_t current_time = (esp_timer_get_time() / 1000) - start_time_ms;
            
            if (current_time < 3000) {
                hello();
            } else if (current_time < 6000) {
                littleJump();
            } else {
                resetAllServoPos();
            }
        }
        
        // Feed the watchdog to prevent Task WDT trigger
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}
