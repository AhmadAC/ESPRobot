#include "servo_controller.h"
#include "sensor_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "SERVO";

#define SERVO_LOW_LEFT_PIN   GPIO_NUM_12 // CH0 (Back Left)
#define SERVO_HIGH_RIGHT_PIN GPIO_NUM_10 // CH1 (Front Right)
#define SERVO_HIGH_LEFT_PIN  GPIO_NUM_11 // CH2 (Front Left)
#define SERVO_LOW_RIGHT_PIN  GPIO_NUM_9  // CH3 (Back Right)

#define SERVO_LOW_LEFT_CH    LEDC_CHANNEL_0
#define SERVO_HIGH_RIGHT_CH  LEDC_CHANNEL_1
#define SERVO_HIGH_LEFT_CH   LEDC_CHANNEL_2
#define SERVO_LOW_RIGHT_CH   LEDC_CHANNEL_3

// State Variables
static int32_t target_low_left   = 90;
static int32_t target_high_right = 90;
static int32_t target_high_left  = 90;
static int32_t target_low_right  = 90;

static int32_t offset_low_left   = 0;
static int32_t offset_high_right = 0;
static int32_t offset_high_left  = 0;
static int32_t offset_low_right  = 0;

static int active_animation = 0; // 0=None, 1=Walk Forward, 2=Walk Back

static void write_servo_calibrated(ledc_channel_t channel, int32_t target_angle, int32_t offset) {
    int32_t final_angle = target_angle + offset;
    if (final_angle < 0) final_angle = 0;
    if (final_angle > 180) final_angle = 180;
    
    uint32_t duty = 204 + ((1024 - 204) * final_angle) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void apply_all_servos() {
    write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
    write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
    write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
    write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);
}

static void servo_animation_task(void *pv) {
    int pos = 0;
    int dir = 1;
    
    while(1) {
        if (sensor_is_safety_locked()) {
            target_low_left = 90; target_high_right = 90;
            target_high_left = 90; target_low_right = 90;
            apply_all_servos();
            active_animation = 0; // Cancel animation on safety trip
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        if (active_animation == 1 || active_animation == 2) {
            pos += (5 * dir);
            if (pos >= 180) { pos = 180; dir = -1; }
            if (pos <= 0)   { pos = 0;   dir = 1; }
            
            int pos2 = 180 - pos;
            
            if (active_animation == 1) { // Forward Walk Loop
                target_low_right = pos2;
                target_high_right = pos;
                target_low_left = pos2;
                target_high_left = pos;
            } else { // Backward Walk Loop
                target_low_right = pos;
                target_high_right = pos2;
                target_low_left = pos;
                target_high_left = pos2;
            }
            apply_all_servos();
            vTaskDelay(pdMS_TO_TICKS(20)); // Animation frame rate
        } else {
            vTaskDelay(pdMS_TO_TICKS(50)); // Idle wait
        }
    }
}

void servo_controller_init() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_i32(my_handle, "ll_off", &offset_low_left);
        nvs_get_i32(my_handle, "hr_off", &offset_high_right);
        nvs_get_i32(my_handle, "hl_off", &offset_high_left);
        nvs_get_i32(my_handle, "lr_off", &offset_low_right);
        nvs_close(my_handle);
    }

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_13_BIT;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 50;  
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t chan_cfg = {};
    chan_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    chan_cfg.intr_type  = LEDC_INTR_DISABLE;
    chan_cfg.timer_sel  = LEDC_TIMER_0;
    chan_cfg.duty = 0; chan_cfg.hpoint = 0;

    chan_cfg.gpio_num = SERVO_LOW_LEFT_PIN;   chan_cfg.channel = SERVO_LOW_LEFT_CH;   ledc_channel_config(&chan_cfg);
    chan_cfg.gpio_num = SERVO_HIGH_RIGHT_PIN; chan_cfg.channel = SERVO_HIGH_RIGHT_CH; ledc_channel_config(&chan_cfg);
    chan_cfg.gpio_num = SERVO_HIGH_LEFT_PIN;  chan_cfg.channel = SERVO_HIGH_LEFT_CH;  ledc_channel_config(&chan_cfg);
    chan_cfg.gpio_num = SERVO_LOW_RIGHT_PIN;  chan_cfg.channel = SERVO_LOW_RIGHT_CH;  ledc_channel_config(&chan_cfg);

    apply_all_servos();
    
    // Start animation loop thread
    xTaskCreate(servo_animation_task, "anim_task", 4096, NULL, 5, NULL);
}

void servo_set_target(const char* id, int angle) {
    if (sensor_is_safety_locked()) return;
    active_animation = 0; // Manual override stops animations
    
    if (strcmp(id, "low_left") == 0)   target_low_left = angle;
    if (strcmp(id, "high_right") == 0) target_high_right = angle;
    if (strcmp(id, "high_left") == 0)  target_high_left = angle;
    if (strcmp(id, "low_right") == 0)  target_low_right = angle;
    
    if (strcmp(id, "all") == 0) {
        target_low_left = angle; target_high_right = angle; target_high_left = angle; target_low_right = angle;
    }
    if (strcmp(id, "front") == 0) {
        target_high_left = angle; target_high_right = angle;
    }
    if (strcmp(id, "back") == 0) {
        target_low_left = angle; target_low_right = angle;
    }
    
    apply_all_servos();
}

void servo_set_action(const char* action_name) {
    if (sensor_is_safety_locked()) return;
    
    if (strcmp(action_name, "forward") == 0) {
        active_animation = 1;
    } else if (strcmp(action_name, "backward") == 0) {
        active_animation = 2;
    } else {
        active_animation = 0; // Stop looping animations for static poses
        
        if (strcmp(action_name, "sit") == 0) {
            target_high_left = 180; target_high_right = 180; // Front 180
            target_low_left = 0;    target_low_right = 0;    // Back 0
        } else if (strcmp(action_name, "stand") == 0) {
            target_high_left = 0; target_high_right = 0; // Front 0
            target_low_left = 0;  target_low_right = 0;  // Back 0
        } else if (strcmp(action_name, "stretch_down") == 0) {
            target_high_left = 180; target_high_right = 180; // Front 180
            target_low_left = 90;   target_low_right = 90;   // Back 90
        } else if (strcmp(action_name, "stretch_back") == 0) {
            target_high_left = 90;  target_high_right = 90;  // Front 90
            target_low_left = 180;  target_low_right = 180;  // Back 180
        } else if (strcmp(action_name, "stop") == 0) {
            target_high_left = 90; target_high_right = 90;
            target_low_left = 90;  target_low_right = 90;
        }
        apply_all_servos();
    }
}

void servo_get_angles(int* ll, int* hr, int* hl, int* lr) {
    *ll = target_low_left; *hr = target_high_right;
    *hl = target_high_left; *lr = target_low_right;
}

void servo_get_offsets(int* ll, int* hr, int* hl, int* lr) {
    *ll = offset_low_left; *hr = offset_high_right;
    *hl = offset_high_left; *lr = offset_low_right;
}

void servo_set_offsets(int ll, int hr, int hl, int lr, bool save_to_nvs) {
    offset_low_left = ll; offset_high_right = hr;
    offset_high_left = hl; offset_low_right = lr;
    
    apply_all_servos();
    
    if (save_to_nvs) {
        nvs_handle_t my_handle;
        if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
            nvs_set_i32(my_handle, "ll_off", offset_low_left);
            nvs_set_i32(my_handle, "hr_off", offset_high_right);
            nvs_set_i32(my_handle, "hl_off", offset_high_left);
            nvs_set_i32(my_handle, "lr_off", offset_low_right);
            nvs_commit(my_handle);
            nvs_close(my_handle);
        }
    }
}