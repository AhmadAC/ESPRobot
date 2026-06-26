#pragma once
#include <stdint.h>
#include <stdbool.h>

void servo_controller_init();
void servo_set_target(const char* id, int angle);
void servo_set_target_silent(const char* id, int angle);
void servo_apply_targets();
void servo_set_action(const char* action_name);
void servo_set_action_bypass(const char* action_name);
void servo_get_angles(int* ll, int* hr, int* hl, int* lr);
void servo_get_offsets(int* ll, int* hr, int* hl, int* lr);

// Calibration APIs
char* servo_get_calibrations_json();
char* servo_get_calibrations_cpp();
void servo_set_calibration(const char* target, int ll, int hr, int hl, int lr, bool save_to_nvs);