#pragma once
#include <stdint.h>

void servo_controller_init();
void servo_set_target(const char* id, int angle);
void servo_set_action(const char* action_name);
void servo_get_angles(int* ll, int* hr, int* hl, int* lr);
void servo_get_offsets(int* ll, int* hr, int* hl, int* lr);
void servo_set_offsets(int ll, int hr, int hl, int lr, bool save_to_nvs);