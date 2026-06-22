#pragma once
#include <stdbool.h>
#include <stdint.h>

void sensor_monitor_init();
bool sensor_is_enabled();
void sensor_set_enabled(bool enabled);
int32_t sensor_get_threshold();
void sensor_set_threshold(int32_t threshold);
float sensor_get_distance();
bool sensor_is_safety_locked();