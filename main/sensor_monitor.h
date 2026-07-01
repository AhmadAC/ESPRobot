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

// Reaction time modifier
int32_t sensor_get_reaction_time();
void sensor_set_reaction_time(int32_t time_ms);

// Programmatic Sensor Target Routines
const char* sensor_get_tripped_action();
const char* sensor_get_cleared_action();
void sensor_set_actions(const char* tripped, const char* cleared);

// Audio Action Routing
const char* sensor_get_tripped_audio();
const char* sensor_get_cleared_audio();
void sensor_set_audio_actions(const char* tripped, const char* cleared);