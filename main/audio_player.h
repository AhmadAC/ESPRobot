#pragma once
#include <stdint.h>
#include <stdbool.h>

void audio_player_init();
void audio_set_volume(int volume_pct); // 0 to 100
int audio_get_volume();
void audio_play(const char* sound_name);
void audio_stop();