#include "audio_player.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "AUDIO";

// Pinout mapping for ESP32-S3 Zero
#define I2S_BCLK_PIN GPIO_NUM_6
#define I2S_LRC_PIN  GPIO_NUM_7
#define I2S_DIN_PIN  GPIO_NUM_8

static i2s_chan_handle_t tx_chan;
static int32_t current_volume = 50; // 0-100 Default volume
static QueueHandle_t audio_queue;

// Access the embedded WAV file created by CMake
extern const uint8_t dog_barking_wav_start[] asm("_binary_dog_barking_wav_start");
extern const uint8_t dog_barking_wav_end[]   asm("_binary_dog_barking_wav_end");

static void load_audio_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_i32(my_handle, "aud_vol", &current_volume);
        nvs_close(my_handle);
    }
    if (current_volume < 0) current_volume = 0;
    if (current_volume > 100) current_volume = 100;
}

static void audio_task(void *pvParameter) {
    char sound_req[32];
    
    while (1) {
        // Wait until a sound play request is received in the queue
        if (xQueueReceive(audio_queue, &sound_req, portMAX_DELAY)) {
            
            const uint8_t* wav_start = NULL;
            const uint8_t* wav_end = NULL;
            
            if (strcmp(sound_req, "dog_bark") == 0) {
                wav_start = dog_barking_wav_start;
                wav_end = dog_barking_wav_end;
            }
            
            if (wav_start != NULL && wav_end > wav_start) {
                
                uint32_t sample_rate = 44100;
                uint16_t channels = 2;
                uint16_t bit_depth = 16;
                const uint8_t* audio_data = NULL;
                uint32_t data_size = 0;

                // Dynamic WAV RIFF Chunk Parser (Safely skips LIST/INFO metadata chunks)
                if (strncmp((const char*)wav_start, "RIFF", 4) == 0) {
                    size_t offset = 12; // Skip RIFF header, file size, and WAVE format
                    
                    while (offset < (size_t)(wav_end - wav_start) - 8) {
                        char chunk_id[5] = {0};
                        memcpy(chunk_id, wav_start + offset, 4);
                        uint32_t chunk_size = *(uint32_t*)(wav_start + offset + 4);
                        offset += 8;

                        if (strcmp(chunk_id, "fmt ") == 0) {
                            channels = *(uint16_t*)(wav_start + offset + 2);
                            sample_rate = *(uint32_t*)(wav_start + offset + 4);
                            bit_depth = *(uint16_t*)(wav_start + offset + 14);
                        } else if (strcmp(chunk_id, "data") == 0) {
                            audio_data = wav_start + offset;
                            data_size = chunk_size;
                            break; // We found the audio payload, stop parsing
                        }
                        offset += chunk_size; // Skip unknown chunks
                    }
                }

                if (audio_data != NULL && data_size > 0) {
                    ESP_LOGI(TAG, "Playing %s: %lu Hz, %d bit, %d ch | Size: %lu bytes", 
                             sound_req, sample_rate, bit_depth, channels, data_size);
                    
                    // Reconfigure I2S Clock to match the WAV file perfectly
                    i2s_channel_disable(tx_chan);
                    
                    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
                    i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
                    
                    // MAX98357A requires standard PHILIPS formatting, NOT MSB/Left-Justified
                    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, 
                        channels == 2 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO
                    );
                    i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg);
                    
                    i2s_channel_enable(tx_chan);
                    
                    // Prevent reading past the embedded binary end if file is corrupted
                    if (audio_data + data_size > wav_end) {
                        data_size = wav_end - audio_data;
                    }
                    
                    int16_t* pcm_samples = (int16_t*)audio_data;
                    size_t num_samples = data_size / 2; // 16-bit integers
                    
                    float vol_mult = (float)current_volume / 100.0f;
                    
                    // Create a small buffer to scale volume in RAM without blocking
                    const size_t CHUNK_SAMPLES = 512; 
                    int16_t out_buffer[CHUNK_SAMPLES];
                    size_t bytes_written;
                    
                    for (size_t i = 0; i < num_samples; i += CHUNK_SAMPLES) {
                        size_t chunk_size = (num_samples - i < CHUNK_SAMPLES) ? (num_samples - i) : CHUNK_SAMPLES;
                        
                        // Software Volume Application
                        for (size_t j = 0; j < chunk_size; j++) {
                            if (current_volume == 0) {
                                out_buffer[j] = 0; // Absolute Mute
                            } else {
                                out_buffer[j] = (int16_t)(pcm_samples[i + j] * vol_mult);
                            }
                        }
                        
                        esp_err_t err = i2s_channel_write(tx_chan, out_buffer, chunk_size * 2, &bytes_written, portMAX_DELAY);
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "I2S Buffer Write Failed: %s", esp_err_to_name(err));
                            break; // Abort this track
                        }
                    }
                    
                    // Flush the DMA buffers to ensure the sound finishes cleanly
                    i2s_channel_write(tx_chan, NULL, 0, &bytes_written, portMAX_DELAY);
                    ESP_LOGI(TAG, "Playback finished.");
                    
                } else {
                    ESP_LOGE(TAG, "Invalid WAV structure: Could not find 'fmt ' or 'data' chunk.");
                }
            }
        }
    }
}

void audio_player_init() {
    load_audio_nvs();
    audio_queue = xQueueCreate(5, sizeof(char) * 32);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_LRC_PIN,
            .dout = I2S_DIN_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false
            },
        },
    };
    
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    // Run audio handling strictly on Core 1 to avoid interrupting Servo PWM
    xTaskCreatePinnedToCore(audio_task, "audio_task", 4096, NULL, 4, NULL, 1);
    
    ESP_LOGI(TAG, "I2S Audio Player Initialized. Pins: BCLK=%d, LRC=%d, DIN=%d", I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DIN_PIN);
}

void audio_set_volume(int volume_pct) {
    if (volume_pct < 0) volume_pct = 0;
    if (volume_pct > 100) volume_pct = 100;
    current_volume = (int32_t)volume_pct;
    
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, "aud_vol", current_volume);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

int audio_get_volume() {
    return (int)current_volume;
}

void audio_play(const char* sound_name) {
    if (current_volume == 0) return; // Completely mute protection
    
    char req[32];
    strncpy(req, sound_name, sizeof(req) - 1);
    req[sizeof(req) - 1] = '\0';
    
    // Add to queue (non-blocking). If queue is full (spamming play), it ignores the request.
    xQueueSend(audio_queue, &req, 0); 
}

void audio_stop() {
    xQueueReset(audio_queue); // Clears any pending plays
    i2s_channel_disable(tx_chan);
    i2s_channel_enable(tx_chan); // Re-enable for next time
    ESP_LOGI(TAG, "Audio playback stopped.");
}