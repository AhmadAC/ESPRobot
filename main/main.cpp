#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "cJSON.h"

static const char *TAG = "ESPROBOT";
httpd_handle_t server = NULL;

// --- Pin Definitions ---
#define SERVO_LOW_LEFT_PIN   GPIO_NUM_12 // CH0
#define SERVO_HIGH_RIGHT_PIN GPIO_NUM_10 // CH1
#define SERVO_HIGH_LEFT_PIN  GPIO_NUM_11 // CH2
#define SERVO_LOW_RIGHT_PIN  GPIO_NUM_9  // CH3

// --- LEDC Channels ---
#define SERVO_LOW_LEFT_CH    LEDC_CHANNEL_0
#define SERVO_HIGH_RIGHT_CH  LEDC_CHANNEL_1
#define SERVO_HIGH_LEFT_CH   LEDC_CHANNEL_2
#define SERVO_LOW_RIGHT_CH   LEDC_CHANNEL_3

// --- Volatile Servo States & NVS Calibration Offsets ---
int32_t target_low_left   = 90;
int32_t target_high_right = 90;
int32_t target_high_left  = 90;
int32_t target_low_right  = 90;

int32_t offset_low_left   = 0;
int32_t offset_high_right = 0;
int32_t offset_high_left  = 0;
int32_t offset_low_right  = 0;

// --- Servo Logic ---
void write_servo_calibrated(ledc_channel_t channel, int32_t target_angle, int32_t offset) {
    int32_t final_angle = target_angle + offset;
    if (final_angle < 0) final_angle = 0;
    if (final_angle > 180) final_angle = 180;
    
    // Duty calculation: 13-bit resolution (0 to 8191)
    // 0 deg = ~0.5ms pulse -> 2.5% of 20ms period -> 204 duty value
    // 180 deg = ~2.5ms pulse -> 12.5% of 20ms period -> 1024 duty value
    uint32_t duty = 204 + ((1024 - 204) * final_angle) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void init_servos() {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_13_BIT;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 50;  
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t chan_cfg = {};
    chan_cfg.speed_mode     = LEDC_LOW_SPEED_MODE;
    chan_cfg.intr_type      = LEDC_INTR_DISABLE;
    chan_cfg.timer_sel      = LEDC_TIMER_0;
    chan_cfg.duty           = 0;
    chan_cfg.hpoint         = 0;

    // Register all 4 servos
    chan_cfg.gpio_num = SERVO_LOW_LEFT_PIN;   chan_cfg.channel = SERVO_LOW_LEFT_CH;   ledc_channel_config(&chan_cfg);
    chan_cfg.gpio_num = SERVO_HIGH_RIGHT_PIN;  chan_cfg.channel = SERVO_HIGH_RIGHT_CH;  ledc_channel_config(&chan_cfg);
    chan_cfg.gpio_num = SERVO_HIGH_LEFT_PIN;   chan_cfg.channel = SERVO_HIGH_LEFT_CH;   ledc_channel_config(&chan_cfg);
    chan_cfg.gpio_num = SERVO_LOW_RIGHT_PIN;   chan_cfg.channel = SERVO_LOW_RIGHT_CH;   ledc_channel_config(&chan_cfg);

    // Apply initial target configurations with offsets
    write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
    write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
    write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
    write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);
}

void save_offsets_to_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, "ll_off", offset_low_left);
        nvs_set_i32(my_handle, "hr_off", offset_high_right);
        nvs_set_i32(my_handle, "hl_off", offset_high_left);
        nvs_set_i32(my_handle, "lr_off", offset_low_right);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Offsets saved to NVS: LL=%ld, HR=%ld, HL=%ld, LR=%ld", 
                 offset_low_left, offset_high_right, offset_high_left, offset_low_right);
    }
}

void load_offsets_from_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_i32(my_handle, "ll_off", &offset_low_left);
        nvs_get_i32(my_handle, "hr_off", &offset_high_right);
        nvs_get_i32(my_handle, "hl_off", &offset_high_left);
        nvs_get_i32(my_handle, "lr_off", &offset_low_right);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Loaded Offsets: LL=%ld, HR=%ld, HL=%ld, LR=%ld", 
                 offset_low_left, offset_high_right, offset_high_left, offset_low_right);
    }
}

// --- Native USB REPL / Debug Listener Task ---
void console_read_task(void *pvParameter) {
    ESP_LOGI("REPL", "USB CDC REPL Listener Started.");
    
    // Set standard input to non-blocking mode so it doesn't freeze the task
    int fd = fileno(stdin);
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    uint8_t buf[64];
    while (1) {
        int len = read(fd, buf, sizeof(buf));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (buf[i] == 0x03) { // Ctrl+C
                    ESP_LOGW("REPL", "--> [Interrupt] Ctrl+C received!");
                    ESP_LOGW("REPL", "--> Halting and centering all servos.");
                    
                    target_low_left = 90; target_high_right = 90;
                    target_high_left = 90; target_low_right = 90;
                    
                    write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
                    write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
                    write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
                    write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);

                } else if (buf[i] == 0x04) { // Ctrl+D
                    ESP_LOGW("REPL", "--> [Soft Reset] Ctrl+D received! Rebooting in 1 second...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else if (buf[i] >= 32 && buf[i] <= 126) {
                    // Echo standard characters back for debug visibility
                    ESP_LOGI("REPL", "--> Received byte: '%c' (0x%02X)", buf[i], buf[i]);
                }
            }
        }
        // Poll every 20ms
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Web Server Core Code ---
void delayed_reboot_task(void *pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

static esp_err_t index_get_handler(httpd_req_t *req) {
    const char* html = R"raw_html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>ESPRobot Dashboard</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
        body { font-family: 'Segoe UI', Arial, sans-serif; background: #eef2f3; margin: 0; padding: 20px; color: #333; }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; color: #2c3e50; margin-bottom: 30px; }
        .card { background: white; border-radius: 12px; padding: 25px; box-shadow: 0 4px 15px rgba(0,0,0,0.05); margin-bottom: 25px; }
        h2 { border-bottom: 2px solid #3498db; padding-bottom: 10px; color: #2c3e50; margin-top: 0; }
        .grid { display: grid; grid-template-columns: 1fr; gap: 20px; }
        @media (min-width: 600px) { .grid { grid-template-columns: 1fr 1fr; } }
        .ctrl-group { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #3498db; }
        .ctrl-header { display: flex; justify-content: space-between; font-weight: bold; margin-bottom: 8px; }
        label { display: block; margin-bottom: 8px; font-weight: bold; }
        input[type=range] { width: 100%; margin-bottom: 15px; cursor: pointer; }
        button { background: #3498db; color: white; border: none; padding: 12px 20px; border-radius: 6px; cursor: pointer; font-size: 15px; width: 100%; transition: 0.2s; }
        button:hover { background: #2980b9; }
        select, input[type=password], input[type=number] { width: 100%; padding: 10px; box-sizing: border-box; border: 1px solid #ccc; border-radius: 6px; margin-bottom: 15px; }
        .pass-container { display: flex; gap: 10px; align-items: center; margin-bottom: 15px; }
        .pass-container input { margin-bottom: 0; }
        .badge { background: #e74c3c; color: white; padding: 3px 8px; border-radius: 12px; font-size: 11px; }
        #status { font-weight: bold; color: #16a085; text-align: center; margin-top: 10px; }
    </style>
</head>
<body>
<div class='container'>
    <h1>ESPRobot Web Controller</h1>

    <!-- Wi-Fi Settings -->
    <div class='card'>
        <h2>Wi-Fi Provisioning</h2>
        <div class='grid'>
            <div>
                <button onclick='scan()'>Scan Wi-Fi Networks</button>
                <p id='status'></p>
            </div>
            <div>
                <label>Target SSID:</label>
                <select id='ssid'></select>
                
                <label>Wi-Fi Password:</label>
                <div class='pass-container'>
                    <input type='password' id='pass' placeholder='Enter password'>
                    <input type='checkbox' onclick="let p=document.getElementById('pass'); p.type=(p.type==='password')?'text':'password';"> <small>Show</small>
                </div>
                <button style='background: #27ae60;' onclick='saveWiFi()'>Save & Reboot</button>
            </div>
        </div>
    </div>

    <!-- Active Robot Servo Control -->
    <div class='card'>
        <h2>Servo Kinematics Control</h2>
        <div class='grid'>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Left Leg (IO12)</span><span id='val_low_left'>90&deg;</span></div>
                <input type='range' id='low_left' min='0' max='180' value='90' oninput='moveServo("low_left", this.value)'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Right Shoulder (IO10)</span><span id='val_high_right'>90&deg;</span></div>
                <input type='range' id='high_right' min='0' max='180' value='90' oninput='moveServo("high_right", this.value)'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Left Shoulder (IO11)</span><span id='val_high_left'>90&deg;</span></div>
                <input type='range' id='high_left' min='0' max='180' value='90' oninput='moveServo("high_left", this.value)'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Right Leg (IO9)</span><span id='val_low_right'>90&deg;</span></div>
                <input type='range' id='low_right' min='0' max='180' value='90' oninput='moveServo("low_right", this.value)'>
            </div>
        </div>

        <h3 style='margin-top:25px;'>Group Synchronized Control</h3>
        <div class='grid'>
            <div class='ctrl-group' style='border-left-color: #f1c40f;'>
                <div class='ctrl-header'><span>All Leg Motors (Combined)</span><span id='val_all'>90&deg;</span></div>
                <input type='range' id='all' min='0' max='180' value='90' oninput='moveServo("all", this.value)'>
            </div>
            <div class='ctrl-group' style='border-left-color: #2ecc71;'>
                <div class='ctrl-header'><span>Front Pairs (High Left & Right)</span><span id='val_front'>90&deg;</span></div>
                <input type='range' id='front' min='0' max='180' value='90' oninput='moveServo("front", this.value)'>
            </div>
            <div class='ctrl-group' style='border-left-color: #e67e22;'>
                <div class='ctrl-header'><span>Back Pairs (Low Left & Right)</span><span id='val_back'>90&deg;</span></div>
                <input type='range' id='back' min='0' max='180' value='90' oninput='moveServo("back", this.value)'>
            </div>
        </div>
    </div>

    <!-- Alignment Software Calibration -->
    <div class='card'>
        <h2>Software Calibration (NVS Saved Offsets)</h2>
        <p><small style='color: #7f8c8d;'>Compensate for misalignment during leg linkage assembly here. Offset value is applied directly on top of the sweep commands.</small></p>
        <div class='grid'>
            <div>
                <label>Low Left Offset:</label>
                <input type='number' id='off_low_left' min='-45' max='45' value='0' onchange='updateOffsets()'>
                
                <label>High Right Offset:</label>
                <input type='number' id='off_high_right' min='-45' max='45' value='0' onchange='updateOffsets()'>
            </div>
            <div>
                <label>High Left Offset:</label>
                <input type='number' id='off_high_left' min='-45' max='45' value='0' onchange='updateOffsets()'>
                
                <label>Low Right Offset:</label>
                <input type='number' id='off_low_right' min='-45' max='45' value='0' onchange='updateOffsets()'>
            </div>
        </div>
        <button style='background: #e67e22; margin-top:15px;' onclick='saveOffsets()'>Save Calibration to Persistent Memory</button>
    </div>
</div>

<script>
    function scan() {
        document.getElementById('status').innerText = 'Scanning Wi-Fi...';
        fetch('/scan').then(r => r.json()).then(d => {
            let s = document.getElementById('ssid');
            s.innerHTML = '';
            d.forEach(n => { s.innerHTML += '<option value="'+n+'">'+n+'</option>'; });
            document.getElementById('status').innerText = 'Found ' + d.length + ' network(s)';
        });
    }

    function saveWiFi() {
        let s = document.getElementById('ssid').value;
        let p = document.getElementById('pass').value;
        if(!s) { alert('Please scan & select an SSID.'); return; }
        fetch('/save', {method: 'POST', body: JSON.stringify({ssid: s, pass: p})})
        .then(() => { alert('Network saved successfully! Robot rebooting in 2 seconds.'); });
    }

    function moveServo(id, angle) {
        document.getElementById('val_' + id).innerHTML = angle + '&deg;';
        fetch('/servo', {
            method: 'POST',
            body: JSON.stringify({id: id, angle: parseInt(angle)})
        });
    }

    // Pull current system configurations on document load
    window.onload = function() {
        fetch('/angles').then(r => r.json()).then(data => {
            // Populate Sliders
            for (let [leg, stats] of Object.entries(data)) {
                let el = document.getElementById(leg);
                if(el) { el.value = stats.angle; document.getElementById('val_' + leg).innerHTML = stats.angle + '&deg;'; }
                let offEl = document.getElementById('off_' + leg);
                if(offEl) { offEl.value = stats.offset; }
            }
        });
    }

    function updateOffsets() {
        let ll = parseInt(document.getElementById('off_low_left').value) || 0;
        let hr = parseInt(document.getElementById('off_high_right').value) || 0;
        let hl = parseInt(document.getElementById('off_high_left').value) || 0;
        let lr = parseInt(document.getElementById('off_low_right').value) || 0;

        fetch('/calibrate', {
            method: 'POST',
            body: JSON.stringify({low_left: ll, high_right: hr, high_left: hl, low_right: lr, save: false})
        });
    }

    function saveOffsets() {
        let ll = parseInt(document.getElementById('off_low_left').value) || 0;
        let hr = parseInt(document.getElementById('off_high_right').value) || 0;
        let hl = parseInt(document.getElementById('off_high_left').value) || 0;
        let lr = parseInt(document.getElementById('off_low_right').value) || 0;

        fetch('/calibrate', {
            method: 'POST',
            body: JSON.stringify({low_left: ll, high_right: hr, high_left: hl, low_right: lr, save: true})
        }).then(() => { alert('Calibration parameters written securely to Flash NVS.'); });
    }
</script>
</body>
</html>
)raw_html";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t scan_get_handler(httpd_req_t *req) {
    esp_wifi_scan_stop();
    wifi_scan_config_t scan_config = {};
    esp_wifi_scan_start(&scan_config, true);
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    wifi_ap_record_t *ap_info = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
    esp_wifi_scan_get_ap_records(&ap_count, ap_info);

    cJSON *root = cJSON_CreateArray();
    for(int i = 0; i < ap_count; i++) {
        cJSON_AddItemToArray(root, cJSON_CreateString((char*)ap_info[i].ssid));
    }
    char* json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(ap_info); 
    cJSON_Delete(root); 
    free(json_str);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
    char buf[200];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if(json) {
        const char *ssid = cJSON_GetObjectItem(json, "ssid")->valuestring;
        const char *pass = cJSON_GetObjectItem(json, "pass")->valuestring;
        nvs_handle_t my_handle;
        nvs_open("storage", NVS_READWRITE, &my_handle);
        nvs_set_str(my_handle, "wifi_ssid", ssid);
        nvs_set_str(my_handle, "wifi_pass", pass);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Saved SSID: %s", ssid);
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
        xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
    }
    return ESP_OK;
}

static esp_err_t servo_post_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json) {
        const char *id = cJSON_GetObjectItem(json, "id")->valuestring;
        int32_t angle = cJSON_GetObjectItem(json, "angle")->valueint;
        
        if (strcmp(id, "low_left") == 0) {
            target_low_left = angle;
            write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
        } else if (strcmp(id, "high_right") == 0) {
            target_high_right = angle;
            write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
        } else if (strcmp(id, "high_left") == 0) {
            target_high_left = angle;
            write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
        } else if (strcmp(id, "low_right") == 0) {
            target_low_right = angle;
            write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);
        } else if (strcmp(id, "all") == 0) {
            target_low_left = angle; target_high_right = angle; target_high_left = angle; target_low_right = angle;
            write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
            write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
            write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
            write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);
        } else if (strcmp(id, "front") == 0) {
            target_high_left = angle; target_high_right = angle;
            write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
            write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
        } else if (strcmp(id, "back") == 0) {
            target_low_left = angle; target_low_right = angle;
            write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
            write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);
        }
        
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t calibrate_post_handler(httpd_req_t *req) {
    char buf[200];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json) {
        offset_low_left   = cJSON_GetObjectItem(json, "low_left")->valueint;
        offset_high_right = cJSON_GetObjectItem(json, "high_right")->valueint;
        offset_high_left  = cJSON_GetObjectItem(json, "high_left")->valueint;
        offset_low_right  = cJSON_GetObjectItem(json, "low_right")->valueint;
        bool save_to_nvs  = cJSON_HasObjectItem(json, "save") ? cJSON_GetObjectItem(json, "save")->valueint : false;

        // Apply calibrated offset adjustments immediately
        write_servo_calibrated(SERVO_LOW_LEFT_CH, target_low_left, offset_low_left);
        write_servo_calibrated(SERVO_HIGH_RIGHT_CH, target_high_right, offset_high_right);
        write_servo_calibrated(SERVO_HIGH_LEFT_CH, target_high_left, offset_high_left);
        write_servo_calibrated(SERVO_LOW_RIGHT_CH, target_low_right, offset_low_right);

        if (save_to_nvs) {
            save_offsets_to_nvs();
        }

        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t angles_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    cJSON *ll = cJSON_CreateObject();
    cJSON_AddNumberToObject(ll, "angle", target_low_left);
    cJSON_AddNumberToObject(ll, "offset", offset_low_left);
    cJSON_AddItemToObject(root, "low_left", ll);

    cJSON *hr = cJSON_CreateObject();
    cJSON_AddNumberToObject(hr, "angle", target_high_right);
    cJSON_AddNumberToObject(hr, "offset", offset_high_right);
    cJSON_AddItemToObject(root, "high_right", hr);

    cJSON *hl = cJSON_CreateObject();
    cJSON_AddNumberToObject(hl, "angle", target_high_left);
    cJSON_AddNumberToObject(hl, "offset", offset_high_left);
    cJSON_AddItemToObject(root, "high_left", hl);

    cJSON *lr = cJSON_CreateObject();
    cJSON_AddNumberToObject(lr, "angle", target_low_right);
    cJSON_AddNumberToObject(lr, "offset", offset_low_right);
    cJSON_AddItemToObject(root, "low_right", lr);

    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    cJSON_Delete(root);
    free(json_str);
    return ESP_OK;
}

void start_control_server() {
    if (server == NULL) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        // Set maximum payload sizes to accommodate calibration files or scans
        config.max_uri_handlers = 8;
        
        if (httpd_start(&server, &config) == ESP_OK) {
            httpd_uri_t uri_index = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_scan  = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_save  = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_servo = { .uri = "/servo", .method = HTTP_POST, .handler = servo_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_cal   = { .uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_angs  = { .uri = "/angles", .method = HTTP_GET, .handler = angles_get_handler, .user_ctx = NULL };
            
            httpd_register_uri_handler(server, &uri_index);
            httpd_register_uri_handler(server, &uri_scan);
            httpd_register_uri_handler(server, &uri_save);
            httpd_register_uri_handler(server, &uri_servo);
            httpd_register_uri_handler(server, &uri_cal);
            httpd_register_uri_handler(server, &uri_angs);
            
            ESP_LOGI(TAG, "Dashboard Server initialized successfully on port %d", config.server_port);
        }
    }
}

// --- Main Application Setup ---
extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize Network interfaces once
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Load Calibration parameters immediately
    load_offsets_from_nvs();

    // Initialize Servo Hardware
    init_servos();

    // Start the REPL Listener Task so Host PC writes do not time out
    xTaskCreate(console_read_task, "console_read_task", 4096, NULL, 5, NULL);

    // Configure both station and AP Mode config profiles
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "ESPRobot_Config");
    ap_config.ap.ssid_len = strlen("ESPRobot_Config");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN; // Configured as open for easy calibration access

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Auto-Connect Station if configurations exist
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        char ssid[32]; char pass[64];
        size_t s_len = sizeof(ssid); size_t p_len = sizeof(pass);
        if (nvs_get_str(my_handle, "wifi_ssid", ssid, &s_len) == ESP_OK &&
            nvs_get_str(my_handle, "wifi_pass", pass, &p_len) == ESP_OK) {
            
            ESP_LOGI(TAG, "Connecting to saved network: %s", ssid);
            wifi_config_t sta_config = {};
            strcpy((char*)sta_config.sta.ssid, ssid);
            strcpy((char*)sta_config.sta.password, pass);
            
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "No Wi-Fi credentials found.");
        }
        nvs_close(my_handle);
    }

    // Launch Dashboard Control Panel
    start_control_server();
}