#include "web_server.h"
#include "wifi_manager.h"
#include "sensor_monitor.h"
#include "servo_controller.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WEB";
httpd_handle_t server = NULL;

static void delayed_reboot_task(void *pvParameter) {
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
        .ctrl-group { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #3498db; transition: opacity 0.3s; }
        .ctrl-header { display: flex; justify-content: space-between; font-weight: bold; margin-bottom: 8px; }
        label { display: block; margin-bottom: 8px; font-weight: bold; }
        input[type=range] { width: 100%; margin-bottom: 15px; cursor: pointer; }
        input[type=range]:disabled { cursor: not-allowed; opacity: 0.5; }
        button { background: #3498db; color: white; border: none; padding: 12px 20px; border-radius: 6px; cursor: pointer; font-size: 15px; width: 100%; transition: 0.2s; }
        button:hover { background: #2980b9; }
        select, input[type=password], input[type=number] { width: 100%; padding: 10px; box-sizing: border-box; border: 1px solid #ccc; border-radius: 6px; margin-bottom: 15px; }
        .pass-container { display: flex; gap: 10px; align-items: center; margin-bottom: 15px; }
        .pass-container input { margin-bottom: 0; }
        #status { font-weight: bold; color: #16a085; text-align: center; margin-top: 10px; }
        #lock_banner { display: none; background: #e74c3c; color: white; padding: 12px; border-radius: 8px; text-align: center; font-weight: bold; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(231, 76, 60, 0.3); }
        .locked-mode .ctrl-group, .locked-mode .btn-grid button { opacity: 0.6; pointer-events: none; }
        .btn-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; }
        .btn-stop { background: #e74c3c !important; } .btn-stop:hover { background: #c0392b !important; }
    </style>
</head>
<body>
<div class='container'>
    <h1>ESPRobot Web Controller</h1>

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

    <div class='card' id='servo_card'>
        <h2>Robot Actions & Kinematics</h2>
        <div id='lock_banner'>⚠️ MOTORS LOCKED: Obstacle Detected!</div>
        
        <div class='btn-grid' style='margin-bottom: 25px;'>
            <button onclick='doAction("forward")'>Walk Forward</button>
            <button onclick='doAction("backward")'>Walk Backward</button>
            <button class='btn-stop' onclick='doAction("stop")'>STOP</button>
            <button onclick='doAction("sit")' style='background:#f39c12;'>Sit</button>
            <button onclick='doAction("stand")' style='background:#f39c12;'>Stand Up</button>
            <button onclick='doAction("stretch_down")' style='background:#9b59b6;'>Stretch Down</button>
            <button onclick='doAction("stretch_back")' style='background:#9b59b6;'>Stretch Back</button>
        </div>

        <div class='grid'>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Left Leg (IO12)</span><span id='val_low_left'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='low_left' min='0' max='180' value='90' oninput='moveServo("low_left", this.value)' onmousedown='dragStart("low_left")' onmouseup='dragEnd()' ontouchstart='dragStart("low_left")' ontouchend='dragEnd()'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Right Shoulder (IO10)</span><span id='val_high_right'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='high_right' min='0' max='180' value='90' oninput='moveServo("high_right", this.value)' onmousedown='dragStart("high_right")' onmouseup='dragEnd()' ontouchstart='dragStart("high_right")' ontouchend='dragEnd()'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Left Shoulder (IO11)</span><span id='val_high_left'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='high_left' min='0' max='180' value='90' oninput='moveServo("high_left", this.value)' onmousedown='dragStart("high_left")' onmouseup='dragEnd()' ontouchstart='dragStart("high_left")' ontouchend='dragEnd()'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Right Leg (IO9)</span><span id='val_low_right'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='low_right' min='0' max='180' value='90' oninput='moveServo("low_right", this.value)' onmousedown='dragStart("low_right")' onmouseup='dragEnd()' ontouchstart='dragStart("low_right")' ontouchend='dragEnd()'>
            </div>
        </div>
    </div>

    <div class='card'>
        <h2>Pose Calibration & Hardware Zeroing</h2>
        <div class='grid'>
            <div class='ctrl-group' style='border-left-color: #e67e22;'>
                <label>Target to Calibrate:</label>
                <select id='calib_target' onchange='loadCalibTarget()' style='font-weight:bold; cursor:pointer;'>
                    <option value='offsets'>1. Hardware Zeroing (Offsets)</option>
                    <option value='sit'>2. Pose: Sit</option>
                    <option value='stand'>3. Pose: Stand Up</option>
                    <option value='stretch_down'>4. Pose: Stretch Down</option>
                    <option value='stretch_back'>5. Pose: Stretch Back</option>
                    <option value='stop'>6. Pose: Stop (Default)</option>
                </select>
                <p style='font-size: 11px; color: #7f8c8d; margin-top:5px;'>First, use Hardware Zeroing to make all motors physically 90&deg;. Then calibrate your custom limits for the individual buttons.</p>
            </div>
            <div class='ctrl-group' style='border-left-color: #34495e; text-align:center;'>
                <p style='margin-top:0; font-size:13px; font-weight:bold;'>Compile Defaults into Firmware</p>
                <p style='margin-top:0; font-size:11px; color:#7f8c8d;'>Save your finalized configuration to a C++ file to hardcode the settings securely for your next compile.</p>
                <button style='background: #34495e; padding: 10px;' onclick='downloadCalib()'>⬇️ Download C++ Configuration</button>
            </div>
        </div>
        <div class='grid' style='margin-top:15px;'>
            <div>
                <label>Low Left Motor Limit:</label>
                <input type='number' id='cal_ll' value='0'>
                <label>High Right Motor Limit:</label>
                <input type='number' id='cal_hr' value='0'>
            </div>
            <div>
                <label>High Left Motor Limit:</label>
                <input type='number' id='cal_hl' value='0'>
                <label>Low Right Motor Limit:</label>
                <input type='number' id='cal_lr' value='0'>
            </div>
        </div>
        <div class='grid' style='margin-top:15px;'>
            <button style='background: #3498db;' onclick='testCalib()'>Test Pose / Offset</button>
            <button style='background: #e67e22;' onclick='saveCalib()'>Save to Robot NVS Storage</button>
        </div>
    </div>

    <div class='card'>
        <h2>Ultrasonic Obstacle Safety Monitor</h2>
        <div class='grid'>
            <div class='ctrl-group' style='border-left-color: #e74c3c;'>
                <label>Sensor Toggle Status:</label>
                <button id='btn_sensor' onclick='toggleSensor()'>Enable Sensor</button>
                <p style='font-size: 16px; margin-top: 15px;'>Live Distance: <span id='live_dist' style='font-weight: bold; color: #34495e;'>--</span> cm</p>
            </div>
            <div class='ctrl-group' style='border-left-color: #9b59b6;'>
                <div class='ctrl-header'><span>Safety Halt Threshold</span><span id='val_threshold'>20cm</span></div>
                <input type='range' id='threshold' min='5' max='100' value='20' onchange='updateThreshold(this.value)'>
                <p style='margin: 0; font-size: 11px; color: #7f8c8d; line-height: 1.4;'>Lock all leg and shoulder motors to 90&deg; automatically if an obstacle appears closer than this limit.</p>
            </div>
        </div>
    </div>

</div>

<script>
    let localSensorEnabled = false; 
    let servoTimeouts = {}; 
    let allCalibrations = {};
    let activeDrag = null;

    function fetchJSON(url, bodyData) {
        return fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(bodyData)
        });
    }

    function dragStart(id) { activeDrag = id; }
    function dragEnd() { activeDrag = null; }

    function fetchCalibrations() {
        fetch('/calibrations_json').then(r => r.json()).then(data => {
            allCalibrations = data;
            loadCalibTarget();
        });
    }

    function loadCalibTarget() {
        let tgt = document.getElementById('calib_target').value;
        if (allCalibrations[tgt]) {
            document.getElementById('cal_ll').value = allCalibrations[tgt].ll;
            document.getElementById('cal_hr').value = allCalibrations[tgt].hr;
            document.getElementById('cal_hl').value = allCalibrations[tgt].hl;
            document.getElementById('cal_lr').value = allCalibrations[tgt].lr;
        }
    }

    function testCalib() {
        let tgt = document.getElementById('calib_target').value;
        fetchJSON('/calibrate', {
            target: tgt,
            ll: parseInt(document.getElementById('cal_ll').value) || 0,
            hr: parseInt(document.getElementById('cal_hr').value) || 0,
            hl: parseInt(document.getElementById('cal_hl').value) || 0,
            lr: parseInt(document.getElementById('cal_lr').value) || 0,
            save: false
        });
    }

    function saveCalib() {
        let tgt = document.getElementById('calib_target').value;
        fetchJSON('/calibrate', {
            target: tgt,
            ll: parseInt(document.getElementById('cal_ll').value) || 0,
            hr: parseInt(document.getElementById('cal_hr').value) || 0,
            hl: parseInt(document.getElementById('cal_hl').value) || 0,
            lr: parseInt(document.getElementById('cal_lr').value) || 0,
            save: true
        }).then(()=> {
            alert('Calibration Updated & Saved to ESP32 Storage!');
            fetchCalibrations();
        });
    }

    function downloadCalib() { window.location.href = '/download_cal'; }

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
        if(!s) return;
        fetchJSON('/save', {ssid: s, pass: p}).then(() => alert('Saved. Rebooting.'));
    }

    function doAction(act) {
        fetchJSON('/action', {action: act});
    }

    function moveServo(id, angle) {
        document.getElementById('val_' + id).innerHTML = angle + '&deg;';
        clearTimeout(servoTimeouts[id]);
        servoTimeouts[id] = setTimeout(() => {
            fetchJSON('/servo', {id: id, angle: parseInt(angle)});
        }, 100); 
    }

    function toggleSensor() {
        localSensorEnabled = !localSensorEnabled;
        
        // Optimistic UI Update ensures the button snaps instantly without waiting for network response
        let btn = document.getElementById('btn_sensor');
        if (localSensorEnabled) {
            btn.innerText = "Disable Sensor"; btn.style.background = "#e74c3c";
        } else {
            btn.innerText = "Enable Sensor"; btn.style.background = "#27ae60";
        }
        
        sendSensorConfig();
    }

    function updateThreshold(val) {
        document.getElementById('val_threshold').innerText = val + "cm";
        sendSensorConfig();
    }

    function sendSensorConfig() {
        let thresh = parseInt(document.getElementById('threshold').value) || 20;
        fetchJSON('/sensor', {enabled: localSensorEnabled, threshold: thresh});
    }

    function updateStatus() {
        fetch('/angles')
            .then(r => r.json())
            .then(data => {
                let srvCard = document.getElementById('servo_card');
                let srvSliders = document.querySelectorAll('.srv-slider');
                
                if (data.safety_lock) {
                    document.getElementById('lock_banner').style.display = 'block';
                    srvCard.classList.add('locked-mode');
                    srvSliders.forEach(slider => slider.disabled = true);
                } else {
                    document.getElementById('lock_banner').style.display = 'none';
                    srvCard.classList.remove('locked-mode');
                    srvSliders.forEach(slider => slider.disabled = false);
                }

                for (let [leg, stats] of Object.entries(data)) {
                    if (typeof stats === 'object') {
                        let el = document.getElementById(leg);
                        // Update slider values ONLY if the user isn't currently dragging them
                        if(el && (leg !== activeDrag || data.safety_lock)) { 
                            el.value = stats.angle; 
                            document.getElementById('val_' + leg).innerHTML = stats.angle + '&deg;'; 
                        }
                    }
                }

                let liveSpan = document.getElementById('live_dist');
                if (data.sensor_enabled) {
                    liveSpan.innerText = data.sensor_distance >= 0 ? data.sensor_distance.toFixed(1) : "Out of Range";
                    liveSpan.style.color = (data.sensor_distance > 0 && data.sensor_distance < data.sensor_threshold) ? "#e74c3c" : "#27ae60";
                } else {
                    liveSpan.innerText = "Disabled";
                    liveSpan.style.color = "#7f8c8d";
                }

                // Only sync the button state to the backend state if the user isn't actively interacting with it
                if (document.activeElement !== document.getElementById('btn_sensor')) {
                    localSensorEnabled = data.sensor_enabled;
                    let btn = document.getElementById('btn_sensor');
                    if (data.sensor_enabled) {
                        btn.innerText = "Disable Sensor"; btn.style.background = "#e74c3c";
                    } else {
                        btn.innerText = "Enable Sensor"; btn.style.background = "#27ae60";
                    }
                }

                if (document.activeElement !== document.getElementById('threshold')) {
                    document.getElementById('threshold').value = data.sensor_threshold;
                    document.getElementById('val_threshold').innerText = data.sensor_threshold + "cm";
                }
                
                // Chain updates sequentially so connections don't bottleneck if Wi-Fi drops
                setTimeout(updateStatus, 500); 
            })
            .catch(err => {
                // Wait longer to recover if a network drop occurs
                setTimeout(updateStatus, 1500); 
            });
    }

    window.onload = function() { 
        fetchCalibrations();
        updateStatus(); // Starts the recursive chaining process instead of blind setInterval
    }
</script>
</body>
</html>
)raw_html";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Reusable macro/function to safely ingest POST payloads and prevent socket hanging
static esp_err_t get_post_json(httpd_req_t *req, cJSON **json_out) {
    *json_out = NULL;
    int total_len = req->content_len;
    if (total_len >= 512 || total_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid payload size");
        return ESP_FAIL;
    }
    
    char* buf = (char*)malloc(total_len + 1);
    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            free(buf);
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[total_len] = '\0';
    
    *json_out = cJSON_Parse(buf);
    free(buf);
    
    if (*json_out == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t scan_get_handler(httpd_req_t *req) {
    char* json_str = wifi_scan_networks_json();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
    cJSON *json = NULL;
    if (get_post_json(req, &json) == ESP_OK) {
        cJSON *ssid_item = cJSON_GetObjectItem(json, "ssid");
        cJSON *pass_item = cJSON_GetObjectItem(json, "pass");
        if (ssid_item && pass_item) {
            wifi_save_credentials(ssid_item->valuestring, pass_item->valuestring);
            xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
        }
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t servo_post_handler(httpd_req_t *req) {
    cJSON *json = NULL;
    if (get_post_json(req, &json) == ESP_OK) {
        cJSON *id_item = cJSON_GetObjectItem(json, "id");
        cJSON *angle_item = cJSON_GetObjectItem(json, "angle");
        if (id_item && angle_item) {
            servo_set_target(id_item->valuestring, angle_item->valueint);
        }
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t action_post_handler(httpd_req_t *req) {
    cJSON *json = NULL;
    if (get_post_json(req, &json) == ESP_OK) {
        cJSON *act_item = cJSON_GetObjectItem(json, "action");
        if (act_item) {
            ESP_LOGI(TAG, "UI Triggered Action: %s", act_item->valuestring);
            servo_set_action(act_item->valuestring);
        }
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t calibrations_json_get_handler(httpd_req_t *req) {
    char* json_str = servo_get_calibrations_json();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    return ESP_OK;
}

static esp_err_t download_cal_get_handler(httpd_req_t *req) {
    char* cpp_code = servo_get_calibrations_cpp();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"esprobot_defaults.txt\"");
    httpd_resp_send(req, cpp_code, strlen(cpp_code));
    free(cpp_code);
    return ESP_OK;
}

static esp_err_t calibrate_post_handler(httpd_req_t *req) {
    cJSON *json = NULL;
    if (get_post_json(req, &json) == ESP_OK) {
        cJSON *tgt_item = cJSON_GetObjectItem(json, "target");
        cJSON *ll_item = cJSON_GetObjectItem(json, "ll");
        cJSON *hr_item = cJSON_GetObjectItem(json, "hr");
        cJSON *hl_item = cJSON_GetObjectItem(json, "hl");
        cJSON *lr_item = cJSON_GetObjectItem(json, "lr");
        cJSON *sv_item = cJSON_GetObjectItem(json, "save");

        if (tgt_item && ll_item && hr_item && hl_item && lr_item) {
            bool save_to_nvs = sv_item ? (cJSON_IsTrue(sv_item) || sv_item->valueint != 0) : false;
            servo_set_calibration(tgt_item->valuestring, ll_item->valueint, hr_item->valueint, hl_item->valueint, lr_item->valueint, save_to_nvs);
        }
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t sensor_post_handler(httpd_req_t *req) {
    cJSON *json = NULL;
    if (get_post_json(req, &json) == ESP_OK) {
        cJSON *en_item = cJSON_GetObjectItem(json, "enabled");
        if (en_item) {
            sensor_set_enabled(cJSON_IsTrue(en_item) || (en_item->valueint != 0));
        }
        cJSON *th_item = cJSON_GetObjectItem(json, "threshold");
        if (th_item) {
            sensor_set_threshold(th_item->valueint);
        }
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t angles_get_handler(httpd_req_t *req) {
    int ang_ll, ang_hr, ang_hl, ang_lr;
    int off_ll, off_hr, off_hl, off_lr;
    
    servo_get_angles(&ang_ll, &ang_hr, &ang_hl, &ang_lr);
    servo_get_offsets(&off_ll, &off_hr, &off_hl, &off_lr);

    cJSON *root = cJSON_CreateObject();
    
    cJSON *ll = cJSON_CreateObject(); cJSON_AddNumberToObject(ll, "angle", ang_ll); cJSON_AddNumberToObject(ll, "offset", off_ll); cJSON_AddItemToObject(root, "low_left", ll);
    cJSON *hr = cJSON_CreateObject(); cJSON_AddNumberToObject(hr, "angle", ang_hr); cJSON_AddNumberToObject(hr, "offset", off_hr); cJSON_AddItemToObject(root, "high_right", hr);
    cJSON *hl = cJSON_CreateObject(); cJSON_AddNumberToObject(hl, "angle", ang_hl); cJSON_AddNumberToObject(hl, "offset", off_hl); cJSON_AddItemToObject(root, "high_left", hl);
    cJSON *lr = cJSON_CreateObject(); cJSON_AddNumberToObject(lr, "angle", ang_lr); cJSON_AddNumberToObject(lr, "offset", off_lr); cJSON_AddItemToObject(root, "low_right", lr);

    cJSON_AddBoolToObject(root, "sensor_enabled", sensor_is_enabled());
    cJSON_AddBoolToObject(root, "safety_lock", sensor_is_safety_locked());
    cJSON_AddNumberToObject(root, "sensor_distance", sensor_get_distance());
    cJSON_AddNumberToObject(root, "sensor_threshold", sensor_get_threshold());

    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    
    // Kept keep-alive open intentionally to drastically improve TCP pool speeds
    httpd_resp_send(req, json_str, strlen(json_str));
    
    cJSON_Delete(root);
    free(json_str);
    return ESP_OK;
}

void web_server_init() {
    if (server == NULL) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        
        // Optimize configuration for swift request processing and thread isolation
        config.max_uri_handlers = 12;
        config.core_id = 1;                             // Restrict the HTTP server strictly to Core 1
        config.task_priority = 5;                       // Standardized to baseline HTTP daemon logic
        config.stack_size = 8192;                       // Guarantee stack space for processing JSON inputs safely
        config.lru_purge_enable = true;                 // Purge stale sockets dynamically to maintain active tunnels
        config.max_open_sockets = 10;                   // Higher ceiling array size allowing concurrent fetch requests smoothly
        config.recv_wait_timeout = 3;                   // Swift socket release on unresponsive clients
        config.send_wait_timeout = 3;                   // Swift socket release on slow transfers
        
        if (httpd_start(&server, &config) == ESP_OK) {
            httpd_uri_t uri_index   = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_scan    = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_save    = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_servo   = { .uri = "/servo", .method = HTTP_POST, .handler = servo_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_act     = { .uri = "/action", .method = HTTP_POST, .handler = action_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_cal     = { .uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_caljson = { .uri = "/calibrations_json", .method = HTTP_GET, .handler = calibrations_json_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_dlcal   = { .uri = "/download_cal", .method = HTTP_GET, .handler = download_cal_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_angs    = { .uri = "/angles", .method = HTTP_GET, .handler = angles_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_sensor  = { .uri = "/sensor", .method = HTTP_POST, .handler = sensor_post_handler, .user_ctx = NULL };
            
            httpd_register_uri_handler(server, &uri_index);
            httpd_register_uri_handler(server, &uri_scan);
            httpd_register_uri_handler(server, &uri_save);
            httpd_register_uri_handler(server, &uri_servo);
            httpd_register_uri_handler(server, &uri_act);
            httpd_register_uri_handler(server, &uri_cal);
            httpd_register_uri_handler(server, &uri_caljson);
            httpd_register_uri_handler(server, &uri_dlcal);
            httpd_register_uri_handler(server, &uri_angs);
            httpd_register_uri_handler(server, &uri_sensor);
            
            ESP_LOGI(TAG, "Dashboard Server initialized successfully on port %d", config.server_port);
        }
    }
}