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
                <input type='range' class='srv-slider' id='low_left' min='0' max='180' value='90' oninput='moveServo("low_left", this.value)'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Right Shoulder (IO10)</span><span id='val_high_right'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='high_right' min='0' max='180' value='90' oninput='moveServo("high_right", this.value)'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Left Shoulder (IO11)</span><span id='val_high_left'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='high_left' min='0' max='180' value='90' oninput='moveServo("high_left", this.value)'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Right Leg (IO9)</span><span id='val_low_right'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='low_right' min='0' max='180' value='90' oninput='moveServo("low_right", this.value)'>
            </div>
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

    <div class='card'>
        <h2>Software Calibration (NVS Saved Offsets)</h2>
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
    let localSensorEnabled = false; 
    let servoTimeouts = {}; 

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
        fetch('/save', { method: 'POST', body: JSON.stringify({ssid: s, pass: p}) })
        .then(() => alert('Saved. Rebooting.'));
    }

    function doAction(act) {
        fetch('/action', { method: 'POST', body: JSON.stringify({action: act}) });
    }

    function moveServo(id, angle) {
        document.getElementById('val_' + id).innerHTML = angle + '&deg;';
        clearTimeout(servoTimeouts[id]);
        servoTimeouts[id] = setTimeout(() => {
            fetch('/servo', { method: 'POST', body: JSON.stringify({id: id, angle: parseInt(angle)}) });
        }, 40); 
    }

    function toggleSensor() {
        localSensorEnabled = !localSensorEnabled;
        sendSensorConfig();
    }

    function updateThreshold(val) {
        document.getElementById('val_threshold').innerText = val + "cm";
        sendSensorConfig();
    }

    function sendSensorConfig() {
        let thresh = parseInt(document.getElementById('threshold').value) || 20;
        fetch('/sensor', { method: 'POST', body: JSON.stringify({enabled: localSensorEnabled, threshold: thresh}) });
    }

    function updateStatus() {
        fetch('/angles').then(r => r.json()).then(data => {
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
                    // Live polling animates the sliders if they aren't currently being dragged
                    if(el && (document.activeElement !== el || data.safety_lock)) { 
                        el.value = stats.angle; 
                        document.getElementById('val_' + leg).innerHTML = stats.angle + '&deg;'; 
                    }
                    let offEl = document.getElementById('off_' + leg);
                    if(offEl && document.activeElement !== offEl) { 
                        offEl.value = stats.offset; 
                    }
                }
            }

            localSensorEnabled = data.sensor_enabled;
            let liveSpan = document.getElementById('live_dist');
            if (data.sensor_enabled) {
                liveSpan.innerText = data.sensor_distance >= 0 ? data.sensor_distance.toFixed(1) : "Out of Range";
                liveSpan.style.color = (data.sensor_distance > 0 && data.sensor_distance < data.sensor_threshold) ? "#e74c3c" : "#27ae60";
            } else {
                liveSpan.innerText = "Disabled";
                liveSpan.style.color = "#7f8c8d";
            }

            let btn = document.getElementById('btn_sensor');
            if (data.sensor_enabled) {
                btn.innerText = "Disable Sensor"; btn.style.background = "#e74c3c";
            } else {
                btn.innerText = "Enable Sensor"; btn.style.background = "#27ae60";
            }

            if (document.activeElement !== document.getElementById('threshold')) {
                document.getElementById('threshold').value = data.sensor_threshold;
                document.getElementById('val_threshold').innerText = data.sensor_threshold + "cm";
            }
        });
    }

    window.onload = function() { updateStatus(); setInterval(updateStatus, 500); }

    function updateOffsets() {
        let ll = parseInt(document.getElementById('off_low_left').value) || 0;
        let hr = parseInt(document.getElementById('off_high_right').value) || 0;
        let hl = parseInt(document.getElementById('off_high_left').value) || 0;
        let lr = parseInt(document.getElementById('off_low_right').value) || 0;
        fetch('/calibrate', { method: 'POST', body: JSON.stringify({low_left: ll, high_right: hr, high_left: hl, low_right: lr, save: false}) });
    }

    function saveOffsets() {
        let ll = parseInt(document.getElementById('off_low_left').value) || 0;
        let hr = parseInt(document.getElementById('off_high_right').value) || 0;
        let hl = parseInt(document.getElementById('off_high_left').value) || 0;
        let lr = parseInt(document.getElementById('off_low_right').value) || 0;
        fetch('/calibrate', { method: 'POST', body: JSON.stringify({low_left: ll, high_right: hr, high_left: hl, low_right: lr, save: true}) })
        .then(() => alert('Calibration Saved.'));
    }
</script>
</body>
</html>
)raw_html";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
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
    char buf[200];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)(sizeof(buf) - 1)) ? remaining : (int)(sizeof(buf) - 1);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if(json) {
        cJSON *ssid_item = cJSON_GetObjectItem(json, "ssid");
        cJSON *pass_item = cJSON_GetObjectItem(json, "pass");
        if (ssid_item && pass_item) {
            wifi_save_credentials(ssid_item->valuestring, pass_item->valuestring);
            httpd_resp_sendstr(req, "OK");
            xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
        }
        cJSON_Delete(json);
    }
    return ESP_OK;
}

static esp_err_t servo_post_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)(sizeof(buf) - 1)) ? remaining : (int)(sizeof(buf) - 1);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json) {
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
    char buf[128];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)(sizeof(buf) - 1)) ? remaining : (int)(sizeof(buf) - 1);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json) {
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

static esp_err_t calibrate_post_handler(httpd_req_t *req) {
    char buf[200];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)(sizeof(buf) - 1)) ? remaining : (int)(sizeof(buf) - 1);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json) {
        int ll = 0, hr = 0, hl = 0, lr = 0;
        servo_get_offsets(&ll, &hr, &hl, &lr); // Get current to override below

        cJSON *ll_item = cJSON_GetObjectItem(json, "low_left");
        cJSON *hr_item = cJSON_GetObjectItem(json, "high_right");
        cJSON *hl_item = cJSON_GetObjectItem(json, "high_left");
        cJSON *lr_item = cJSON_GetObjectItem(json, "low_right");
        cJSON *sv_item = cJSON_GetObjectItem(json, "save");

        if (ll_item) ll = ll_item->valueint;
        if (hr_item) hr = hr_item->valueint;
        if (hl_item) hl = hl_item->valueint;
        if (lr_item) lr = lr_item->valueint;

        bool save_to_nvs = sv_item ? (cJSON_IsTrue(sv_item) || sv_item->valueint != 0) : false;
        servo_set_offsets(ll, hr, hl, lr, save_to_nvs);

        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t sensor_post_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;
    size_t recv_len = (remaining < (int)(sizeof(buf) - 1)) ? remaining : (int)(sizeof(buf) - 1);
    if ((ret = httpd_req_recv(req, buf, recv_len)) <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json) {
        cJSON *en_item = cJSON_GetObjectItem(json, "enabled");
        if (en_item) {
            sensor_set_enabled(cJSON_IsTrue(en_item) || (en_item->valueint != 0));
        }
        cJSON *th_item = cJSON_GetObjectItem(json, "threshold");
        if (th_item) {
            sensor_set_threshold(th_item->valueint);
        }

        ESP_LOGI(TAG, "Web UI Updated Sensor -> Enabled: %s, Threshold: %ldcm", 
                 sensor_is_enabled() ? "YES" : "NO", sensor_get_threshold());

        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    } else {
        httpd_resp_send_500(req);
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
    httpd_resp_send(req, json_str, strlen(json_str));
    
    cJSON_Delete(root);
    free(json_str);
    return ESP_OK;
}

void web_server_init() {
    if (server == NULL) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.max_uri_handlers = 12;
        
        if (httpd_start(&server, &config) == ESP_OK) {
            httpd_uri_t uri_index  = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_scan   = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_save   = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_servo  = { .uri = "/servo", .method = HTTP_POST, .handler = servo_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_act    = { .uri = "/action", .method = HTTP_POST, .handler = action_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_cal    = { .uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_angs   = { .uri = "/angles", .method = HTTP_GET, .handler = angles_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_sensor = { .uri = "/sensor", .method = HTTP_POST, .handler = sensor_post_handler, .user_ctx = NULL };
            
            httpd_register_uri_handler(server, &uri_index);
            httpd_register_uri_handler(server, &uri_scan);
            httpd_register_uri_handler(server, &uri_save);
            httpd_register_uri_handler(server, &uri_servo);
            httpd_register_uri_handler(server, &uri_act);
            httpd_register_uri_handler(server, &uri_cal);
            httpd_register_uri_handler(server, &uri_angs);
            httpd_register_uri_handler(server, &uri_sensor);
            
            ESP_LOGI(TAG, "Dashboard Server initialized successfully on port %d", config.server_port);
        }
    }
}