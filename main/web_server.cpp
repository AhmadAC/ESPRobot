#include "web_server.h"
#include "wifi_manager.h"
#include "sensor_monitor.h"
#include "servo_controller.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include <string.h>

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
        :root { --primary: #0ea5e9; --bg: #0f172a; --card: #1e293b; --text: #f1f5f9; }
        body { font-family: -apple-system, sans-serif; background: var(--bg); color: var(--text); padding: 15px; margin: 0; }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; color: var(--primary); margin-bottom: 30px; }
        .card { background: var(--card); border-radius: 16px; padding: 25px; border: 1px solid #334155; margin-bottom: 25px; box-sizing: border-box; }
        h2 { border-bottom: 1px solid #334155; padding-bottom: 10px; color: var(--primary); margin-top: 0; margin-bottom: 15px; }
        .grid { display: grid; grid-template-columns: 1fr; gap: 20px; }
        @media (min-width: 600px) { .grid { grid-template-columns: 1fr 1fr; } }
        
        .ctrl-group { background: #0f172a; padding: 15px; border-radius: 12px; border: 1px solid #334155; border-left: 4px solid #3b82f6; transition: opacity 0.3s; }
        .ctrl-header { display: flex; justify-content: space-between; font-weight: bold; margin-bottom: 8px; color: #94a3b8; font-size: 0.9rem;}
        label { display: block; margin-bottom: 8px; font-weight: bold; font-size: 0.9rem; color: #cbd5e1; }
        
        input[type=range] { width: 100%; margin-bottom: 15px; cursor: pointer; accent-color: var(--primary); }
        input[type=range]:disabled { cursor: not-allowed; opacity: 0.5; }
        
        button { background: #3b82f6; color: white; border: none; padding: 12px 15px; border-radius: 10px; cursor: pointer; font-size: 15px; width: 100%; transition: transform 0.1s, opacity 0.2s; box-sizing: border-box; font-weight: bold; }
        button:hover { opacity: 0.9; }
        button:active { transform: scale(0.96); opacity: 0.8; }
        
        select, input[type=password], input[type=number] { width: 100%; padding: 12px; box-sizing: border-box; border: 1px solid #475569; border-radius: 10px; background: #0f172a; color: white; margin-bottom: 15px; }
        .pass-container { display: flex; gap: 10px; align-items: center; margin-bottom: 15px; }
        .pass-container input[type=password], .pass-container input[type=text] { margin-bottom: 0; flex-grow: 1; }
        .pass-container input[type=checkbox] { width: auto; margin: 0; }
        
        #status { font-weight: bold; color: #10b981; text-align: center; margin-top: 10px; font-size: 0.9rem; }
        #lock_banner { display: none; background: #7f1d1d; color: #fca5a5; padding: 12px; border-radius: 10px; text-align: center; font-weight: bold; margin-bottom: 20px; border: 1px solid #ef4444; }
        
        .locked-mode .ctrl-group, .locked-mode .btn-grid button, .locked-mode .sync-header { opacity: 0.5; pointer-events: none; }
        .btn-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 10px; margin-bottom: 25px; }
        .sync-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; }
        
        .btn-green { background: #10b981; }
        .btn-teal { background: #14b8a6; }
        .btn-purple { background: #8b5cf6; }
        .btn-orange { background: #f59e0b; }
        .btn-red { background: #ef4444; }
        .btn-gray { background: #475569; }
        .btn-dark { background: #1e293b; border: 1px solid #334155; }
        
        .text-sm { font-size: 0.85rem; color: #94a3b8; margin-top: 0; }
        .status-bar { padding: 12px; border-radius: 10px; font-weight: bold; text-align: center; font-size: 0.85rem; border: 1px solid; text-transform: uppercase; background: #172554; color: #93c5fd; border-color: #3b82f6; margin-bottom: 20px; }
    </style>
</head>
<body>
<div class='container'>
    <div class="status-bar">ESPRobot Control Dashboard</div>

    <div class='card'>
        <h2>Wi-Fi Provisioning</h2>
        <div class='grid'>
            <div>
                <button class='btn-gray' onclick='scan()'>Scan Wi-Fi Networks</button>
                <p id='status'>Ready to Scan</p>
            </div>
            <div>
                <label>Target SSID:</label>
                <select id='ssid'><option value=''>-- Select --</option></select>
                <label>Wi-Fi Password:</label>
                <div class='pass-container'>
                    <input type='password' id='pass' placeholder='Enter password'>
                    <input type='checkbox' onclick="let p=document.getElementById('pass'); p.type=(p.type==='password')?'text':'password';"> <small class="text-sm">Show</small>
                </div>
                <button class='btn-green' onclick='saveWiFi()'>Save and Connect</button>
            </div>
        </div>
        <hr style='border-color:#334155; margin: 25px 0;'>
        <div class='text-sm' style='margin-bottom:10px;'>Quick Boot Mode Switch</div>
        <div class='grid'>
            <button class='btn-orange' onclick='forceAP()'>Force AP Mode</button>
            <button class='btn-green' onclick='forceWiFi()'>Use Saved Wi-Fi</button>
        </div>
    </div>

    <div class='card' id='servo_card'>
        <h2>Robot Actions and Kinematics</h2>
        <div id='lock_banner'>WARNING: MOTORS LOCKED - Obstacle Detected!</div>
        
        <div class='btn-grid'>
            <button onclick='doAction("forward")'>Walk Forward</button>
            <button onclick='doAction("backward")'>Walk Backward</button>
            <button class='btn-teal' onclick='doAction("step_forward")'>Step Forward</button>
            <button class='btn-teal' onclick='doAction("step_backward")'>Step Backward</button>
            <button class='btn-teal' onclick='doAction("leap_forward")'>Leap Forward</button>
            <button class='btn-purple' onclick='doAction("left_wave")'>Left Wave</button>
            <button class='btn-purple' onclick='doAction("right_wave")'>Right Wave</button>
            <button class='btn-purple' onclick='doAction("back_left_wave")'>Back Left Wave</button>
            <button class='btn-purple' onclick='doAction("back_right_wave")'>Back Right Wave</button>
            <button class='btn-orange' onclick='doAction("crawl")'>Forward Crawl</button>
            <button class='btn-red' onclick='doAction("stop")'>STOP</button>
            <button class='btn-orange' onclick='doAction("sit")'>Sit</button>
            <button class='btn-orange' onclick='doAction("stand")'>Stand Up</button>
            <button class='btn-purple' onclick='doAction("stretch_down")'>Stretch Down</button>
            <button class='btn-purple' onclick='doAction("stretch_back")'>Stretch Back</button>
        </div>

        <div class='sync-header'>
            <h3 style='margin:0; font-size: 16px; color: var(--primary);'>Manual Joint Control</h3>
            <button id='btn_sync' onclick='toggleSync()' class='btn-gray' style='width: auto; padding: 8px 15px; margin: 0;'>Sync Legs: OFF</button>
        </div>

        <div class='grid'>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Left Leg (IO12)</span><span id='val_low_left'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='low_left' min='0' max='180' value='90' oninput='moveServo("low_left", this.value)' onmousedown='dragStart("low_left")' onmouseup='dragEnd()' ontouchstart='dragStart("low_left")' ontouchend='dragEnd()'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Left Shoulder (IO11)</span><span id='val_high_left'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='high_left' min='0' max='180' value='90' oninput='moveServo("high_left", this.value)' onmousedown='dragStart("high_left")' onmouseup='dragEnd()' ontouchstart='dragStart("high_left")' ontouchend='dragEnd()'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>Low Right Leg (IO9)</span><span id='val_low_right'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='low_right' min='0' max='180' value='90' oninput='moveServo("low_right", this.value)' onmousedown='dragStart("low_right")' onmouseup='dragEnd()' ontouchstart='dragStart("low_right")' ontouchend='dragEnd()'>
            </div>
            <div class='ctrl-group'>
                <div class='ctrl-header'><span>High Right Shoulder (IO10)</span><span id='val_high_right'>90&deg;</span></div>
                <input type='range' class='srv-slider' id='high_right' min='0' max='180' value='90' oninput='moveServo("high_right", this.value)' onmousedown='dragStart("high_right")' onmouseup='dragEnd()' ontouchstart='dragStart("high_right")' ontouchend='dragEnd()'>
            </div>
        </div>
    </div>

    <div class='card'>
        <h2>Pose Calibration & Zeroing</h2>
        <div class='grid'>
            <div class='ctrl-group' style='border-left-color: #f59e0b;'>
                <label>Target to Calibrate:</label>
                <select id='calib_target' onchange='loadCalibTarget()' style='font-weight:bold; cursor:pointer;'>
                    <option value='offsets'>1. Hardware Zeroing (Offsets)</option>
                    <option value='sit'>2. Pose: Sit</option>
                    <option value='stand'>3. Pose: Stand Up</option>
                    <option value='stretch_down'>4. Pose: Stretch Down</option>
                    <option value='stretch_back'>5. Pose: Stretch Back</option>
                    <option value='stop'>6. Pose: Stop (Default)</option>
                </select>
                <p class='text-sm' style='margin-top:10px; line-height:1.4;'>First, use Hardware Zeroing to make all motors physically 90&deg;. Then calibrate limits for buttons.</p>
                <button class='btn-red' style='margin-top: 15px;' onclick='resetCal()'>Reset NVS to Firmware Defaults</button>
            </div>
            <div class='ctrl-group' style='border-left-color: #94a3b8; text-align:center;'>
                <p style='margin-top:0; font-size:13px; font-weight:bold;'>Compile Defaults into Firmware</p>
                <p class='text-sm' style='line-height:1.4;'>Save your finalized configuration to a C++ file to hardcode settings securely for your next compile.</p>
                <button class='btn-dark' style='margin-top: 10px;' onclick='downloadCalib()'>Download C++ Config</button>
            </div>
        </div>
        <div class='grid' style='margin-top:15px;'>
            <div>
                <label>Low Left Motor Limit:</label>
                <input type='number' id='cal_ll' value='0'>
                <label>Low Right Motor Limit:</label>
                <input type='number' id='cal_lr' value='0'>
            </div>
            <div>
                <label>High Left Motor Limit:</label>
                <input type='number' id='cal_hl' value='0'>
                <label>High Right Motor Limit:</label>
                <input type='number' id='cal_hr' value='0'>
            </div>
        </div>
        <div class='grid' style='margin-top:15px;'>
            <button class='btn-blue' onclick='testCalib()'>Test Pose or Offset</button>
            <button class='btn-orange' onclick='saveCalib()'>Save to Robot NVS Storage</button>
        </div>
    </div>

    <div class='card'>
        <h2>Ultrasonic Obstacle Safety Monitor</h2>
        <div class='grid'>
            <div class='ctrl-group' style='border-left-color: #ef4444;'>
                <label>Sensor Toggle Status:</label>
                <button id='btn_sensor' onclick='toggleSensor()'>Enable Sensor</button>
                <p style='font-size: 16px; margin-top: 15px;'>Live Distance: <span id='live_dist' style='font-weight: bold; color: var(--primary);'>--</span> cm</p>
            </div>
            <div class='ctrl-group' style='border-left-color: #8b5cf6;'>
                <div class='ctrl-header'><span>Safety Halt Threshold</span><span id='val_threshold'>20cm</span></div>
                <input type='range' id='threshold' min='5' max='100' value='20' onchange='updateThreshold(this.value)'>
                <p class='text-sm' style='margin-bottom: 15px; line-height: 1.4;'>Triggers immediately when distance crosses limit.</p>
                
                <div class='ctrl-header'><span>Reaction Delay</span><span id='val_reaction'>0ms</span></div>
                <input type='range' id='reaction' min='0' max='5000' step='100' value='0' onchange='updateReaction(this.value)'>
                <p class='text-sm' style='line-height: 1.4;'>Wait time between detection and automatic action execution.</p>
            </div>
        </div>
        <div class='grid' style='margin-top:15px;'>
            <div class='ctrl-group' style='border-left-color: #f59e0b;'>
                <label>Action When Tripped (Obstacle Detected):</label>
                <select id='sensor_tripped_action' onchange='sendSensorConfig()'>
                    <option value='stop'>Stop (Default)</option>
                    <option value='forward'>Walk Forward</option>
                    <option value='backward'>Walk Backward</option>
                    <option value='step_forward'>Step Forward</option>
                    <option value='step_backward'>Step Backward</option>
                    <option value='leap_forward'>Leap Forward</option>
                    <option value='left_wave'>Left Wave</option>
                    <option value='right_wave'>Right Wave</option>
                    <option value='back_left_wave'>Back Left Wave</option>
                    <option value='back_right_wave'>Back Right Wave</option>
                    <option value='crawl'>Forward Crawl</option>
                    <option value='sit'>Sit</option>
                    <option value='stand'>Stand Up</option>
                    <option value='stretch_down'>Stretch Down</option>
                    <option value='stretch_back'>Stretch Back</option>
                    <option value='none'>None (Do nothing)</option>
                </select>
            </div>
            <div class='ctrl-group' style='border-left-color: #10b981;'>
                <label>Action When Cleared (Obstacle Removed):</label>
                <select id='sensor_cleared_action' onchange='sendSensorConfig()'>
                    <option value='stand'>Stand Up</option>
                    <option value='forward'>Walk Forward</option>
                    <option value='backward'>Walk Backward</option>
                    <option value='step_forward'>Step Forward</option>
                    <option value='step_backward'>Step Backward</option>
                    <option value='leap_forward'>Leap Forward</option>
                    <option value='left_wave'>Left Wave</option>
                    <option value='right_wave'>Right Wave</option>
                    <option value='back_left_wave'>Back Left Wave</option>
                    <option value='back_right_wave'>Back Right Wave</option>
                    <option value='crawl'>Forward Crawl</option>
                    <option value='stop'>Stop</option>
                    <option value='sit'>Sit</option>
                    <option value='stretch_down'>Stretch Down</option>
                    <option value='stretch_back'>Stretch Back</option>
                    <option value='none'>None (Do nothing)</option>
                </select>
            </div>
        </div>
    </div>

</div>

<script>
    let localSensorEnabled = false; 
    let syncEnabled = false;
    let servoTimeouts = {}; 
    let allCalibrations = {};
    let activeDrag = null;
    const legMap = { 'low_left': 'll', 'high_right': 'hr', 'high_left': 'hl', 'low_right': 'lr' };

    function fetchJSON(url, bodyData) {
        return fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(bodyData)
        });
    }

    function forceAP(){
        if(confirm("Switch to AP Mode and reboot?")) fetch('/switch_to_ap', { method: 'POST' }).then(() => alert('Rebooting...'));
    }
    function forceWiFi(){
        if(confirm("Switch to Wi-Fi Mode and reboot?")) fetch('/switch_to_wifi', { method: 'POST' }).then(() => alert('Rebooting...'));
    }

    function dragStart(id) { activeDrag = id; }
    function dragEnd() { activeDrag = null; }

    function toggleSync() {
        syncEnabled = !syncEnabled;
        let btn = document.getElementById('btn_sync');
        if (syncEnabled) {
            btn.innerText = "Sync Legs: ON";
            btn.classList.remove('btn-gray'); btn.classList.add('btn-green');
        } else {
            btn.innerText = "Sync Legs: OFF";
            btn.classList.remove('btn-green'); btn.classList.add('btn-gray');
        }
    }

    function isDragging(leg) {
        if (leg === activeDrag) return true;
        if (syncEnabled && activeDrag) {
            if (activeDrag === 'high_left' && leg === 'high_right') return true;
            if (activeDrag === 'high_right' && leg === 'high_left') return true;
            if (activeDrag === 'low_left' && leg === 'low_right') return true;
            if (activeDrag === 'low_right' && leg === 'low_left') return true;
        }
        return false;
    }

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
            alert('Calibration Updated and Saved to ESP32 Storage!');
            fetchCalibrations();
        });
    }

    function downloadCalib() { window.location.href = '/download_cal'; }

    function resetCal() {
        if (confirm("Are you sure you want to clear NVS storage? This will clear custom poses/offsets and reboot to reload firmware defaults.")) {
            fetchJSON('/reset_cal', {}).then(() => {
                alert("NVS partitions erased successfully. Robot is now rebooting...");
            });
        }
    }

    function scan() {
        document.getElementById('status').innerText = 'Scanning Wi-Fi...';
        fetch('/scan').then(r => r.json()).then(d => {
            let s = document.getElementById('ssid');
            s.innerHTML = '<option value="">-- Select --</option>';
            d.forEach(n => { s.innerHTML += '<option value="'+n+'">'+n+'</option>'; });
            document.getElementById('status').innerText = 'Found ' + d.length + ' network(s)';
        }).catch(() => document.getElementById('status').innerText = 'Scan Error');
    }

    function saveWiFi() {
        let s = document.getElementById('ssid').value;
        let p = document.getElementById('pass').value;
        if(!s) return alert('Select SSID');
        fetchJSON('/save', {ssid: s, pass: p}).then(() => alert('Saved. Rebooting...'));
    }

    function doAction(act) {
        if (allCalibrations[act]) {
            let p = allCalibrations[act];
            for (let leg in legMap) {
                let val = p[legMap[leg]];
                let el = document.getElementById(leg);
                if (el) el.value = val;
                let valEl = document.getElementById('val_' + leg);
                if (valEl) valEl.innerHTML = val + '&deg;';
            }
        }
        fetchJSON('/action', {action: act});
    }

    function moveServo(id, angle) {
        document.getElementById('val_' + id).innerHTML = angle + '&deg;';
        
        let payload = {};
        payload[legMap[id]] = parseInt(angle);
        
        if (syncEnabled) {
            let pairedId = null;
            if (id === 'high_left') pairedId = 'high_right';
            else if (id === 'high_right') pairedId = 'high_left';
            else if (id === 'low_left') pairedId = 'low_right';
            else if (id === 'low_right') pairedId = 'low_left';

            if (pairedId) {
                let pairedEl = document.getElementById(pairedId);
                if (pairedEl.value !== angle) {
                    pairedEl.value = angle;
                    document.getElementById('val_' + pairedId).innerHTML = angle + '&deg;';
                    payload[legMap[pairedId]] = parseInt(angle);
                }
            }
        }

        clearTimeout(servoTimeouts['bulk']);
        servoTimeouts['bulk'] = setTimeout(() => {
            fetchJSON('/servo', payload);
        }, 30);
    }

    function toggleSensor() {
        localSensorEnabled = !localSensorEnabled;
        let btn = document.getElementById('btn_sensor');
        if (localSensorEnabled) {
            btn.innerText = "Disable Sensor"; btn.classList.remove('btn-green'); btn.classList.add('btn-red');
        } else {
            btn.innerText = "Enable Sensor"; btn.classList.remove('btn-red'); btn.classList.add('btn-green');
        }
        sendSensorConfig();
    }

    function updateThreshold(val) {
        document.getElementById('val_threshold').innerText = val + "cm";
        sendSensorConfig();
    }

    function updateReaction(val) {
        document.getElementById('val_reaction').innerText = val + "ms";
        sendSensorConfig();
    }

    function sendSensorConfig() {
        let thresh = parseInt(document.getElementById('threshold').value) || 20;
        let react = parseInt(document.getElementById('reaction').value) || 0;
        let tripAct = document.getElementById('sensor_tripped_action').value;
        let clearAct = document.getElementById('sensor_cleared_action').value;
        fetchJSON('/sensor', {
            enabled: localSensorEnabled, 
            threshold: thresh,
            reaction_time: react,
            tripped_action: tripAct,
            cleared_action: clearAct
        });
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
                        if(el && (!isDragging(leg) || data.safety_lock)) { 
                            el.value = stats.angle; 
                            document.getElementById('val_' + leg).innerHTML = stats.angle + '&deg;'; 
                        }
                    }
                }

                let liveSpan = document.getElementById('live_dist');
                if (data.sensor_enabled) {
                    liveSpan.innerText = data.sensor_distance >= 0 ? data.sensor_distance.toFixed(1) : "Out of Range";
                    liveSpan.style.color = (data.sensor_distance > 0 && data.sensor_distance < data.sensor_threshold) ? "#ef4444" : "#10b981";
                } else {
                    liveSpan.innerText = "Disabled";
                    liveSpan.style.color = "#94a3b8";
                }

                if (document.activeElement !== document.getElementById('btn_sensor')) {
                    localSensorEnabled = data.sensor_enabled;
                    let btn = document.getElementById('btn_sensor');
                    if (data.sensor_enabled) {
                        btn.innerText = "Disable Sensor"; btn.className = "btn-red";
                    } else {
                        btn.innerText = "Enable Sensor"; btn.className = "btn-green";
                    }
                }

                if (document.activeElement !== document.getElementById('threshold')) {
                    document.getElementById('threshold').value = data.sensor_threshold;
                    document.getElementById('val_threshold').innerText = data.sensor_threshold + "cm";
                }

                if (document.activeElement !== document.getElementById('reaction')) {
                    document.getElementById('reaction').value = data.sensor_reaction_time || 0;
                    document.getElementById('val_reaction').innerText = (data.sensor_reaction_time || 0) + "ms";
                }

                if (document.activeElement !== document.getElementById('sensor_tripped_action')) {
                    document.getElementById('sensor_tripped_action').value = data.sensor_tripped_action || 'stop';
                }

                if (document.activeElement !== document.getElementById('sensor_cleared_action')) {
                    document.getElementById('sensor_cleared_action').value = data.sensor_cleared_action || 'stand';
                }
                
                setTimeout(updateStatus, 800); 
            })
            .catch(err => {
                setTimeout(updateStatus, 2000); 
            });
    }

    window.onload = function() { 
        fetchCalibrations();
        updateStatus(); 
    }
</script>
</body>
</html>
)raw_html";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t captive_portal_redirect(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
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
            
            // Clear AP force flag when saving new credentials
            nvs_handle_t my_handle;
            if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
                nvs_set_u8(my_handle, "force_ap", 0);
                nvs_commit(my_handle);
                nvs_close(my_handle);
            }

            xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
        }
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "OK");
    }
    return ESP_OK;
}

static esp_err_t switch_ap_post_handler(httpd_req_t *req) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, "force_ap", 1);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    httpd_resp_sendstr(req, "OK");
    xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t switch_wifi_post_handler(httpd_req_t *req) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, "force_ap", 0);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    httpd_resp_sendstr(req, "OK");
    xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t servo_post_handler(httpd_req_t *req) {
    cJSON *json = NULL;
    if (get_post_json(req, &json) == ESP_OK) {
        cJSON *ll = cJSON_GetObjectItem(json, "ll");
        cJSON *lr = cJSON_GetObjectItem(json, "lr");
        cJSON *hl = cJSON_GetObjectItem(json, "hl");
        cJSON *hr = cJSON_GetObjectItem(json, "hr");
        
        cJSON *id_item = cJSON_GetObjectItem(json, "id");
        cJSON *angle_item = cJSON_GetObjectItem(json, "angle");
        
        if (ll || lr || hl || hr) {
            if (ll) servo_set_target_silent("low_left", ll->valueint);
            if (lr) servo_set_target_silent("low_right", lr->valueint);
            if (hl) servo_set_target_silent("high_left", hl->valueint);
            if (hr) servo_set_target_silent("high_right", hr->valueint);
            servo_apply_targets();
        } else if (id_item && angle_item) {
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
        cJSON *re_item = cJSON_GetObjectItem(json, "reaction_time");
        if (re_item) {
            sensor_set_reaction_time(re_item->valueint);
        }
        cJSON *trip_item = cJSON_GetObjectItem(json, "tripped_action");
        cJSON *clear_item = cJSON_GetObjectItem(json, "cleared_action");
        if (trip_item && clear_item) {
            sensor_set_actions(trip_item->valuestring, clear_item->valuestring);
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
    cJSON_AddNumberToObject(root, "sensor_reaction_time", sensor_get_reaction_time());
    
    // Safety Routine Metadata Export
    cJSON_AddStringToObject(root, "sensor_tripped_action", sensor_get_tripped_action());
    cJSON_AddStringToObject(root, "sensor_cleared_action", sensor_get_cleared_action());

    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    
    // Kept keep-alive open intentionally to drastically improve TCP pool speeds
    httpd_resp_send(req, json_str, strlen(json_str));
    
    cJSON_Delete(root);
    free(json_str);
    return ESP_OK;
}

static esp_err_t reset_cal_post_handler(httpd_req_t *req) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_erase_all(my_handle);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGW(TAG, "NVS Storage Erased. Rebooting...");
    }
    xTaskCreate(delayed_reboot_task, "reboot_task", 2048, NULL, 5, NULL);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// Graceful Favicon Handler to suppress missing /favicon.ico logging pollution
static esp_err_t favicon_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

void web_server_init() {
    if (server == NULL) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        
        // Optimize configuration for swift request processing and thread isolation
        config.uri_match_fn = httpd_uri_match_wildcard; // Allows wildcard redirects for Captive Portal
        config.max_uri_handlers = 20;                   // Expand limits to fit our new routing
        config.core_id = 1;                             // Restrict the HTTP server strictly to Core 1
        config.task_priority = 5;                       // Standardized to baseline HTTP daemon logic
        config.stack_size = 10240;                      // Guarantee stack space for processing JSON inputs safely
        config.lru_purge_enable = true;                 // Purge stale sockets dynamically to maintain active tunnels
        config.max_open_sockets = 7;                    // Max out default LWIP sockets without requiring sdkconfig rebuilds
        
        // --- High-Latency/Fringe Wi-Fi Resilience ---
        config.recv_wait_timeout = 15;                  // 15 seconds to tolerate packet loss on -85dBm Wi-Fi connections
        config.send_wait_timeout = 15;                  // 15 seconds to tolerate packet loss on -85dBm Wi-Fi connections
        
        if (httpd_start(&server, &config) == ESP_OK) {
            httpd_uri_t uri_index    = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_scan     = { .uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_save     = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_switchap = { .uri = "/switch_to_ap", .method = HTTP_POST, .handler = switch_ap_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_switchwi = { .uri = "/switch_to_wifi", .method = HTTP_POST, .handler = switch_wifi_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_servo    = { .uri = "/servo", .method = HTTP_POST, .handler = servo_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_act      = { .uri = "/action", .method = HTTP_POST, .handler = action_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_cal      = { .uri = "/calibrate", .method = HTTP_POST, .handler = calibrate_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_caljson  = { .uri = "/calibrations_json", .method = HTTP_GET, .handler = calibrations_json_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_dlcal    = { .uri = "/download_cal", .method = HTTP_GET, .handler = download_cal_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_angs     = { .uri = "/angles", .method = HTTP_GET, .handler = angles_get_handler, .user_ctx = NULL };
            httpd_uri_t uri_sensor   = { .uri = "/sensor", .method = HTTP_POST, .handler = sensor_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_resetcal = { .uri = "/reset_cal", .method = HTTP_POST, .handler = reset_cal_post_handler, .user_ctx = NULL };
            httpd_uri_t uri_favicon  = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler, .user_ctx = NULL };
            
            // Standard OS Captive Portal Endpoints
            httpd_uri_t uri_cp1      = { .uri = "/generate_204", .method = HTTP_GET, .handler = captive_portal_redirect, .user_ctx = NULL };
            httpd_uri_t uri_cp2      = { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_portal_redirect, .user_ctx = NULL };
            httpd_uri_t uri_cp3      = { .uri = "/ncsi.txt", .method = HTTP_GET, .handler = captive_portal_redirect, .user_ctx = NULL };
            
            httpd_register_uri_handler(server, &uri_index);
            httpd_register_uri_handler(server, &uri_scan);
            httpd_register_uri_handler(server, &uri_save);
            httpd_register_uri_handler(server, &uri_switchap);
            httpd_register_uri_handler(server, &uri_switchwi);
            httpd_register_uri_handler(server, &uri_servo);
            httpd_register_uri_handler(server, &uri_act);
            httpd_register_uri_handler(server, &uri_cal);
            httpd_register_uri_handler(server, &uri_caljson);
            httpd_register_uri_handler(server, &uri_dlcal);
            httpd_register_uri_handler(server, &uri_angs);
            httpd_register_uri_handler(server, &uri_sensor);
            httpd_register_uri_handler(server, &uri_resetcal);
            httpd_register_uri_handler(server, &uri_favicon);
            httpd_register_uri_handler(server, &uri_cp1);
            httpd_register_uri_handler(server, &uri_cp2);
            httpd_register_uri_handler(server, &uri_cp3);
            
            ESP_LOGI(TAG, "Dashboard Server initialized successfully on port %d", config.server_port);
        }
    }
}