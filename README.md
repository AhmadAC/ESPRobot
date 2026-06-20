### Project Architecture Overview
The **ESPRobot** system consists of an ESP32-S3-Zero controller programmed in C++ using the **ESP-IDF v5.2** framework. The robot features:
* **4-Leg Servo Control** with individual, group, and pair sweep modes.
* **Software-based Calibration Offsets** stored persistently in memory to align the legs.
* **APSTA Dual Wi-Fi Connection** allowing you to control the robot locally or configure its connection to a home router.
* **Automated CI/CD Compilation Pipeline** to compile the firmware and generate 1-click web browser installation binaries.

---

### File 1: `main/main.cpp` (The Core Firmware)
This is the main application that compiles directly onto your ESP32-S3-Zero. It is broken down into five major functional areas:

#### A. Pin Configuration and Servo Setup
The script assigns physical GPIO pins to the robot's hardware components:
* **`GPIO 12` (Low Left Leg)** $\rightarrow$ LEDC Channel 0
* **`GPIO 10` (High Right Shoulder)** $\rightarrow$ LEDC Channel 1
* **`GPIO 11` (High Left Shoulder)** $\rightarrow$ LEDC Channel 2
* **`GPIO 9` (Low Right Leg)** $\rightarrow$ LEDC Channel 3
* **`GPIO 4` (Ultrasonic Trig)** & **`GPIO 5` (Ultrasonic Echo)** (allocated for distance sensing)

#### B. Hardware-Accelerated PWM Driver (`write_servo_calibrated`)
Standard hobby servos require a repeating pulse frame at **50Hz** (20ms period). Instead of using standard processor-intensive delay loops, the code configures the ESP32-S3’s hardware **LEDC (LED Controller)** peripheral:
* It sets up a **13-bit timer** (providing 8,192 distinct levels of duty cycle resolution).
* When a motion command is executed, it calculates `Target Angle + Calibration Offset`.
* It clamps the final value between `[0, 180]` to protect the mechanical linkages from binding.
* It translates the angle to a duty cycle: 0° maps to 204 steps (`~0.5ms` pulse width) and 180° maps to 1024 steps (`~2.5ms` pulse width).

#### C. Persistent Storage Management (NVS)
Using the ESP-IDF Non-Volatile Storage (NVS) APIs, the code manages two sets of persistent records across system restarts:
* **Leg Calibrations**: Stores offset integers (`ll_off`, `hr_off`, `hl_off`, `lr_off`) so manual link alignments persist through reboot cycles.
* **Station Wi-Fi Credentials**: Stores the SSID and passphrase for connecting to an external Wi-Fi router.

#### D. APSTA Dual-Radio Operations (`app_main`)
During startup, the system activates the Wi-Fi transceiver in simultaneous **APSTA (Access Point + Station)** mode:
* **SoftAP**: Hosts an open configuration network named `ESPRobot_Config` (operating on IP `192.168.4.1`).
* **Station**: Reads the storage block in NVS. If credentials exist, it attempts to connect to your home Wi-Fi station in the background. This ensures you can always reach the controller, even if the robot loses connection to your home router.

#### E. Web Server API Router and HTML Payload
A lightweight Web Server runs on port 80, serving an interactive Javascript dashboard (embedded directly within the C++ code as a raw multiline string) and processing several REST API endpoints:
* **`GET /`**: Delivers the CSS/HTML/JS control dashboard to your phone or PC.
* **`GET /angles`**: Pulls the active physical angles and loaded NVS offset parameters to sync the slider interface state when a browser connects.
* **`GET /scan`**: Executes a blocking background Wi-Fi network scan, compiling found router names into a JSON list.
* **`POST /save`**: Takes new Wi-Fi credentials, writes them to NVS, and initiates a safe, delayed system reboot task (`delayed_reboot_task`) after a 2-second grace period.
* **`POST /servo`**: Intercepts real-time sliding events. It parses JSON payloads (e.g., `{"id": "front", "angle": 120}`) and adjusts the single leg, pair (front/back), or all leg motors together.
* **`POST /calibrate`**: Accepts calibration offset values, applies them instantly so you can visually verify alignment, and optionally writes them permanently to NVS flash.

---

### File 2: Build Scripts (`CMakeLists.txt`)
ESP-IDF uses CMake to determine project properties and track build dependencies.
* **Root `CMakeLists.txt`**: Declares that this is an ESP-IDF project and names the final application `cobot_esp32_s3`.
* **Component `main/CMakeLists.txt`**: Instructs the compiler to pull and compile the `main.cpp` source code and expose current-directory headers.

---

### File 3: `sdkconfig.defaults` (System Configuration)
This configuration file instructs the compiler to override default ESP-IDF settings before compiling:
* **`CONFIG_FREERTOS_HZ=1000`**: By default, the FreeRTOS system scheduler ticks at 100Hz (every 10ms). Standard task delays (`vTaskDelay`) can only resolve in multiples of 10ms at this rate. By raising the frequency to **1000Hz** (1ms ticks), the software can execute high-precision, sub-10ms timing tasks, which helps provide smooth, jitter-free sweeps for your 9g servo legs.

---

### File 4: `.github/workflows/main.yml` (CI/CD Compilation Pipeline)
This script automates the compiling process using **GitHub Actions** every time you push code updates or open pull requests to your `main` or `master` branches:
1. **Runner Environment**: Runs on a clean virtual Linux container containing the official **`espressif/idf:release-v5.2`** development environment.
2. **Setup and Formatting check**: Checks out your repository code and sweeps the project directory for custom `sdkconfig.defaults` configuration overrides, repairing any formatting inconsistencies.
3. **Firmware Building**: Targets the **ESP32-S3** core architecture and runs `idf.py build` to compile the C++ source files into machine code.
4. **Binary Processing**: Isolates and copies the generated binary outputs (`bootloader.bin`, `partition-table.bin`, and `cobot_esp32_s3.bin`) to an output directory called `build_out/`.
5. **Web Flasher Manifest Generation**: Programmatically writes a `manifest.json` file. WebSerial installation tools (like ESP Web Flasher) use this manifest file to flash your ESP32-S3-Zero with a single click directly from a compatible web browser.
6. **Artifact Upload**: Bundles the entire compiled installation package and uploads it as a GitHub artifact named `esprobot-binaries` for easy download.
