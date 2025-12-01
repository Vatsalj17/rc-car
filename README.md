# ESP32 ADAS WiFi RC Car

**Team:** Group No. 65 (Yuvraj Kumar, Ume Kulsoom, Vatsal Jaiswal)  
**Device:** ESP32 (ESP-IDF Framework)

## 1. Project Overview
This project implements a WiFi-controlled RC car with **ADAS (Advanced Driver Assistance Systems)** features. It operates in two modes:
1.  **Manual Mode:** User controls the car via a Web Interface or App. Includes collision prevention (stops automatically if an obstacle is < 18cm).
2.  **Automatic Mode:** The car drives itself, detecting obstacles and turning right automatically to avoid collisions.

The system uses **FreeRTOS** to handle motor control, sensor readings, and the HTTP web server concurrently.

## 2. Hardware Architecture

| Component | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **Motor IN1** | GPIO 12 | Left Motor Logic Input 1 |
| **Motor IN2** | GPIO 13 | Left Motor Logic Input 2 |
| **Motor IN3** | GPIO 14 | Right Motor Logic Input 1 |
| **Motor IN4** | GPIO 15 | Right Motor Logic Input 2 |
| **Motor ENA** | GPIO 27 | Left Motor PWM Speed Control |
| **Motor ENB** | GPIO 33 | Right Motor PWM Speed Control |
| **Ultrasonic Trig**| GPIO 25 | Sensor Trigger Pulse |
| **Ultrasonic Echo**| GPIO 26 | Sensor Echo Input |

**Key Specifications:**
* **PWM Frequency:** 1 kHz (LEDC High Speed Mode)
* **PWM Resolution:** 13-bit (Values 0 - 8191)
* **WiFi Mode:** SoftAP (Access Point)
* **SSID:** `Khilona` (Open Network)
* **IP Address:** `192.168.4.1`

## 3. Installation & Flashing

1.  **Prerequisites:** Install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).
2.  **Clone/Download** this repository.
3.  **Build and Flash:**
    ```bash
    idf.py set-target esp32
    idf.py build
    idf.py -p (YOUR_PORT) flash monitor
    ```

## 4. API Reference (HTTP Endpoints)

The car exposes a RESTful API. You can control it by sending HTTP GET requests to `http://192.168.4.1`.

### Mode Switching
| Action | Endpoint | Description |
| :--- | :--- | :--- |
| **Switch to Auto** | `/switch_auto` | Enables autonomous driving loop. |
| **Switch to Manual**| `/switch_manual` | Disables auto loop; stops motors. |

### Manual Control
*Base URL:* `/cmd?do=[command]`

| Command | Action |
| :--- | :--- |
| `forward` | Move forward at manual speed. |
| `backward`| Move backward at manual speed. |
| `left` | Turn left (tank turn). |
| `right` | Turn right (tank turn). |
| `stop` | Stop all motors. |

### Speed Control
| Parameter | Endpoint | Range |
| :--- | :--- | :--- |
| **Manual Speed** | `/set_manual_speed?val=[40-100]` | Sets duty cycle %. |
| **Auto Speed** | `/set_auto_speed?val=[50-90]` | Sets cruising speed %. |

### Auto Mode Control
*Base URL:* `/auto?do=[action]`

| Action | Description |
| :--- | :--- |
| `pause` | Pauses the auto-driving logic (stops car). |
| `resume` | Resumes the auto-driving logic. |

## 5. Feature Breakdown

### RTOS Task Management
* **`app_main`**: Initializes GPIO, PWM, WiFi, and Web Server.
* **`ultrasonic_task`**: A dedicated FreeRTOS task that continuously pings the sensor. It handles the "reflex" actions (emergency stop in Manual, obstacle avoidance turn in Auto) to ensure low latency.

### Safety Mechanisms
* **Manual Safety:** Even if the user presses "Forward", the firmware overrides the command if the ultrasonic sensor detects an object closer than **18 cm**.
* **Timeout Handling:** If the sensor disconnects, the code defaults to a safe state to prevent runaway behavior.
