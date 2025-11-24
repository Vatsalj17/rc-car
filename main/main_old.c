// main.c
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_camera.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "camera_pins.h"
#include "freertos/queue.h"

static const char *TAG = "cam_car";

/* ---------- MOTOR PIN CONFIG - adjust if needed ---------- */
#define MOTOR_IN1   GPIO_NUM_12
#define MOTOR_IN2   GPIO_NUM_13
#define MOTOR_IN3   GPIO_NUM_14
#define MOTOR_IN4   GPIO_NUM_15
#define MOTOR_ENA   GPIO_NUM_27  // PWM for motor A
#define MOTOR_ENB   GPIO_NUM_33  // PWM for motor B

/* LEDC (PWM) settings */
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // duty resolution
#define LEDC_FREQUENCY          1000             // 1 kHz

/* Simple motor directions */
typedef enum {
    STOP = 0,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
} drive_dir_t;

typedef struct {
    drive_dir_t dir;
    uint8_t speed;
} motor_cmd_t;

// A handle for our command queue
static QueueHandle_t motor_cmd_queue = NULL;


/* ----- Camera config for AI-Thinker module ----- */
/* If you already have camera_pins.h or your own config,
   prefer to use it. Below is a default AI-Thinker mapping. */

camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sscb_sda   = SIOD_GPIO_NUM,
    .pin_sscb_scl   = SIOC_GPIO_NUM,
    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,
    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,
    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_VGA,
    .jpeg_quality   = 12,
    .fb_count       = 2,
    // .grab_mode      = CAMERA_GRAB_LATEST
};

/* ----- Motor helper functions ----- */
static void motor_gpio_init(void)
{
    gpio_reset_pin(MOTOR_IN1);
    gpio_reset_pin(MOTOR_IN2);
    gpio_reset_pin(MOTOR_IN3);
    gpio_reset_pin(MOTOR_IN4);

    gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN4, GPIO_MODE_OUTPUT);

    /* LEDC (PWM) init for ENA and ENB */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // channel for motor A (ENA)
    ledc_channel_config_t ledc_channel_a = {
        .gpio_num = MOTOR_ENA,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_a);

    // channel for motor B (ENB)
    ledc_channel_config_t ledc_channel_b = {
        .gpio_num = MOTOR_ENB,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_b);
}

static void set_pwm_duty(uint8_t channel, uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    uint32_t duty = (max_duty * duty_percent) / 100;
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

static void motor_set_stop(void)
{
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);
    set_pwm_duty(LEDC_CHANNEL_0, 0);
    set_pwm_duty(LEDC_CHANNEL_1, 0);
}

static void motor_set_forward(uint8_t speed)
{
    // Motor A forward
    gpio_set_level(MOTOR_IN1, 1);
    gpio_set_level(MOTOR_IN2, 0);
    // Motor B forward
    gpio_set_level(MOTOR_IN3, 1);
    gpio_set_level(MOTOR_IN4, 0);
    set_pwm_duty(LEDC_CHANNEL_0, speed);
    set_pwm_duty(LEDC_CHANNEL_1, speed);
}

static void motor_set_backward(uint8_t speed)
{
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 1);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 1);
    set_pwm_duty(LEDC_CHANNEL_0, speed);
    set_pwm_duty(LEDC_CHANNEL_1, speed);
}

static void motor_turn_left(uint8_t speed)
{
    // left turn by spinning right wheel forward and left wheel slower/back
    // Right wheel = motor B -> forward
    gpio_set_level(MOTOR_IN1, 0); // left wheel stop/back
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 1);
    gpio_set_level(MOTOR_IN4, 0);
    set_pwm_duty(LEDC_CHANNEL_0, 0);   // left motor off
    set_pwm_duty(LEDC_CHANNEL_1, speed);
}

static void motor_turn_right(uint8_t speed)
{
    // Right wheel stop, left wheel forward
    gpio_set_level(MOTOR_IN1, 1);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);
    set_pwm_duty(LEDC_CHANNEL_0, speed);
    set_pwm_duty(LEDC_CHANNEL_1, 0);
}

/* ----- HTTP server & handlers ----- */

/* small index page with video and buttons */
static const char index_html[] = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>ESP32-CAM Car</title><style>button{width:100px;height:60px;margin:6px;font-size:16px}</style></head><body>"
    "<h3>ESP32-CAM Remote</h3>"
    "<img id='stream' src='/stream' style='max-width:100%%;height:auto;display:block'/>"
    "<div>"
    "<button onclick=\"cmd('forward')\">Forward</button>"
    "<button onclick=\"cmd('backward')\">Backward</button>"
    "<button onclick=\"cmd('left')\">Left</button>"
    "<button onclick=\"cmd('right')\">Right</button>"
    "<button onclick=\"cmd('stop')\">Stop</button>"
    "</div>"
    "<div>Speed: <input id='speed' type='range' min='0' max='100' value='70' oninput='spd.value=value'><output id='spd'>70</output></div>"
    "<script>"
    "function cmd(c){var s=document.getElementById('speed').value;fetch('/control?cmd='+c+'&speed='+s).catch(console.error)}"
    "</script></body></html>";

/* stream handler: multipart MJPEG */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    char part_buf[64];
    const char *boundary = "frame";
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    if (res != ESP_OK) return res;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            
            // ============ THE FIX ============
            // The camera failed. Instead of looping forever,
            // we will stop the stream. This frees the server task
            // and prevents a server-wide crash. The client
            // (browser) will simply see the stream stop.
            // =================================
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a second
            break; // EXIT THE LOOP
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf),
            "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            boundary, (unsigned)fb->len);

        // Check if sending fails (e.g., client disconnected)
        if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
            esp_camera_fb_return(fb);
            break; // Client disconnected, exit loop
        }

        if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
            esp_camera_fb_return(fb);
            break; // Client disconnected, exit loop
        }

        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            esp_camera_fb_return(fb);
            break; // Client disconnected, exit loop
        }

        esp_camera_fb_return(fb);

        // Delay is now only on *success*
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Stream handler ending");
    return ESP_OK;
}

/* control handler for motor commands */
static void motor_control_task(void *pvParameters)
{
    motor_cmd_t cmd;
    ESP_LOGI(TAG, "Motor control task started");

    while (true) {
        // Wait forever for a command to arrive in the queue
        if (xQueueReceive(motor_cmd_queue, &cmd, portMAX_DELAY)) {
            
            // We got a command. Execute it.
            // This is the *only* task that calls the motor functions.
            switch (cmd.dir) {
                case FORWARD:
                    motor_set_forward(cmd.speed);
                    break;
                case BACKWARD:
                    motor_set_backward(cmd.speed);
                    break;
                case LEFT:
                    motor_turn_left(cmd.speed);
                    break;
                case RIGHT:
                    motor_turn_right(cmd.speed);
                    break;
                case STOP:
                default:
                    motor_set_stop();
                    break;
            }
        }
    }
}

// --- END ADD ---
// static esp_err_t control_handler(httpd_req_t *req)
// {
//     char buf[64];
//     if (httpd_req_get_url_query_len(req) > sizeof(buf)-1) {
//         httpd_resp_send_500(req);
//         return ESP_FAIL;
//     }
//
//     int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
//     if (ret != ESP_OK) {
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
//         return ESP_FAIL;
//     }
//
//     char cmd[16] = {0};
//     char speed_s[8] = {0};
//     httpd_query_key_value(buf, "cmd", cmd, sizeof(cmd));
//     httpd_query_key_value(buf, "speed", speed_s, sizeof(speed_s));
//     int speed = 70;
//     if (speed_s[0]) speed = atoi(speed_s);
//     if (speed < 0) speed = 0;
//     if (speed > 100) speed = 100;
//
//     if (strcmp(cmd, "forward") == 0) {
//         motor_set_forward((uint8_t)speed);
//     } else if (strcmp(cmd, "backward") == 0) {
//         motor_set_backward((uint8_t)speed);
//     } else if (strcmp(cmd, "left") == 0) {
//         motor_turn_left((uint8_t)speed);
//     } else if (strcmp(cmd, "right") == 0) {
//         motor_turn_right((uint8_t)speed);
//     } else {
//         motor_set_stop();
//     }
//
//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_send(req, "{\"ok\":true}\n", HTTPD_RESP_USE_STRLEN);
//     return ESP_OK;
// }
/* control handler for motor commands */
static esp_err_t control_handler(httpd_req_t *req)
{
    char buf[64];
    if (httpd_req_get_url_query_len(req) > sizeof(buf)-1) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    // --- THIS IS THE NEW LOGIC ---
    motor_cmd_t cmd = { .dir = STOP, .speed = 0 };
    char cmd_str[16] = {0};
    char speed_str[8] = {0};

    httpd_query_key_value(buf, "cmd", cmd_str, sizeof(cmd_str));
    httpd_query_key_value(buf, "speed", speed_str, sizeof(speed_str));
    
    int speed_val = 70;
    if (speed_str[0]) speed_val = atoi(speed_str);
    if (speed_val < 0) speed_val = 0;
    if (speed_val > 100) speed_val = 100;
    cmd.speed = (uint8_t)speed_val;

    if (strcmp(cmd_str, "forward") == 0) {
        cmd.dir = FORWARD;
    } else if (strcmp(cmd_str, "backward") == 0) {
        cmd.dir = BACKWARD;
    } else if (strcmp(cmd_str, "left") == 0) {
        cmd.dir = LEFT;
    } else if (strcmp(cmd_str, "right") == 0) {
        cmd.dir = RIGHT;
    } else {
        cmd.dir = STOP;
    }

    // Send the command to the queue. This is non-blocking and returns
    // immediately. It does NOT wait for the motor task.
    xQueueSend(motor_cmd_queue, &cmd, (TickType_t)0);

    // --- END NEW LOGIC ---

    // Immediately send response. The httpd task is now free.
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}\n", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* index page handler */
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
};
static httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
};
static httpd_uri_t control_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = control_handler,
    .user_ctx = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);
        httpd_register_uri_handler(server, &control_uri);
    }
    return server;
}

/* ----- Wi-Fi setup (soft-AP mode, easiest for mobile control) ----- */
static void initialise_wifi_softap(void)
{
    // SoftAP SSID/password - change if you like
    const char *ssid = "Khilona";
    const char *pass = "timepass";

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "",
            .ssid_len = 0,
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strncpy((char*)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strncpy((char*)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
    if (strlen(pass) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Soft-AP started. SSID:%s password:%s", ssid, pass);
}

/* ----- app_main ----- */
void app_main(void)
{
    nvs_flash_init();
    // --- ADD THIS SECTION ---
    // Create the motor command queue
    motor_cmd_queue = xQueueCreate(10, sizeof(motor_cmd_t));
    if (motor_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create motor_cmd_queue");
        return;
    }
    
    // Create and pin the motor control task to Core 1
    // (httpd and camera driver often run on Core 0)
    xTaskCreatePinnedToCore(
        motor_control_task,   // Function to implement the task
        "motor_task",         // Name of the task
        2048,                 // Stack size in words
        NULL,                 // Task input parameter
        5,                    // Priority of the task
        NULL,                 // Task handle
        1                     // Pin to Core 1
    );
    ESP_LOGI(TAG, "Init camera");
    // if (esp_camera_init(&camera_config) != ESP_OK) {
    //     ESP_LOGE(TAG, "Camera init failed");
    //     return;
    // }

    motor_gpio_init();
    motor_set_stop();

    initialise_wifi_softap(); // easiest: phone connects to ESP softap

    start_webserver();
    // motor_set_forward(70);

    ESP_LOGI(TAG, "Ready! Connect a phone to the AP and open http://192.168.4.1");
}
