/*
 * Mini Project (ADAS Car Implementation)
 * Group No. 65
 * Made by:- Yuvraj Kumar, Ume Kulsoom, Vatsal Jaiswal
 */



#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "esp_http_server.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "rom/ets_sys.h"
#include "esp_err.h"

static const char *TAG = "toy_car";

/* ------------------------- PIN CONFIG ------------------------- */
#define IN1_GPIO 12
#define IN2_GPIO 13
#define IN3_GPIO 14
#define IN4_GPIO 15
#define ENA_GPIO 27
#define ENB_GPIO 33

#define TRIG_PIN 25
#define ECHO_PIN 26

/* ------------------------- SPEED + THRESHOLDS ------------------------- */
#define MANUAL_MIN_SPEED 40
#define MANUAL_MAX_SPEED 100

#define AUTO_MIN_SPEED 50
#define AUTO_MAX_SPEED 90
#define AUTO_TURN_SPEED 40

#define MANUAL_STOP_THRESHOLD_CM 18.0f
#define AUTO_TURN_THRESHOLD_CM   25.0f

#define PWM_MAX_DUTY 8191

/* ------------------------- GLOBAL STATE ------------------------- */
typedef enum { STATE_STOP = 0, STATE_FORWARD, STATE_BACKWARD, STATE_LEFT, STATE_RIGHT } motor_state_t;
static volatile motor_state_t g_state = STATE_STOP;

typedef enum { MODE_MANUAL = 0, MODE_AUTOMATIC = 1 } car_mode_t;
static volatile car_mode_t g_mode = MODE_MANUAL;

static volatile int g_auto_running = 1;
static volatile int g_manual_speed = 70;   // default (within 40-100)
static volatile int g_auto_speed = 70;     // default (within 50-90)

/* ------------------------- PWM & MOTOR ------------------------- */
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT
#define LEDC_CH_0       LEDC_CHANNEL_0
#define LEDC_CH_1       LEDC_CHANNEL_1
#define LEDC_FREQ_HZ    1000

static void ledc_init_pwm(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch0 = {
        .gpio_num = ENA_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CH_0,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch0);

    ledc_channel_config_t ch1 = {
        .gpio_num = ENB_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CH_1,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch1);
}

static inline int pct_to_duty(int p) { return (p * PWM_MAX_DUTY) / 100; }

static void pwm_set(int ch, int pct) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    ledc_set_duty(LEDC_MODE, ch, pct_to_duty(pct));
    ledc_update_duty(LEDC_MODE, ch);
}

static void motor_gpio_init(void) {
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL<<IN1_GPIO)|(1ULL<<IN2_GPIO)|(1ULL<<IN3_GPIO)|(1ULL<<IN4_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);
    ledc_init_pwm();
}

/* Motor commands use the current speeds: manual commands use g_manual_speed, automatic forward uses g_auto_speed */
static void motor_set_stop(void) {
    gpio_set_level(IN1_GPIO,0);
    gpio_set_level(IN2_GPIO,0);
    gpio_set_level(IN3_GPIO,0);
    gpio_set_level(IN4_GPIO,0);
    pwm_set(LEDC_CH_0,0);
    pwm_set(LEDC_CH_1,0);
    g_state = STATE_STOP;
    ESP_LOGI(TAG, "CMD: STOP");
}

static void motor_set_forward(int spd){
    gpio_set_level(IN1_GPIO,1);
    gpio_set_level(IN2_GPIO,0);
    gpio_set_level(IN3_GPIO,1);
    gpio_set_level(IN4_GPIO,0);
    pwm_set(LEDC_CH_0,spd);
    pwm_set(LEDC_CH_1,spd);
    g_state = STATE_FORWARD;
    ESP_LOGI(TAG, "CMD: FORWARD speed=%d", spd);
}

static void motor_set_backward(int spd){
    gpio_set_level(IN1_GPIO,0);
    gpio_set_level(IN2_GPIO,1);
    gpio_set_level(IN3_GPIO,0);
    gpio_set_level(IN4_GPIO,1);
    pwm_set(LEDC_CH_0,spd);
    pwm_set(LEDC_CH_1,spd);
    g_state = STATE_BACKWARD;
    ESP_LOGI(TAG, "CMD: BACKWARD speed=%d", spd);
}

static void motor_turn_left(int spd){
    gpio_set_level(IN1_GPIO,0);
    gpio_set_level(IN2_GPIO,1);
    gpio_set_level(IN3_GPIO,1);
    gpio_set_level(IN4_GPIO,0);
    pwm_set(LEDC_CH_0,spd);
    pwm_set(LEDC_CH_1,spd);
    g_state = STATE_LEFT;
    ESP_LOGI(TAG, "CMD: LEFT speed=%d", spd);
}

static void motor_turn_right(int spd){
    gpio_set_level(IN1_GPIO,1);
    gpio_set_level(IN2_GPIO,0);
    gpio_set_level(IN3_GPIO,0);
    gpio_set_level(IN4_GPIO,1);
    pwm_set(LEDC_CH_0,spd);
    pwm_set(LEDC_CH_1,spd);
    g_state = STATE_RIGHT;
    ESP_LOGI(TAG, "CMD: RIGHT speed=%d", spd);
}

/* ------------------------- ULTRASONIC ------------------------- */
static float get_distance_cm(void) {
    gpio_set_level(TRIG_PIN,0); ets_delay_us(2);
    gpio_set_level(TRIG_PIN,1); ets_delay_us(10);
    gpio_set_level(TRIG_PIN,0);

    int wait=0;
    while(gpio_get_level(ECHO_PIN)==0){ if(++wait>30000) return -1; ets_delay_us(1);}

    int count=0;
    while(gpio_get_level(ECHO_PIN)==1){ if(++count>60000) break; ets_delay_us(1);}

    if(count==0) return -1;
    return (count*0.0343f)/2.0f;
}

/* ------------------------- ULTRASONIC TASK ------------------------- */
static void ultrasonic_task(void *p){
    (void)p;
    ESP_LOGI(TAG, "Ultrasonic task started");
    while(1){
        float d = get_distance_cm();

        if(d < 0){
            ESP_LOGW(TAG, "Sensor timeout");
            // On timeout: conservative stop only in manual and when moving forward/left/right
            if(g_mode == MODE_MANUAL &&
               (g_state == STATE_FORWARD || g_state == STATE_LEFT || g_state == STATE_RIGHT)){
                motor_set_stop();
            }
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }

        /* AUTOMATIC MODE */
        if(g_mode == MODE_AUTOMATIC && g_auto_running){
            if(d < AUTO_TURN_THRESHOLD_CM){
                // Obstacle: turn right at hardcoded slow speed
                motor_turn_right(AUTO_TURN_SPEED);
                // short turn pulse, then re-evaluate
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            } else {
                // Path clear: go forward with auto speed slider value
                motor_set_forward(g_auto_speed);
            }
        }

        /* MANUAL SAFETY: only prevent crashes when moving forward/left/right (not backward) */
        if(g_mode == MODE_MANUAL){
            if(d < MANUAL_STOP_THRESHOLD_CM &&
               (g_state == STATE_FORWARD || g_state == STATE_LEFT || g_state == STATE_RIGHT)){
                motor_set_stop();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

/* ------------------------- HTML (pages include sliders & JS) ------------------------- */
static const char *manual_html =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Toy Car — MANUAL</title>"
"<style>button{width:110px;height:60px;margin:6px;font-size:16px;}input[type=range]{width:80%}</style>"
"</head><body>"
"<h3>MANUAL MODE</h3>"
"<div>"
"<label>Speed: <span id='mval'>70</span>%</label><br>"
"<input id='mslider' type='range' min='40' max='100' value='70' oninput='mval.innerText=value; setManualSpeed(value)'>"
"</div>"
"<div>"
"<button onclick=\"cmd('forward')\">Forward</button>"
"<button onclick=\"cmd('backward')\">Backward</button><br>"
"<button onclick=\"cmd('left')\">Left</button>"
"<button onclick=\"cmd('right')\">Right</button><br>"
"<button onclick=\"cmd('stop')\">Stop</button>"
"</div><hr>"
"<button onclick=\"location.href='/switch_auto'\">Switch to AUTOMATIC</button>"
"<script>"
"function cmd(c){fetch('/cmd?do='+c);} "
"function setManualSpeed(v){fetch('/set_manual_speed?val='+v);} "
"</script></body></html>";

static const char *auto_html =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Toy Car — AUTO</title>"
"<style>button{width:140px;height:60px;margin:6px;font-size:16px;}input[type=range]{width:80%}</style>"
"</head><body>"
"<h3>AUTOMATIC MODE</h3>"
"<div>"
"<label>Auto Speed: <span id='aval'>70</span>%</label><br>"
"<input id='aslider' type='range' min='50' max='90' value='70' oninput='aval.innerText=value; setAutoSpeed(value)'>"
"</div>"
"<div>"
"<button onclick=\"fetch('/auto?do=pause')\">Pause</button>"
"<button onclick=\"fetch('/auto?do=resume')\">Resume</button>"
"</div><hr>"
"<button onclick=\"location.href='/switch_manual'\">Exit Automatic (Go Manual)</button>"
"<script>"
"function setAutoSpeed(v){fetch('/set_auto_speed?val='+v);} "
"</script></body></html>";

/* ------------------------- HTTP HANDLERS ------------------------- */

static esp_err_t root_get_handler(httpd_req_t *req) {
    if (g_mode == MODE_MANUAL) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, manual_html, strlen(manual_html));
    } else {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, auto_html, strlen(auto_html));
    }
    return ESP_OK;
}

/* /cmd?do=forward|backward|left|right|stop  -> uses current manual speed for manual commands */
static esp_err_t cmd_handler(httpd_req_t *req) {
    char buf[128];
    char val[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, "do", val, sizeof(val)) == ESP_OK) {
            ESP_LOGI(TAG, "HTTP CMD: %s", val);
            if (strcmp(val, "forward") == 0) {
                motor_set_forward(g_manual_speed);
            } else if (strcmp(val, "backward") == 0) {
                motor_set_backward(g_manual_speed);
            } else if (strcmp(val, "left") == 0) {
                motor_turn_left(g_manual_speed);
            } else if (strcmp(val, "right") == 0) {
                motor_turn_right(g_manual_speed);
            } else if (strcmp(val, "stop") == 0) {
                motor_set_stop();
            }
        }
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* change manual speed: /set_manual_speed?val=NN  (40-100 enforced) */
static esp_err_t set_manual_speed_handler(httpd_req_t *req) {
    char buf[64], val[16];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, "val", val, sizeof(val)) == ESP_OK) {
            int v = atoi(val);
            if (v < MANUAL_MIN_SPEED) v = MANUAL_MIN_SPEED;
            if (v > MANUAL_MAX_SPEED) v = MANUAL_MAX_SPEED;
            g_manual_speed = v;
            ESP_LOGI(TAG, "Manual speed set to %d", v);
        }
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* change auto speed: /set_auto_speed?val=NN  (50-90 enforced) */
static esp_err_t set_auto_speed_handler(httpd_req_t *req) {
    char buf[64], val[16];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, "val", val, sizeof(val)) == ESP_OK) {
            int v = atoi(val);
            if (v < AUTO_MIN_SPEED) v = AUTO_MIN_SPEED;
            if (v > AUTO_MAX_SPEED) v = AUTO_MAX_SPEED;
            g_auto_speed = v;
            ESP_LOGI(TAG, "Auto speed set to %d", v);
        }
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* /switch_auto and /switch_manual to toggle modes */
static esp_err_t switch_auto_handler(httpd_req_t *req) {
    g_mode = MODE_AUTOMATIC;
    g_auto_running = 1;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, auto_html, strlen(auto_html));
    ESP_LOGI(TAG, "Switched to AUTOMATIC (running)");
    return ESP_OK;
}

static esp_err_t switch_manual_handler(httpd_req_t *req) {
    g_mode = MODE_MANUAL;
    motor_set_stop();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, manual_html, strlen(manual_html));
    ESP_LOGI(TAG, "Switched to MANUAL");
    return ESP_OK;
}

/* /auto?do=pause|resume */
static esp_err_t auto_control_handler(httpd_req_t *req) {
    char buf[128], val[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, "do", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "pause") == 0) {
                g_auto_running = 0;
                motor_set_stop();
                ESP_LOGI(TAG, "AUTO paused");
            } else if (strcmp(val, "resume") == 0) {
                g_auto_running = 1;
                ESP_LOGI(TAG, "AUTO resumed");
            }
        }
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* Start HTTP server and register URIs */
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &root);

    httpd_uri_t cmd_uri = { .uri = "/cmd", .method = HTTP_GET, .handler = cmd_handler };
    httpd_register_uri_handler(server, &cmd_uri);

    httpd_uri_t manual_speed_uri = { .uri = "/set_manual_speed", .method = HTTP_GET, .handler = set_manual_speed_handler };
    httpd_register_uri_handler(server, &manual_speed_uri);

    httpd_uri_t auto_speed_uri = { .uri = "/set_auto_speed", .method = HTTP_GET, .handler = set_auto_speed_handler };
    httpd_register_uri_handler(server, &auto_speed_uri);

    httpd_uri_t switch_auto = { .uri = "/switch_auto", .method = HTTP_GET, .handler = switch_auto_handler };
    httpd_register_uri_handler(server, &switch_auto);

    httpd_uri_t switch_manual = { .uri = "/switch_manual", .method = HTTP_GET, .handler = switch_manual_handler };
    httpd_register_uri_handler(server, &switch_manual);

    httpd_uri_t auto_uri = { .uri = "/auto", .method = HTTP_GET, .handler = auto_control_handler };
    httpd_register_uri_handler(server, &auto_uri);

    return server;
}

/* ------------------------- SIMPLE Wi-Fi SOFTAP ------------------------- */
static void initialise_wifi_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Khilona",
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
            .pmf_cfg = {
                .capable = false,
                .required = false
            }
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi SoftAP started. Connect to Khilona");
}

/* ------------------------- APP MAIN ------------------------- */
void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    motor_gpio_init();
    motor_set_stop();

    /* Ultrasonic pins */
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_conf);

    /* WiFi + server */
    initialise_wifi_softap();
    start_webserver();

    /* Defaults: manual page shown, manual default speed 70, auto default 70 (within ranges) */
    g_mode = MODE_MANUAL;
    if (g_manual_speed < MANUAL_MIN_SPEED) g_manual_speed = MANUAL_MIN_SPEED;
    if (g_manual_speed > MANUAL_MAX_SPEED) g_manual_speed = MANUAL_MAX_SPEED;
    if (g_auto_speed < AUTO_MIN_SPEED) g_auto_speed = AUTO_MIN_SPEED;
    if (g_auto_speed > AUTO_MAX_SPEED) g_auto_speed = AUTO_MAX_SPEED;
    g_auto_running = 0; /* per your earlier request: default manual; when switched to auto it should run by default */

    xTaskCreate(ultrasonic_task, "ultra", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Car ready. Default: MANUAL. Connect to Khilona and open http://192.168.4.1/");
}
