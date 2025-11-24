/* main.c — Dual-mode ESP32 toy car firmware
 *
 * Features:
 *  - Manual mode (default): full manual controls via web UI + crash prevention (manual threshold)
 *  - Automatic mode: auto-running by default, turns RIGHT repeatedly until path clear
 *  - Separate thresholds for manual and automatic
 *  - Ultrasonic sensor continuously running in a FreeRTOS task
 *  - Simple HTTP server (softAP expected) with pages for manual/auto
 *
 * Pins: adjust the #defines below to match your wiring.
 *
 * WARNING: Make sure ECHO pin is level-shifted to 3.3V!
 */

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"

static const char* TAG = "toy_car";

/* ------------------------- PIN CONFIG ------------------------- */
/* Motor driver (from your logs) */
#define IN1_GPIO 12
#define IN2_GPIO 13
#define IN3_GPIO 14
#define IN4_GPIO 15
#define ENA_GPIO 27	 // PWM channel A
#define ENB_GPIO 33	 // PWM channel B

/* Ultrasonic sensor */
#define TRIG_PIN 25
#define ECHO_PIN 26

/* ------------------------- BEHAVIOR CONFIG ------------------------- */
#define MANUAL_STOP_THRESHOLD_CM 18.0f
#define AUTO_TURN_THRESHOLD_CM 25.0f
#define AUTO_DEFAULT_SPEED 70  // percent (0-100)
#define PWM_MAX_DUTY 8191	   // 13-bit duty for ledc

/* ------------------------- GLOBAL STATE ------------------------- */
typedef enum { STATE_STOP = 0,
			   STATE_FORWARD,
			   STATE_BACKWARD,
			   STATE_LEFT,
			   STATE_RIGHT } motor_state_t;
static volatile motor_state_t g_state = STATE_STOP;

typedef enum { MODE_MANUAL = 0,
			   MODE_AUTOMATIC = 1 } car_mode_t;
static volatile car_mode_t g_mode = MODE_MANUAL;

/* When in automatic and switched to auto, this flag controls run/pause */
static volatile int g_auto_running = 1;

/* ------------------------- HELPER: PWM / Motor control ------------------------- */
/* Use LEDC channels for ENA and ENB PWM */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_CH_0 LEDC_CHANNEL_0
#define LEDC_CH_1 LEDC_CHANNEL_1
#define LEDC_FREQ_HZ 1000

static void ledc_init_pwm(void) {
	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_MODE,
		.duty_resolution = LEDC_DUTY_RES,
		.timer_num = LEDC_TIMER,
		.freq_hz = LEDC_FREQ_HZ,
		.clk_cfg = LEDC_AUTO_CLK};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ch0 = {
		.gpio_num = ENA_GPIO,
		.speed_mode = LEDC_MODE,
		.channel = LEDC_CH_0,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0};
	ledc_channel_config(&ch0);

	ledc_channel_config_t ch1 = {
		.gpio_num = ENB_GPIO,
		.speed_mode = LEDC_MODE,
		.channel = LEDC_CH_1,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0};
	ledc_channel_config(&ch1);
}

static inline int percent_to_duty(int pct) {
	if (pct <= 0) return 0;
	if (pct >= 100) return PWM_MAX_DUTY;
	return (int)((pct / 100.0f) * PWM_MAX_DUTY);
}

static void motor_gpio_init(void) {
	gpio_config_t out_conf = {
		.pin_bit_mask = (1ULL << IN1_GPIO) | (1ULL << IN2_GPIO) | (1ULL << IN3_GPIO) | (1ULL << IN4_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE};
	gpio_config(&out_conf);

	ledc_init_pwm();
}

static void set_pwm_speed(int channel, int percent) {
	int duty = percent_to_duty(percent);
	ledc_set_duty(LEDC_MODE, channel, duty);
	ledc_update_duty(LEDC_MODE, channel);
}

/* Motor control: set IN pins appropriately and set PWM duty */
static void motor_set_stop(void) {
	gpio_set_level(IN1_GPIO, 0);
	gpio_set_level(IN2_GPIO, 0);
	gpio_set_level(IN3_GPIO, 0);
	gpio_set_level(IN4_GPIO, 0);
	set_pwm_speed(LEDC_CH_0, 0);
	set_pwm_speed(LEDC_CH_1, 0);
	g_state = STATE_STOP;
	ESP_LOGI(TAG, "CMD: STOP");
}

static void motor_set_forward(int spd_percent) {
	// Left motor forward: IN1=1, IN2=0 ; Right motor forward: IN3=1, IN4=0
	gpio_set_level(IN1_GPIO, 1);
	gpio_set_level(IN2_GPIO, 0);
	gpio_set_level(IN3_GPIO, 1);
	gpio_set_level(IN4_GPIO, 0);
	set_pwm_speed(LEDC_CH_0, spd_percent);
	set_pwm_speed(LEDC_CH_1, spd_percent);
	g_state = STATE_FORWARD;
	ESP_LOGI(TAG, "CMD: FORWARD | ENA(%d) ENB(%d)", spd_percent, spd_percent);
}

static void motor_set_backward(int spd_percent) {
	gpio_set_level(IN1_GPIO, 0);
	gpio_set_level(IN2_GPIO, 1);
	gpio_set_level(IN3_GPIO, 0);
	gpio_set_level(IN4_GPIO, 1);
	set_pwm_speed(LEDC_CH_0, spd_percent);
	set_pwm_speed(LEDC_CH_1, spd_percent);
	g_state = STATE_BACKWARD;
	ESP_LOGI(TAG, "CMD: BACKWARD | ENA(%d) ENB(%d)", spd_percent, spd_percent);
}

static void motor_turn_left(int spd_percent) {
	// spin left: left motor backward, right motor forward
	gpio_set_level(IN1_GPIO, 0);
	gpio_set_level(IN2_GPIO, 1);
	gpio_set_level(IN3_GPIO, 1);
	gpio_set_level(IN4_GPIO, 0);
	set_pwm_speed(LEDC_CH_0, spd_percent);
	set_pwm_speed(LEDC_CH_1, spd_percent);
	g_state = STATE_LEFT;
	ESP_LOGI(TAG, "CMD: LEFT | ENA(%d) ENB(%d)", spd_percent, spd_percent);
}

static void motor_turn_right(int spd_percent) {
	// spin right: left motor forward, right motor backward
	gpio_set_level(IN1_GPIO, 1);
	gpio_set_level(IN2_GPIO, 0);
	gpio_set_level(IN3_GPIO, 0);
	gpio_set_level(IN4_GPIO, 1);
	set_pwm_speed(LEDC_CH_0, spd_percent);
	set_pwm_speed(LEDC_CH_1, spd_percent);
	g_state = STATE_RIGHT;
	ESP_LOGI(TAG, "CMD: RIGHT | ENA(%d) ENB(%d)", spd_percent, spd_percent);
}

/* ------------------------- ULTRASONIC ------------------------- */
/* make sure TRIG_PIN set as output, ECHO_PIN as input (with level-shift) */
static float get_distance_cm(void) {
	// send 10us pulse
	gpio_set_level(TRIG_PIN, 0);
	ets_delay_us(2);
	gpio_set_level(TRIG_PIN, 1);
	ets_delay_us(10);
	gpio_set_level(TRIG_PIN, 0);

	// wait for echo start
	int wait = 0;
	while (gpio_get_level(ECHO_PIN) == 0) {
		if (++wait > 30000) return -1;	// timeout ~30 ms
		ets_delay_us(1);
	}

	// measure high time
	int count = 0;
	while (gpio_get_level(ECHO_PIN) == 1) {
		if (++count > 60000) break;	 // safety cap
		ets_delay_us(1);
	}
	if (count == 0) return -1;

	// distance cm = (time_us * speed_of_sound_cm_per_us) / 2
	// speed of sound ~ 0.0343 cm/us -> dist = (count * 0.001 ms?) but count is microseconds here
	float distance_cm = (count * 0.0343f) / 2.0f;
	return distance_cm;
}

/* ------------------------- ULTRASONIC TASK (AUTOMATION & SAFETY) ------------------------- */
static void ultrasonic_task(void* pvParameters) {
	(void)pvParameters;
	ESP_LOGI(TAG, "Ultrasonic task started");

	while (1) {
		float d = get_distance_cm();
		if (d < 0) {
			ESP_LOGW(TAG, "Sensor timeout, stopping motors for safety.");
			// On timeout: be conservative. If in manual and moving forward/left/right, stop.
			if (g_mode == MODE_MANUAL &&
				(g_state == STATE_FORWARD || g_state == STATE_LEFT || g_state == STATE_RIGHT)) {
				motor_set_stop();
			}
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		// AUTOMATIC MODE
		if (g_mode == MODE_AUTOMATIC && g_auto_running) {
			// If close to obstacle, keep turning right until path is free
			if (d < AUTO_TURN_THRESHOLD_CM) {
				ESP_LOGI(TAG, "AUTO: obstacle %.1f cm → turning right", d);
				motor_turn_right(AUTO_DEFAULT_SPEED);
				// turn for a short burst then check again
				vTaskDelay(pdMS_TO_TICKS(250));	 // small right-turn pulse
				// after that loop will recheck distance; if still blocked, it turns right again
				continue;
			} else {
				// path clear - go forward
				motor_set_forward(AUTO_DEFAULT_SPEED);
			}
		}

		// MANUAL SAFETY: should prevent crashes while allowing manual control
		if (g_mode == MODE_MANUAL) {
			if (d < MANUAL_STOP_THRESHOLD_CM) {
				// if obstacle is too near, stop immediately (regardless of manual command)
				ESP_LOGW(TAG, "MANUAL: Obstacle %.1f cm - forcing STOP", d);
				motor_set_stop();
			}
			// else do nothing; manual controls can set motor states
		}

		vTaskDelay(pdMS_TO_TICKS(80));
	}
}

/* ------------------------- SIMPLE HTTP UI ------------------------- */
/* Very small pages. Manual page has movement buttons + "Switch to Auto".
   Auto page has Pause / Resume / Exit Auto. */

static const char* manual_html =
	"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
	"<title>Toy Car - MANUAL</title><style>button{width:110px;height:60px;margin:6px;font-size:16px}</style></head><body>"
	"<h3>MANUAL MODE</h3>"
	"<div>"
	"<button onclick=\"cmd('forward')\">Forward</button>"
	"<button onclick=\"cmd('backward')\">Backward</button>"
	"<br>"
	"<button onclick=\"cmd('left')\">Left</button>"
	"<button onclick=\"cmd('right')\">Right</button>"
	"<br><button onclick=\"cmd('stop')\">Stop</button>"
	"</div>"
	"<hr>"
	"<div>"
	"<button onclick=\"location.href='/switch_auto'\">Switch to AUTOMATIC</button>"
	"</div>"
	"<script>"
	"function cmd(c){fetch('/cmd?do='+c).then(()=>{}).catch(()=>{});}"
	"</script></body></html>";

static const char* auto_html =
	"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
	"<title>Toy Car - AUTO</title><style>button{width:140px;height:60px;margin:6px;font-size:16px}</style></head><body>"
	"<h3>AUTOMATIC MODE</h3>"
	"<div>"
	"<button onclick=\"fetch('/auto?do=pause')\">Pause</button>"
	"<button onclick=\"fetch('/auto?do=resume')\">Resume</button>"
	"<br><hr>"
	"<button onclick=\"location.href='/switch_manual'\">Exit Automatic (Go Manual)</button>"
	"</div>"
	"</body></html>";

/* HTTP handlers */
static esp_err_t root_get_handler(httpd_req_t* req) {
	if (g_mode == MODE_MANUAL) {
		httpd_resp_set_type(req, "text/html");
		httpd_resp_send(req, manual_html, strlen(manual_html));
	} else {
		httpd_resp_set_type(req, "text/html");
		httpd_resp_send(req, auto_html, strlen(auto_html));
	}
	return ESP_OK;
}

/* /cmd?do=forward/backward/left/right/stop */
static esp_err_t cmd_handler(httpd_req_t* req) {
	char buf[128];
	char cmd[32];
	if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
		if (httpd_query_key_value(buf, "do", cmd, sizeof(cmd)) == ESP_OK) {
			if (strcmp(cmd, "forward") == 0)
				motor_set_forward(70);
			else if (strcmp(cmd, "backward") == 0)
				motor_set_backward(60);
			else if (strcmp(cmd, "left") == 0)
				motor_turn_left(65);
			else if (strcmp(cmd, "right") == 0)
				motor_turn_right(65);
			else if (strcmp(cmd, "stop") == 0)
				motor_set_stop();
			ESP_LOGI(TAG, "HTTP Request Received: cmd='%s'", cmd);
		}
	}
	httpd_resp_sendstr(req, "OK");
	return ESP_OK;
}

/* /switch_auto -> switch to automatic and show auto page (auto runs by default) */
static esp_err_t switch_auto_handler(httpd_req_t* req) {
	g_mode = MODE_AUTOMATIC;
	g_auto_running = 1;
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, auto_html, strlen(auto_html));
	ESP_LOGI(TAG, "Mode switched to AUTOMATIC (running).");
	return ESP_OK;
}

/* /switch_manual -> switch to manual mode and show manual page */
static esp_err_t switch_manual_handler(httpd_req_t* req) {
	g_mode = MODE_MANUAL;
	motor_set_stop();  // be safe
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, manual_html, strlen(manual_html));
	ESP_LOGI(TAG, "Mode switched to MANUAL.");
	return ESP_OK;
}

/* /auto?do=pause or /auto?do=resume */
static esp_err_t auto_control_handler(httpd_req_t* req) {
	char buf[128], val[32];
	if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
		if (httpd_query_key_value(buf, "do", val, sizeof(val)) == ESP_OK) {
			if (strcmp(val, "pause") == 0) {
				g_auto_running = 0;
				motor_set_stop();
				ESP_LOGI(TAG, "AUTO: paused");
			} else if (strcmp(val, "resume") == 0) {
				g_auto_running = 1;
				ESP_LOGI(TAG, "AUTO: resumed");
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

	httpd_uri_t root = {
		.uri = "/",
		.method = HTTP_GET,
		.handler = root_get_handler,
		.user_ctx = NULL};
	httpd_register_uri_handler(server, &root);

	httpd_uri_t cmd_uri = {
		.uri = "/cmd",
		.method = HTTP_GET,
		.handler = cmd_handler};
	httpd_register_uri_handler(server, &cmd_uri);

	httpd_uri_t switch_auto = {
		.uri = "/switch_auto",
		.method = HTTP_GET,
		.handler = switch_auto_handler};
	httpd_register_uri_handler(server, &switch_auto);

	httpd_uri_t switch_manual = {
		.uri = "/switch_manual",
		.method = HTTP_GET,
		.handler = switch_manual_handler};
	httpd_register_uri_handler(server, &switch_manual);

	httpd_uri_t auto_uri = {
		.uri = "/auto",
		.method = HTTP_GET,
		.handler = auto_control_handler};
	httpd_register_uri_handler(server, &auto_uri);

	return server;
}

/* ------------------------- SIMPLE Wi-Fi SOFTAP (minimal) ------------------------- */
static void initialise_wifi_softap(void) {
	esp_netif_init();
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = "ToyCar_AP",
			.ssid_len = 0,
			.channel = 1,
			.authmode = WIFI_AUTH_OPEN,
			.max_connection = 4,
			.pmf_cfg = {
				.capable = false,
				.required = false}},
	};

	esp_wifi_set_mode(WIFI_MODE_AP);
	esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
	esp_wifi_start();

	ESP_LOGI(TAG, "WiFi SoftAP started. Connect to ToyCar_AP");
}

/* ------------------------- APP MAIN ------------------------- */
void app_main(void) {
	// NVS and core setup
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		nvs_flash_init();
	}

	// Motor pins & PWM
	motor_gpio_init();
	motor_set_stop();

	// Ultrasonic pins
	gpio_config_t trig_conf = {
		.pin_bit_mask = (1ULL << TRIG_PIN),
		.mode = GPIO_MODE_OUTPUT};
	gpio_config(&trig_conf);
	gpio_config_t echo_conf = {
		.pin_bit_mask = (1ULL << ECHO_PIN),
		.mode = GPIO_MODE_INPUT};
	gpio_config(&echo_conf);

	// Wi-Fi softAP
	initialise_wifi_softap();

	// Start web server
	start_webserver();

	// Default mode manual (per requirement)
	g_mode = MODE_MANUAL;
	g_auto_running = 0;

	// Ultrasonic task: continuous monitoring
	xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, NULL);

	ESP_LOGI(TAG, "Car ready. Default: MANUAL. Connect to ToyCar_AP and open http://192.168.4.1/");
}
