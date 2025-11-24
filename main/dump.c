#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"

static const char* TAG = "toy_car";

// Motor Pins
#define MOTOR_IN1 GPIO_NUM_12
#define MOTOR_IN2 GPIO_NUM_13
#define MOTOR_IN3 GPIO_NUM_14
#define MOTOR_IN4 GPIO_NUM_15
#define MOTOR_ENA GPIO_NUM_27
#define MOTOR_ENB GPIO_NUM_33

// Motor PWM Config
#define LEDC_TIMER LEDC_TIMER_1
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 1000

#define TRIG_PIN GPIO_NUM_25
#define ECHO_PIN GPIO_NUM_26
#define OBSTACLE_THRESHOLD_CM 20.0
#define MAX_ECHO_TIMEOUT_US 25000

typedef enum {
	MODE_MANUAL,
	MODE_AUTOMATIC
} car_mode_t;

typedef enum {
	AUTO_STOPPED,
	AUTO_RUNNING
} auto_state_t;

volatile car_mode_t g_car_mode = MODE_MANUAL;
volatile auto_state_t g_auto_state = AUTO_STOPPED;

#define MANUAL_STOP_THRESHOLD_CM 20.0
#define AUTO_MANEUVER_THRESHOLD_CM 35.0
#define AUTO_DEFAULT_SPEED 70

enum State {
	STOP,
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
};
volatile enum State state = STOP;

static void motor_gpio_init(void) {
	gpio_reset_pin(MOTOR_IN1);
	gpio_reset_pin(MOTOR_IN2);
	gpio_reset_pin(MOTOR_IN3);
	gpio_reset_pin(MOTOR_IN4);

	gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_IN3, GPIO_MODE_OUTPUT);
	gpio_set_direction(MOTOR_IN4, GPIO_MODE_OUTPUT);

	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_MODE,
		.timer_num = LEDC_TIMER,
		.duty_resolution = LEDC_DUTY_RES,
		.freq_hz = LEDC_FREQUENCY,
		.clk_cfg = LEDC_AUTO_CLK};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel_a = {
		.gpio_num = MOTOR_ENA,
		.speed_mode = LEDC_MODE,
		.channel = LEDC_CHANNEL_2,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0};
	ledc_channel_config(&ledc_channel_a);

	ledc_channel_config_t ledc_channel_b = {
		.gpio_num = MOTOR_ENB,
		.speed_mode = LEDC_MODE,
		.channel = LEDC_CHANNEL_3,
		.timer_sel = LEDC_TIMER,
		.duty = 0,
		.hpoint = 0};
	ledc_channel_config(&ledc_channel_b);
}

static void set_pwm_duty(uint8_t channel, uint8_t duty_percent) {
	if (duty_percent > 100) duty_percent = 100;
	uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
	uint32_t duty = (max_duty * duty_percent) / 100;
	ledc_set_duty(LEDC_MODE, channel, duty);
	ledc_update_duty(LEDC_MODE, channel);
}

static void log_motor_pin_states(const char* action, int in1, int in2, int in3, int in4, uint8_t speed_ena, uint8_t speed_enb) {
	ESP_LOGI(TAG, "CMD: %-8s | IN1(12): %d | IN2(13): %d | IN3(14): %d | IN4(15): %d | ENA(27): %3u%% | ENB(33): %3u%%",
			 action, in1, in2, in3, in4, speed_ena, speed_enb);
}

static void motor_set_stop(void) {
	state = STOP;
	gpio_set_level(MOTOR_IN1, 0);
	gpio_set_level(MOTOR_IN2, 0);
	gpio_set_level(MOTOR_IN3, 0);
	gpio_set_level(MOTOR_IN4, 0);
	set_pwm_duty(LEDC_CHANNEL_2, 0);
	set_pwm_duty(LEDC_CHANNEL_3, 0);
	log_motor_pin_states("STOP", 0, 0, 0, 0, 0, 0);
}

static void motor_set_forward(uint8_t speed) {
	state = FORWARD;
	gpio_set_level(MOTOR_IN1, 1);
	gpio_set_level(MOTOR_IN2, 0);
	gpio_set_level(MOTOR_IN3, 1);
	gpio_set_level(MOTOR_IN4, 0);
	set_pwm_duty(LEDC_CHANNEL_2, speed);
	set_pwm_duty(LEDC_CHANNEL_3, speed);
	log_motor_pin_states("FORWARD", 1, 0, 1, 0, speed, speed);
}

static void motor_set_backward(uint8_t speed) {
	state = BACKWARD;
	gpio_set_level(MOTOR_IN1, 0);
	gpio_set_level(MOTOR_IN2, 1);
	gpio_set_level(MOTOR_IN3, 0);
	gpio_set_level(MOTOR_IN4, 1);
	set_pwm_duty(LEDC_CHANNEL_2, speed);
	set_pwm_duty(LEDC_CHANNEL_3, speed);
	log_motor_pin_states("BACKWARD", 0, 1, 0, 1, speed, speed);
}

static void motor_turn_left(uint8_t speed) {
	state = LEFT;
	gpio_set_level(MOTOR_IN1, 0);
	gpio_set_level(MOTOR_IN2, 0);
	gpio_set_level(MOTOR_IN3, 1);
	gpio_set_level(MOTOR_IN4, 0);
	set_pwm_duty(LEDC_CHANNEL_2, 0);
	set_pwm_duty(LEDC_CHANNEL_3, speed);
	log_motor_pin_states("LEFT", 0, 0, 1, 0, speed, speed);
}

static void motor_turn_right(uint8_t speed) {
	state = RIGHT;
	gpio_set_level(MOTOR_IN1, 1);
	gpio_set_level(MOTOR_IN2, 0);
	gpio_set_level(MOTOR_IN3, 0);
	gpio_set_level(MOTOR_IN4, 0);
	set_pwm_duty(LEDC_CHANNEL_2, speed);
	set_pwm_duty(LEDC_CHANNEL_3, 0);
	log_motor_pin_states("RIGHT", 1, 0, 0, 0, speed, speed);
}

static void send_trigger_pulse(void) {
	gpio_set_level(TRIG_PIN, 0);
	ets_delay_us(2);
	gpio_set_level(TRIG_PIN, 1);
	ets_delay_us(10);
	gpio_set_level(TRIG_PIN, 0);
}

static float get_distance_cm(void) {
	send_trigger_pulse();  // This is your existing function

	int64_t start_time_us = 0;
	int64_t end_time_us = 0;
	int timeout_counter = 0;

	// 1. Wait for Echo pin to go HIGH
	while (gpio_get_level(ECHO_PIN) == 0) {
		if (timeout_counter++ > MAX_ECHO_TIMEOUT_US) return -1.0;  // Timeout
		ets_delay_us(1);
	}

	// 2. Measure the pulse duration
	start_time_us = esp_timer_get_time();
	timeout_counter = 0;  // Reset timeout
	while (gpio_get_level(ECHO_PIN) == 1) {
		if (timeout_counter++ > MAX_ECHO_TIMEOUT_US) return -1.0;  // Timeout
		ets_delay_us(1);
	}
	end_time_us = esp_timer_get_time();

	uint32_t duration_us = (uint32_t)(end_time_us - start_time_us);
	return (duration_us * 0.0343) / 2.0;
}

void ultrasonic_test_task(void* pvParameters) {
	gpio_config_t trig_conf = {
		.pin_bit_mask = (1ULL << TRIG_PIN),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&trig_conf);

	gpio_config_t echo_conf = {
		.pin_bit_mask = (1ULL << ECHO_PIN),
		.mode = GPIO_MODE_INPUT,
	};
	gpio_config(&echo_conf);

	vTaskDelay(pdMS_TO_TICKS(1000));
	ESP_LOGI(TAG, "Ultrasonic sensor task started.");

	while (1) {
		float distance_cm = get_distance_cm();
		if (distance_cm < 0) {
			ESP_LOGW(TAG, "Sensor timeout, stopping motors for safety.");
            if (state != STOP) motor_set_stop();
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;  // Skip this loop iteration
		}

		if (g_car_mode == MODE_AUTOMATIC) {
			if (g_auto_state == AUTO_RUNNING) {
				// We are in Auto-Run mode, execute autonomous logic
				if (distance_cm < AUTO_MANEUVER_THRESHOLD_CM) {
					// Obstacle detected! Turn right.
					ESP_LOGI(TAG, "AUTO: Obstacle (%.1fcm). Turning Right.", distance_cm);
					motor_turn_right(AUTO_DEFAULT_SPEED);
				} else {
					// Way is clear! Go forward.
					ESP_LOGI(TAG, "AUTO: Clear (%.1fcm). Going Forward.", distance_cm);
					motor_set_forward(AUTO_DEFAULT_SPEED);
				}
			} else {
				// We are in Auto-Stop (Paused) mode.
                if (state != STOP) motor_set_stop();
			}

		} else {
			// --- We are in MANUAL mode ---
			// The sensor just acts as a safety override.
			if (distance_cm < MANUAL_STOP_THRESHOLD_CM &&
				(state == FORWARD || state == RIGHT || state == LEFT)) {
				ESP_LOGW(TAG, "MANUAL: Obstacle detected at %.1f cm! Forcing stop!", distance_cm);
				motor_set_stop();
			}
		}

		vTaskDelay(pdMS_TO_TICKS(100));	 // Loop every 100ms
	}
}

static const char manual_page_html[] =
	"<!DOCTYPE html><html><head><meta name'viewport' content='width=device-width,initial-scale=1'/>"
	"<title>ESP32 Car - MANUAL</title><style>body{font-family:sans-serif;text-align:center;} button{width:120px;height:70px;margin:6px;font-size:16px} .mode{background-color:#f00;color:white;padding:10px;}</style></head><body>"
	"<h3>MANUAL MODE</h3>"
	"<div>"
	"<button onclick=\"cmd('forward')\">Forward</button>"
	"<button onclick=\"cmd('backward')\">Backward</button>"
	"<button onclick=\"cmd('left')\">Right</button>"  // Note: Your original file had these two buttons swapped in the HTML
	"<button onclick=\"cmd('right')\">Left</button>"
	"<button onclick=\"cmd('stop')\">Stop</button>"
	"</div>"
	"<div>Speed: <input id='speed' type='range' min='0' max='100' value='70' oninput='spd.value=value'><output id='spd'>70</output></div>"
	"<hr>"
	"<div><button class='mode' onclick=\"location.href='/set_mode?auto=1'\">Go to AUTOMATIC</button></div>"	 // NEW Button
	"<script>"
	"function cmd(c){var s=document.getElementById('speed').value;fetch('/control?cmd='+c+'&speed='+s).catch(console.error)}"
	"</script></body></html>";

static const char automatic_page_html[] =
	"<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
	"<title>ESP32 Car - AUTO</title><style>body{font-family:sans-serif;text-align:center;} button{width:120px;height:70px;margin:6px;font-size:16px} .mode{background-color:#080;color:white;padding:10px;}</style></head><body>"
	"<h3>AUTOMATIC MODE</h3>"
	"<div>"
	"<button onclick=\"fetch('/auto_cmd?cmd=run')\">Resume</button>"
	"<button onclick=\"fetch('/auto_cmd?cmd=stop')\">Pause</button>"
	"</div>"
	"<hr>"
	"<div><button class='mode' onclick=\"location.href='/set_mode?auto=0'\">Go to MANUAL</button></div>"
	"</body></html>";

static esp_err_t control_handler(httpd_req_t* req) {
	// Only accept manual commands if we are in manual mode
	if (g_car_mode != MODE_MANUAL) {
		ESP_LOGW(TAG, "Ignoring manual command, car is in AUTO mode.");
		httpd_resp_send_500(req);  // Send an error
		return ESP_FAIL;
	}
	char buf[32];
	if (httpd_req_get_url_query_len(req) > sizeof(buf) - 1) {
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
	if (ret != ESP_OK) {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
		return ESP_FAIL;
	}

	char cmd_str[16] = {0};
	char speed_str[8] = {0};

	httpd_query_key_value(buf, "cmd", cmd_str, sizeof(cmd_str));
	httpd_query_key_value(buf, "speed", speed_str, sizeof(speed_str));

	int speed_val = 30;
	if (speed_str[0]) speed_val = atoi(speed_str);
	if (speed_val < 0) speed_val = 0;
	if (speed_val > 100) speed_val = 100;

	ESP_LOGI(TAG, "HTTP Request Received: cmd='%s', speed=%d", cmd_str, speed_val);

	if (strcmp(cmd_str, "forward") == 0) {
		// We will let the ultrasonic task handle the stop,
		// but we could also check the state here if we stored distance globally.
		// For now, the background task is a simpler override.
		if (state == BACKWARD) {
			motor_set_stop();
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
		motor_set_forward((uint8_t)speed_val);
	}
	// ... (rest of the motor control logic is unchanged) ...
	else if (strcmp(cmd_str, "backward") == 0) {
		if (state == FORWARD || state == LEFT || state == RIGHT) {
			motor_set_stop();
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
		motor_set_backward((uint8_t)speed_val);
	} else if (strcmp(cmd_str, "left") == 0) {
		if (state == BACKWARD) {
			motor_set_stop();
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
		motor_turn_left((uint8_t)speed_val);
	} else if (strcmp(cmd_str, "right") == 0) {
		if (state == BACKWARD) {
			motor_set_stop();
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
		motor_turn_right((uint8_t)speed_val);
	} else {
		motor_set_stop();
	}

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}\n", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t* req) {
	httpd_resp_set_type(req, "text/html");

	if (g_car_mode == MODE_MANUAL) {
		httpd_resp_send(req, manual_page_html, HTTPD_RESP_USE_STRLEN);
	} else {
		httpd_resp_send(req, automatic_page_html, HTTPD_RESP_USE_STRLEN);
	}

	return ESP_OK;
}

static esp_err_t mode_set_handler(httpd_req_t* req) {
	char buf[16];
	httpd_req_get_url_query_str(req, buf, sizeof(buf));
	
	char param[8];
	httpd_query_key_value(buf, "auto", param, sizeof(param));

	if (strcmp(param, "1") == 0) {
		// --- Switch to AUTO ---
		g_car_mode = MODE_AUTOMATIC;
		g_auto_state = AUTO_RUNNING; // Per your request, default to RUNNING
		ESP_LOGI(TAG, "MODE SWITCH: -> AUTOMATIC (RUNNING)");
	} else {
		// --- Switch to MANUAL ---
		g_car_mode = MODE_MANUAL;
		g_auto_state = AUTO_STOPPED;
		ESP_LOGI(TAG, "MODE SWITCH: -> MANUAL");
		motor_set_stop(); // IMPORTANT: Stop motors when switching to manual
	}

	// Redirect the browser back to the main '/' page
	httpd_resp_set_status(req, "302 Found");
	httpd_resp_set_hdr(req, "Location", "/");
	httpd_resp_send(req, NULL, 0); // Send empty body to finalize redirect
	return ESP_OK;
}

// --- NEW: Handler to Pause/Resume Auto mode ---
static esp_err_t auto_cmd_handler(httpd_req_t* req) {
	char buf[16];
	httpd_req_get_url_query_str(req, buf, sizeof(buf));
	
	char param[8];
	httpd_query_key_value(buf, "cmd", param, sizeof(param));

	if (strcmp(param, "run") == 0) {
		g_auto_state = AUTO_RUNNING;
		ESP_LOGI(TAG, "AUTO_CMD: RESUME");
	} else if (strcmp(param, "stop") == 0) {
		g_auto_state = AUTO_STOPPED;
		ESP_LOGI(TAG, "AUTO_CMD: PAUSE");
		motor_set_stop(); // Stop the motors when pausing
	}
	
	httpd_resp_send(req, "{\"ok\":true}\n", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static httpd_uri_t index_uri = {
	.uri = "/",
	.method = HTTP_GET,
	.handler = index_handler,
	.user_ctx = NULL};

static httpd_uri_t control_uri = {
	.uri = "/control",
	.method = HTTP_GET,
	.handler = control_handler,
	.user_ctx = NULL};

// --- NEW: URI definitions for our new handlers ---
static httpd_uri_t mode_set_uri = {
	.uri = "/set_mode",
	.method = HTTP_GET,
	.handler = mode_set_handler,
	.user_ctx = NULL
};

static httpd_uri_t auto_cmd_uri = {
	.uri = "/auto_cmd",
	.method = HTTP_GET,
	.handler = auto_cmd_handler,
	.user_ctx = NULL
};

static httpd_handle_t start_webserver(void) {
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.server_port = 80;
	config.lru_purge_enable = true;

	config.stack_size = 8192;

	httpd_handle_t server = NULL;
	if (httpd_start(&server, &config) == ESP_OK) {
		httpd_register_uri_handler(server, &index_uri);
		httpd_register_uri_handler(server, &control_uri);
        // NEW
        httpd_register_uri_handler(server, &mode_set_uri);
		httpd_register_uri_handler(server, &auto_cmd_uri);
	}
	return server;
}

static void initialise_wifi_softap(void) {
	const char* ssid = "Khilona";
	const char* pass = "timepass";

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
			.authmode = WIFI_AUTH_WPA_WPA2_PSK},
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

void app_main(void) {
	nvs_flash_init();
	motor_gpio_init();
	motor_set_stop();
	initialise_wifi_softap();
	start_webserver();

	// This task will run on the other CPU core (Core 1) by default
	xTaskCreate(ultrasonic_test_task, "ultrasonic_task", 4096, NULL, 5, NULL);

	ESP_LOGI(TAG, "Completed! IP:-  http://192.168.4.1");
}
