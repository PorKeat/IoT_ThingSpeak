/** 
 * Project: 
 * Author: 
 * Created on 
 * Hardware platform: Keyestudio ESP32 Dev Board (ESP32 DevKit 38 pins)
 * Software framework: ESP-IDF v5.4.0 (VS Code Extension 1.9.1)
 * Description:
 *  
 * 
 * 
 */

/*------------------------------------------------------------------------------
 HEADERS
------------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "my_data.h"
#include "cJSON.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "ultrasonic.h"
#include "esp_timer.h"

/*------------------------------------------------------------------------------
PROGRAM CONSTANTS
------------------------------------------------------------------------------*/
// Name Tag For Function
#define TAG "ServoMotor"
// API
#define WRITE_ULTRASONIC_API "JAZNX114ESE9G465"
#define WRITE_PIR_API "0UDMEUVJMS6YQ118"
#define WRITE_BACK "2GFHPT1S2DZLVPHH"
#define READ_API "8FB4KKVVEHU5ALBV"
#define CHANNEL "2987995"
// Pin
#define SERVO_GPIO GPIO_NUM_13
#define LED_GPIO GPIO_NUM_2
#define PIR_SENSOR_GPIO GPIO_NUM_4
#define TRIGGER_GPIO GPIO_NUM_5
#define ECHO_GPIO GPIO_NUM_18
// Other
#define MAX_DISTANCE_CM 500 // 5m max
#define ULTRASONIC_SAMPLE_COUNT 5
#define DISTANCE_CHANGE_THRESHOLD_CM 20.0   // Only accept if change is greater than this
#define STABLE_DETECTION_INTERVAL_MS 2000   // Minimum time between valid detection events
/*------------------------------------------------------------------------------
 GLOBAL VARIABLES
------------------------------------------------------------------------------*/
static bool gate_is_open = false;
static float last_distance_cm = -1;
static int last_led_state = 0; // 0 = OFF, 1 = ON
static int64_t last_gate_action_time_us = 0;

/*------------------------------------------------------------------------------
 FUNCTION DECLARATIONS
------------------------------------------------------------------------------*/
/* Configuration */
void pwm_init(void);
/* Normal Function */
void gate_control_task(void *pvParameters);
// Read Value
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_connection();
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt);
static void read_gate();
static void read_led();
// Servo
uint32_t angle_to_duty_cycle(uint8_t angle);
void open_gate();
void close_gate();
void send_garage_data_back(int value);
// LED
void toggle_led(uint8_t state);
void read_led();
// HC-SR04
float get_ultrasonic_distance_cm();
void monitor_ultrasonic_task(void *pvParameters);
void send_task(void *pvParameters);
void send_ultrasonic_to_thingspeak(int value);
// HC-SR501
void configure_pir_sensor();
void monitor_pir_sensor();
void send_pir_to_thingspeak(int value);
/*------------------------------------------------------------------------------
 MAIN FUNCTION
------------------------------------------------------------------------------*/
void app_main(void)
{
    nvs_flash_init();
    wifi_connection();
    pwm_init();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("WIFI was initiated ...........\n\n");

    xTaskCreate(gate_control_task, "gate_control_task", 4096, NULL, 5, NULL);
    xTaskCreate(monitor_ultrasonic_task, "monitor_ultrasonic_task", 4096, NULL, 4, NULL);
    xTaskCreate(send_task, "send_task", 4096, NULL, 3, NULL);
}


/*------------------------------------------------------------------------------
 FUNCTION DEFINITIONS
------------------------------------------------------------------------------*/

void gate_control_task(void *pvParameters) {
    while (1) {
        read_gate();
        read_led();
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check every 10 seconds
    }
}

/* Function for read data from ThingSpeak */

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation 					s1.1
    esp_event_loop_create_default();     // event loop 			                s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station 	                    s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // 					                    s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_set_mode(WIFI_MODE_STA);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
    {
        // Log raw response for debugging
        char *json_data = (char *)evt->data;
        printf("Raw response: %.*s\n", evt->data_len, json_data);

        // Check if data is empty or too short
        if (evt->data_len == 0 || json_data == NULL) {
            printf("Empty or null response from ThingSpeak\n");
            break;
        }

        // Parse JSON
        cJSON *root = cJSON_ParseWithLength(json_data, evt->data_len);
        if (root == NULL) {
            printf("Error parsing JSON\n");
            break;
        }

        // Get field1 (for gate) and field2 (for LED)
        cJSON *field1 = cJSON_GetObjectItem(root, "field1");
        cJSON *field2 = cJSON_GetObjectItem(root, "field2");

        // Process field1 for gate control
        if (cJSON_IsString(field1) && field1->valuestring != NULL && (strcmp(field1->valuestring, "0") == 0 || strcmp(field1->valuestring, "1") == 0)) {
            printf("Field1 value: %s\n", field1->valuestring);
            if (atoi(field1->valuestring)) {
                if (!gate_is_open) {
                    open_gate();
                    gate_is_open = true;
                    ESP_LOGI(TAG, "Gate opened");
                }
            } else {
                if (gate_is_open) {
                    close_gate();
                    gate_is_open = false;
                    ESP_LOGI(TAG, "Gate closed");
                }
            }
        } else {
            printf("field1 is missing, not a string, or invalid value\n");
        }

        // Process field2 for LED control
        if (cJSON_IsString(field2) && field2->valuestring != NULL && (strcmp(field2->valuestring, "0") == 0 || strcmp(field2->valuestring, "1") == 0)) {
            printf("Field2 value: %s\n", field2->valuestring);
            int check = atoi(field2->valuestring);
            toggle_led(check);
        } else {
            printf("field2 is missing, not a string, or invalid value\n");
        }

        cJSON_Delete(root);
        break;
    }
    case HTTP_EVENT_ERROR:
        printf("HTTP client error\n");
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void read_gate()
{
    esp_http_client_config_t config_get = {
        .url = "http://api.thingspeak.com/channels/"CHANNEL"/fields/1/last.json?api_key="READ_API,
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        printf("HTTP request failed for read_gate: %s\n", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

static void read_led()
{
    esp_http_client_config_t config_get = {
        .url = "http://api.thingspeak.com/channels/"CHANNEL"/fields/2/last.json?api_key="READ_API,
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        printf("HTTP request failed for read_led: %s\n", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

/* Function for Servo */

uint32_t angle_to_duty_cycle(uint8_t angle) {
    if (angle > 180) angle = 180;
    // Map angle (0° -> 0.5ms, 180° -> 2.5ms) for 50Hz (20ms period)
    uint32_t pulse_width_us = 500 + (angle * 2000 / 180); // 500us to 2500us
    uint32_t duty = (pulse_width_us * 4096) / 20000; // 4096 (12-bit) / 20ms
    return duty;
}

void pwm_init(void){
    // Configure PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure PWM channel
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
}

void open_gate(){
    for(int angle = 65; angle <= 170; angle += 2) {
        ESP_LOGI(TAG, "Moving to %d degrees", angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty_cycle(angle));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void close_gate(){
    for(int angle = 170; angle >= 65; angle -= 2) {
        ESP_LOGI(TAG, "Moving to %d degrees", angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty_cycle(angle));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    gate_is_open = false;
    last_gate_action_time_us = esp_timer_get_time();  // ✅ Save time
    send_garage_data_back(0);
    ESP_LOGI(TAG, "Gate closed and notified ThingSpeak");
}



void send_garage_data_back(int value) {
    char url[256];
    snprintf(url, sizeof(url),
        "http://api.thingspeak.com/update?api_key=%s&field1=%d&field2=%d", 
        WRITE_BACK, value, last_led_state);

    esp_http_client_config_t config = {
        .url = url,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}


/* Function to control LED */

void toggle_led(uint8_t state) {
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    if (state == 1) {
        gpio_set_level(LED_GPIO, 1);
        ESP_LOGI(TAG, "LED ON");
    } else {
        gpio_set_level(LED_GPIO, 0);
        ESP_LOGI(TAG, "LED OFF");
    }
    last_led_state = state;
}


/* Function to control HC-SR04 */

float get_ultrasonic_distance_cm() {
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    float distance;
    esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
    if (res == ESP_OK) {
        return distance * 100.0;
    } else {
        ESP_LOGW(TAG, "Ultrasonic error: %s", esp_err_to_name(res));
        return -1.0;
    }
}

void monitor_ultrasonic_task(void *pvParameters) {
    while (1) {
        last_distance_cm = get_ultrasonic_distance_cm();
        if (last_distance_cm > 0) {
            ESP_LOGI(TAG, "Distance: %.2f cm", last_distance_cm);

            float threshold = (last_led_state == 1) ? 20.0 : 12.0;

            int64_t now = esp_timer_get_time();
            bool recently_closed = (now - last_gate_action_time_us) < 8000000; // 8 seconds in microseconds

            if (!recently_closed && last_distance_cm < threshold) {
                if (gate_is_open) {
                    close_gate();
                }
                printf("\nCar Detected! (Threshold: %.1f cm)\n", threshold);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}




void send_task(void *pvParameters)
{
    while (true)
    {
        if (!gate_is_open && last_distance_cm >= 0) {  // <-- Only send if gate is closed
            send_ultrasonic_to_thingspeak((int)last_distance_cm);
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Send every 5 seconds
    }
}


void send_ultrasonic_to_thingspeak(int value) {
    char url[256];
    snprintf(url, sizeof(url),
        "http://api.thingspeak.com/update?api_key=%s&field1=%d", WRITE_ULTRASONIC_API, value);

    esp_http_client_config_t config = {
        .url = url,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

/* Function to control HC-SR501 */

void configure_pir_sensor() {
    gpio_set_direction(PIR_SENSOR_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIR_SENSOR_GPIO, GPIO_PULLDOWN_ONLY);
    printf("✅ Monitoring PIR on GPIO %d\n", PIR_SENSOR_GPIO);
}

void monitor_pir_sensor() {
    int last_state = 0;
    while (1) {
        int state = gpio_get_level(PIR_SENSOR_GPIO);

        if (state != last_state) {
            if (state) {
                printf("ALERT: Motion detected!\n");
                send_pir_to_thingspeak(1);  // Send "1" to ThingSpeak to indicate motion
            } else {
                printf("Clear.\n");
                send_pir_to_thingspeak(0);  // Send "0" to ThingSpeak to indicate no motion
            }
            last_state = state;
            if (state) vTaskDelay(pdMS_TO_TICKS(500));  // Debounce on detection
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Polling interval
    }
}

void send_pir_to_thingspeak(int value) {
    char url[256];
    snprintf(url, sizeof(url),
        "http://api.thingspeak.com/update?api_key=%s&field1=%d", WRITE_PIR_API, value);

    esp_http_client_config_t config = {
        .url = url,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}