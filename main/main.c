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

/*------------------------------------------------------------------------------
PROGRAM CONSTANTS
------------------------------------------------------------------------------*/
#define TAG "ServoMotor"
#define SERVO_GPIO GPIO_NUM_13
#define LED_GPIO GPIO_NUM_2

/*------------------------------------------------------------------------------
 GLOBAL VARIABLES
------------------------------------------------------------------------------*/
static bool gate_is_open = false;

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
static void rest_get();
// Servo
uint32_t angle_to_duty_cycle(uint8_t angle);
void open_gate();
void close_gate();
// LED
void toggle_led(uint8_t state);
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

}


/*------------------------------------------------------------------------------
 FUNCTION DEFINITIONS
------------------------------------------------------------------------------*/

void gate_control_task(void *pvParameters) {
    while (1) {
        rest_get();
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check every 10 seconds
    }
}


/*  Function for read data from ThingSpeak  */

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
        char *json_data = (char *)evt->data;

        cJSON *root = cJSON_ParseWithLength(json_data, evt->data_len);
        if (root == NULL) {
            printf("Error parsing JSON\n");
            break;
        }

        cJSON *field1 = cJSON_GetObjectItem(root, "field1");
        if (cJSON_IsString(field1) && field1->valuestring != NULL) {
            printf("Field1 value: %s\n", field1->valuestring);
            if (strcmp(field1->valuestring, "0") == 0 || strcmp(field1->valuestring, "1") == 0) {
                toggle_led(atoi(field1->valuestring));
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
                printf("Field1 is not 0 or 1\n");
            }
        } else {
            printf("field1 is missing or not a string\n");
        }

        cJSON_Delete(root);
        break;
    }

    default:
        break;
    }
    return ESP_OK;
}

static void rest_get()
{
    esp_http_client_config_t config_get = {
        .url = "http://api.thingspeak.com/channels/2987995/fields/1/last.json?api_key=8FB4KKVVEHU5ALBV",
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

/* Function for Servo */

uint32_t angle_to_duty_cycle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    // Map angle (0° -> 0.5ms, 180° -> 2.5ms)
    float pulse_width = 0.5 + (angle / 180.0) * 2.0;
    // Convert pulse width to duty cycle (12-bit resolution, 50Hz)
    uint32_t duty = (pulse_width / 20.0) * 4096;
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
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


void close_gate(){
    for(int angle = 170; angle >= 65; angle -= 2) {
        ESP_LOGI(TAG, "Moving to %d degrees", angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty_cycle(angle));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


void toggle_led(uint8_t state) {
    gpio_set_direction(LED_GPIO,GPIO_MODE_OUTPUT);
    if (state == 1) {
        gpio_set_level(LED_GPIO, 1);  // Turn LED ON
        ESP_LOGI(TAG, "LED ON");
    } else {
        gpio_set_level(LED_GPIO, 0);  // Turn LED OFF
        ESP_LOGI(TAG, "LED OFF");
    }
}
