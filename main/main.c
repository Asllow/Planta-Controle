#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "shared_resources.h"
#include "wifi_manager.h"
#include "esp_timer.h"
#include "control_task.h"
#include "udp_client_task.h"

QueueHandle_t data_queue;
SemaphoreHandle_t g_setpoint_mutex;

#define WIFI_SSID       "Planta_Controle"
#define WIFI_PASSWORD   "12345678"

static const char *TAG = "APP_MAIN";

volatile float g_current_setpoint = 0.0f;
volatile float g_sensor_max_voltage_mv = 3100.0f;
volatile int64_t g_last_valid_communication_ms = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando Planta de Controle - Modo Simplificado.");

    data_queue = xQueueCreate(3000, sizeof(control_data_t));
    g_setpoint_mutex = xSemaphoreCreateMutex();
    g_last_valid_communication_ms = esp_timer_get_time() / 1000;

    if(data_queue == NULL || g_setpoint_mutex == NULL){
        ESP_LOGE(TAG, "Falha crítica na criação de recursos.");
        return;
    }

    xTaskCreatePinnedToCore(
                    control_loop_task,   
                    "Tarefa de Controle",
                    4096,                
                    NULL,                
                    10,                  
                    NULL,                
                    0);                  

    ESP_LOGI(TAG, "Criando Rede Wi-Fi (Modo AP)...");
    wifi_init_ap(WIFI_SSID, WIFI_PASSWORD);

    xTaskCreatePinnedToCore(
                    communication_task,     
                    "Tarefa de Comunicação",
                    16384,                  
                    NULL,                   
                    5,                      
                    NULL,                   
                    1);                     

    ESP_LOGI(TAG, "Sistema inicializado. Tarefas separadas por Core.");
}