#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "shared_resources.h"
#include "wifi_manager.h"
#include "esp_timer.h"
#include "control_task.h"
#include "http_client_task.h"

QueueHandle_t data_queue;
SemaphoreHandle_t g_setpoint_mutex;

#define WIFI_SSID       "Rede_Planta"
#define WIFI_PASSWORD   "Rede_Planta"

static const char *TAG = "APP_MAIN";

// Mantemos apenas as variáveis globais essenciais para o controle/comunicação
volatile float g_current_setpoint = 0.0f;
volatile float g_sensor_max_voltage_mv = 3100.0f;
volatile int64_t g_last_valid_communication_ms = 0;

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando Planta de Controle - Modo Simplificado.");

    // 1. Criação de Recursos (Fila e Mutex)
    data_queue = xQueueCreate(3000, sizeof(control_data_t));
    g_setpoint_mutex = xSemaphoreCreateMutex();
    g_last_valid_communication_ms = esp_timer_get_time() / 1000;

    if(data_queue == NULL || g_setpoint_mutex == NULL){
        ESP_LOGE(TAG, "Falha crítica na criação de recursos.");
        return;
    }

    // 2. Tarefa de Controle (Core 0 - Tempo Real)
    // Aqui roda a malha aberta, leitura do tacogerador e PWM
    xTaskCreatePinnedToCore(
                    control_loop_task,   
                    "Tarefa de Controle",
                    4096,                
                    NULL,                
                    10,                  
                    NULL,                
                    0);                  

    ESP_LOGI(TAG, "Controle ativo. Iniciando conexão Wi-Fi...");
    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);

    // 3. Tarefa de Comunicação (Core 1 - Rede)
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