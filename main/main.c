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


#define WIFI_SSID       "TITANIC"
#define WIFI_PASSWORD   "$NovaPescaLivre332@;"

static const char *TAG = "APP_MAIN";

volatile float g_current_setpoint = 0.0f;
volatile float g_sensor_max_voltage_mv = 3100.0f;
volatile int64_t g_last_valid_communication_ms = 0;

// Definição das variáveis de debug
volatile uint32_t g_debug_samples_count = 0;
volatile uint32_t g_debug_batches_sent = 0;
volatile uint32_t g_debug_http_errors = 0;

/**
 * @brief Monitor do sistema.
 * * Responsabilidades:
 * 1. Monitorar a taxa de criação de dados.
 * 2. Monitorar o estado da fila.
 * 3. Monitorar a taxa de erros e envios.
 */
void system_monitor_task(void *pvParameter) {
    uint32_t last_samples = 0;
    
    while(1) {
        // Dorme por 2 segundos (2000 ms)
        vTaskDelay(pdMS_TO_TICKS(2000)); 

        // 1. Captura o estado atual
        uint32_t current_samples = g_debug_samples_count;
        uint32_t current_sent = g_debug_batches_sent;
        uint32_t current_errors = g_debug_http_errors;
        
        // Quantas mensagens estão esperando na fila?
        UBaseType_t items_in_queue = uxQueueMessagesWaiting(data_queue);
        UBaseType_t spaces_left = uxQueueSpacesAvailable(data_queue);
        
        // 2. Calcula a taxa de amostragem real (Hz)
        // (Amostras novas / 2 segundos)
        float sample_rate = (float)(current_samples - last_samples) / 2.0f;
        last_samples = current_samples;

        // 3. Verifica saúde da fila (Porcentagem de ocupação)
        float queue_usage = ((float)items_in_queue / (items_in_queue + spaces_left)) * 100.0f;

        // 4. Imprime RELATÓRIO CONSOLIDADO
        // Formato: [AUDITORIA] Taxa: 200Hz | Fila: 5% (50 items) | Envios: 120 | Erros: 0
        ESP_LOGW("AUDITORIA", "Taxa: %.1f Hz | Fila: %d (%.1f%%) | Envios: %lu | Erros: %lu", 
                sample_rate, 
                (int)items_in_queue, 
                queue_usage, 
                (unsigned long)current_sent, 
                (unsigned long)current_errors);

        // ALERTA DE GARGALO
        if (queue_usage > 80.0f) {
            ESP_LOGE("AUDITORIA", "!!! PERIGO: A fila está enchendo mais rápido do que esvazia !!!");
        }
    }
}

/**
 * @brief Ponto de entrada da aplicação.
 * * Responsabilidades:
 * 1. Inicializar o Wi-Fi.
 * 2. Criar os recursos de comunicação inter-core (Fila e Mutex).
 * 3. Criar e "pinar" as tarefas de controle e comunicação em seus respectivos cores.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando aplicação da Planta de Controle.");

    data_queue = xQueueCreate(1000, sizeof(control_data_t));
    g_setpoint_mutex = xSemaphoreCreateMutex();
    
    // Inicializa variáveis globais de segurança
    g_last_valid_communication_ms = esp_timer_get_time() / 1000;

    if(data_queue == NULL || g_setpoint_mutex == NULL){
        ESP_LOGE(TAG, "Falha crítica na criação de recursos.");
        return;
    }

    // Core 0 é ideal para tarefas de tempo real e controle de periféricos.
    xTaskCreatePinnedToCore(
                    control_loop_task,   // Função da tarefa
                    "Tarefa de Controle",// Nome
                    4096,                // Tamanho da stack
                    NULL,                // Parâmetros
                    10,                  // Prioridade alta para o controle
                    NULL,                // Handle da tarefa
                    0);                  // Core ID 0

    ESP_LOGI(TAG, "Controle ativo. Iniciando conexão Wi-Fi...");


    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);

    // Core 1 é ideal para tarefas "pesadas" como o stack de rede.
    xTaskCreatePinnedToCore(
                    communication_task,     // Função da tarefa
                    "Tarefa de Comunicação",// Nome
                    16384,                   // Stack maior por causa do Wi-Fi/TLS
                    NULL,                   // Parâmetros
                    5,                      // Prioridade normal
                    NULL,                   // Handle da tarefa
                    1);                     // Core ID 1

    xTaskCreatePinnedToCore(
                    system_monitor_task,
                    "Monitor",
                    4096,
                    NULL,
                    1, // Prioridade baixa
                    NULL,
                    1);

    ESP_LOGI(TAG, "Aplicação inicializada e rodando.");
}