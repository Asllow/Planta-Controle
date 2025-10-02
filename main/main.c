#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "shared_resources.h"
#include "wifi_manager.h"
#include "control_task.h"
#include "http_client_task.h"

// --- Definição das Variáveis Globais ---
// Estas são as definições reais das variáveis declaradas como 'extern' em shared_resources.h

QueueHandle_t data_queue;
SemaphoreHandle_t g_setpoint_mutex;
volatile float g_current_setpoint = 0.0f; // Começa com 0 para segurança

// Definição da variável de calibração, com um valor inicial padrão/de bancada
volatile float g_sensor_max_voltage_mv = 3100.0f;

#define WIFI_SSID       "TITANIC"
#define WIFI_PASSWORD   "$NovaPescaLivre332@;"

static const char *TAG = "APP_MAIN";

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

    // 1. Conecta ao Wi-Fi (esta função é bloqueante)
    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);

    // 2. Cria a Fila para comunicar os dados.
    // Terá espaço para 10 pacotes de dados.
    data_queue = xQueueCreate(10, sizeof(control_data_t));
    if(data_queue == NULL){
        ESP_LOGE(TAG, "Falha ao criar a fila.");
        return; // Falha crítica
    }

    // 3. Cria o Mutex para proteger a variável de setpoint
    g_setpoint_mutex = xSemaphoreCreateMutex();
    if(g_setpoint_mutex == NULL){
        ESP_LOGE(TAG, "Falha ao criar o mutex.");
        return; // Falha crítica
    }

    ESP_LOGI(TAG, "Recursos criados. Iniciando tarefas principais...");

    // 4. Cria a tarefa de controle e a "pina" no Core 0 (PRO_CPU)
    // Core 0 é ideal para tarefas de tempo real e controle de periféricos.
    xTaskCreatePinnedToCore(
                    control_loop_task,   // Função da tarefa
                    "Tarefa de Controle",// Nome
                    4096,                // Tamanho da stack
                    NULL,                // Parâmetros
                    10,                  // Prioridade alta para o controle
                    NULL,                // Handle da tarefa
                    0);                  // Core ID 0

    // 5. Cria a tarefa de comunicação e a "pina" no Core 1 (APP_CPU)
    // Core 1 é ideal para tarefas "pesadas" como o stack de rede.
    xTaskCreatePinnedToCore(
                    communication_task,     // Função da tarefa
                    "Tarefa de Comunicação",// Nome
                    8192,                   // Stack maior por causa do Wi-Fi/TLS
                    NULL,                   // Parâmetros
                    5,                      // Prioridade normal
                    NULL,                   // Handle da tarefa
                    1);                     // Core ID 1

    ESP_LOGI(TAG, "Aplicação inicializada e rodando.");
}