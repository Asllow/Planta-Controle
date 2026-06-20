/**
 * @file main.c
 * @brief Ponto de entrada da aplicação. Orquestra a inicialização de subsistemas
 * e o escalonamento das tarefas no FreeRTOS.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

/* Inclusões da Nova Arquitetura */
#include "wifi_manager.h"
#include "udp_comms_task.h"
#include "ipc_manager.h"
#include "control_task.h"

/* Credenciais do Ponto de Acesso (SoftAP) */
#define APP_MAIN_WIFI_SSID       "Planta_Controle_AP"
#define APP_MAIN_WIFI_PASSWORD   "12345678"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando Planta de Controlo - Arquitetura HAL/IPC");

    /* 1. Inicialização do Gestor de Comunicação Inter-Processos (IPC) */
    if (!IPC_MGR_Init()) {
        ESP_LOGE(TAG, "Falha critica na alocacao de recursos IPC. Sistema abortado.");
        return;
    }

    /* 2. Tarefa de Controlo de Tempo Real (Fixo no Core 0 para determinismo) */
    xTaskCreatePinnedToCore(
        control_loop_task,   
        "Tarefa_Controlo",
        4096,                
        NULL,                
        10,                  
        NULL,                
        0
    );                  

    ESP_LOGI(TAG, "Controlo ativo. Iniciando infraestrutura de Rede (SoftAP)...");
    
    /* 3. Inicialização do Wi-Fi em modo Access Point */
    if (WIFI_MGR_InitAP(APP_MAIN_WIFI_SSID, APP_MAIN_WIFI_PASSWORD) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o Access Point.");
        return;
    }

    /* 4. Tarefa de Transmissão de Telemetria via UDP (Core 1) */
    xTaskCreatePinnedToCore(
        UDP_COMMS_TxTask,     
        "Tarefa_Tx_UDP",
        8192,                  
        NULL,                   
        5,                      
        NULL,                   
        1
    );                     

    /* 5. Tarefa de Recepção de Comandos via UDP (Core 1) */
    xTaskCreatePinnedToCore(
        UDP_COMMS_RxTask,     
        "Tarefa_Rx_UDP",
        4096,                  
        NULL,                   
        6,                      
        NULL,                   
        1
    );

    ESP_LOGI(TAG, "Orquestracao inicializada com sucesso. Sistema a operar em modo AP.");
}