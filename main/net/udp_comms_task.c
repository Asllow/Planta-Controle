/**
 * @file udp_comms_task.c
 * @brief Implementação das tarefas de comunicação UDP via Serialização Binária Loteada.
 */

#include "udp_comms_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "ipc_manager.h"
#include <string.h>

static const char *TAG_TX = "UDP_TX";
static const char *TAG_RX = "UDP_RX";

#define UDP_COMMS_PORT_TX       5000
#define UDP_COMMS_PORT_RX       5001
#define UDP_COMMS_BROADCAST_IP  "192.168.4.255"

/* Loteamento Estrito: 5 Amostras x 36 bytes = 180 bytes por pacote UDP */
#define UDP_COMMS_BATCH_SIZE    5 

/**
 * @brief Processa internamente o comando binário recebido.
 * @param[in] new_setpoint Referência de tensão (float) recebida via rede.
 */
static void UDP_COMMS_ProcessCommand(float new_setpoint)
{
    IPC_MGR_SetSetpoint(new_setpoint);
}

void UDP_COMMS_TxTask(void *pvParameter)
{
    ESP_LOGI(TAG_TX, "Tarefa de Transmissao Binaria iniciada (Zero-Copy Batching).");

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG_TX, "Falha critica na criacao do socket de TX.");
        vTaskDelete(NULL);
    }

    int broadcast_enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    struct sockaddr_in tx_addr;
    tx_addr.sin_family = AF_INET;
    tx_addr.sin_port = htons(UDP_COMMS_PORT_TX);
    inet_pton(AF_INET, UDP_COMMS_BROADCAST_IP, &tx_addr.sin_addr.s_addr);

    control_data_t tx_buffer[UDP_COMMS_BATCH_SIZE];

    while (1) {
        int samples_collected = 0;

        if (IPC_MGR_DequeueTelemetry(&tx_buffer[samples_collected], portMAX_DELAY)) {
            samples_collected++;

            while (samples_collected < UDP_COMMS_BATCH_SIZE) {
                if (IPC_MGR_DequeueTelemetry(&tx_buffer[samples_collected], pdMS_TO_TICKS(2))) {
                    samples_collected++;
                } else {
                    break; 
                }
            }

            sendto(sock, tx_buffer, samples_collected * sizeof(control_data_t), 0, (struct sockaddr *)&tx_addr, sizeof(tx_addr));
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

void UDP_COMMS_RxTask(void *pvParameter)
{
    ESP_LOGI(TAG_RX, "Tarefa de Recepcao iniciada (Binary Listening).");

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG_RX, "Falha critica na criacao do socket de RX.");
        vTaskDelete(NULL);
    }

    struct sockaddr_in rx_addr;
    rx_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_port = htons(UDP_COMMS_PORT_RX);

    if (bind(sock, (struct sockaddr *)&rx_addr, sizeof(rx_addr)) < 0) {
        ESP_LOGE(TAG_RX, "Falha no bind da porta de recepcao.");
        close(sock);
        vTaskDelete(NULL);
    }

    float incoming_setpoint_buffer;

    while (1) {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);
        
        int len = recvfrom(sock, &incoming_setpoint_buffer, sizeof(float), 0, (struct sockaddr *)&source_addr, &socklen);
        
        if (len == sizeof(float)) {
            UDP_COMMS_ProcessCommand(incoming_setpoint_buffer);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}