/**
 * @file udp_comms_task.c
 * @brief Implementação das tarefas de comunicação UDP desacopladas usando IPC.
 */

#include "udp_comms_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "ipc_manager.h"
#include <string.h>

static const char *TAG_TX = "UDP_TX";
static const char *TAG_RX = "UDP_RX";

#define UDP_COMMS_PORT_TX       5000
#define UDP_COMMS_PORT_RX       5001
#define UDP_COMMS_BROADCAST_IP  "192.168.4.255"
#define UDP_COMMS_BATCH_SIZE    25
#define UDP_COMMS_BUFFER_SIZE   4096

/**
 * @brief Processa internamente o payload JSON recebido.
 * @param[in] payload String contendo a estrutura JSON do controlador remoto.
 */
static void UDP_COMMS_ProcessCommand(const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    if (root != NULL) {
        cJSON *setpoint_item = cJSON_GetObjectItem(root, "new_setpoint");
        if (cJSON_IsNumber(setpoint_item)) {
            float new_setpoint = setpoint_item->valuedouble;
            
            /* Inserção limpa e encapsulada na máquina de estados via IPC */
            IPC_MGR_SetSetpoint(new_setpoint);
        }
        cJSON_Delete(root);
    }
}

void UDP_COMMS_TxTask(void *pvParameter)
{
    ESP_LOGI(TAG_TX, "Tarefa de Transmissao iniciada (Push Streaming).");

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

    control_data_t *sample_buffer = malloc(UDP_COMMS_BATCH_SIZE * sizeof(control_data_t));
    char *tx_buffer = malloc(UDP_COMMS_BUFFER_SIZE);

    if (sample_buffer == NULL || tx_buffer == NULL) {
        ESP_LOGE(TAG_TX, "Falha de alocacao de memoria.");
        vTaskDelete(NULL);
    }

    while (1) {
        int num_samples = 0;
        
        /* Bloqueia até a primeira amostra estar disponível usando a API do IPC */
        if (IPC_MGR_DequeueTelemetry(&sample_buffer[num_samples], portMAX_DELAY)) {
            num_samples++;
            
            /* Drena a fila agrupando dados para eficiência da camada MAC da rede */
            while (num_samples < UDP_COMMS_BATCH_SIZE) {
                if (IPC_MGR_DequeueTelemetry(&sample_buffer[num_samples], 2)) {
                    num_samples++;
                } else {
                    break;
                }
            }

            int offset = 0;
            offset += snprintf(tx_buffer + offset, UDP_COMMS_BUFFER_SIZE - offset, "[");
            
            for (int i = 0; i < num_samples; i++) {
                offset += snprintf(tx_buffer + offset, UDP_COMMS_BUFFER_SIZE - offset, 
                    "{\"timestamp_amostra_ms\":%.0f,\"valor_adc\":%d,\"tensao_mv\":%.2f,\"sinal_controle\":%.2f,\"tensao_estimada_mv\":%.2f,\"erro_obs_mv\":%.2f,\"estado_1\":%.4f,\"estado_2\":%.4f,\"estado_3\":%.4f}%s",
                    (double)sample_buffer[i].timestamp_amostra_ms,
                    sample_buffer[i].valor_adc_raw,
                    (float)sample_buffer[i].tensao_mv,
                    sample_buffer[i].sinal_controle,
                    sample_buffer[i].tensao_estimada_mv,
                    sample_buffer[i].erro_obs_mv,
                    sample_buffer[i].estado_1,
                    sample_buffer[i].estado_2,
                    sample_buffer[i].estado_3,
                    (i == num_samples - 1) ? "" : ","
                );
            }
            snprintf(tx_buffer + offset, UDP_COMMS_BUFFER_SIZE - offset, "]");

            sendto(sock, tx_buffer, strlen(tx_buffer), 0, (struct sockaddr *)&tx_addr, sizeof(tx_addr));
        }
    }

    free(sample_buffer);
    free(tx_buffer);
    close(sock);
    vTaskDelete(NULL);
}

void UDP_COMMS_RxTask(void *pvParameter)
{
    ESP_LOGI(TAG_RX, "Tarefa de Recepcao iniciada (Listening).");

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

    char rx_buffer[512];

    while (1) {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);
        
        /* Função de bloqueio profundo: O núcleo aguarda passivamente um datagrama UDP */
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len > 0) {
            rx_buffer[len] = '\0';
            UDP_COMMS_ProcessCommand(rx_buffer);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}