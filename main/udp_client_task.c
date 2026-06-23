#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "shared_resources.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "UDP_TASK";

#define UDP_PORT 5000
#define BATCH_SIZE 25

void communication_task(void *pvParameter) {
    ESP_LOGI(TAG, "Tarefa de Comunicação UDP (Binary) iniciada.");

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Erro ao criar socket");
        vTaskDelete(NULL);
    }
    int broadcastEnable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(UDP_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.4.255");

    control_data_t *sample_buffer = malloc(BATCH_SIZE * sizeof(control_data_t));

    while(1) {
        float new_setpoint;
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        
        int len = recvfrom(sock, &new_setpoint, sizeof(float), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len == sizeof(float)) {
            if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
                g_current_setpoint = new_setpoint;
                xSemaphoreGive(g_setpoint_mutex);
            }
            g_last_valid_communication_ms = esp_timer_get_time() / 1000;
            dest_addr.sin_addr.s_addr = source_addr.sin_addr.s_addr;
        }

        int num_samples = 0;
        if (xQueueReceive(data_queue, &sample_buffer[num_samples], pdMS_TO_TICKS(5)) == pdPASS) {
            num_samples++;
            while (num_samples < BATCH_SIZE) {
                if (xQueueReceive(data_queue, &sample_buffer[num_samples], 0) == pdPASS) {
                    num_samples++;
                } else {
                    break;
                }
            }
            sendto(sock, sample_buffer, num_samples * sizeof(control_data_t), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
    }
}