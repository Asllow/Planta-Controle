#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "cJSON.h"
#include "shared_resources.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "WEBSOCKET_TASK";

#define SERVER_WS_URL "ws://192.168.137.1:5000/ws" 
#define BATCH_SIZE 250 

static void _websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conexao estabelecida com o Servidor Python");
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Conexao perdida");
            break;
            
        case WEBSOCKET_EVENT_DATA:
            if (data->op_code == 0x01 && data->data_len > 0) { 
                g_last_valid_communication_ms = esp_timer_get_time() / 1000;
                
                char *resp_str = malloc(data->data_len + 1);
                if (resp_str) {
                    memcpy(resp_str, data->data_ptr, data->data_len);
                    resp_str[data->data_len] = '\0';
                    
                    cJSON *response_root = cJSON_Parse(resp_str);
                    if (response_root) {
                        cJSON *setpoint_item = cJSON_GetObjectItem(response_root, "new_setpoint");
                        if (cJSON_IsNumber(setpoint_item)) {
                            float new_setpoint = setpoint_item->valuedouble;
                            if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
                                g_current_setpoint = new_setpoint;
                                xSemaphoreGive(g_setpoint_mutex);
                            }
                        }
                        cJSON_Delete(response_root);
                    }
                    free(resp_str);
                }
            }
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "Erro de rede no WebSocket");
            break;
    }
}

void communication_task(void *pvParameter) {
    ESP_LOGI(TAG, "Tarefa de Comunicacao SINCRONIZADA iniciada.");

    control_data_t *sample_buffer = malloc(BATCH_SIZE * sizeof(control_data_t));
    int max_len = BATCH_SIZE * 180 + 100;
    char *json_payload = malloc(max_len);

    if (sample_buffer == NULL || json_payload == NULL) {
        ESP_LOGE(TAG, "Erro de alocacao!");
        vTaskDelete(NULL);
    }

    esp_websocket_client_config_t websocket_cfg = {
        .uri = SERVER_WS_URL,
        .buffer_size = max_len + 512,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 5000
    };

    esp_websocket_client_handle_t client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, _websocket_event_handler, (void *)client);
    esp_websocket_client_start(client);

    while(1) {
        int num_samples = 0;
        
        if (xQueueReceive(data_queue, &sample_buffer[num_samples], portMAX_DELAY) == pdPASS) {
            num_samples++;
            
            while (num_samples < BATCH_SIZE) {
                if (xQueueReceive(data_queue, &sample_buffer[num_samples], pdMS_TO_TICKS(5)) == pdPASS) {
                    num_samples++;
                } else {
                    break; 
                }
            }

            if (esp_websocket_client_is_connected(client)) {
                int offset = 0;
                offset += snprintf(json_payload + offset, max_len - offset, "[");
                
                for (int i = 0; i < num_samples; i++) {
#ifdef ENABLE_OBSERVER_DEBUG
                    offset += snprintf(json_payload + offset, max_len - offset, 
                        "{\"timestamp_amostra_ms\":%.0f,\"valor_adc\":%d,\"tensao_mv\":%.2f,\"sinal_controle\":%.2f,\"tensao_estimada_mv\":%.2f,\"erro_obs_mv\":%.2f}%s",
                        (double)sample_buffer[i].timestamp_amostra_ms,
                        (int)sample_buffer[i].valor_adc_raw,
                        (float)sample_buffer[i].tensao_mv,
                        (float)sample_buffer[i].sinal_controle,
                        (float)sample_buffer[i].tensao_estimada_mv,
                        (float)sample_buffer[i].erro_obs_mv,
                        (i == num_samples - 1) ? "" : ","
                    );
#else
                    offset += snprintf(json_payload + offset, max_len - offset, 
                        "{\"timestamp_amostra_ms\":%.0f,\"valor_adc\":%d,\"tensao_mv\":%.2f,\"sinal_controle\":%.2f}%s",
                        (double)sample_buffer[i].timestamp_amostra_ms,
                        (int)sample_buffer[i].valor_adc_raw,
                        (float)sample_buffer[i].tensao_mv,
                        (float)sample_buffer[i].sinal_controle,
                        (i == num_samples - 1) ? "" : ","
                    );
#endif
                }
                
                snprintf(json_payload + offset, max_len - offset, "]");

                esp_websocket_client_send_text(client, json_payload, strlen(json_payload), portMAX_DELAY);
                g_last_valid_communication_ms = esp_timer_get_time() / 1000;
                
            } else {
                vTaskDelay(pdMS_TO_TICKS(1000));
                UBaseType_t items_waiting = uxQueueMessagesWaiting(data_queue);
                if (items_waiting > 1000) {
                    xQueueReset(data_queue);
                    ESP_LOGW(TAG, "FILA RESETADA. WS Offline.");
                }
            }
        }
    }
    
    free(sample_buffer);
    free(json_payload);
    esp_websocket_client_destroy(client);
}