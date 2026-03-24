#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "shared_resources.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "HTTP_CLIENT_TASK";

#define SERVER_URL "http://192.168.137.1:5000/data"
#define BATCH_SIZE 50

typedef struct {
    char buffer[128];
    int buffer_len;
} http_response_data_t;

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    http_response_data_t *response_data = (http_response_data_t*)evt->user_data;
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (response_data->buffer_len + evt->data_len < sizeof(response_data->buffer)) {
                memcpy(response_data->buffer + response_data->buffer_len, evt->data, evt->data_len);
                response_data->buffer_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            response_data->buffer[response_data->buffer_len] = '\0';
            break;
        default: break;
    }
    return ESP_OK;
}

void communication_task(void *pvParameter) {
    ESP_LOGI(TAG, "Tarefa de Comunicação OTIMIZADA iniciada.");

    control_data_t sample_buffer[BATCH_SIZE];
    char *json_payload = NULL;
    http_response_data_t response_data = {0};

    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .event_handler = _http_event_handler,
        .user_data = &response_data,
        .timeout_ms = 2000,
        .keep_alive_enable = true,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    while(1) {
        if (xQueueReceive(data_queue, &sample_buffer[0], portMAX_DELAY) == pdPASS) {

            int num_samples = 1;
            
            while (num_samples < BATCH_SIZE) {
                if (xQueueReceive(data_queue, &sample_buffer[num_samples], 10) == pdPASS) {
                    num_samples++;
                } else {
                    break;
                }
            }

            cJSON *root = cJSON_CreateArray();
            for (int i = 0; i < num_samples; i++) {
                cJSON *sample_json = cJSON_CreateObject();
                cJSON_AddNumberToObject(sample_json, "timestamp_amostra_ms", (double)sample_buffer[i].timestamp_amostra_ms);
                cJSON_AddNumberToObject(sample_json, "valor_adc", sample_buffer[i].valor_adc_raw);
                cJSON_AddNumberToObject(sample_json, "tensao_mv", sample_buffer[i].tensao_mv);
                cJSON_AddNumberToObject(sample_json, "sinal_controle", sample_buffer[i].sinal_controle);
                cJSON_AddItemToArray(root, sample_json);
            }
            json_payload = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);

            if (json_payload != NULL) {
                memset(&response_data, 0, sizeof(http_response_data_t));
                
                esp_http_client_set_post_field(client, json_payload, strlen(json_payload));

                esp_err_t err = esp_http_client_perform(client);

                if (err == ESP_OK) {
                    g_last_valid_communication_ms = esp_timer_get_time() / 1000;

                    if (response_data.buffer_len > 0) {
                        cJSON *response_root = cJSON_Parse(response_data.buffer);
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
                    }
                } else {
                    ESP_LOGE(TAG, "Erro HTTP: %s", esp_err_to_name(err));
                    
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    UBaseType_t items_waiting = uxQueueMessagesWaiting(data_queue);
                    if (items_waiting > 800) {
                        xQueueReset(data_queue);
                        ESP_LOGW(TAG, "!!! FILA RESETADA !!! Descartados %d itens.", items_waiting);
                    }
                }
                
                free(json_payload);
            }
        }
    }
    esp_http_client_cleanup(client);
}