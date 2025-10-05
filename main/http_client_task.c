// main/http_client_task.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "shared_resources.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "HTTP_CLIENT_TASK";

#define SERVER_URL "http://192.168.100.132:5000/data"

// --- Estrutura para passar dados para o event handler ---
typedef struct {
    char buffer[128];
    int buffer_len;
} http_response_data_t;

/**
 * @brief Manipulador de eventos para o cliente HTTP.
 *
 * Este é o "espião" que captura os dados da resposta.
 * Quando um evento HTTP_EVENT_ON_DATA ocorre, copiamos o pedaço de
 * dados recebido para o nosso buffer.
 */
esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    http_response_data_t *response_data = (http_response_data_t*)evt->user_data;

    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            // Garante que não vamos estourar o buffer
            if (response_data->buffer_len + evt->data_len < sizeof(response_data->buffer)) {
                memcpy(response_data->buffer + response_data->buffer_len, evt->data, evt->data_len);
                response_data->buffer_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            // Adiciona o terminador nulo ao final do buffer quando a resposta termina
            response_data->buffer[response_data->buffer_len] = '\0';
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 * @brief Tarefa de comunicação, agora usando o event handler para ler a resposta.
 */
void communication_task(void *pvParameter) {
    ESP_LOGI(TAG, "Tarefa de Comunicação iniciada no Core %d", xPortGetCoreID());

    control_data_t sample_buffer[10];
    char *json_payload = NULL;
    http_response_data_t response_data = {0};

    while(1) {
        // 1. Espera pelo menos um item chegar na fila
        if (xQueueReceive(data_queue, &sample_buffer[0], portMAX_DELAY) == pdPASS) {
            
            int num_samples = 1;
            // 2. Coleta os outros itens que já estão na fila (batching)
            while (num_samples < 10 && xQueueReceive(data_queue, &sample_buffer[num_samples], 0) == pdPASS) {
                num_samples++;
            }
            ESP_LOGI(TAG, "Agregando %d amostras para envio.", num_samples);

            // 3. Constrói o array JSON com as amostras coletadas
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
                esp_http_client_config_t config = {
                    .url = SERVER_URL,
                    .event_handler = _http_event_handler,
                    .user_data = &response_data,
                    .timeout_ms = 5000,
                };

                esp_http_client_handle_t client = esp_http_client_init(&config);
                esp_http_client_set_method(client, HTTP_METHOD_POST);
                esp_http_client_set_header(client, "Content-Type", "application/json");
                esp_http_client_set_post_field(client, json_payload, strlen(json_payload));

                esp_err_t err = esp_http_client_perform(client);

                // 4. Processa a resposta do servidor
                if (err == ESP_OK) {
                    if (response_data.buffer_len > 0) {
                        cJSON *response_root = cJSON_Parse(response_data.buffer);
                        if (response_root) {
                            // Procura por um novo setpoint
                            cJSON *setpoint_item = cJSON_GetObjectItem(response_root, "new_setpoint");
                            if (cJSON_IsNumber(setpoint_item)) {
                                float new_setpoint = setpoint_item->valuedouble;
                                if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
                                    g_current_setpoint = new_setpoint;
                                    xSemaphoreGive(g_setpoint_mutex);
                                    ESP_LOGI(TAG, "SUCESSO: Novo setpoint (%.2f) recebido.", new_setpoint);
                                }
                            }
                            
                            // Procura por uma nova frequência
                            cJSON *freq_item = cJSON_GetObjectItem(response_root, "new_frequency");
                            if (cJSON_IsNumber(freq_item)) {
                                uint32_t new_freq = (uint32_t)freq_item->valuedouble;
                                if (new_freq > 0) {
                                    g_new_frequency_hz = new_freq; // Sinaliza para a outra tarefa
                                    ESP_LOGI(TAG, "SUCESSO: Novo comando de frequência (%" PRIu32 " Hz) recebido.", new_freq);
                                }
                            }
                            cJSON_Delete(response_root);
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "Falha na requisição HTTP: %s", esp_err_to_name(err));
                }
                esp_http_client_cleanup(client);
                free(json_payload);
            }
        }
    }
}