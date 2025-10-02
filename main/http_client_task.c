// main/http_client_task.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "shared_resources.h"
#include <inttypes.h>

static const char *TAG = "HTTP_CLIENT_TASK";

// IMPORTANTE: Altere para o IP do computador onde o dashboard Python está rodando
#define SERVER_URL "http://192.168.1.10:5000/data"

// Handler de eventos HTTP para fins de depuração (opcional)
esp_err_t _http_event_handler(esp_http_client_event_t *evt) { return ESP_OK; }

/**
 * @brief Tarefa de comunicação, rodando em background.
 * * Esta tarefa é responsável por:
 * 1. Aguardar dados chegarem na fila (vindo do Core 0).
 * 2. Formatar os dados em um payload JSON.
 * 3. Enviar os dados para o servidor via HTTP POST.
 * 4. Ler a resposta do servidor para obter novos comandos de setpoint.
 * 5. Fazer o parse do JSON da resposta.
 * 6. Atualizar a variável global de setpoint de forma segura.
 */
void communication_task(void *pvParameter) {
    ESP_LOGI(TAG, "Tarefa de Comunicação iniciada no Core %d", xPortGetCoreID());

    control_data_t received_data;
    char json_payload[256];
    char response_buffer[128];

    while(1) {
        // Bloqueia indefinidamente até que um item chegue na fila.
        // Esta é uma forma muito eficiente de aguardar, sem consumir CPU.
        if (xQueueReceive(data_queue, &received_data, portMAX_DELAY) == pdPASS) {
            
            // 1. Monta o pacote JSON para enviar ao servidor
            snprintf(json_payload, sizeof(json_payload), 
                    "{\"timestamp_amostra_ms\": %lld, \"valor_adc\": %d, \"tensao_mv\": %" PRIu32 ", \"sinal_controle\": %.2f}",
                    received_data.timestamp_amostra_ms,
                    received_data.valor_adc_raw,
                    received_data.tensao_mv,
                    received_data.sinal_controle);
            
            // 2. Configura e executa a requisição HTTP
            esp_http_client_config_t config = {
                .url = SERVER_URL,
                .event_handler = _http_event_handler,
                .timeout_ms = 5000, // Timeout de 5 segundos
            };
            esp_http_client_handle_t client = esp_http_client_init(&config);
            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_header(client, "Content-Type", "application/json");
            esp_http_client_set_post_field(client, json_payload, strlen(json_payload));

            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                int status_code = esp_http_client_get_status_code(client);
                if (status_code == 200) {
                    // 3. Lê o corpo da resposta para obter o comando
                    int read_len = esp_http_client_read_response(client, response_buffer, sizeof(response_buffer) - 1);
                    if (read_len > 0) {
                        response_buffer[read_len] = '\0';
                        
                        // 4. Faz o parse do JSON recebido
                        cJSON *root = cJSON_Parse(response_buffer);
                        if (root) {
                            cJSON *setpoint_item = cJSON_GetObjectItem(root, "new_setpoint");
                            if (cJSON_IsNumber(setpoint_item)) {
                                float new_setpoint = setpoint_item->valuedouble;
                                
                                // 5. Atualiza a variável global de forma segura
                                if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
                                    g_current_setpoint = new_setpoint;
                                    xSemaphoreGive(g_setpoint_mutex);
                                    ESP_LOGI(TAG, "Novo setpoint recebido: %.2f %%", new_setpoint);
                                }
                            }
                            cJSON_Delete(root); // Libera a memória do objeto JSON
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "Erro no servidor, status HTTP: %d", status_code);
                }
            } else {
                ESP_LOGE(TAG, "Falha na requisição HTTP: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(client); // Libera os recursos do cliente HTTP
        }
    }
}