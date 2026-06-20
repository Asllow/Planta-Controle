/**
 * @file ipc_manager.c
 * @brief Implementação do Gestor de Comunicação Inter-Processos (IPC).
 */

#include "ipc_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "IPC_MGR";

#define TELEMETRY_QUEUE_SIZE 3000

static QueueHandle_t s_telemetry_queue = NULL;
static SemaphoreHandle_t s_setpoint_mutex = NULL;
static float s_current_setpoint = 0.0f;

bool IPC_MGR_Init(void) {
    s_telemetry_queue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(control_data_t));
    s_setpoint_mutex = xSemaphoreCreateMutex();

    if (s_telemetry_queue == NULL || s_setpoint_mutex == NULL) {
        ESP_LOGE(TAG, "Falha critica na alocacao de recursos do IPC.");
        return false;
    }
    return true;
}

void IPC_MGR_SetSetpoint(float setpoint) {
    if (s_setpoint_mutex != NULL) {
        if (xSemaphoreTake(s_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
            s_current_setpoint = setpoint;
            xSemaphoreGive(s_setpoint_mutex);
        }
    }
}

float IPC_MGR_GetSetpoint(void) {
    float temp_setpoint = 0.0f;
    if (s_setpoint_mutex != NULL) {
        if (xSemaphoreTake(s_setpoint_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            temp_setpoint = s_current_setpoint;
            xSemaphoreGive(s_setpoint_mutex);
        }
    }
    return temp_setpoint;
}

bool IPC_MGR_EnqueueTelemetry(const control_data_t *data) {
    if (s_telemetry_queue == NULL) return false;
    return (xQueueSend(s_telemetry_queue, data, 0) == pdTRUE);
}

bool IPC_MGR_DequeueTelemetry(control_data_t *data, uint32_t timeout_ms) {
    if (s_telemetry_queue == NULL) return false;
    return (xQueueReceive(s_telemetry_queue, data, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}