/**
 * @file control_task.c
 * @brief Tarefa central de tempo real do sistema de controlo.
 * * Orquestra as leituras da HAL, o processamento matemático e 
 * a escrita nos atuadores, ditando a frequência de discretização.
 */

#include "control_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "ipc_manager.h"
#include "hal_motor_pwm.h"
#include "hal_sensor_adc.h"
#include "control_algorithms.h"

static const char *TAG = "CTRL_TASK";

#define CONTROL_LOOP_FREQUENCY_HZ 1000


void control_loop_task(void *pvParameters) {
    ESP_LOGI(TAG, "Inicializando periféricos de Controlo...");
    
    if (!HAL_PWM_Init() || !HAL_ADC_Init()) {
        ESP_LOGE(TAG, "Falha na inicializacao da HAL. Parando a tarefa.");
        vTaskDelete(NULL);
    }

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sampling_time_ticks = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQUENCY_HZ);

    while (1) {
        float current_setpoint = IPC_MGR_GetSetpoint();

        float X1, float X2, float X3 = IPC_MGR_DesiredReference(current_setpoint);

        
        
        uint32_t sensor_voltage_mv = 0;
        int sensor_raw = 0;
        HAL_ADC_ReadVoltage(&sensor_voltage_mv, &sensor_raw);

        float control_effort = MATH_CTRL_ComputeOpenLoop(current_setpoint);

        HAL_PWM_SetDutyCycle(control_effort);

        control_data_t telemetry;
        telemetry.timestamp_amostra_ms = (uint32_t)(esp_timer_get_time() / 1000);
        telemetry.sinal_controle       = current_setpoint; 
        telemetry.tensao_mv            = (float)sensor_voltage_mv;
        telemetry.valor_adc            = (float)sensor_raw;
        telemetry.tensao_estimada_mv   = 0.0f; 
        telemetry.erro_obs_mv          = 0.0f; 
        telemetry.estado_1             = 0.0f; 
        telemetry.estado_2             = 0.0f; 
        telemetry.estado_3             = 0.0f; 

        IPC_MGR_EnqueueTelemetry(&telemetry);

        vTaskDelayUntil(&last_wake_time, sampling_time_ticks);
    }
}