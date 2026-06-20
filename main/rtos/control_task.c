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

#define CONTROL_LOOP_FREQUENCY_HZ 200

void control_loop_task(void *pvParameters) {
    ESP_LOGI(TAG, "Inicializando periféricos de Controlo...");
    
    if (!HAL_PWM_Init() || !HAL_ADC_Init()) {
        ESP_LOGE(TAG, "Falha na inicializacao da HAL. Parando a tarefa.");
        vTaskDelete(NULL);
    }

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sampling_time_ticks = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQUENCY_HZ);

    while (1) {
        /* 1. Aquisição de Dados de Controlo */
        float current_setpoint = IPC_MGR_GetSetpoint();
        
        uint32_t sensor_voltage_mv = 0;
        int sensor_raw = 0;
        HAL_ADC_ReadVoltage(&sensor_voltage_mv, &sensor_raw);

        /* 2. Processamento Matemático (Isolado e Comutável) */
        /* TODO: Quando o LQG for ativado, substituir por MATH_CTRL_ComputeLQG */
        float control_effort = MATH_CTRL_ComputeOpenLoop(current_setpoint);

        /* 3. Atuação Física na Planta */
        HAL_PWM_SetDutyCycle(control_effort);

        /* 4. Telemetria e Diagnóstico */
        control_data_t telemetry;
        telemetry.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        telemetry.valor_adc_raw        = sensor_raw;
        telemetry.tensao_mv            = sensor_voltage_mv;
        telemetry.sinal_controle       = current_setpoint; 
        telemetry.tensao_estimada_mv   = 0.0f; /* Reservado LQG */
        telemetry.erro_obs_mv          = 0.0f; /* Reservado LQG */
        telemetry.estado_1             = 0.0f; /* Reservado LQG */

        IPC_MGR_EnqueueTelemetry(&telemetry);

        /* 5. Suspensão Estrita (Garante o Ts fixo de 5ms) */
        vTaskDelayUntil(&last_wake_time, sampling_time_ticks);
    }
}