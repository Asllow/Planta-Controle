/**
 * @file control_task.c
 * @brief Tarefa central de tempo real do sistema de controlo.
 * * NOTA ARQUITETURAL: A tarefa lê um valor agnóstico genérico (float).
 * O limite físico e semântico é aplicado exclusivamente dentro das funções 
 * matemáticas do control_algorithms.c, garantindo segurança funcional (Failsafe).
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

    ESP_LOGI(TAG, "Iniciando Malha de Tempo Real a 1000Hz (Agnostic Mode).");

    while (1) {
        float generic_reference = IPC_MGR_GetReference();
        
        uint32_t sensor_voltage_mv = 0;
        int sensor_raw = 0;
        HAL_ADC_ReadVoltage(&sensor_voltage_mv, &sensor_raw);

        float control_effort_duty = MATH_CTRL_ComputeOpenLoop(generic_reference);

        HAL_PWM_SetDutyCycle(control_effort_duty);

        control_data_t telemetry;
        telemetry.timestamp_amostra_ms = (uint32_t)(esp_timer_get_time() / 1000);
        telemetry.sinal_controle       = control_effort_duty; 
        telemetry.tensao_mv            = (float)sensor_voltage_mv;
        telemetry.valor_adc            = (float)sensor_raw;

        IPC_MGR_EnqueueTelemetry(&telemetry);

        vTaskDelayUntil(&last_wake_time, sampling_time_ticks);
    }
}