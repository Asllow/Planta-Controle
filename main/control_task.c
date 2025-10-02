/**
 * @file control_task.c
 * @brief Tarefa de controle com múltiplos modos de operação.
 *
 * Esta tarefa opera em um de três modos, definidos pelo setpoint recebido:
 * - Operação Normal (setpoint >= 0): Controle em malha aberta com dados filtrados.
 * - Modo Calibração (setpoint = -1): Rotina para encontrar a tensão máxima do sensor.
 * - Modo Multímetro (setpoint = -2): Modo de diagnóstico para visualizar o filtro em ação.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "shared_resources.h"
#include <inttypes.h>
#include <math.h>

static const char *TAG = "CONTROL_TASK";

// --- Configurações de Hardware e Controle ---
#define MOTOR_PWM_PIN           GPIO_NUM_21
#define MOTOR_PWM_FREQUENCY_HZ  25000
#define SENSOR_ADC_UNIT         ADC_UNIT_2
#define SENSOR_ADC_CHANNEL      ADC2_CHANNEL_2
#define SENSOR_ADC_ATTEN        ADC_ATTEN_DB_11
#define CONTROL_LOOP_FREQUENCY_HZ 20

// --- Parâmetros do Filtro Adaptativo ---
#define ADC_FILTER_ALPHA_LOW  0.2f  // Alfa para suavização (sinal estável)
#define ADC_FILTER_ALPHA_HIGH 0.8f  // Alfa para resposta rápida (sinal mudando)
#define ADC_FILTER_CHANGE_THRESHOLD_MV 250.0f // Limiar de 250mV para considerar "mudança rápida"

// --- Comandos Especiais ---
#define CALIBRATION_COMMAND -1.0f
#define MULTIMETER_COMMAND  -2.0f

// --- Variáveis Estáticas do Módulo ---
static esp_adc_cal_characteristics_t *adc_chars;
static float filtered_voltage_mv = 0.0f; // Armazena o último valor filtrado

// Protótipos das funções estáticas
static void motor_pwm_init(void);
static void motor_set_duty_cycle(float duty_cycle);
static void adc_calibration_init(void);
static void run_auto_calibration(void);
static void run_multimeter_mode(void);
static void run_normal_operation(float setpoint_percent, TickType_t *pLastWakeTime);

/**
 * @brief Tarefa principal que atua como um "despachante" para os modos de operação.
 */
void control_loop_task(void *pvParameter) {
    adc_calibration_init();
    motor_pwm_init();
    ESP_LOGI(TAG, "Tarefa de Controle iniciada no Core %d", xPortGetCoreID());
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float setpoint_percent = 0.0f;

    while(1) {
        // Pega o setpoint mais recente a cada ciclo
        if (xSemaphoreTake(g_setpoint_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            setpoint_percent = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        // Decide qual modo de operação executar
        if (setpoint_percent == CALIBRATION_COMMAND) {
            run_auto_calibration();
            // Reseta o setpoint para um estado seguro após a calibração
            if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
                g_current_setpoint = 0.0f;
                xSemaphoreGive(g_setpoint_mutex);
            }
        } else if (setpoint_percent == MULTIMETER_COMMAND) {
            run_multimeter_mode(); // Esta função é terminal e nunca retorna.
        } else {
            run_normal_operation(setpoint_percent, &xLastWakeTime);
        }
    }
}

/**
 * @brief [MODO 1] Executa a rotina de calibração automática do sensor.
 */
static void run_auto_calibration(void) {
    ESP_LOGW(TAG, "--- INICIANDO MODO DE CALIBração ---");
    motor_set_duty_cycle(100.0f);
    ESP_LOGW(TAG, "Motor a 100%. Aguardando estabilização (3s)...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGW(TAG, "Medindo tensão PICO por 5s...");
    float max_voltage_found = 0;
    int64_t cal_start_time = esp_timer_get_time();
    while((esp_timer_get_time() - cal_start_time) < 5000000) { // 5s em us
        int raw_value;
        if (adc2_get_raw(SENSOR_ADC_CHANNEL, ADC_WIDTH_BIT_12, &raw_value) == ESP_OK) {
            // Usa a tensão NÃO FILTRADA para encontrar o pico real do hardware
            uint32_t current_mv = esp_adc_cal_raw_to_voltage(raw_value, adc_chars);
            if (current_mv > max_voltage_found) {
                max_voltage_found = current_mv;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    motor_set_duty_cycle(0.0f);
    if (max_voltage_found > 1000) {
        g_sensor_max_voltage_mv = max_voltage_found;
        ESP_LOGW(TAG, "--- CALIBRAÇÃO CONCLUÍDA! Novo máximo: %.0f mV ---", g_sensor_max_voltage_mv);
    } else {
        ESP_LOGE(TAG, "--- FALHA NA CALIBRAÇÃO! Tensão medida muito baixa. ---");
    }
}

/**
 * @brief [MODO 2] Entra em modo de diagnóstico "Multímetro".
 */
static void run_multimeter_mode(void) {
    ESP_LOGE(TAG, "--- MODO MULTÍMETRO ATIVADO (PWM @ 100%) ---");
    ESP_LOGE(TAG, "--- ESTE MODO É TERMINAL. RESETE O ESP PARA SAIR. ---");
    motor_set_duty_cycle(100.0f);
    vTaskDelay(pdMS_TO_TICKS(3000)); // Espera estabilizar

    while(1) {
        int raw_value;
        uint32_t current_voltage_mv = 0;
        if (adc2_get_raw(SENSOR_ADC_CHANNEL, ADC_WIDTH_BIT_12, &raw_value) == ESP_OK) {
            current_voltage_mv = esp_adc_cal_raw_to_voltage(raw_value, adc_chars);
        }

        float delta = fabsf(current_voltage_mv - filtered_voltage_mv);
        float alpha = (delta > ADC_FILTER_CHANGE_THRESHOLD_MV) ? ADC_FILTER_ALPHA_HIGH : ADC_FILTER_ALPHA_LOW;
        filtered_voltage_mv = (alpha * current_voltage_mv) + ((1.0f - alpha) * filtered_voltage_mv);

        ESP_LOGI(TAG, "Raw: %4d | Tensão: %4" PRIu32 "mV | Filtrada: %4.0fmV | Alpha: %.2f", 
            raw_value, current_voltage_mv, filtered_voltage_mv, alpha);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief [MODO 3] Executa um ciclo de operação normal.
 */
static void run_normal_operation(float setpoint_percent, TickType_t *pLastWakeTime) {
    control_data_t current_data;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQUENCY_HZ);

    // Garante que o setpoint esteja dentro dos limites válidos
    if (setpoint_percent < 0) setpoint_percent = 0;
    if (setpoint_percent > 100) setpoint_percent = 100;

    current_data.sinal_controle = setpoint_percent;
    motor_set_duty_cycle(current_data.sinal_controle);

    current_data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
    
    int raw_value;
    uint32_t current_voltage_mv = 0;
    if (adc2_get_raw(SENSOR_ADC_CHANNEL, ADC_WIDTH_BIT_12, &raw_value) == ESP_OK) {
        current_data.valor_adc_raw = raw_value;
        current_voltage_mv = esp_adc_cal_raw_to_voltage(raw_value, adc_chars);
    } else {
        current_data.valor_adc_raw = -1;
    }

    float delta = fabsf(current_voltage_mv - filtered_voltage_mv);
    float alpha = (delta > ADC_FILTER_CHANGE_THRESHOLD_MV) ? ADC_FILTER_ALPHA_HIGH : ADC_FILTER_ALPHA_LOW;
    filtered_voltage_mv = (alpha * current_voltage_mv) + ((1.0f - alpha) * filtered_voltage_mv);
    current_data.tensao_mv = (uint32_t)filtered_voltage_mv;

    xQueueSend(data_queue, &current_data, 0);
    vTaskDelayUntil(pLastWakeTime, xFrequency);
}


// --- Funções de Inicialização de Hardware (sem alterações) ---
static void motor_pwm_init(void) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_PIN);
    mcpwm_config_t pwm_config = {
        .frequency = MOTOR_PWM_FREQUENCY_HZ,
        .cmpr_a = 0.0f,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

static void adc_calibration_init(void) {
    adc2_config_channel_atten(SENSOR_ADC_CHANNEL, SENSOR_ADC_ATTEN);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(SENSOR_ADC_UNIT, SENSOR_ADC_ATTEN, ADC_WIDTH_BIT_12, 0, adc_chars);
}

static void motor_set_duty_cycle(float duty_cycle) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
}