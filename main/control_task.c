/**
 * @file control_task.c
 * @brief Versão final e estável, com modos de teste bloqueantes que exigem reset para sair.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "shared_resources.h"
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "CONTROL_TASK";

// --- Configurações ---
#define MOTOR_PWM_PIN           GPIO_NUM_21
#define MOTOR_PWM_FREQUENCY_HZ  10000
#define SENSOR_ADC_UNIT         ADC_UNIT_2
#define SENSOR_ADC_CHANNEL      ADC_CHANNEL_2
#define SENSOR_ADC_ATTEN        ADC_ATTEN_DB_12
#define CONTROL_LOOP_FREQUENCY_HZ 20

// --- Parâmetros do Filtro Adaptativo ---
#define ADC_FILTER_ALPHA_LOW  0.2f
#define ADC_FILTER_ALPHA_HIGH 0.8f
#define ADC_FILTER_CHANGE_THRESHOLD_MV 300.0f

// --- Comandos Especiais ---
#define CALIBRATION_COMMAND -1.0f
#define MULTIMETER_COMMAND  -2.0f
#define SWEEP_COMMAND       -3.0f
#define FREQ_SWEEP_COMMAND  -4.0f
#define FILTER_TOGGLE_COMMAND -5.0f

// --- Handles e Variáveis Globais do Módulo ---
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc2_cali_handle = NULL;
static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper = NULL;
static mcpwm_cmpr_handle_t comparator = NULL;
static mcpwm_gen_handle_t generator = NULL;
static uint32_t timer_resolution_hz;
static uint32_t mcpwm_period_ticks = 0;

// --- Variáveis de Estado ---
static bool g_filter_enabled = true;
static float filtered_voltage_mv = 0.0f;

// --- Protótipos das funções estáticas ---
static void motor_pwm_init(uint32_t frequency);
static void motor_pwm_deinit(void);
static void motor_set_duty_cycle(float duty_cycle);
static void adc_calibration_init(void);
static void run_auto_calibration(void);
static void run_multimeter_mode(void);
static void run_sweep_mode(void);
static void run_frequency_sweep_mode(void);
static void run_normal_operation(float setpoint_percent);


void control_loop_task(void *pvParameter) {
    adc_calibration_init();
    motor_pwm_init(MOTOR_PWM_FREQUENCY_HZ); 
    ESP_LOGI(TAG, "Tarefa de Controle iniciada.");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float setpoint_percent = 0.0f;

    while(1) {
        if (g_new_frequency_hz > 0) {
            ESP_LOGW(TAG, "Reconfigurando PWM para %" PRIu32 " Hz...", g_new_frequency_hz);
            motor_pwm_init(g_new_frequency_hz);
            g_new_frequency_hz = 0;
        }

        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            setpoint_percent = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        if (setpoint_percent == CALIBRATION_COMMAND) {
            run_auto_calibration();
        } else if (setpoint_percent == MULTIMETER_COMMAND) {
            run_multimeter_mode();
        } else if (setpoint_percent == SWEEP_COMMAND) {
            run_sweep_mode();
        } else if (setpoint_percent == FREQ_SWEEP_COMMAND) {
            run_frequency_sweep_mode();
        } else if (setpoint_percent == FILTER_TOGGLE_COMMAND) {
            g_filter_enabled = !g_filter_enabled;
            ESP_LOGW(TAG, "Filtro digital %s. Reiniciando para o modo normal.", g_filter_enabled ? "ATIVADO" : "DESATIVADO");
            if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
                g_current_setpoint = 0.0f;
                xSemaphoreGive(g_setpoint_mutex);
            }
        }
        
        run_normal_operation(setpoint_percent);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQUENCY_HZ));
    }
}

static void run_auto_calibration(void) {
    ESP_LOGW(TAG, "--- MODO DE CALIBRAÇÃO ---");
    motor_set_duty_cycle(100.0f);
    ESP_LOGW(TAG, "Motor a 100%. Estabilizando por 3s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGW(TAG, "Medindo tensão PICO por 5s...");
    int max_voltage_found_mv = 0;
    for (int i = 0; i < 50; i++) {
        int raw_value, current_mv;
        if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
            if (adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_mv) == ESP_OK) {
                if (current_mv > max_voltage_found_mv) max_voltage_found_mv = current_mv;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    motor_set_duty_cycle(0.0f);
    if (max_voltage_found_mv > 1000) {
        g_sensor_max_voltage_mv = (float)max_voltage_found_mv;
        ESP_LOGW(TAG, "--- CALIBRAÇÃO CONCLUÍDA! Novo máximo: %.0f mV ---", g_sensor_max_voltage_mv);
    } else {
        ESP_LOGE(TAG, "--- FALHA NA CALIBRAÇÃO! ---");
    }

    ESP_LOGW(TAG, "Modo concluído. O sistema ficará inerte. Pressione RESET para usar outro modo.");
    while(1) { vTaskDelay(portMAX_DELAY); }
}

static void run_multimeter_mode(void) {
    ESP_LOGW(TAG, "--- MODO MULTÍMETRO ---. Pressione RESET para sair.");
    motor_set_duty_cycle(100.0f);
    uint32_t max_voltage_mv = 0;

    while (true) {
        int raw_value, current_voltage_mv = 0;
        if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
            adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_voltage_mv);
        }
        if (current_voltage_mv > max_voltage_mv) max_voltage_mv = current_voltage_mv;
        
        ESP_LOGI(TAG, "Duty: 100%% | Tensão: %4d mV | Máxima: %4" PRIu32 " mV", current_voltage_mv, max_voltage_mv);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void run_sweep_mode(void) {
    ESP_LOGW(TAG, "--- MODO DE VARREDURA DE DUTY CYCLE ---. Pressione RESET para sair.");
    
    while(true) { 
        ESP_LOGW(TAG, "Varredura: Subindo (0%% -> 100%%)...");
        for (float duty_cycle = 0.0f; duty_cycle <= 100.0f; duty_cycle += 1.0f) {
            motor_set_duty_cycle(duty_cycle);
            int raw_value, voltage_mv = 0;
            if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
                adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &voltage_mv);
                ESP_LOGI(TAG, "Duty: %3.0f%% | Tensão: %4d mV", duty_cycle, voltage_mv);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        ESP_LOGW(TAG, "Varredura: Descendo (100%% -> 0%%)...");
        for (float duty_cycle = 100.0f; duty_cycle >= 0.0f; duty_cycle -= 1.0f) {
            motor_set_duty_cycle(duty_cycle);
            int raw_value, voltage_mv = 0;
            if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
                adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &voltage_mv);
                ESP_LOGI(TAG, "Duty: %3.0f%% | Tensão: %4d mV", duty_cycle, voltage_mv);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        ESP_LOGW(TAG, "Varredura: Ciclo completo. Reiniciando em 2s...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void run_frequency_sweep_mode(void) {
    ESP_LOGW(TAG, "--- MODO DE VARREDURA DE FREQUÊNCIAS ---. Pressione RESET para sair.");
    motor_set_duty_cycle(0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    const uint32_t frequencies_to_test[] = {
        1000, 5000, 10000, 15000, 20000, 25000, 30000, 35000
    };
    int num_frequencies = sizeof(frequencies_to_test) / sizeof(frequencies_to_test[0]);

    printf("---INICIO-DOS-DADOS---\n");
    printf("Frequencia(Hz)");
    for(int i = 0; i <= 100; i++) { printf(",Duty_%d%%", i); }
    printf("\n");

    for (int i = 0; i < num_frequencies; i++) {
        uint32_t current_frequency = frequencies_to_test[i];
        ESP_LOGI(TAG, "Coletando dados para %" PRIu32 " Hz...", current_frequency);
        
        uint32_t voltage_readings[101];
        motor_pwm_init(current_frequency);
        vTaskDelay(pdMS_TO_TICKS(100));

        for (int duty_percent = 0; duty_percent <= 100; duty_percent++) {
            motor_set_duty_cycle(duty_percent);
            vTaskDelay(pdMS_TO_TICKS(50));
            
            int raw_adc, voltage_mv = 0;
            if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_adc) == ESP_OK) {
                adc_cali_raw_to_voltage(adc2_cali_handle, raw_adc, &voltage_mv);
            }
            voltage_readings[duty_percent] = voltage_mv;
        }

        printf("%" PRIu32, current_frequency);
        for (int j = 0; j <= 100; j++) { printf(",%" PRIu32, voltage_readings[j]); }
        printf("\n");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("---FIM-DOS-DADOS---\n");
    ESP_LOGW(TAG, "Teste de varredura de frequências completo.");
    ESP_LOGW(TAG, "O sistema ficará inerte. Pressione RESET para usar outro modo.");

    motor_pwm_deinit();
    motor_pwm_init(MOTOR_PWM_FREQUENCY_HZ);

    if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
        g_current_setpoint = 0.0f;
        xSemaphoreGive(g_setpoint_mutex);
    }
    
    while(1) { vTaskDelay(portMAX_DELAY); }
}

static void run_normal_operation(float setpoint_percent) {
    if (setpoint_percent < 0) setpoint_percent = 0;
    if (setpoint_percent > 100) setpoint_percent = 100;
    
    control_data_t current_data;
    current_data.sinal_controle = setpoint_percent;
    motor_set_duty_cycle(current_data.sinal_controle);
    current_data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
    
    int raw_value, current_voltage_mv = 0;
    if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
        current_data.valor_adc_raw = raw_value;
        adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_voltage_mv);
    } else {
        current_data.valor_adc_raw = -1;
    }

    if (g_filter_enabled) {
        float delta = fabsf(current_voltage_mv - filtered_voltage_mv);
        float alpha = (delta > ADC_FILTER_CHANGE_THRESHOLD_MV) ? ADC_FILTER_ALPHA_HIGH : ADC_FILTER_ALPHA_LOW;
        filtered_voltage_mv = (alpha * current_voltage_mv) + ((1.0f - alpha) * filtered_voltage_mv);
        current_data.tensao_mv = (uint32_t)filtered_voltage_mv;
    } else {
        current_data.tensao_mv = (uint32_t)current_voltage_mv;
    }
    
    xQueueSend(data_queue, &current_data, 0);
}

static void motor_pwm_deinit(void) {
    if (timer) {
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(timer);
        mcpwm_del_generator(generator);
        mcpwm_del_comparator(comparator);
        mcpwm_del_operator(oper);
        mcpwm_del_timer(timer);
        timer = NULL; oper = NULL; comparator = NULL; generator = NULL;
    }
}

static void motor_pwm_init(uint32_t frequency) {
    motor_pwm_deinit();
    mcpwm_timer_config_t timer_config = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = (10 * 1000 * 1000) / frequency,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    timer_resolution_hz = timer_config.resolution_hz;
    mcpwm_period_ticks = timer_config.period_ticks;

    mcpwm_operator_config_t oper_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t gen_handle = NULL;
    mcpwm_generator_config_t generator_config = { .gen_gpio_num = MOTOR_PWM_PIN };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &gen_handle));
    generator = gen_handle;
    
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void motor_set_duty_cycle(float duty_cycle) {
    if (comparator == NULL) return;
    uint32_t compare_value = (duty_cycle / 100.0f) * mcpwm_period_ticks;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, compare_value));
}

static void adc_calibration_init(void) {
    adc_oneshot_unit_init_cfg_t init_config2 = { .unit_id = SENSOR_ADC_UNIT, .ulp_mode = ADC_ULP_MODE_DISABLE };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = SENSOR_ADC_ATTEN };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, SENSOR_ADC_CHANNEL, &config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = SENSOR_ADC_UNIT,
        .chan = SENSOR_ADC_CHANNEL,
        .atten = SENSOR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_handle);
    if (ret != ESP_OK) {
        adc2_cali_handle = NULL;
        ESP_LOGW(TAG, "Calibração do ADC falhou (eFuse pode não estar gravado).");
    }
}