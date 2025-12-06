/**
 * @file control_task.c
 * @brief Tarefa de controle com frequência de PWM fixa e modos de teste.
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
#define MOTOR_PWM_FREQUENCY_HZ  1000 // Frequência agora é fixa
#define SENSOR_ADC_UNIT         ADC_UNIT_2
#define SENSOR_ADC_CHANNEL      ADC_CHANNEL_2
#define SENSOR_ADC_ATTEN        ADC_ATTEN_DB_12
#define CONTROL_LOOP_FREQUENCY_HZ 200

// --- Comandos Especiais ---
#define CALIBRATION_COMMAND -1.0f
#define MULTIMETER_COMMAND  -2.0f
#define SWEEP_COMMAND       -3.0f

// --- Handles e Variáveis Globais do Módulo ---
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc2_cali_handle = NULL;
static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper = NULL;
static mcpwm_cmpr_handle_t comparator = NULL;
static mcpwm_gen_handle_t generator = NULL;
static uint32_t timer_resolution_hz;
static uint32_t mcpwm_period_ticks = 0;

// --- Variáveis de Estado para os Modos ---
static enum { SWEEP_DIR_UP, SWEEP_DIR_DOWN } g_sweep_direction = SWEEP_DIR_UP;
static float g_sweep_duty_cycle = 0.0f;
static enum { CALIB_STATE_IDLE, CALIB_STATE_STABILIZING, CALIB_STATE_MEASURING } g_calib_state = CALIB_STATE_IDLE;
static int64_t g_calib_timer_start = 0;
static int g_calib_max_voltage = 0;

// --- Protótipos das funções estáticas ---
static void motor_pwm_init(void); // Alterado: não recebe mais frequência
static void motor_pwm_deinit(void);
static void motor_set_duty_cycle(float duty_cycle);
static void adc_calibration_init(void);
static bool run_auto_calibration(void);
static void run_multimeter_mode(void);
static void run_sweep_mode(void);
static void run_normal_operation(float setpoint_percent);


void control_loop_task(void *pvParameter) {
    adc_calibration_init();
    motor_pwm_init();
    ESP_LOGI(TAG, "Tarefa de Controle iniciada.");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float setpoint_percent = 0.0f;
    float last_setpoint = 0.0f;

    while(1) {
        
        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            setpoint_percent = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        if (setpoint_percent != last_setpoint) {
            ESP_LOGW(TAG, "Modo alterado para %.1f", setpoint_percent);
            g_calib_state = CALIB_STATE_IDLE;
            g_sweep_direction = SWEEP_DIR_UP;
            g_sweep_duty_cycle = 0.0f;
            last_setpoint = setpoint_percent;
        }

        if (setpoint_percent == CALIBRATION_COMMAND) {
            if (run_auto_calibration()) {
                if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
                    g_current_setpoint = 0.0f;
                    xSemaphoreGive(g_setpoint_mutex);
                }
            }
        } else if (setpoint_percent == MULTIMETER_COMMAND) {
            run_multimeter_mode();
        } else if (setpoint_percent == SWEEP_COMMAND) {
            run_sweep_mode();
        } else {
            run_normal_operation(setpoint_percent);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQUENCY_HZ));
    }
}

static bool run_auto_calibration(void) {
    switch (g_calib_state) {
        case CALIB_STATE_IDLE:
            ESP_LOGW(TAG, "Calibração: Iniciando... Motor a 100%.");
            motor_set_duty_cycle(100.0f);
            g_calib_timer_start = esp_timer_get_time();
            g_calib_max_voltage = 0;
            g_calib_state = CALIB_STATE_STABILIZING;
            return false;

        case CALIB_STATE_STABILIZING:
            if ((esp_timer_get_time() - g_calib_timer_start) > 3000000) { // 3s
                ESP_LOGW(TAG, "Calibração: Medindo por 5s...");
                g_calib_timer_start = esp_timer_get_time();
                g_calib_state = CALIB_STATE_MEASURING;
            }
            return false;

        case CALIB_STATE_MEASURING:
            if ((esp_timer_get_time() - g_calib_timer_start) < 5000000) { // 5s
                int raw_value, current_mv;
                if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK &&
                    adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_mv) == ESP_OK) {
                    if (current_mv > g_calib_max_voltage) g_calib_max_voltage = current_mv;
                }
            } else {
                motor_set_duty_cycle(0.0f);
                if (g_calib_max_voltage > 1000) {
                    g_sensor_max_voltage_mv = (float)g_calib_max_voltage;
                    ESP_LOGW(TAG, "--- CALIBRAÇÃO CONCLUÍDA! Novo máximo: %.0f mV ---", g_sensor_max_voltage_mv);
                } else {
                    ESP_LOGE(TAG, "--- FALHA NA CALIBRAÇÃO! ---");
                }
                g_calib_state = CALIB_STATE_IDLE;
                return true; // Terminou!
            }
            return false;
    }
    return true; // Estado inválido, termina.
}

static void run_multimeter_mode(void) {
    motor_set_duty_cycle(100.0f);
    int raw_value, current_voltage_mv = 0;
    if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
        adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_voltage_mv);
    }
    ESP_LOGI(TAG, "Multímetro | Tensão: %4d mV", current_voltage_mv);
}

static void run_sweep_mode(void) {
    if (g_sweep_direction == SWEEP_DIR_UP) {
        g_sweep_duty_cycle += 0.5f;
        if (g_sweep_duty_cycle >= 100.0f) {
            g_sweep_duty_cycle = 100.0f;
            g_sweep_direction = SWEEP_DIR_DOWN;
        }
    } else { // SWEEP_DIR_DOWN
        g_sweep_duty_cycle -= 0.5f;
        if (g_sweep_duty_cycle <= 0.0f) {
            g_sweep_duty_cycle = 0.0f;
            g_sweep_direction = SWEEP_DIR_UP;
        }
    }

    motor_set_duty_cycle(g_sweep_duty_cycle);
    int raw_value, voltage_mv = 0;
    if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
        adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &voltage_mv);
    }
    ESP_LOGI(TAG, "Sweep | Duty: %5.1f%% | Tensão: %4d mV", g_sweep_duty_cycle, voltage_mv);
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

    current_data.tensao_mv = (uint32_t)current_voltage_mv;
    
    if (xQueueSend(data_queue, &current_data, 0) == pdTRUE) {
        g_debug_samples_count++; 
    }
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

static void motor_pwm_init(void) {
    motor_pwm_deinit();
    mcpwm_timer_config_t timer_config = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = (10 * 1000 * 1000) / MOTOR_PWM_FREQUENCY_HZ, // Usa a constante
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