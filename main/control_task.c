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

// --- PARÂMETROS DO CONTROLADOR PI (Malha Fechada) ---
// Planta: K=0.811, tau=0.135s | Ts=5ms
// Método: Síntese Direta
#define PI_Q0      2.422f
#define PI_Q1     -2.334f
#define MAX_VOLTAGE_OUT 3.1f // CORREÇÃO: Tensão de saturação do ADC do S3 (12dB)

// --- Variáveis de Estado do Controlador ---
static float g_u_prev = 0.0f; // Ação de controle anterior (u[k-1])
static float g_e_prev = 0.0f; // Erro anterior (e[k-1])

// --- Comandos Especiais ---
#define CALIBRATION_COMMAND -1.0f
#define MULTIMETER_COMMAND  -2.0f
#define SWEEP_COMMAND       -3.0f

#define SAFETY_TIMEOUT_MS 5000

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

// --- Variáveis do Multímetro ---
static int g_multimeter_max = 0;           // Pico Máximo da Sessão (Desde que ligou o modo)
static int g_rolling_buffer[5] = {0};      // Buffer dos últimos 5 segundos (1 slot por segundo)
static int g_rolling_idx = 0;              // Índice atual do buffer
static int g_temp_sec_max = 0;             // Máximo do segundo atual (sendo construído)
static int64_t g_last_roll_time = 0;       // Cronômetro para girar o buffer

// --- Protótipos das funções estáticas ---
static void motor_pwm_init(void); // Alterado: não recebe mais frequência
static void motor_pwm_deinit(void);
static void motor_set_duty_cycle(float duty_cycle);
static void adc_calibration_init(void);
static bool run_auto_calibration(void);
static void run_multimeter_mode(void);
static void run_sweep_mode(void);
//static void run_normal_operation(float setpoint_percent);

/**
 * @brief Executa o controlador PI Digital em Malha Fechada.
 * @param target_voltage Setpoint desejado em VOLTS (ex: 1.0 para 1V).
 */
static void run_closed_loop_control(float target_voltage) {
    // 1. Limita setpoint por segurança (0V a 3.1V)
    if (target_voltage < 0.0f) target_voltage = 0.0f;
    if (target_voltage > MAX_VOLTAGE_OUT) target_voltage = MAX_VOLTAGE_OUT;
    
    control_data_t current_data;
    current_data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
    
    int raw_value, current_voltage_mv = 0;
    float current_voltage_v = 0.0f;

    // 2. Leitura do Sensor (Feedback)
    if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
        adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_voltage_mv);
        current_data.valor_adc_raw = raw_value;
        current_data.tensao_mv = (uint32_t)current_voltage_mv;
        
        // CONVERSÃO CRÍTICA: mV -> Volts para bater com o modelo matemático
        current_voltage_v = current_voltage_mv / 1000.0f; 
    } else {
        current_data.valor_adc_raw = -1;
        current_data.tensao_mv = 0;
        current_voltage_v = 0.0f;
    }

    // 3. Cálculo do Erro
    float error = target_voltage - current_voltage_v;

    // 4. Equação de Diferenças do PI (u[k] = u[k-1] + q0*e[k] + q1*e[k-1])
    float u = g_u_prev + (PI_Q0 * error) + (PI_Q1 * g_e_prev);

    // 5. Saturação (Anti-windup) e Proteção
    if (u > MAX_VOLTAGE_OUT) u = MAX_VOLTAGE_OUT;
    if (u < 0.0f)            u = 0.0f;

    // 6. Atuação na Planta (Conversão Volts -> Duty Cycle %)
    // Se 3.1V é o máximo que o sistema "vê", assumimos isso como 100% de esforço
    float duty_cycle_percent = (u / MAX_VOLTAGE_OUT) * 100.0f;
    
    // Clamp final de segurança do PWM
    if (duty_cycle_percent > 100.0f) duty_cycle_percent = 100.0f;
    
    motor_set_duty_cycle(duty_cycle_percent);

    // 7. Atualização dos Estados (Memória)
    g_u_prev = u;
    g_e_prev = error;

    // 8. Telemetria
    // Enviamos o duty cycle (%) para monitorar o esforço do motor
    current_data.sinal_controle = duty_cycle_percent; 
    
    if (xQueueSend(data_queue, &current_data, 0) == pdTRUE) {
        g_debug_samples_count++; 
    }
}

void control_loop_task(void *pvParameter) {
    adc_calibration_init();
    motor_pwm_init();
    ESP_LOGI(TAG, "Tarefa de Controle (Malha Fechada PI) iniciada.");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float setpoint_value = 0.0f; // Agora representa VOLTS (se > 0)
    float last_setpoint = 0.0f;

    while(1) {
        // Leitura do Setpoint (Protegida por Mutex)
        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            setpoint_value = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        // --- DETECÇÃO DE MUDANÇA DE MODO/SETPOINT ---
        if (setpoint_value != last_setpoint) {
            ESP_LOGW(TAG, "Setpoint alterado para %.2f V", setpoint_value);
            
            // 1. Zera memória do controlador (PI) para evitar solavancos
            g_u_prev = 0.0f;
            g_e_prev = 0.0f;
            
            // 2. Reseta estados dos modos de teste (Multímetro/Sweep)
            g_calib_state = CALIB_STATE_IDLE;
            g_sweep_direction = SWEEP_DIR_UP;
            g_sweep_duty_cycle = 0.0f;
            
            // 3. Reseta variáveis do Multímetro
            g_multimeter_max = 0;
            memset(g_rolling_buffer, 0, sizeof(g_rolling_buffer));
            g_rolling_idx = 0;
            g_temp_sec_max = 0;
            g_last_roll_time = esp_timer_get_time();

            last_setpoint = setpoint_value;
        }

        // --- LÓGICA DE SEGURANÇA (Watchdog) ---
        if (setpoint_value >= 0.0f) {
            int64_t now = esp_timer_get_time() / 1000;
            if ((now - g_last_valid_communication_ms) > SAFETY_TIMEOUT_MS) {
                if (setpoint_value != 0.0f) {
                    ESP_LOGE(TAG, "Failsafe! Sem rede há 5s. Zerando.");
                    setpoint_value = 0.0f;
                    if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
                        g_current_setpoint = 0.0f;
                        xSemaphoreGive(g_setpoint_mutex);
                    }
                }
            }
        }

        // --- SELETOR DE MODOS ---
        if (setpoint_value == CALIBRATION_COMMAND) {
            if (run_auto_calibration()) {
                if (xSemaphoreTake(g_setpoint_mutex, portMAX_DELAY) == pdTRUE) {
                    g_current_setpoint = 0.0f;
                    xSemaphoreGive(g_setpoint_mutex);
                }
            }
        } else if (setpoint_value == MULTIMETER_COMMAND) {
            run_multimeter_mode();
        } else if (setpoint_value == SWEEP_COMMAND) {
            run_sweep_mode();
        } else {
            // MODO MALHA FECHADA
            // O valor positivo agora é tratado como Tensão Alvo (Volts)
            run_closed_loop_control(setpoint_value);
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
    
    // Leitura (Roda a 200Hz)
    if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
        adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &current_voltage_mv);
    }

    // 1. Atualiza Pico da Sessão (Máximo absoluto)
    if (current_voltage_mv > g_multimeter_max) {
        g_multimeter_max = current_voltage_mv;
    }

    // 2. Lógica Inteligente dos Últimos 5 Segundos
    // Primeiro, descobre qual o maior valor DESTE segundo atual
    if (current_voltage_mv > g_temp_sec_max) {
        g_temp_sec_max = current_voltage_mv;
    }

    int64_t now_us = esp_timer_get_time(); // Tempo em microsegundos
    
    // Se passou 1 segundo, salva o "vencedor" no histórico e abre nova votação
    if ((now_us - g_last_roll_time) > 1000000) {
        g_rolling_buffer[g_rolling_idx] = g_temp_sec_max; // Salva
        g_rolling_idx = (g_rolling_idx + 1) % 5;          // Avança circularmente (0..4)
        
        g_temp_sec_max = 0; // Reseta para o próximo segundo
        g_last_roll_time = now_us;
    }

    // Calcula quem é o maior dos últimos 5 segundos (varre o histórico + atual)
    int max_5s = g_temp_sec_max;
    for (int i = 0; i < 5; i++) {
        if (g_rolling_buffer[i] > max_5s) {
            max_5s = g_rolling_buffer[i];
        }
    }

    // 3. Print Formatado (a cada 500ms)
    static int64_t last_log_time_ms = 0;
    int64_t now_ms = now_us / 1000;
    
    if ((now_ms - last_log_time_ms) > 500) {
        ESP_LOGI(TAG, "Mult: Atual %4d mV | Max(5s) %4d mV | Max(Total) %4d mV", 
                    current_voltage_mv, max_5s, g_multimeter_max);
        last_log_time_ms = now_ms;
    }
}

static void run_sweep_mode(void) {
    // Controla a velocidade da varredura (atualiza a cada 50ms)
    static int64_t last_step_time = 0;
    int64_t now = esp_timer_get_time() / 1000;

    if ((now - last_step_time) > 50) {
        if (g_sweep_direction == SWEEP_DIR_UP) {
            g_sweep_duty_cycle += 0.5f;
            if (g_sweep_duty_cycle >= 100.0f) {
                g_sweep_duty_cycle = 100.0f;
                g_sweep_direction = SWEEP_DIR_DOWN;
            }
        } else { 
            g_sweep_duty_cycle -= 0.5f;
            if (g_sweep_duty_cycle <= 0.0f) {
                g_sweep_duty_cycle = 0.0f;
                g_sweep_direction = SWEEP_DIR_UP;
            }
        }
        last_step_time = now;
    }

    motor_set_duty_cycle(g_sweep_duty_cycle);
    
    // Leitura apenas para log visual
    int raw_value, voltage_mv = 0;
    if (adc_oneshot_read(adc2_handle, SENSOR_ADC_CHANNEL, &raw_value) == ESP_OK) {
        adc_cali_raw_to_voltage(adc2_cali_handle, raw_value, &voltage_mv);
    }
    
    // Log do Sweep limitado a 200ms para não travar o terminal
    static int64_t last_log_sweep = 0;
    if ((now - last_log_sweep) > 200) {
        ESP_LOGI(TAG, "Sweep | Duty: %5.1f%% | Tensão: %4d mV", g_sweep_duty_cycle, voltage_mv);
        last_log_sweep = now;
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