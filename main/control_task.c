#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "shared_resources.h"

// --- CONFIGURAÇÕES DO HARDWARE ---
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (21)
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      (5000)

#define ADC_UNIT            ADC_UNIT_2
#define ADC_CHANNEL         ADC_CHANNEL_2

// --- CONSTANTES DO CONTROLADOR AVANÇADO ---
static const double LIE_2_COEFS[2] = {-3431.576952, -48353.038951};
static const double LGLF1_VAL = 10740.865534;

// NOVA ADIÇÃO: Matriz T para conversão rigorosa da referência X -> Z
static const double MATRIZ_T[2][2] = {
    {0.166734, 0.000000},
    {-0.308273, 145.968363},
};

static const double MATRIZ_T_INV[2][2] = {
    {5.997565, 0.000000},
    {0.012666, 0.006851},
};

// Ganhos Ajustados para Eliminar o Erro Estacionário sem Bang-Bang
static const double K_Z_CTRL[2] = {3000.0, 60.0}; 
static const double K_I_CTRL = -5000.0; // Integrador mais forte para cravar a referência         

static const double Ad_C_OBS[2][2] = {
    {1.000000, 0.010000},
    {0.000000, 1.000000},
};
static const double Bd_C_OBS[2][1] = {
    {0.000050},
    {0.010000},
};

// Observador suavizado para rejeitar o ruído do ADC na derivada
static const double L_OBS[2][1] = {
    {0.800000},
    {0.400000}, // Drasticamente reduzido para evitar explosões na derivada e bang-bang
};

#define TS_SEGUNDOS 0.01   // 100 Hz (10 ms)

#define MAX_VOLTAGE_OUT 5.0
#define RL_OHMS 180.0

/*
 * ATENÇÃO: Se você NÃO tem um divisor resistivo físico para baixar 5V para 3.3V, 
 * o DIVIDER_RATIO DEVE SER 1.0. Se estiver a usar um divisor, altere de volta.
 */
#define DIVIDER_RATIO 1.0 

// Variáveis Globais Externas
extern QueueHandle_t data_queue;
extern SemaphoreHandle_t g_setpoint_mutex;
extern volatile float g_current_setpoint;

// Handles e calibração
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;
bool do_calibration = false;

// Inicialização do LEDC
static void ledc_init(void){
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LEDC_OUTPUT_IO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// Inicialização do ADC
static void adc_init(void){
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_calibration = true;
    } else {
        ESP_LOGW("ADC", "Calibração não disponível, usando conversão linear");
    }
}

// Tarefa de Controle Principal
void control_loop_task(void *pvParameters){
    ledc_init();
    adc_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    double z_est[2] = {0.0, 0.0};
    double integral_error = 0.0;
    
    while(1){
        // 1. Setpoint
        float current_setpoint = 0.0f;
        if (xSemaphoreTake(g_setpoint_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            current_setpoint = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }
        double r = (double)current_setpoint;

        // ====================================================================
        // 1.5 CÁLCULO DINÂMICO DA REFERÊNCIA (X_ref -> Z_ref usando MATRIZ_T)
        // ====================================================================
        // Calcula a Velocidade (w_des) e a Corrente (i1_des) equivalentes à tensão r
        double w_des = ((2.793557 + RL_OHMS) / 0.169322) * (r / RL_OHMS);
        double A11_num = (0.000518 / 0.000365) + (0.169322 * 0.169322) / (0.000365 * (2.793557 + RL_OHMS));
        double A12_num = 0.319541 / 0.000365;
        double i1_des = (A11_num / A12_num) * w_des;

        // Transformação X -> Z
        double z_ref[2];
        z_ref[0] = MATRIZ_T[0][0] * w_des + MATRIZ_T[0][1] * i1_des;
        z_ref[1] = MATRIZ_T[1][0] * w_des + MATRIZ_T[1][1] * i1_des;
        // z_ref[0] será numericamente igual a 'r'
        // z_ref[1] será numericamente igual a 0.0

        // 2. Leitura do ADC
        int adc_raw = 0;
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));

        if (do_calibration) {
            adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
        } else {
            voltage_mv = (int)((double)adc_raw * 3300.0 / 4095.0);
        }

        // Tensão real na planta
        double y = (voltage_mv / 1000.0) * DIVIDER_RATIO;

        // 3. Inovação do observador
        double y_est = z_est[0];
        double erro_obs = y - y_est;

        // ZONA MORTA (DEADBAND) - Proteção contra ruído do ADC
        if (erro_obs > -0.005 && erro_obs < 0.005) {
            erro_obs = 0.0;
        }

        // 4. Integração do erro de tracking (Usando a referência convertida rigorosamente)
        double erro_tracking_real = z_ref[0] - y; 

        if (r > 0.01) {
            integral_error += erro_tracking_real * TS_SEGUNDOS;
            
            // Anti-Windup Clamping Absoluto
            if (integral_error > 15.0) integral_error = 15.0;
            if (integral_error < -15.0) integral_error = -15.0;

        } else {
            integral_error = 0.0;
            z_est[0] = 0.0; 
            z_est[1] = 0.0;
        }

        // 5. Lei de controle LQI (Com a referência z_ref calculada por T)
        double erro_z1 = z_ref[0] - y; 
        double erro_z2 = z_ref[1] - z_est[1];
        double v = (K_Z_CTRL[0] * erro_z1 + K_Z_CTRL[1] * erro_z2) - (K_I_CTRL * integral_error);

        // 6. Transformação canônica → original (Z -> X)
        double x_est[2];
        x_est[0] = MATRIZ_T_INV[0][0] * z_est[0] + MATRIZ_T_INV[0][1] * z_est[1];
        x_est[1] = MATRIZ_T_INV[1][0] * z_est[0] + MATRIZ_T_INV[1][1] * z_est[1];

        // 7. Feedback Linearization
        double lie_2 = LIE_2_COEFS[0] * x_est[0] + LIE_2_COEFS[1] * x_est[1];
        double u_calc = (v - lie_2) / LGLF1_VAL;

        double u = u_calc;
        if (u < 0.0) u = 0.0;
        if (u > MAX_VOLTAGE_OUT) u = MAX_VOLTAGE_OUT;

        // Proteção do observador: recalcula v_real caso u tenha saturado
        double v_real = v;
        if (u != u_calc) {
            v_real = u * LGLF1_VAL + lie_2;
        }

        // 9. Predição do observador
        double z_next[2];
        z_next[0] = Ad_C_OBS[0][0] * z_est[0] + Ad_C_OBS[0][1] * z_est[1] + Bd_C_OBS[0][0] * v_real + L_OBS[0][0] * erro_obs;
        z_next[1] = Ad_C_OBS[1][0] * z_est[0] + Ad_C_OBS[1][1] * z_est[1] + Bd_C_OBS[1][0] * v_real + L_OBS[1][0] * erro_obs;
        z_est[0] = z_next[0];
        z_est[1] = z_next[1];

        // 10. Atuação PWM
        uint32_t target_duty = (uint32_t)((u / MAX_VOLTAGE_OUT) * 8191.0);
        if (r < 0.01) target_duty = 0; 

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, target_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        // 11. Telemetria
        control_data_t data;
        data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        data.valor_adc_raw = adc_raw;
        data.tensao_mv = voltage_mv;
        data.sinal_controle = (float)((u / MAX_VOLTAGE_OUT) * 100.0);

        #ifdef ENABLE_OBSERVER_DEBUG
        data.tensao_estimada_mv = (float)(y_est * 1000.0 / DIVIDER_RATIO); 
        data.erro_obs_mv = (float)(erro_obs * 1000.0);
        data.estado_1 = (float)x_est[0]; // Agora envia o estado físico real (Velocidade)
        data.estado_2 = (float)x_est[1]; // Agora envia o estado físico real (Corrente do motor)
        data.estado_3 = (float)(y / RL_OHMS); 
        #endif

        xQueueSend(data_queue, &data, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}