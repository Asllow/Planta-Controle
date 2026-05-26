#include <stdio.h>
#include <math.h>
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

// ===================== CONFIGURAÇÕES PWM E ADC =====================
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (21)
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY      (5000)
#define DUTY_MAX            8191.0f

#define ADC_UNIT            ADC_UNIT_2
#define ADC_CHANNEL         ADC_CHANNEL_2

// ===================== PARÂMETROS DO LQR =====================
#define TS_S                0.01f       // 10 ms
#define V_MAX_MOTOR         4.56f
#define U_EQ                2.28f
#define Y_EQ                1.5950f
#define PWM_DEADZONE        0.38f       // Zona morta de 0% a 38%

// Ganhos do LQR (Pesos Q=[1, 10, 1, 50], R=100)
static const float K1 = 0.5173250f;
static const float K2 = 0.0839720f;
static const float K3 = 0.2635852f;
static const float Ki = 0.6965920f;

// Ganhos do Observador (Dinâmica ts ~= 100ms para filtrar ruído)
static const float Ld1 = 3.00959763f;
static const float Ld2 = -0.00618515f;
static const float Ld3 = 5.55681462f;

// Matrizes do modelo discretizado
static const float Ad11 = 0.9674186971f, Ad12 = 0.0000722515f, Ad13 = -0.0031540712f;
static const float Ad21 = 0.0000910280f, Ad22 = 0.3773350032f, Ad23 = 0.0019191017f;
static const float Ad31 = 0.0813853026f, Ad32 = -0.0393046794f, Ad33 = 0.9995799090f;

static const float Bd1 = 0.0563917634f, Bd2 = 0.0000018771f, Bd3 = 0.0023458866f;

// ===================== RECURSOS COMPARTILHADOS =====================
extern QueueHandle_t data_queue;
extern SemaphoreHandle_t g_setpoint_mutex;
extern volatile float g_current_setpoint;

// Variáveis de calibração
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;
bool do_calibration = false;

// ===================== INICIALIZAÇÃO DE HARDWARE =====================
static void ledc_init(void){
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void adc_init(void){
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
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
    }
}

// ===================== TASK DE CONTROLE =====================
void control_loop_task(void *pvParameters){
    ledc_init();
    adc_init();

    // Estados do observador e controlador
    float x1_hat = 0.0f, x2_hat = 0.0f, x3_hat = 0.0f;
    float x_integral = 0.0f;
    float u_control_desvio = 0.0f;

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1){
        // 1. Leitura do Setpoint via IHM (Tratado em Volts)
        float current_setpoint = 0.0f;
        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            current_setpoint = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        // 2. Leitura e Conversão do ADC
        int adc_raw = 0;
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
        if (do_calibration) {
            adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
        }

        float y_medido = voltage_mv / 1000.0f;
        float y_desvio = y_medido - Y_EQ;

        // 3. Equações do Observador de Estados
        float y_hat = 9.0f * x2_hat;
        float erro_obs = y_desvio - y_hat;
        
        float novo_x1 = Ad11*x1_hat + Ad12*x2_hat + Ad13*x3_hat + Bd1*u_control_desvio + Ld1*erro_obs;
        float novo_x2 = Ad21*x1_hat + Ad22*x2_hat + Ad23*x3_hat + Bd2*u_control_desvio + Ld2*erro_obs;
        float novo_x3 = Ad31*x1_hat + Ad32*x2_hat + Ad33*x3_hat + Bd3*u_control_desvio + Ld3*erro_obs;

        // 4. Lógica de Controle LQR
        float u_total = 0.0f;
        
        if (current_setpoint > 0.1f) { 
            float referencia_desvio = current_setpoint - Y_EQ;
            
            // Ação Integral com Anti-Windup
            float erro_ctrl = referencia_desvio - y_desvio;
            x_integral += erro_ctrl * TS_S;
            if (x_integral > 3.0f) x_integral = 3.0f;
            if (x_integral < -3.0f) x_integral = -3.0f;

            // Lei de Controle de Realimentação de Estados
            float ux = K1*novo_x1 + K2*novo_x2 + K3*novo_x3;
            u_control_desvio = -ux + Ki * x_integral;
            
            u_total = U_EQ + u_control_desvio;
            
            // Saturação Direta
            if (u_total > V_MAX_MOTOR) u_total = V_MAX_MOTOR;
            if (u_total < 0.0f) u_total = 0.0f;
        } else {
            // Failsafe / Desligado
            u_total = 0.0f;
            u_control_desvio = 0.0f;
            x_integral = 0.0f; 
            novo_x1 = 0.0f; novo_x2 = 0.0f; novo_x3 = 0.0f;
        }

        // Atualização dos estados para o próximo ciclo
        x1_hat = novo_x1;
        x2_hat = novo_x2;
        x3_hat = novo_x3;

        // 5. Aplicação do PWM e Zona Morta
        float pwm_percent = u_total / V_MAX_MOTOR;
        if (pwm_percent > 0.0f && pwm_percent < PWM_DEADZONE) {
            pwm_percent = 0.0f; 
        }
        
        uint32_t target_duty = (uint32_t)(pwm_percent * DUTY_MAX);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, target_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        // 6. Estruturação dos Dados para Telemetria
        control_data_t data;
        data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        data.valor_adc_raw = adc_raw;
        data.tensao_mv = voltage_mv;
        data.sinal_controle = pwm_percent * 100.0f; 

#ifdef ENABLE_OBSERVER_DEBUG
        data.tensao_estimada_mv = (y_hat + Y_EQ) * 1000.0f;
        data.erro_obs_mv = erro_obs * 1000.0f;
#endif

        // Envio para a Fila (Core 1 processará o envio via WebSocket)
        xQueueSend(data_queue, &data, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}