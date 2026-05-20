#include <stdio.h>
#include <math.h> // Para fmaxf, fminf e fabs
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

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (21)
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      (5000)

#define ADC_UNIT            ADC_UNIT_2
#define ADC_CHANNEL         ADC_CHANNEL_2

// -------------------------------------------------------------
// CONSTANTES DO MODELO E CONTROLE (A, B, C, K, L)
// Calculadas para Ts = 1ms
// -------------------------------------------------------------
const float A_11 = 0.0982f,   A_12 = 1.0f;
const float A_21 = -0.7868f,  A_22 = 0.0f;

const float B_1 = -0.7875f;
const float B_2 = 36.9610f;

const float C_1 = 1.0f, C_2 = 0.0f; // C = [1 0]

// Ganhos do Controlador (K e ka)
const float K_1 = -0.2285f;
const float K_2 =  0.0385f;
const float k_a =  0.0006f; 

// Ganhos do Observador de Luenberger (L)
const float L_1 = -0.9018f;
const float L_2 = -1.0368f;

// Ponto de operação DC (Detrend)
const float Y_EQ = 1598.75f; // Ponto de equilibrio de malha aberta (mV)

// -------------------------------------------------------------

extern QueueHandle_t data_queue;
extern SemaphoreHandle_t g_setpoint_mutex;
extern volatile float g_current_setpoint;

uint32_t LEDC_DUTY = 0;

adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;
bool do_calibration = false;

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

// -------------------------------------------------------------
// TAREFA PRINCIPAL DO LOOP DE CONTROLE (1 ms)
// -------------------------------------------------------------
void control_loop_task(void *pvParameters){
    ledc_init();
    adc_init();

    // Estados do Controlador/Observador (k)
    float x_hat_1 = 0.0f;
    float x_hat_2 = 0.0f;
    float x_int   = 0.0f; // Memória do integrador

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1){
        // 1. OBTER REFERÊNCIA (IHM - Valor Livre Digitado)
        float valor_digitado = 0.0f;
        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            valor_digitado = g_current_setpoint; 
            xSemaphoreGive(g_setpoint_mutex);
        }

        // --- TRATAMENTO DO VALOR DA IHM ---
        // Garante que o valor digitado fique dentro de 0% e 100%
        if (valor_digitado > 100.0f) {
            valor_digitado = 100.0f;
        } else if (valor_digitado < 0.0f) {
            valor_digitado = 0.0f;
        }

        // Traduz de Porcentagem para Milivolts (mV). 
        const float MAX_TENSAO_MV = 3100.0f; 
        float current_setpoint_mV = (valor_digitado / 100.0f) * MAX_TENSAO_MV;

        // Controle só ativa se o usuário pedir mais de 1%
        bool controle_ativo = (valor_digitado > 1.0f);

        // 2. LER SENSOR E RETIRAR PONTO DE OPERAÇÃO
        int adc_raw = 0;
        int voltage_mV = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
        if (do_calibration) {
            adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mV);
        }

        float y_medido = (float)voltage_mV;
        float delta_y = y_medido - Y_EQ;      
        float delta_ref = current_setpoint_mV - Y_EQ; 

        float erro = 0.0f;
        float delta_u_total = 0.0f; 

        // 3. LEI DE CONTROLE (Realimentação de Estados Estimados + Integrador)
        if (controle_ativo) {
            erro = delta_ref - delta_y;
            
            // u_estado = -K * x_hat
            float u_estado = -(K_1 * x_hat_1 + K_2 * x_hat_2);
            
            // u_int = ka * x_int
            float u_int = k_a * x_int;
            
            delta_u_total = u_estado + u_int;
        } else {
            delta_u_total = 0.0f;
            x_int = 0.0f; // Reseta o integrador se desligado
        }

        // Saturar o delta_u em limites físicos de variação (Ex: +/- 50% de banda de PWM)
        float u_sat = fmaxf(-50.0f, fminf(50.0f, delta_u_total));

        // 4. ATUALIZAR O INTEGRAL COM ANTI-WINDUP
        if (controle_ativo) {
            // Só acumula o erro se não estiver saturado
            if (fabs(delta_u_total) <= 50.0f) {
                x_int += erro;
            }
        }

        // 5. ATUALIZAR O OBSERVADOR (Estimativa para k+1)
        float y_hat = x_hat_1; // Porque C = [1 0]
        float erro_obs = delta_y - y_hat;

        float x_hat_1_novo = (A_11 * x_hat_1) + (A_12 * x_hat_2) + (B_1 * u_sat) + (L_1 * erro_obs);
        float x_hat_2_novo = (A_21 * x_hat_1) + (A_22 * x_hat_2) + (B_2 * u_sat) + (L_2 * erro_obs);

        x_hat_1 = x_hat_1_novo;
        x_hat_2 = x_hat_2_novo;

        // 6. TRADUZIR O ESFORÇO DE CONTROLE (u_sat) PARA O MUNDO REAL DA PLANTA (PWM)
        float effective_setpoint = 0.0f;
        
        if (controle_ativo) {
            // O u_sat é variação. Voltamos a adicionar os 50% do ponto de equilíbrio DC.
            float u_real_pwm = u_sat + 50.0f; 
            
            // Garante que a saída final não passe de 100% nem caia de 0%
            effective_setpoint = fmaxf(0.0f, fminf(100.0f, u_real_pwm));

            // Aplica a zona morta exigida pelo hardware (se pediu pra rodar, mínimo é 38%)
            if (effective_setpoint < 38.0f) {
                effective_setpoint = 38.0f; 
            }
        } else {
            effective_setpoint = 0.0f;
        }

        // 7. APLICAR PWM
        uint32_t target_duty = (uint32_t)((effective_setpoint / 100.0f) * 8191.0f);
        LEDC_DUTY = target_duty;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        // 8. ENVIAR DADOS (Para Log/Gráfico IHM)
        control_data_t data;
        data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        data.valor_adc_raw = adc_raw;
        data.tensao_mv = voltage_mV;
        data.sinal_controle = effective_setpoint;

#ifdef ENABLE_OBSERVER_DEBUG
        data.tensao_estimada_mv = y_hat + Y_EQ; 
        data.erro_obs_mv = erro_obs; 
#endif

        xQueueSend(data_queue, &data, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}