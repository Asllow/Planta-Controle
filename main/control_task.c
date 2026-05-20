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

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (21)
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      (5000)

#define ADC_UNIT            ADC_UNIT_2
#define ADC_CHANNEL         ADC_CHANNEL_2

// =============================================================
// CONSTANTES DO MODELO E GANHOS (Ts = 10ms)
// =============================================================
const float A_11 =  1.6075f, A_12 = 1.0f;
const float A_21 = -0.6372f, A_22 = 0.0f;

const float B_1 =  0.9066f;  
const float B_2 = -0.6093f;  

const float C_1 = 1.0f;
const float C_2 = 0.0f;

const float L_1 =  0.7575f;
const float L_2 = -0.4572f;

const float K_1 =  0.6966f;
const float K_2 =  0.6138f;
const float K_i = -0.0333f;

const float PWM_EQ = 50.0f;
const float Y_EQ   = 1600.0f; 
// =============================================================

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

// =============================================================
// TAREFA PRINCIPAL DO LOOP DE CONTROLE (Ts = 10 ms)
// =============================================================
void control_loop_task(void *pvParameters){
    ledc_init();
    adc_init();

    float x_hat_1 = 0.0f;
    float x_hat_2 = 0.0f;
    float erro_integral = 0.0f;
    
    bool controle_ativo_anterior = false;

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1){
        // 1. OBTER REFERÊNCIA (IHM)
        float valor_digitado = 0.0f;
        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            valor_digitado = g_current_setpoint; 
            xSemaphoreGive(g_setpoint_mutex);
        }

        // --- TRATAMENTO DO VALOR DA IHM DIRETO EM mV ---
        float current_setpoint_mV_absoluto = valor_digitado; 
        
        // Limita o alvo para não queimar nada
        if(current_setpoint_mV_absoluto > 3100.0f) current_setpoint_mV_absoluto = 3100.0f;
        
        float current_setpoint_mV_delta = current_setpoint_mV_absoluto - Y_EQ;

        // O controle só liga se você digitar mais de 500mV.
        bool controle_ativo = (current_setpoint_mV_absoluto > 500.0f);

        // 2. LER SENSOR 
        int adc_raw = 0;
        int voltage_mV = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
        if (do_calibration) {
            adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mV);
        }
        
        float y_medido_delta = (float)voltage_mV - Y_EQ;

        // PULO DO GATO: Reset limpo ao iniciar o controle
        if (controle_ativo && !controle_ativo_anterior) {
            x_hat_1 = 0.0f;
            x_hat_2 = 0.0f;
            erro_integral = 0.0f;
        }
        controle_ativo_anterior = controle_ativo;

        // 3. CÁLCULO DA LEI DE CONTROLE PI 
        float pwm_saturado = 0.0f;
        float delta_u_aplicado = 0.0f; 

        if (controle_ativo) {
            float erro = current_setpoint_mV_delta - y_medido_delta;

            // A) Qual seria o PWM agora ANTES de integrar o erro atual?
            float delta_u_prop = - (K_1 * x_hat_1 + K_2 * x_hat_2);
            float pwm_tentativa = PWM_EQ + delta_u_prop - (K_i * erro_integral);

            // B) Lógica ANTI-WINDUP INDUSTRIAL (Perfeita, sem travas matemáticas artificiais)
            bool pode_integrar = true;
            if (pwm_tentativa >= 100.0f && erro > 0.0f) pode_integrar = false; 
            if (pwm_tentativa <= 38.0f && erro < 0.0f) pode_integrar = false;  

            if (pode_integrar) {
                erro_integral += erro;
            }
            // O LIMITADOR MATEMÁTICO QUE CAUSOU O PROBLEMA FOI REMOVIDO DAQUI!

            // C) Recalcula a Lei de Controle FINAL 
            float delta_u_final = delta_u_prop - (K_i * erro_integral);
            float pwm_total = PWM_EQ + delta_u_final;
            
            // D) Saturação Física e envio para a planta
            pwm_saturado = fmaxf(38.0f, fminf(100.0f, pwm_total));
            delta_u_aplicado = pwm_saturado - PWM_EQ;
            
        } else {
            // Motor desligado
            pwm_saturado = 0.0f;
            delta_u_aplicado = 0.0f - PWM_EQ; 
        }

        // 5. OBSERVADOR DE LUENBERGER 
        float y_hat_delta = (C_1 * x_hat_1) + (C_2 * x_hat_2); 
        float erro_obs = y_medido_delta - y_hat_delta;

        float x_hat_1_novo = (A_11 * x_hat_1) + (A_12 * x_hat_2) + (B_1 * delta_u_aplicado) + (L_1 * erro_obs);
        float x_hat_2_novo = (A_21 * x_hat_1) + (A_22 * x_hat_2) + (B_2 * delta_u_aplicado) + (L_2 * erro_obs);

        x_hat_1 = x_hat_1_novo;
        x_hat_2 = x_hat_2_novo;

        // 6. ENVIAR PARA O MOTOR (PWM FÍSICO)
        float effective_setpoint = pwm_saturado;

        uint32_t target_duty = (uint32_t)((effective_setpoint / 100.0f) * 8191.0f);
        LEDC_DUTY = target_duty;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        // 7. PREPARAR PACOTE DA IHM
        control_data_t data;
        data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        data.valor_adc_raw = adc_raw;
        data.tensao_mv = voltage_mV;
        data.sinal_controle = effective_setpoint;

#ifdef ENABLE_OBSERVER_DEBUG
        // Transforma o estado delta para o mundo absoluto para a interface web
        data.tensao_estimada_mv = y_hat_delta + Y_EQ; 
        data.erro_obs_mv = erro_obs; 
#endif

        xQueueSend(data_queue, &data, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}