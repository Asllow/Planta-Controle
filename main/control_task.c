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

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (21)
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      (5000)

#define ADC_UNIT            ADC_UNIT_2
#define ADC_CHANNEL         ADC_CHANNEL_2

//Mutex e semáforo para lidar com com recursos compartilhado de envio IHM
extern QueueHandle_t data_queue;
extern SemaphoreHandle_t g_setpoint_mutex;
extern volatile float g_current_setpoint;

uint32_t LEDC_DUTY = 0;

//Variáveis de calibração
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;
bool do_calibration = false;

//Função de início do ledc para PWM
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

//Função de início e calibração do ADC
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

//Função de Controle
void control_loop_task(void *pvParameters){
    ledc_init();
    adc_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1){
        //Tratar setpoint recebido pela IHM
        float current_setpoint = 0.0f;
        if (xSemaphoreTake(g_setpoint_mutex, 0) == pdTRUE) {
            current_setpoint = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        float effective_setpoint = 0.0f;
        if (current_setpoint > 0.0f) {
            effective_setpoint = 38.0f + (current_setpoint * 0.62f);
        }

        if (current_setpoint > 100.0f) {
            current_setpoint = 100.0f;
        } else if (current_setpoint < 38.0f) {
            current_setpoint = 0.0f;
        }

        //Converter para bits
        uint32_t target_duty = (uint32_t)((effective_setpoint / 100.0f) * 8191.0f);

        //Setar o duty
        LEDC_DUTY = target_duty;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        //Fazer a leitura
        int adc_raw = 0;
        int voltage = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
        if (do_calibration) {
            adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage);
        }

        //Preparar os dados e enviar
        control_data_t data;
        data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        data.valor_adc_raw = adc_raw;
        data.tensao_mv = voltage;
        data.sinal_controle = current_setpoint;

        xQueueSend(data_queue, &data, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}