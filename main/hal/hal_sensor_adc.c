/**
 * @file hal_sensor_adc.c
 * @brief Implementação da HAL para leitura do Tacogerador via ADC_ONESHOT do ESP32.
 */
#include "hal_sensor_adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define HAL_ADC_UNIT            ADC_UNIT_2
#define HAL_ADC_CHANNEL         ADC_CHANNEL_2
#define HAL_ADC_ATTEN           ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static bool s_is_calibrated = false;

bool HAL_ADC_Init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = HAL_ADC_UNIT,
    };
    if (adc_oneshot_new_unit(&init_config, &s_adc_handle) != ESP_OK) return false;

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = HAL_ADC_ATTEN,
    };
    if (adc_oneshot_config_channel(s_adc_handle, HAL_ADC_CHANNEL, &config) != ESP_OK) return false;

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = HAL_ADC_UNIT,
        .chan = HAL_ADC_CHANNEL,
        .atten = HAL_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc_cali_handle) == ESP_OK) {
        s_is_calibrated = true;
    }
    return true;
}

bool HAL_ADC_ReadVoltage(uint32_t *voltage_mv, int *raw_value) {
    if (s_adc_handle == NULL) return false;
    
    int raw = 0;
    int mv = 0;
    
    if (adc_oneshot_read(s_adc_handle, HAL_ADC_CHANNEL, &raw) != ESP_OK) {
        return false;
    }
    
    if (s_is_calibrated) {
        adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &mv);
    }
    
    if (voltage_mv != NULL) *voltage_mv = (uint32_t)mv;
    if (raw_value != NULL) *raw_value = raw;
    
    return true;
}