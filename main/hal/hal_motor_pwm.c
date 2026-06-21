/**
 * @file hal_motor_pwm.c
 * @brief Implementação da HAL para o PWM utilizando o periférico LEDC do ESP32.
 */
#include "hal_motor_pwm.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define HAL_LEDC_TIMER          LEDC_TIMER_0
#define HAL_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define HAL_LEDC_OUTPUT_IO      21
#define HAL_LEDC_CHANNEL        LEDC_CHANNEL_0
#define HAL_LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define HAL_LEDC_FREQUENCY      5000
#define HAL_LEDC_MAX_DUTY_VAL   8191.0f

bool HAL_PWM_Init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = HAL_LEDC_MODE,
        .timer_num        = HAL_LEDC_TIMER,
        .duty_resolution  = HAL_LEDC_DUTY_RES,
        .freq_hz          = HAL_LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&ledc_timer) != ESP_OK) return false;
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = HAL_LEDC_MODE,
        .channel        = HAL_LEDC_CHANNEL,
        .timer_sel      = HAL_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = HAL_LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    if (ledc_channel_config(&ledc_channel) != ESP_OK) return false;

    return true;
}

void HAL_PWM_SetDutyCycle(float duty_percent) {
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint32_t target_duty = (uint32_t)((duty_percent / 100.0f) * HAL_LEDC_MAX_DUTY_VAL);
    
    ledc_set_duty(HAL_LEDC_MODE, HAL_LEDC_CHANNEL, target_duty);
    ledc_update_duty(HAL_LEDC_MODE, HAL_LEDC_CHANNEL);
}