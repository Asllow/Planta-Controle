// main/control_task.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "shared_resources.h"

static const char *TAG = "CONTROL_TASK";

// --- Configurações de Hardware ---
// Motor (Atuador)
#define MOTOR_PWM_GPIO_A 26 // Pino para o sinal de PWM do motor
#define MOTOR_PWM_GPIO_B 27 // Pino para direção (se usar Ponte-H)

// Sensor (Tacogerador)
#define SENSOR_ADC_CHANNEL ADC1_CHANNEL_6 // Verifique o pino do seu ESP32-S3 (ex: GPIO7)
#define SENSOR_ADC_ATTEN   ADC_ATTEN_DB_11 // Atenuação para a faixa de 0-3100mV

// --- Variáveis do Módulo ---
static esp_adc_cal_characteristics_t *adc_chars;

/**
 * @brief Inicializa o periférico MCPWM para gerar o sinal de controle do motor.
 */
static void motor_pwm_init(void) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_GPIO_A);
    mcpwm_config_t pwm_config = {
        .frequency = 5000, // 5 KHz de frequência
        .cmpr_a = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

/**
 * @brief Aplica um duty cycle ao motor.
 * @param duty_cycle Valor de 0.0 a 100.0.
 */
static void motor_set_duty_cycle(float duty_cycle) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

/**
 * @brief Configura o ADC e o driver de calibração para leituras precisas.
 */
static void adc_calibration_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_ADC_CHANNEL, SENSOR_ADC_ATTEN);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, SENSOR_ADC_ATTEN, ADC_WIDTH_BIT_12, 0, adc_chars);
}

/**
 * @brief Tarefa principal de controle, rodando em frequência fixa.
 * * Esta tarefa é responsável por:
 * 1. Ler o setpoint atual de forma segura.
 * 2. Coletar dados do sensor (ADC, Tensão).
 * 3. Calcular o sinal de controle (algoritmo P).
 * 4. Aplicar o sinal ao motor (PWM).
 * 5. Enviar os dados coletados para a tarefa de comunicação via fila.
 */
void control_loop_task(void *pvParameter) {
    // --- Inicializações específicas desta tarefa ---
    adc_calibration_init();
    motor_pwm_init();
    ESP_LOGI(TAG, "Tarefa de Controle iniciada no Core %d", xPortGetCoreID());

    control_data_t current_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Frequência de 20 Hz (50ms)

    float setpoint_percent = 0;

    while(1) {
        // --- Início do Ciclo de Controle ---
        
        // 1. Pega o valor mais atual do setpoint de forma segura
        if (xSemaphoreTake(g_setpoint_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            setpoint_percent = g_current_setpoint;
            xSemaphoreGive(g_setpoint_mutex);
        }

        // 2. Coleta de dados com timestamp
        current_data.timestamp_amostra_ms = esp_timer_get_time() / 1000;
        current_data.valor_adc_raw = adc1_get_raw(SENSOR_ADC_CHANNEL);
        current_data.tensao_mv = esp_adc_cal_raw_to_voltage(current_data.valor_adc_raw, adc_chars);

        // 3. Lógica do Controlador (Exemplo de Controlador Proporcional - P)
        float velocidade_atual_percent = ((float)current_data.tensao_mv / 3100.0f) * 100.0f;
        if (velocidade_atual_percent > 100.0f) velocidade_atual_percent = 100.0f;
        
        float Kp = 0.8f; // Ganho Proporcional (precisará ser ajustado/tunado)
        float error = setpoint_percent - velocidade_atual_percent;
        current_data.sinal_controle = Kp * error;
        
        // Saturação do sinal de controle para os limites do PWM (0-100%)
        if (current_data.sinal_controle > 100.0f) current_data.sinal_controle = 100.0f;
        if (current_data.sinal_controle < 0.0f) current_data.sinal_controle = 0.0f;

        // 4. Aplica o sinal de controle ao atuador (motor)
        motor_set_duty_cycle(current_data.sinal_controle);

        // 5. Envia o pacote de dados para a fila do Core 1
        if (xQueueSend(data_queue, &current_data, 0) != pdPASS) {
            ESP_LOGW(TAG, "Falha ao enviar para a fila. Fila cheia.");
        }

        // Aguarda pelo próximo ciclo, mantendo a frequência constante.
        // Essencial para a estabilidade do sistema de controle.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}