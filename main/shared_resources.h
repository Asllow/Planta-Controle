// main/shared_resources.h

#ifndef SHARED_RESOURCES_H
#define SHARED_RESOURCES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>

/**
 * @brief Estrutura para passar os dados de controle do Core 0 (controle) para o Core 1 (comunicação).
 * * Esta estrutura contém um pacote completo de dados coletados em um único ciclo de controle.
 */
typedef struct {
    int64_t timestamp_amostra_ms; // Timestamp em milissegundos de quando a amostra foi tirada (desde o boot).
    int valor_adc_raw;            // O valor bruto lido do ADC (ex: 0-4095).
    uint32_t tensao_mv;           // A tensão calculada em milivolts, após calibração.
    float sinal_controle;         // O sinal de controle calculado e aplicado ao motor (%).
} control_data_t;


// --- Declarações de Variáveis e Handles Globais ---
// Usamos 'extern' para dizer ao compilador que estas variáveis existem,
// mas são definidas em outro arquivo (no nosso caso, em main.c).

/**
 * @brief Fila para enviar pacotes de dados do Core 0 para o Core 1.
 * É o principal meio de comunicação entre as tarefas.
 */
extern QueueHandle_t data_queue;

/**
 * @brief Mutex para proteger o acesso à variável de setpoint global.
 * Garante que a leitura/escrita do setpoint seja atômica e segura entre os cores.
 */
extern SemaphoreHandle_t g_setpoint_mutex;

/**
 * @brief Armazena o valor de setpoint atual recebido do servidor.
 * É 'volatile' para indicar ao compilador que seu valor pode mudar a qualquer momento.
 */
extern volatile float g_current_setpoint;

/**
 * @brief Constante de Calibração.
 */ 
extern volatile float g_sensor_max_voltage_mv;

/**
 * @brief Armazena o valor de uma nova frequência do pwm do motor.
 */
extern volatile uint32_t g_new_frequency_hz;

#endif // SHARED_RESOURCES_H