#ifndef SHARED_RESOURCES_H
#define SHARED_RESOURCES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>

#define ENABLE_OBSERVER_DEBUG

// O atributo 'packed' garante que não haverá preenchimento (padding) de memória vazio,
// o que é essencial para enviar a struct diretamente via rede como array de bytes.
typedef struct __attribute__((packed)) {
    int64_t timestamp_amostra_ms;
    int32_t valor_adc_raw;      
    uint32_t tensao_mv;
    float sinal_controle;
#ifdef ENABLE_OBSERVER_DEBUG
    float tensao_estimada_mv;
    float erro_obs_mv;
    float estado_1;
    float estado_2;
    float estado_3;
#endif
} control_data_t;

extern QueueHandle_t data_queue;
extern SemaphoreHandle_t g_setpoint_mutex;
extern volatile float g_current_setpoint;
extern volatile int64_t g_last_valid_communication_ms;

#endif // SHARED_RESOURCES_H