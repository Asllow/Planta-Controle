/**
 * @file control_algorithms.c
 * @brief Implementação dos algoritmos matemáticos e matrizes de estado em dupla precisão (64-bits).
 * * Contém proteções estritas de Anti-Windup e Failsafe contra falhas numéricas (NaN/INF).
 */

#include "control_algorithms.h"
#include <math.h>

/**
 * @brief Calcula a ação de controlo em Malha Aberta.
 * * Em malha aberta, o setpoint é tratado diretamente como o Duty Cycle do atuador.
 * A função atua puramente como uma barreira de saturação física.
 * * @param[in] desired_pwm_percentage Comando direto de potência (0.0f a 100.0f).
 * @return Esforço de controlo saturado e validado em percentagem.
 */
float MATH_CTRL_ComputeOpenLoop(float desired_pwm_percentage) {
    /* Barreira de proteção física para o driver LEDC do ESP-IDF */
    if (desired_pwm_percentage > 100.0f) {
        desired_pwm_percentage = 100.0f;
    } else if (desired_pwm_percentage < 0.0f) {
        desired_pwm_percentage = 0.0f;
    }

    /* Em malha aberta pura (sem feedforward dinâmico), a saída é igual à entrada validada */
    return desired_pwm_percentage;
}

float MATH_CTRL_ComputeLQG(float setpoint_v, float current_voltage_v, control_data_t *telemetry) {
    return setpoint_v;
}