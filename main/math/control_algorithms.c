/**
 * @file control_algorithms.c
 * @brief Implementação dos algoritmos matemáticos de controle.
 */
#include "control_algorithms.h"

float MATH_CTRL_ComputeOpenLoop(float desired_setpoint) {
    if (desired_setpoint > 100.0f) desired_setpoint = 100.0f;
    if (desired_setpoint < 0.0f) desired_setpoint = 0.0f;

    float effective_effort = 0.0f;
    if (desired_setpoint > 0.0f) {
        effective_effort = 38.0f + (desired_setpoint * 0.62f);
    }
    return effective_effort;
}

float MATH_CTRL_ComputeFeedbackLinearization(float raw_voltage) {
    /* TODO: Inserir a matriz/polinómio de difeomorfismo T(x) calculada. */
    return raw_voltage; 
}

float MATH_CTRL_ComputeLQG(float setpoint, float current_voltage, control_data_t *telemetry) {
    /* TODO: 
     * 1. Propagar Modelo de Estado do Observador.
     * 2. Calcular Erro de Estimação (L * (y - C*x_hat)).
     * 3. Calcular Ganho LQR (u = -K*x_hat + N*r).
     * 4. Preencher a estrutura telemetry com x_hat_1, x_hat_2.
     */
    return 0.0f; 
}