/**
 * @file control_algorithms.h
 * @brief Algoritmos matemáticos independentes de arquitetura.
 * * Contém a implementação dos controladores em Malha Aberta e os 
 * preparativos estruturais para Feedback Linearization e LQR/LQG.
 */
#ifndef CONTROL_ALGORITHMS_H
#define CONTROL_ALGORITHMS_H

#include <stdint.h>
#include "ipc_manager.h"

/**
 * @brief Calcula a ação de controlo baseada na compensação de Zona Morta (Malha Aberta).
 * @param[in] desired_setpoint Setpoint exigido de 0 a 100%.
 * @return Esforço de controlo em percentagem para aplicação na planta.
 */
float MATH_CTRL_ComputeOpenLoop(float desired_setpoint);

/**
 * @brief Placeholder para o Difeomorfismo (Feedback Linearization).
 * @param[in] raw_voltage Tensão lida no sistema não-linear.
 * @return Variável de estado linearizada (z).
 */
float MATH_CTRL_ComputeFeedbackLinearization(float raw_voltage);

/**
 * @brief Placeholder para o Controlador de Ganhos Quadráticos Lineares (LQG/LQR).
 * @param[in] setpoint Referência desejada.
 * @param[in] current_voltage Leitura de feedback.
 * @param[out] telemetry Estrutura atualizada com os estados do Observador Luenberger.
 * @return Ação de controlo ideal computada pelo ganho de realimentação de estados (K).
 */
float MATH_CTRL_ComputeLQG(float setpoint, float current_voltage, control_data_t *telemetry);

#endif /* CONTROL_ALGORITHMS_H */