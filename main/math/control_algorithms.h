/**
 * @file control_algorithms.h
 * @brief Algoritmos matemáticos independentes de arquitetura para a malha de controlo.
 * * Contém a implementação do Controlador de Ganhos Quadráticos Lineares (LQR/LQI),
 * do Observador de Estados (LQG) e as transformações de Difeomorfismo para
 * linearização por realimentação (Feedback Linearization).
 */

#ifndef CONTROL_ALGORITHMS_H
#define CONTROL_ALGORITHMS_H

#include <stdint.h>
#include "ipc_manager.h"

/**
 * @brief Calcula a ação de controlo baseada na compensação de Zona Morta (Malha Aberta).
 * @param[in] desired_setpoint Setpoint exigido.
 * @return Esforço de controlo em percentagem (0.0f a 100.0f).
 */
float MATH_CTRL_ComputeOpenLoop(float desired_setpoint);

/**
 * @brief Computa a lei de controle em malha fechada via LQI, Difeomorfismo e LQG.
 * * Executa o gerador dinâmico de referências físicas, a integração de erro com 
 * proteção Anti-Windup direcional, a lei de controle LQR no espaço linearizado (z),
 * e propaga o estado do sistema oculto através da integração otimizada de Euler (O(1)).
 * * @param[in] setpoint_v Referência de tensão desejada para o gerador (Volts).
 * @param[in] current_voltage_v Tensão real medida na saída do tacogerador (Volts).
 * @param[out] telemetry Estrutura de telemetria a ser preenchida com os estados do observador.
 * @return Ação de controle mapeada em percentagem de Duty Cycle (0.0f a 100.0f).
 */
float MATH_CTRL_ComputeLQG(float setpoint_v, float current_voltage_v, control_data_t *telemetry);

#endif /* CONTROL_ALGORITHMS_H */