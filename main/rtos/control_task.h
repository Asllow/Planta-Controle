/**
 * @file control_task.h
 * @brief Interface da tarefa de controlo de tempo real.
 * * Expõe a assinatura da tarefa responsável por ditar o período
 * de amostragem, adquirir dados, calcular a lei de controlo e atuar na planta.
 */

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/**
 * @brief Tarefa principal do ciclo de controlo.
 *
 * Executa a amostragem da HAL, cálculo do esforço de controlo via 
 * biblioteca matemática e atuação na planta via PWM, garantindo um 
 * período de amostragem rigoroso e determinístico.
 *
 * @param[in] pvParameters Parâmetros da tarefa (padrão FreeRTOS, não utilizado).
 */
void control_loop_task(void *pvParameters);

#endif /* CONTROL_TASK_H */