/**
 * @file udp_comms_task.h
 * @brief Interface do subsistema de comunicação UDP via Serialização Binária.
 */

#ifndef UDP_COMMS_TASK_H
#define UDP_COMMS_TASK_H

/**
 * @brief Tarefa dedicada à transmissão contínua de telemetria (C-Struct).
 * @param[in] pvParameter Parâmetro padrão do FreeRTOS.
 */
void UDP_COMMS_TxTask(void *pvParameter);

/**
 * @brief Tarefa dedicada à recepção de comandos externos binários (float).
 * @param[in] pvParameter Parâmetro padrão do FreeRTOS.
 */
void UDP_COMMS_RxTask(void *pvParameter);

#endif /* UDP_COMMS_TASK_H */