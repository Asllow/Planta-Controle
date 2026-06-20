/**
 * @file udp_comms_task.h
 * @brief Interface do subsistema de comunicação UDP (Telemetria e Comandos).
 */

#ifndef UDP_COMMS_TASK_H
#define UDP_COMMS_TASK_H

/**
 * @brief Tarefa dedicada à transmissão contínua (Streaming) de telemetria.
 * * Consome os dados da fila estruturada e os envia em lote via UDP Broadcast.
 * * @param[in] pvParameter Parâmetro padrão do FreeRTOS.
 */
void UDP_COMMS_TxTask(void *pvParameter);

/**
 * @brief Tarefa dedicada à recepção de comandos externos.
 * * Bloqueia passivamente aguardando pacotes UDP contendo novos setpoints
 * ou configurações de matrizes de controle (LQG/LQR).
 * * @param[in] pvParameter Parâmetro padrão do FreeRTOS.
 */
void UDP_COMMS_RxTask(void *pvParameter);

#endif /* UDP_COMMS_TASK_H */