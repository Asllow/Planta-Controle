/**
 * @file control_task.h
 * @brief Interface da tarefa de controlo de tempo real.
 */

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/**
 * @brief Tarefa principal do ciclo de controlo.
 * @param[in] pvParameters Parâmetros da tarefa.
 */
void control_loop_task(void *pvParameters);

#endif /* CONTROL_TASK_H */