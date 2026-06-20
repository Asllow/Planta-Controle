/**
 * @file hal_motor_pwm.h
 * @brief Interface da Camada de Abstração de Hardware para o sinal de PWM.
 */
#ifndef HAL_MOTOR_PWM_H
#define HAL_MOTOR_PWM_H

#include <stdbool.h>

/**
 * @brief Inicializa o periférico gerador de PWM.
 * @return true em caso de sucesso.
 */
bool HAL_PWM_Init(void);

/**
 * @brief Define o ciclo de trabalho (Duty Cycle) do PWM.
 * @param[in] duty_percent Valor de 0.0 a 100.0%.
 */
void HAL_PWM_SetDutyCycle(float duty_percent);

#endif /* HAL_MOTOR_PWM_H */