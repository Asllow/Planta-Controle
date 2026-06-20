/**
 * @file hal_sensor_adc.h
 * @brief Interface da Camada de Abstração de Hardware para leitura do Tacogerador.
 */
#ifndef HAL_SENSOR_ADC_H
#define HAL_SENSOR_ADC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Inicializa o conversor Analógico-Digital e os perfis de calibração.
 * @return true em caso de sucesso.
 */
bool HAL_ADC_Init(void);

/**
 * @brief Efetua uma leitura do ADC e devolve os valores brutos e convertidos.
 * @param[out] voltage_mv Ponteiro para armazenar a tensão calibrada em milivolts.
 * @param[out] raw_value Ponteiro para armazenar o valor bruto do ADC.
 * @return true em caso de leitura válida.
 */
bool HAL_ADC_ReadVoltage(uint32_t *voltage_mv, int *raw_value);

#endif /* HAL_SENSOR_ADC_H */