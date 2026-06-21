/**
 * @file ipc_manager.h
 * @brief Gestor de Comunicação Inter-Processos (IPC).
 * * Abstrai as filas e semáforos do FreeRTOS, fornecendo
 * uma interface segura para a troca de dados entre núcleos.
 */

#ifndef IPC_MANAGER_H
#define IPC_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Estrutura unificada de telemetria e estado do observador.
 * * Estruturada para envio binário direto (Little-Endian, 36 bytes).
 * O atributo 'packed' impede a inserção de bytes de alinhamento pelo compilador.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_amostra_ms;
    float sinal_controle;
    float tensao_mv;
    float valor_adc;
    float tensao_estimada_mv;
    float erro_obs_mv;
    float estado_1;
    float estado_2;
    float estado_3;
} control_data_t;

/**
 * @brief Inicializa os recursos do FreeRTOS (Filas e Mutexes).
 * @return true se o sucesso for alcançado, false caso contrário.
 */
bool IPC_MGR_Init(void);

/**
 * @brief Define de forma segura a referência genérica recebida da rede.
 * @param[in] ref Valor de referência agnóstico (float).
 */
void IPC_MGR_SetReference(float ref);

/**
 * @brief Obtém de forma segura a referência atual para a malha.
 * @return Valor da referência agnóstica.
 */
float IPC_MGR_GetReference(void);

/**
 * @brief Insere um novo pacote de dados na fila de telemetria.
 * @param[in] data Ponteiro para a estrutura de dados a enviar.
 * @return true se inserido com sucesso, false se a fila estiver cheia.
 */
bool IPC_MGR_EnqueueTelemetry(const control_data_t *data);

/**
 * @brief Retira um pacote de dados da fila de telemetria.
 * @param[out] data Ponteiro onde os dados recebidos serão armazenados.
 * @param[in] timeout_ms Tempo máximo de espera em milissegundos.
 * @return true se os dados foram lidos com sucesso, false em caso de timeout.
 */
bool IPC_MGR_DequeueTelemetry(control_data_t *data, uint32_t timeout_ms);

#endif /* IPC_MANAGER_H */