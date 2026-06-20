/**
 * @file wifi_manager.h
 * @brief Interface de gerenciamento do periférico Wi-Fi do ESP32-S3.
 * * Este módulo abstrai as rotinas de inicialização e configuração do 
 * rádio Wi-Fi, operando primariamente no modo Access Point (SoftAP).
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"

/**
 * @brief Inicializa o rádio Wi-Fi no modo Access Point (SoftAP).
 * * Configura o servidor DHCP, define as credenciais da rede e inicia o rádio.
 * * @param[in] ssid     Nome da rede (SSID) a ser transmitida (máx 32 caracteres).
 * @param[in] password Senha da rede (mínimo 8 caracteres, máximo 64 caracteres).
 * Se a string for vazia, a rede será configurada como aberta.
 * * @return esp_err_t ESP_OK em caso de sucesso.
 * Códigos de erro padrão ESP-IDF em caso de falha.
 */
esp_err_t WIFI_MGR_InitAP(const char* ssid, const char* password);

#endif /* WIFI_MANAGER_H */