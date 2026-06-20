/**
 * @file wifi_manager.c
 * @brief Implementação do gerenciador Wi-Fi (Modo SoftAP).
 */

#include "wifi_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

static const char* TAG = "WIFI_MGR";

#define WIFI_MGR_MAX_STA_CONN 4
#define WIFI_MGR_CHANNEL      1

/**
 * @brief Manipulador de eventos de sistema para o subsistema Wi-Fi.
 * * Escuta eventos de conexão e desconexão de estações ao SoftAP
 * para fins de diagnóstico e log.
 */
static void WIFI_MGR_EventHandler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) 
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Estacao conectada - MAC: " MACSTR ", AID: %d",
                MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Estacao desconectada - MAC: " MACSTR ", AID: %d",
                MAC2STR(event->mac), event->aid);
    }
}

esp_err_t WIFI_MGR_InitAP(const char* ssid, const char* password) 
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &WIFI_MGR_EventHandler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid),
            .channel = WIFI_MGR_CHANNEL,
            .max_connection = WIFI_MGR_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                    .required = false,
            },
        },
    };

    strncpy((char*)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    
    if (strlen(password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        strncpy((char*)wifi_config.ap.password, password, sizeof(wifi_config.ap.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    /* * Desativa power save no modo AP para evitar aumento de latência na 
     * comunicação TCP/UDP. O ESP32 manterá o rádio ativo permanentemente.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Inicializacao AP concluida. SSID: %s | Canal: %d",
            ssid, WIFI_MGR_CHANNEL);

    return ESP_OK;
}