# main/CMakeLists.txt

idf_component_register(SRCS "main.c" 
                        "wifi_manager.c" 
                        "control_task.c" 
                        "http_client_task.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp_wifi driver esp_event esp_websocket_client cjson nvs_flash esp_netif esp_adc)