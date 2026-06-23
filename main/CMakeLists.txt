# main/CMakeLists.txt

idf_component_register(SRCS "main.c" 
                        "wifi_manager.c" 
                        "control_task.c" 
                        "udp_client_task.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp_wifi driver esp_event lwip nvs_flash esp_netif esp_adc esp_driver_ledc esp_timer)