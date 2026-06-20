# main/CMakeLists.txt

idf_component_register(SRCS "main.c" 
                        "net/wifi_manager.c" 
                        "net/udp_comms_task.c"
                        "rtos/ipc_manager.c"
                        "rtos/control_task.c"
                        "hal/hal_motor_pwm.c"
                        "hal/hal_sensor_adc.c"
                        "math/control_algorithms.c"
                INCLUDE_DIRS "." "net" "rtos" "hal" "math"
                REQUIRES esp_wifi driver esp_event lwip nvs_flash esp_netif esp_adc esp_driver_ledc)