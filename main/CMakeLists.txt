# main/CMakeLists.txt

idf_component_register(SRCS
        "main.c"
        "wifi_manager.c"
        "control_task.c"
        "http_client_task.c"
        INCLUDE_DIRS ".")