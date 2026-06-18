# Componente: services
# Tareas FreeRTOS del sistema. BLE usa el stack NimBLE dentro del componente 'bt'.

idf_component_register(
    SRCS
        "sensor_task.cpp"
        "control_task.cpp"
        "comm_task.cpp"
        "ble_task.cpp"
        "event_queue.cpp"
        "sync_manager.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        "core"
        "drivers"
        "esp_driver_gpio"
        "freertos"
        "log"
        "esp_timer"
        "bt"
        "nvs_flash"
)
