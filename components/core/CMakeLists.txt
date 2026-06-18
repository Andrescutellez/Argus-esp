# Componente: core
# En ESP-IDF 6.0 el componente 'driver' se dividió en esp_driver_gpio, etc.
# pin_config.h incluye driver/gpio.h → requiere esp_driver_gpio.

idf_component_register(
    SRCS
        "state_machine.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        "esp_driver_gpio"
        "freertos"
        "log"
)
