# Componente principal: main
# Punto de entrada del firmware Argus Secure.

idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        "core"
        "drivers"
        "services"
        "utils"
        "nvs_flash"
        "log"
        "freertos"
        "esp_driver_gpio"
)
