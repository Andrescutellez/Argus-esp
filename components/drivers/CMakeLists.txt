# Componente: drivers
# mpu6050_driver usa I2C      → esp_driver_i2c
# a7670_driver usa UART       → esp_driver_uart
# mosfet_control usa GPIO     → esp_driver_gpio (heredado de core)
# LedManager usa GPIO + LEDC  → esp_driver_gpio, esp_driver_ledc
# ActuatorManager usa GPIO    → esp_driver_gpio

idf_component_register(
    SRCS
        "mpu6050_driver.cpp"
        "a7670_driver.cpp"
        "mosfet_control.cpp"
        "LedManager.cpp"
        "ActuatorManager.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        "core"
        "esp_driver_gpio"
        "esp_driver_ledc"
        "esp_driver_i2c"
        "esp_driver_uart"
        "freertos"
        "log"
        "esp_timer"
)
