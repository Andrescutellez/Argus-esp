# Componente: utils
# Utilidades compartidas: macros de logging.
# Componente header-only: no tiene archivos .cpp, solo expone include/logger.h.
# En ESP-IDF 6.0 el componente de logging se llama 'log' (antes 'esp_log').

idf_component_register(
    INCLUDE_DIRS
        "include"
    REQUIRES
        "log"
)
