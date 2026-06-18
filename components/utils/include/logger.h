#pragma once

/**
 * @file logger.h
 * @brief Macros de logging unificadas para el proyecto Argus.
 *
 * PROPÓSITO:
 *   Proporcionar una capa de abstracción delgada sobre ESP_LOG para que el
 *   resto del firmware use ARGUS_LOG* en lugar de ESP_LOG* directamente.
 *   Esta indirección permite cambiar el backend de logging en un solo lugar
 *   sin modificar todos los archivos del proyecto: por ejemplo, agregar
 *   logging a UART remota, SD card, o envío al backend TCP sin tocar el
 *   código de negocio.
 *
 * ESTADO ACTUAL:
 *   Los macros son actualmente wrappers directos de ESP_LOG. No agregan
 *   ningún comportamiento adicional. La capa existe para facilitar la
 *   extensión futura, no para abstraer funcionalidad presente.
 *
 * NIVELES DE LOG:
 *   ARGUS_LOGI (Info): Eventos normales de operación. Inicialización correcta,
 *     cambios de estado, reportes enviados, conexiones establecidas. El operador
 *     no necesita acción.
 *   ARGUS_LOGW (Warning): Situaciones anómalas pero recuperables. GPS fix perdido,
 *     reintento de conexión TCP, evento sospechoso sin confirmar, batería baja.
 *     El operador debe estar al tanto pero el sistema puede continuar.
 *   ARGUS_LOGE (Error): Fallos críticos. Módulo 4G no responde, fallo de hardware,
 *     imposible conectar al servidor después de todos los reintentos. Puede requerir
 *     intervención manual.
 *   ARGUS_LOGD (Debug): Datos detallados para desarrollo y diagnóstico. Valores
 *     raw del sensor, bytes AT individuales, estados internos de buffers.
 *     Se compila a noop en producción si CONFIG_LOG_DEFAULT_LEVEL < DEBUG.
 *
 * CONTROL DEL NIVEL DE LOG:
 *   En tiempo de compilación (menuconfig): CONFIG_LOG_DEFAULT_LEVEL.
 *   En tiempo de ejecución: esp_log_level_set("TAG", ESP_LOG_LEVEL).
 *   esp_log_level_set("*", ESP_LOG_DEBUG) habilita todos los logs en runtime.
 *   esp_log_level_set("A7670", ESP_LOG_NONE) silencia un componente específico.
 *
 * USO:
 *   #include "logger.h"
 *   static const char* TAG = "MiComponente";
 *   ARGUS_LOGI(TAG, "Estado: %d, velocidad: %.1f km/h", state, speed);
 *   ARGUS_LOGE(TAG, "Error crítico: %s", esp_err_to_name(ret));
 *
 * FORMATO DE SALIDA (ESP_LOG por defecto en UART0):
 *   I (12345) MiComponente: Estado: 2, velocidad: 45.3 km/h
 *   │  │       │             └── mensaje formateado
 *   │  │       └── TAG del componente
 *   │  └── timestamp en millisegundos desde boot
 *   └── nivel (I=Info, W=Warn, E=Error, D=Debug)
 *
 * POSIBLES MEJORAS:
 *   1. Agregar macro ARGUS_LOG_HEX(tag, buf, len) para volcar buffers binarios
 *      (útil para depurar paquetes TCP del protocolo Argus).
 *   2. Agregar un hook post-log que envíe mensajes LOGE al backend TCP si la
 *      conexión está activa (log remoto para diagnóstico en campo).
 *   3. Definir un nivel de log mínimo en ARGUS_LOG* independiente de ESP_LOG:
 *      #if ARGUS_LOG_LEVEL >= ARGUS_LOG_INFO → permite silenciar sin menuconfig.
 *
 * @module components/utils/logger
 */

#include "esp_log.h"

// ARGUS_LOGI: eventos normales de operación. El sistema funciona como esperado.
// Usar para: "inicializado", "conectado", "estado cambiado a X", "reporte enviado".
#define ARGUS_LOGI(tag, fmt, ...)  ESP_LOGI(tag, fmt, ##__VA_ARGS__)

// ARGUS_LOGW: situaciones que merecen atención pero no son fatales.
// Usar para: "GPS fix perdido", "reintento #N", "sin respuesta, esperando...".
#define ARGUS_LOGW(tag, fmt, ...)  ESP_LOGW(tag, fmt, ##__VA_ARGS__)

// ARGUS_LOGE: fallos críticos del sistema o del hardware.
// Usar para: "módulo no responde", "todos los reintentos agotados", "fallo de inicialización".
// Siempre visible incluso con nivel de log configurado en WARN o ERROR.
#define ARGUS_LOGE(tag, fmt, ...)  ESP_LOGE(tag, fmt, ##__VA_ARGS__)

// ARGUS_LOGD: datos de diagnóstico detallados solo para desarrollo.
// Se compila a noop si CONFIG_LOG_MAXIMUM_LEVEL < DEBUG (sin overhead en producción).
// Usar para: valores raw del sensor, bytes AT, estado de buffers internos.
#define ARGUS_LOGD(tag, fmt, ...)  ESP_LOGD(tag, fmt, ##__VA_ARGS__)


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/utils/include/logger.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo es una capa fina sobre el sistema de logging de Espressif (ESP_LOG).
   Su propósito actual es puramente de convención: usar ARGUS_LOG* en lugar de
   ESP_LOG* en todo el código del proyecto. Esto permite en el futuro cambiar
   el backend de logging (agregar SD card, envío TCP, etc.) en un solo lugar.

   ACTUALMENTE: los macros son alias directos de ESP_LOG. No agregan overhead.

   GUÍA DE USO:
   Nivel  | Cuándo usarlo                           | Ejemplo
   -------|----------------------------------------|---------------------------------
   LOGI   | Operación normal, éxito                 | "MPU6050 listo: 100Hz, ±2g"
   LOGW   | Algo anómalo pero el sistema continúa   | "GPS fix perdido, usando caché"
   LOGE   | Fallo crítico, puede requerir atención  | "A7670 no responde después de 3 reintentos"
   LOGD   | Solo para depuración en desarrollo      | "AT→ AT+CGNSSINFO ←+CGNSSINFO:..."

   CONTROL DE VERBOSIDAD:
   → En menuconfig: Component config → Log output → Default log verbosity
   → En runtime:    esp_log_level_set("*", ESP_LOG_DEBUG)
   → Por TAG:       esp_log_level_set("A7670", ESP_LOG_NONE)

   DEUDA TÉCNICA:
   1. No hay macro ARGUS_LOG_HEX para buffers binarios (útil para el protocolo TCP).
   2. No hay logging remoto: los LOGE no llegan al operador en tiempo real.
   3. La abstracción es solo nominal — si se quiere extender, hay que redefinir
      los macros para que llamen a una función en lugar de ESP_LOG directamente.

   ═══════════════════════════════════════════════════════════ */
