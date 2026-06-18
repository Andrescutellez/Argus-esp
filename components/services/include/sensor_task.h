#pragma once

/**
 * @file sensor_task.h
 * @brief Interfaz de la tarea FreeRTOS del sensor MPU6050 del sistema Argus.
 *
 * PROPÓSITO:
 *   Declarar la tarea de sensor y sus funciones auxiliares. Esta tarea es
 *   responsable de todo lo relacionado con el MPU6050: inicializarlo, esperar
 *   sus interrupciones, leer sus datos, clasificar el movimiento, y enviar
 *   los eventos correspondientes a control_task.
 *
 * ARQUITECTURA DE DETECCIÓN (pipeline de movimiento):
 *
 *   MPU6050 hardware
 *     → genera señal en pin INT (GPIO27)
 *     → ISR motionISRHandler() [IRAM_ATTR, ~1µs]
 *     → xSemaphoreGiveFromISR(xMotionSemaphore)
 *     → sensorTask despierta de xSemaphoreTake()
 *     → readSensorData() [I2C, ~14 bytes, ~200µs]
 *     → classifyMovement() [lógica, ~10µs]
 *     → xQueueSend(xEventQueue, EventMessage_t) [FreeRTOS, ~5µs]
 *     → control_task procesa el evento
 *
 *   POLLING DE RESPALDO (cada 1s si la ISR no disparó):
 *     Si el semáforo expira sin señal de la ISR, sensorTask igualmente
 *     lee el sensor para detectar movimiento lento que no disparó la ISR
 *     (por debajo del umbral de hardware del MPU6050 pero sobre el umbral
 *     de software definido en SENSITIVITY_TABLE).
 *
 * PRIORIDAD: 5 (la más alta de las tareas del sistema)
 *   Justificación: el sensor de seguridad no puede perder eventos. Si sensorTask
 *   tuviera menor prioridad que control_task, podría perderse una señal de la ISR
 *   si control_task está procesando algo largo.
 *
 * STACK: 4096 bytes
 *   Incluye el stack de xSemaphoreTake, readSensorData (I2C), classifyMovement
 *   (sqrtf con FPU), y xQueueSend. 4096 bytes es suficiente con margen.
 *
 * CORE: 0 (default en ESP-IDF single-core scheduler)
 *
 * VARIABLES CRÍTICAS:
 *   - xMotionSemaphore (interno): semáforo binario dado por la ISR GPIO27.
 *     Si se crea con NULL (malloc falla), la tarea se aborta en init().
 *   - SENSITIVITY_TABLE (interno): tabla de umbrales por nivel de sensibilidad.
 *     5 entradas, indexadas por SensitivityLevel_t (0-4).
 *   - motionSensitivity (system_flags.h): nivel actual, cargado desde NVS al arranque.
 *
 * CONCURRENCIA:
 *   - xMotionSemaphore: dado desde ISR (IRAM_ATTR), tomado en sensorTask.
 *     Es un binary semaphore: si la ISR dispara múltiples veces antes de que
 *     sensorTask consuma el semáforo, las señales extras se colapsan en una.
 *     Esto puede causar que se pierda un evento si la ISR dispara 2 veces
 *     mientras sensorTask está en la I2C. En práctica, los eventos de movimiento
 *     tienen un anti-rebote de 200ms (vTaskDelay al final del loop).
 *   - lastMovementTimestamp: escrito por sensorTask, leído por commTask.
 *     Ver nota de concurrencia en system_flags.h.
 *
 * ISR CONSTRAINTS:
 *   - motionISRHandler() está marcada IRAM_ATTR para garantizar ejecución
 *     en SRAM incluso durante operaciones de flash (que deshabilitan el caché).
 *   - La ISR solo llama xSemaphoreGiveFromISR y portYIELD_FROM_ISR.
 *     Nunca bloquea, nunca lee el sensor (que requiere I2C).
 *
 * @module components/services/sensor_task
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "system_flags.h"

// Handle de la tarea (para suspender/reanudar desde otra tarea si fuera necesario).
// Definido en sensor_task.cpp, inicializado por xTaskCreate() en main.cpp.
extern TaskHandle_t xSensorTaskHandle;

/**
 * @brief Función principal de la tarea FreeRTOS del sensor MPU6050.
 *
 * PROPÓSITO:
 *   Ejecutar el pipeline de detección de movimiento en un loop infinito.
 *   Combina detección por interrupción (ISR) con polling de respaldo (1s timeout).
 *
 * FLUJO INTERNO:
 *   1. loadSensitivityFromNVS() — cargar nivel de sensibilidad guardado.
 *   2. Crear xMotionSemaphore.
 *   3. mpu6050.init() — inicializar I2C y configurar el chip.
 *   4. sensorTask_initISR() — conectar GPIO27 a motionISRHandler.
 *   5. Loop:
 *      a. xSemaphoreTake(xMotionSemaphore, 1000ms) — esperar ISR o timeout.
 *      b. readSensorData() — leer 14 bytes del MPU6050.
 *      c. classifyMovement() — determinar tipo de evento.
 *      d. Si evento válido y shouldProcess → xQueueSend(xEventQueue).
 *      e. vTaskDelay(200ms) — anti-rebote.
 *
 * MANEJO DE ERRORES:
 *   Si el semáforo no se puede crear → vTaskDelete(nullptr).
 *   Si mpu6050.init() falla → vTaskDelete(nullptr).
 *   Si readSensorData() falla → log WARN + vTaskDelay(100ms) + continue.
 *   Si la cola de eventos está llena → log WARN + descartar evento.
 *
 * @param pvParameters No usado (nullptr). Requerido por la firma de tarea FreeRTOS.
 */
void sensorTask(void* pvParameters);

/**
 * @brief Instala la ISR de movimiento en GPIO27 conectándola a xMotionSemaphore.
 *
 * PROPÓSITO:
 *   Separar la instalación de la ISR de la inicialización del sensor para que
 *   el semáforo esté creado antes de que la ISR pueda disparar.
 *   Llamada internamente desde sensorTask() después de crear xMotionSemaphore.
 *
 * FLUJO:
 *   mpu6050.installMotionISR(motionISRHandler, nullptr)
 *   → gpio_config(GPIO27, flanco ascendente, pull-down)
 *   → gpio_install_isr_service(ESP_INTR_FLAG_IRAM)
 *   → gpio_isr_handler_add(GPIO27, motionISRHandler, nullptr)
 *
 * NOTA: El 'arg' de la ISR es nullptr porque la ISR accede a xMotionSemaphore
 *   como variable estática del módulo (visible dentro de sensor_task.cpp).
 *   El semáforo podría pasarse como arg, pero la implementación actual usa
 *   acceso directo a la variable estática.
 */
void sensorTask_initISR();

/**
 * @brief Cambia el nivel de sensibilidad de detección y lo persiste en NVS.
 *
 * PROPÓSITO:
 *   Actualizar motionSensitivity (variable global volatile) y persistir el
 *   nuevo valor en NVS para que sobreviva reboots. El cambio tiene efecto
 *   inmediato: la próxima clasificación de movimiento usará los nuevos umbrales.
 *
 * FLUJO:
 *   1. Sanitizar level (si fuera de rango → SENSITIVITY_MEDIUM).
 *   2. motionSensitivity = level.
 *   3. nvs_open("argus_cfg", NVS_READWRITE) → nvs_set_u8("sensitivity", level) → commit → close.
 *   4. Log con los nuevos umbrales efectivos.
 *
 * THREAD SAFETY:
 *   La escritura de motionSensitivity (enum de 4 bytes) es atómica en ARM.
 *   El acceso a NVS es thread-safe internamente.
 *   Puede llamarse desde bleTask o commTask sin mutex adicional.
 *
 * CORRESPONDENCIA BLE:
 *   SENSITIVITY_VERY_LOW  (0) ← BLE byte 0x10
 *   SENSITIVITY_LOW       (1) ← BLE byte 0x11
 *   SENSITIVITY_MEDIUM    (2) ← BLE byte 0x12 (default)
 *   SENSITIVITY_HIGH      (3) ← BLE byte 0x13
 *   SENSITIVITY_VERY_HIGH (4) ← BLE byte 0x14
 *
 * @param level El nuevo nivel de sensibilidad a aplicar (SensitivityLevel_t 0-4).
 *              Si el valor está fuera de rango, se usa SENSITIVITY_MEDIUM.
 */
void setSensitivityLevel(SensitivityLevel_t level);


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/include/sensor_task.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo declara la "tarea de vigilancia" del sistema Argus. Hay una tarea
   FreeRTOS que corre permanentemente con la máxima prioridad, esperando que el
   sensor de movimiento (MPU6050) detecte algo. Cuando el sensor genera una señal
   eléctrica (interrupción), la tarea se despierta, lee los datos del sensor, y
   decide si el movimiento es suave, brusco, o un impacto.

   La sensibilidad es configurable via BLE desde la app móvil. Un nivel bajo
   requiere movimiento más fuerte para generar una alarma (menos falsas alarmas
   pero puede perderse movimientos sutiles). Un nivel alto detecta cualquier
   micro-movimiento (máxima seguridad pero más falsas alarmas).

   FUNCIONES PÚBLICAS:
   sensorTask()          → tarea FreeRTOS (arrancada por main.cpp)
   sensorTask_initISR()  → conecta GPIO27 a la ISR (llamada internamente)
   setSensitivityLevel() → cambia umbrales de detección + persiste en NVS

   DIAGRAMA DE DETECCIÓN:
   [MPU6050] → GPIO27 → ISR → Semáforo → sensorTask despierta
   sensorTask → readSensorData() → classifyMovement()
     → EVENT_MOVEMENT_SOFT (vibración leve)
     → EVENT_MOVEMENT_HARD (movimiento brusco)
     → EVENT_IMPACT_DETECTED (golpe fuerte)
   → xQueueSend(xEventQueue) → control_task

   ═══════════════════════════════════════════════════════════ */
