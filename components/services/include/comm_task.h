#pragma once

/**
 * @file comm_task.h
 * @brief Interfaz de la tarea FreeRTOS de comunicación GPS + telemetría del sistema Argus.
 *
 * PROPÓSITO:
 *   Declarar la tarea de comunicación y su handle. Esta tarea es responsable de
 *   todo lo relacionado con conectividad saliente: obtener posición GPS del A7670,
 *   construir el payload del protocolo Argus, encolarlo con prioridad según el
 *   estado del sistema, y enviarlo al backend via TCP con backoff automático.
 *
 * ROL EN LA ARQUITECTURA:
 *   commTask cierra el pipeline de datos del sistema. sensorTask detecta movimiento
 *   → controlTask cambia estado → commTask reporta la posición al backend con la
 *   urgencia correspondiente al estado actual.
 *
 *   Jerarquía de tareas:
 *     sensorTask  (prio 5) → detecta movimiento
 *     controlTask (prio 4) → decide y actúa
 *     commTask    (prio 3) → reporta telemetría   ← ESTE ARCHIVO
 *     bleTask     (prio 2) → escucha comandos BLE
 *
 *   Justificación de prioridad 3:
 *     Las operaciones de red (TCP, AT commands, GNSS) son lentas (100ms-3s).
 *     commTask puede pasar segundos bloqueado en I/O sin afectar la detección
 *     de movimiento ni el control de hardware, que tienen prioridades superiores.
 *
 * SUBSISTEMAS INTERNOS (instanciados en comm_task.cpp como static):
 *   - A7670Driver a7670:        driver del modem 4G para GPS y TCP.
 *   - EventQueue  eventQueue:   cola con prioridad y persistencia NVS.
 *   - SyncManager syncManager:  drena eventQueue con backoff exponencial.
 *
 * PROTOCOLO ARGUS (frame TCP):
 *   Formato: "ARGUS|<device_id>|<epoch_ms>|<lat>|<lon>|<crc32_hex>\n"
 *   Ejemplo: "ARGUS|ARGUS-1A2B3C4D|1715027200000|19.432608|-99.133209|3F8A1B2C\n"
 *
 *   CRC32 se calcula sobre: "<device_id>|<epoch_ms>|<lat>|<lon>|argus-dev-secret"
 *   El secreto está hardcodeado en ARGUS_SIGNATURE_SECRET (comm_task.cpp).
 *   NOTA: Este secreto es de desarrollo — en producción debería venir de NVS o
 *   provisioning seguro. Ver deuda técnica en comm_task.cpp.
 *
 * INTERVALOS DE TELEMETRÍA:
 *   PURSUIT:          10s (emergencia — ignora plan)
 *   ALERT:            15s (alerta — ignora plan)
 *   MOVING/IDLE PREMIUM:  30s
 *   MOVING/IDLE FREEMIUM: 3600s (1h)
 *
 * LÓGICA DE INACTIVIDAD:
 *   Si el sistema está en IDLE sin movimiento por más de 5 minutos, la telemetría
 *   se pausa. Solo se envía un keepalive mínimo con la última posición conocida.
 *   Cuando se detecta actividad, se reanuda la telemetría normal. Esto conserva
 *   datos del plan FREEMIUM y batería.
 *
 * STACK: 8192 bytes
 *   Mayor que las demás tareas porque comm_task usa:
 *   - snprintf para buildArgusPacket (buffer tcpPacket[320] en stack)
 *   - sscanf en gpsTimestampToEpoch
 *   - A7670Driver con operaciones UART internas
 *   - SyncManager y EventQueue con sus estados internos
 *   8192 bytes previene stack overflow en las operaciones de parsing de strings.
 *
 * VARIABLES CRÍTICAS (en comm_task.cpp):
 *   - g_deviceId[20]:       ID del dispositivo derivado de la MAC (4 bytes).
 *   - lastKnownGps:         última posición GPS válida (fallback si se pierde el fix).
 *   - hasEverHadFix:        false hasta el primer fix GPS real. Sin fix: no se reporta.
 *   - ARGUS_SIGNATURE_SECRET: secreto CRC32. Ver nota de seguridad arriba.
 *
 * CONCURRENCIA:
 *   - lastMovementTimestamp (system_flags.h): escrito por sensorTask, leído por commTask.
 *     Ver ARQUITECTURA ⚠️ en system_flags.h sobre atomicidad de uint64_t.
 *   - currentSystemState (system_flags.h): escrito por controlTask, leído por commTask.
 *     Lectura directa sin mutex — ver nota en system_flags.h.
 *   - xGpsDataQueue (cap. 1, overwrite): commTask escribe, otros leen.
 *     xQueueOverwrite es atómico — seguro sin mutex adicional.
 *   - xEventQueue: commTask escribe EVENT_GPS_FIX_OK/LOST y EVENT_COMMS_SUCCESS/FAILURE.
 *     xQueueSend con timeout=0 para no bloquear.
 *
 * @module components/services/comm_task
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Handle de la tarea. Definido en comm_task.cpp, inicializado por xTaskCreate() en main.cpp.
extern TaskHandle_t xCommTaskHandle;

/**
 * @brief Función principal de la tarea FreeRTOS de comunicación GPS + telemetría.
 *
 * PROPÓSITO:
 *   Ejecutar el ciclo de telemetría adaptativa: obtener GPS, construir payload,
 *   encolar con prioridad, y drenar la cola via SyncManager con backoff automático.
 *
 * FLUJO DE ARRANQUE:
 *   1. initDeviceId()          → derivar ARGUS-XXXXXXXX de la MAC del ESP32.
 *   2. eventQueue.init()       → inicializar la cola con prioridad.
 *   3. eventQueue.loadFromNVS() → restaurar eventos CRITICAL del boot anterior.
 *   4. syncManager.init()      → inicializar el manager de sincronización.
 *   5. a7670.init()            → inicializar modem A7670 (AT, GNSS, TCP).
 *
 * LOOP (cada 1s via vTaskDelay(1000ms)):
 *   A. Ciclo de telemetría (si han pasado N segundos desde el último envío):
 *      1. Evaluar inactividad (>5min en IDLE sin movimiento → pausa).
 *      2. Si activo: a7670.getGpsPosition() → notifyGpsStatusChange() → xQueueOverwrite().
 *      3. Determinar coordenadas: fix actual, fallback a lastKnownGps, o skip si sin fix nunca.
 *      4. buildArgusPacket() → CRC32 → payload TCP.
 *      5. submitReport() → encolar con prioridad según estado del sistema.
 *   B. SyncManager tick (cada iteración del loop):
 *      1. syncManager.tick(now) → drenar hasta 5 eventos con TCP send + backoff.
 *      2. Detectar cambios en métricas → enviar EVENT_COMMS_SUCCESS/FAILURE a xEventQueue.
 *   C. Guardar NVS (cada 5 minutos):
 *      eventQueue.saveToNVS() → persistir eventos CRITICAL pendientes.
 *   D. a7670.maintain() → watchdog AT del modem para mantener la sesión TCP activa.
 *
 * MANEJO DE ERRORES:
 *   Si a7670.init() falla → log LOGE + continuar (la tarea sigue corriendo para
 *   al menos intentar recuperar el modem en ciclos posteriores con a7670.recover()).
 *   Si GPS sin fix por primera vez → no enviar telemetría (hasEverHadFix=false).
 *   Si GPS sin fix después de fix previo → usar lastKnownGps (epoch=0 → backend usa Date.now()).
 *
 * @param pvParameters No usado (nullptr). Requerido por la firma de tarea FreeRTOS.
 */
void commTask(void* pvParameters);


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/include/comm_task.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   comm_task.h declara la tarea que "habla con el mundo exterior".
   Esta tarea hace dos cosas cada segundo:
   1. Cada N segundos (según urgencia): pide al modem 4G la posición GPS
      y empaqueta esa posición en el protocolo Argus para enviarla al backend.
   2. Cada segundo: intenta enviar los mensajes pendientes en la cola,
      respetando backoff exponencial si hay fallos de red.

   La frecuencia de reportes es adaptativa:
   - Emergencia (persecución): cada 10 segundos
   - Alerta (posible robo): cada 15 segundos
   - Normal con plan premium: cada 30 segundos
   - Freemium sin alerta: solo cada hora

   FUNCIONES PÚBLICAS:
   commTask() → tarea FreeRTOS (arrancada por main.cpp)

   ═══════════════════════════════════════════════════════════ */
