#pragma once

/**
 * @file ble_task.h
 * @brief Interfaz de la tarea FreeRTOS del servidor BLE del sistema Argus.
 *
 * PROPÓSITO:
 *   Declarar la tarea BLE y su handle. Esta tarea inicializa el stack NimBLE,
 *   registra los servicios GATT del sistema Argus, y queda suspendida mientras
 *   la tarea interna del host NimBLE (nimbleHostTask) procesa eventos BLE de
 *   forma autónoma.
 *
 * ROL EN LA ARQUITECTURA:
 *   bleTask es el "puente BLE → eventos del sistema". Cuando la app móvil
 *   Argus envía un comando via BLE, el callback cmdCharAccessCallback() lo
 *   traduce a un EventMessage_t y lo inyecta en xEventQueue para que
 *   controlTask lo procese.
 *
 *   Jerarquía de tareas:
 *     sensorTask    (prio 5) → detecta movimiento
 *     controlTask   (prio 4) → decide y actúa
 *     commTask      (prio 3) → reporta telemetría
 *     bleTask       (prio 2) → escucha comandos BLE  ← ESTE ARCHIVO
 *     nimbleHostTask (prio interna de NimBLE)
 *
 *   Justificación de prioridad 2:
 *     BLE no es crítico en tiempo real. Los comandos BLE son acciones del usuario
 *     (ARM/DISARM) que no tienen requisitos de latencia estrictos (ms son suficientes).
 *     La prioridad baja garantiza que los eventos de sensor y control no sean
 *     interrumpidos por el procesamiento BLE.
 *
 * PROTOCOLO BLE — SERVICIO GATT:
 *   UUID Servicio:        4FAFC201-1FB5-459E-8FCC-C5C9C331914B
 *   UUID Característica:  BEB5483E-36E1-4688-B7F5-EA07361B26A8
 *   Propiedades:          WRITE | WRITE_NO_RSP
 *
 *   Razón para usar WRITE_NO_RSP:
 *     Reduce la latencia de la escritura BLE: la app no espera confirmación ATT.
 *     Aceptable para comandos de control donde la confirmación viene implícitamente
 *     a través del cambio de estado visible en la app (LEDs, notificaciones push).
 *
 * COMANDOS BLE (byte único):
 *   0x01 → EVENT_ARM_CMD          (armar el sistema)
 *   0x02 → EVENT_DISARM_CMD       (desarmar el sistema)
 *   0x03 → EVENT_TRIGGER_ALERT_CMD (disparar alerta manual)
 *   0x10 → setSensitivityLevel(SENSITIVITY_VERY_LOW)
 *   0x11 → setSensitivityLevel(SENSITIVITY_LOW)
 *   0x12 → setSensitivityLevel(SENSITIVITY_MEDIUM)   (default)
 *   0x13 → setSensitivityLevel(SENSITIVITY_HIGH)
 *   0x14 → setSensitivityLevel(SENSITIVITY_VERY_HIGH)
 *   0x20 → Auto-ARM enable (solo PREMIUM). Opcional: 2do byte = minutos (1-60).
 *   0x21 → Auto-ARM disable (solo PREMIUM).
 *
 *   NOTA: Los comandos de sensibilidad (0x10-0x14) NO generan eventos en xEventQueue.
 *   Actúan directamente sobre motionSensitivity (volatile global en system_flags.h)
 *   y persisten el nivel en NVS via setSensitivityLevel(). El efecto es inmediato:
 *   la próxima clasificación de movimiento en sensorTask usa los nuevos umbrales.
 *
 * DISPOSITIVO BLE:
 *   Nombre: "Argus-Secure" (visible en el scan de la app móvil)
 *   Intervalo de advertising: 100ms (itvl_min=itvl_max=160 × 0.625ms = 100ms)
 *   Modo: connectable undirected (cualquier cliente puede conectarse)
 *
 * STACK: 6144 bytes
 *   NimBLE requiere más stack que otras tareas porque:
 *   - El stack NimBLE usa malloc para buffers de mbuf (mblocks de datos BLE).
 *   - La tabla de servicios GATT se registra en el heap via ble_gatts_add_svcs().
 *   - Los callbacks GATT (cmdCharAccessCallback) se ejecutan en el contexto de
 *     nimbleHostTask, no en bleTask — bleTask solo inicializa y se suspende.
 *   6144 bytes es el mínimo recomendado para un servidor NimBLE simple.
 *
 * SEGURIDAD:
 *   ARQUITECTURA:
 *      No hay emparejamiento BLE (bonding/MITM). Cualquier dispositivo en rango
 *      puede conectarse y enviar comandos ARM/DISARM sin autenticacion.
 *      En produccion se debe implementar:
 *      1. ble_hs_cfg.sm_bonding = 1 (bonding persistente)
 *      2. ble_hs_cfg.sm_mitm = 1   (proteccion contra ataques man-in-the-middle)
 *      3. Passkey o OOB pairing para autenticar la app movil.
 *      Sin esto, un atacante con un telefono puede desarmar el sistema.
 *
 * CONCURRENCIA:
 *   - cmdCharAccessCallback() se ejecuta en el contexto de nimbleHostTask (no bleTask).
 *     xQueueSend(xEventQueue, ..., 100ms) es seguro desde cualquier tarea.
 *   - setSensitivityLevel() es thread-safe (escribe un enum de 4 bytes atómicamente
 *     en ARM y luego accede al NVS que tiene mutex interno).
 *   - bleTask se suspende con vTaskSuspend() tras inicializar NimBLE.
 *     nimbleHostTask corre de forma independiente hasta que se llame nimble_port_stop().
 *
 * @module components/services/ble_task
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

// Handle de la tarea. Definido en ble_task.cpp, inicializado por xTaskCreate() en main.cpp.
extern TaskHandle_t xBleTaskHandle;

/**
 * @brief Función principal de la tarea FreeRTOS del servidor BLE.
 *
 * PROPÓSITO:
 *   Inicializar el stack NimBLE, registrar el servicio GATT del sistema Argus,
 *   lanzar la tarea interna nimbleHostTask, y luego suspenderse para no consumir
 *   CPU. Todo el procesamiento BLE ocurre en nimbleHostTask.
 *
 * FLUJO DE ARRANQUE:
 *   1. nimble_port_init()         → inicializar el controlador BT del ESP32.
 *   2. Registrar callbacks:
 *        ble_hs_cfg.sync_cb  = onNimBLESync  (llamado cuando el host está listo)
 *        ble_hs_cfg.reset_cb = onNimBLEReset (llamado tras reset inesperado)
 *   3. ble_svc_gap_device_name_set("Argus-Secure") → nombre en advertising.
 *   4. ble_svc_gap_init() + ble_svc_gatt_init()   → servicios estándar BLE.
 *   5. ble_gatts_count_cfg() + ble_gatts_add_svcs() → registrar tabla GATT.
 *   6. nimble_port_freertos_init(nimbleHostTask)   → lanzar tarea host NimBLE.
 *   7. vTaskSuspend(nullptr) → bleTask se suspende indefinidamente.
 *
 * MANEJO DE ERRORES:
 *   Si nimble_port_init() falla → vTaskDelete(nullptr) (sin BLE, el sistema
 *   sigue funcionando con sensorTask/controlTask/commTask).
 *   Si ble_gatts_count_cfg() o ble_gatts_add_svcs() falla → vTaskDelete(nullptr).
 *
 * NOTA: Después de vTaskSuspend(nullptr), bleTask puede reanudarse en el futuro
 *   con vTaskResume(xBleTaskHandle) para manejar lógica adicional de BLE
 *   (ej. notificaciones INDICATE hacia la app móvil).
 *
 * @param pvParameters No usado (nullptr). Requerido por la firma de tarea FreeRTOS.
 */
void bleTask(void* pvParameters);

/**
 * @brief Retorna true si hay un cliente BLE conectado actualmente.
 *
 * Thread-safe. Puede llamarse desde cualquier tarea FreeRTOS.
 */
bool bleIsConnected();

/**
 * @brief Envía NOTIFY BLE a la app con el estado ARM/DISARM actual.
 *
 * No hace nada si no hay cliente conectado o no habilitó NOTIFY.
 * Llamar desde control_task tras procesar EVENT_ARM_CMD / EVENT_DISARM_CMD.
 *
 * @param armed true = sistema armado (0x01), false = desarmado (0x00).
 */
void bleNotifyState(bool armed);


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/include/ble_task.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   ble_task.h declara la tarea que "escucha" comandos del teléfono del dueño
   via Bluetooth Low Energy. Cuando el dueño toca "Armar" en la app Argus,
   el teléfono envía el byte 0x01 a la característica BLE, que se traduce en
   el evento ARM_CMD y lo pone en la cola de eventos para que controlTask lo
   procese (activando el modo de vigilancia).

   La tarea solo inicializa NimBLE y luego se suspende. El trabajo real lo
   hace nimbleHostTask (una tarea interna de NimBLE) que corre en segundo plano.

   MAPA DE COMANDOS BLE:
   Byte | Acción
   -----|-------
   0x01 | ARM_CMD        → Sistema armado (vigilancia activa)
   0x02 | DISARM_CMD     → Sistema desarmado
   0x03 | TRIGGER_ALERT  → Dispara alerta manual de emergencia
   0x10 | Sensibilidad Muy Baja  (mucho movimiento para alarmar)
   0x11 | Sensibilidad Baja
   0x12 | Sensibilidad Media     (default)
   0x13 | Sensibilidad Alta
   0x14 | Sensibilidad Muy Alta  (mínimo movimiento para alarmar)

   ═══════════════════════════════════════════════════════════ */
