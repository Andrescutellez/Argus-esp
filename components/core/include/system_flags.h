#pragma once

/**
 * @file system_flags.h
 * @brief Variables de estado global compartidas entre todas las tareas del sistema Argus.
 *
 * PROPÓSITO:
 *   Centralizar los flags y variables de estado que necesitan ser visibles desde
 *   múltiples tareas FreeRTOS. Son el "estado del mundo" del sistema: qué está
 *   haciendo la moto ahora mismo, si está armada, qué plan tiene, etc.
 *
 * REGLA DE ESCRITURA:
 *   Solo las tareas autorizadas deben escribir cada variable:
 *   - systemArmed, remoteAlert, currentSystemState → StateMachine (dentro de control_task)
 *   - lastMovementTimestamp → sensorTask (cada vez que detecta movimiento)
 *   - planType → commTask al arranque (leído desde NVS o configuración)
 *   - motionSensitivity → bleTask o commTask (al recibir comando de sensibilidad)
 *   Violar esta regla puede causar estados inconsistentes sin errores de compilación.
 *
 * PROTECCIÓN DE CONCURRENCIA:
 *   - currentSystemState: protegida por xStateMutex cuando se lee/escribe desde
 *     múltiples tareas. Sin el mutex, una transición de estado puede ser vista
 *     parcialmente por otra tarea.
 *   - systemArmed, remoteAlert: volatiles simples. La lectura de un bool es
 *     atómica en ARM Cortex-M; no hay riesgo de leer un valor parcialmente escrito.
 *     Sin embargo, si la lógica de negocio depende de leer AMBAS variables de
 *     forma consistente (ej: "si armado y remoteAlert"), hay un window entre
 *     las dos lecturas donde el estado puede cambiar.
 *   - lastMovementTimestamp: volatile uint64_t. En ARM Cortex-M, las escrituras
 *     de 64 bits NO son atómicas (requieren 2 instrucciones). Si sensorTask
 *     escribe mientras commTask lee, commTask puede leer un valor parcial
 *     (los 32 bits bajos del nuevo y los 32 bits altos del viejo).
 *     ARQUITECTURA ⚠️: lastMovementTimestamp no está protegida contra lectura
 *     parcial de 64 bits. En la práctica, la diferencia de tiempo que se
 *     compara con ella (> 5 minutos) es tan grande que un valor parcialmente
 *     corrupto no causaría un error lógico observado.
 *
 * VARIABLES CRÍTICAS:
 *   - systemArmed: si es false, el sensor puede detectar movimiento pero la
 *     máquina de estados lo ignorará. false = moto desprotegida.
 *   - currentSystemState: fuente de verdad del estado de la moto. Si se corrompe,
 *     la lógica de timers, actuadores y telemetría falla de forma impredecible.
 *   - motionSensitivity: determina qué tan fácil es disparar una alerta.
 *     SENSITIVITY_VERY_HIGH en zona urbana causará muchas falsas alarmas.
 *
 * @module components/core/system_flags
 */

#include <stdbool.h>
#include <stdint.h>
#include "system_events.h"

// true cuando el sistema está armado y vigilando la motocicleta.
// false: sensor_task detecta movimiento pero control_task lo ignora.
// Escrito por: StateMachine.handleIdle() al recibir EVENT_ARM_CMD / EVENT_DISARM_CMD.
extern volatile bool systemArmed;

// true cuando hay una alerta activa disparada remotamente por el operador o backend.
// Diferencia de systemArmed: permite que el operador active ALERT sin necesidad de
// movimiento físico real en la moto.
// Escrito por: StateMachine al recibir EVENT_TRIGGER_ALERT_CMD o EVENT_PURSUIT_CONFIRM.
extern volatile bool remoteAlert;

// true cuando el sistema está en modo geocerca de estacionamiento.
// Efecto: STATE_ALERT NO activa el buzzer — solo envía EVENT al backend.
// El backend evalúa si la moto salió de la geocerca y, de ser así, envía CMD|PARK_MODE_OFF
// que limpia este flag y activa la alarma completa.
// Se limpia también al recibir CMD|DISARM.
// Escrito por: commTask (CMD|PARK_MODE_ON / CMD|PARK_MODE_OFF / CMD|DISARM).
// Leído por: control_task (applyStateEffects + tick de buzzer).
extern volatile bool flagParkMode;

// true cuando se recibió CMD|ENGINE_CUT pero la moto estaba en movimiento.
// Propósito: evitar cortar el motor mientras la moto rueda (riesgo de accidente mortal).
// control_task verifica en cada tick (250ms): si (now - lastHardMovementTimestamp) ≥ 3s
// → ejecuta el corte vía s_motorManualCut + setEngineCut(true) y limpia este flag.
// Se limpia también al recibir EVENT_ENGINE_RESTORE o EVENT_DISARM_CMD.
// Escrito por: commTask (CMD|ENGINE_CUT) y controlTask (al ejecutar o al DISARM/RESTORE).
// Leído por: controlTask en cada tick del loop principal (cada 250ms).
extern volatile bool flagEngineCutPending;

// Estado actual de la máquina de estados (SystemState_t).
// Protegido por xStateMutex cuando se lee desde tareas que necesitan consistencia.
// Escrito por: StateMachine.transitionTo() exclusivamente.
// Leído por: commTask (para decidir intervalo de telemetría), sensorTask (para decidir
// si enviar eventos), applyLedPattern() (para decidir patrón de LEDs).
extern volatile SystemState_t currentSystemState;

// Timestamp en microsegundos (esp_timer_get_time()) del último movimiento o impacto
// detectado por sensor_task. 0 = nunca detectado.
// Inicializado por commTask al arranque a esp_timer_get_time() para que la telemetría
// comience activa sin necesitar movimiento previo.
// Escrito por: sensorTask cuando detectedEvent != EVENT_NONE (SOFT, HARD o IMPACT).
// Leído por: commTask (auto-ARM), control_task (auto-ARM), sensorTask (driveMetrics).
// RIESGO: escritura de uint64_t no es atómica en ARM — ver nota de concurrencia arriba.
extern volatile uint64_t lastMovementTimestamp;

// Timestamp del último evento DURO (MOVEMENT_HARD o IMPACT_DETECTED).
// A diferencia de lastMovementTimestamp, la vibración ambiental (MOVEMENT_SOFT
// generada por tráfico o ISR) NO resetea esta variable.
// Propósito exclusivo: gate de GPS sleep en commTask.
// Lógica: si la moto lleva >5 min sin movimiento duro → pausar telemetría GPS.
// Vibraciones de tráfico no deben impedir el sleep en moto estacionada en ciudad.
// RIESGO: misma falta de atomicidad que lastMovementTimestamp (uint64_t en ARM).
extern volatile uint64_t lastHardMovementTimestamp;

// Plan activo del dispositivo. Determina intervalos de telemetría y features.
// PLAN_FREEMIUM: heartbeat cada 1h, GPS solo en alertas.
// PLAN_PREMIUM:  telemetría cada 30s, GPS continuo en ALERT/PURSUIT.
// Escrito por: commTask al arranque. No cambia en runtime en la implementación actual.
extern volatile PlanType_t planType;

/**
 * @brief Nivel de sensibilidad de la detección de movimiento.
 *
 * PROPÓSITO:
 *   Controlar qué tan sensible es el sistema a los movimientos de la moto.
 *   Afecta los umbrales de clasificación de movimiento en sensor_task.cpp
 *   (tabla SENSITIVITY_TABLE) y se persiste en NVS para sobrevivir reboots.
 *
 * NIVELES Y UMBRALES (accel soft/hard, gyro soft/hard):
 *   VERY_LOW  (0): >0.25g / >0.70g  |  >15°/s  / >60°/s
 *     → Para motos en tráfico pesado. Requiere movimiento muy marcado.
 *   LOW       (1): >0.15g / >0.50g  |  >10°/s  / >40°/s
 *     → Zona semi-urbana. Movimiento claro requerido.
 *   MEDIUM    (2): >0.05g / >0.30g  |  >3°/s   / >20°/s  (DEFAULT)
 *     → Balance entre sensibilidad y falsas alarmas.
 *   HIGH      (3): >0.02g / >0.15g  |  >1°/s   / >10°/s
 *     → Zona tranquila. Detecta vibración leve (mano sobre el manillar).
 *   VERY_HIGH (4): >0.01g / >0.08g  |  >0.5°/s / >5°/s
 *     → Máxima vigilancia. Cualquier micro-movimiento dispara alarma.
 *     ADVERTENCIA: usarlo en zona urbana causará múltiples falsas alarmas.
 *
 * PERSISTENCIA:
 *   NVS namespace "argus_cfg", key "sensitivity" (uint8_t).
 *   Cargado en sensorTask al arrancar. Escrito por setSensitivityLevel().
 *
 * COMANDOS BLE:
 *   BLE byte 0x10 → VERY_LOW
 *   BLE byte 0x11 → LOW
 *   BLE byte 0x12 → MEDIUM
 *   BLE byte 0x13 → HIGH
 *   BLE byte 0x14 → VERY_HIGH
 *
 * COMANDOS TCP (desde backend / operador):
 *   "SENSITIVITY_VERY_LOW" / "LOW" / "MEDIUM" / "HIGH" / "VERY_HIGH"
 */
typedef enum {
    SENSITIVITY_VERY_LOW  = 0,
    SENSITIVITY_LOW       = 1,
    SENSITIVITY_MEDIUM    = 2,  // Default al arranque si no hay valor en NVS
    SENSITIVITY_HIGH      = 3,
    SENSITIVITY_VERY_HIGH = 4,
} SensitivityLevel_t;

// Nivel de sensibilidad activo. Escrito por setSensitivityLevel() (sensorTask.cpp).
// Leído por classifyMovement() en cada detección de movimiento.
// La variable es volatile porque sensorTask y bleTask pueden acceder desde
// tareas diferentes.
extern volatile SensitivityLevel_t motionSensitivity;

// ─── Auto-ARM por inactividad (solo PLAN_PREMIUM) ────────────────────────────
//
// Cuando autoArmEnabled=true y el sistema lleva >autoArmDelayMs sin movimiento,
// controlTask inyecta EVENT_ARM_CMD automáticamente.
// Escrito por: bleTask (comandos BLE 0x20/0x21).
// Leído por: controlTask en cada ciclo del loop.
//
// autoArmDelayMs: delay en milisegundos antes de armar (default=300000 = 5 min).
// DEUDA: Debería persistirse en NVS ("argus_cfg"/"auto_arm_min").
extern volatile bool     autoArmEnabled;
extern volatile uint32_t autoArmDelayMs;

// ─── Métricas de conducción ───────────────────────────────────────────────────
//
// Acumuladas por sensorTask en cada lectura del MPU6050, independientemente
// de si el sistema está armado o no. commTask las lee y resetea al enviar
// cada frame DRIVE junto con el GPS periódico.
//
// peakAccelDev: pico de desviación de la magnitud de aceleración respecto a 1g,
//   en unidades g. En reposo = 0. Frenada fuerte = ~0.5g. Sirve como proxy de
//   agresividad de conducción.
// peakGyroMag: pico de la magnitud del vector giroscopio en °/s. Giro brusco
//   de manillar puede superar 50°/s. Proxy de agresividad en curvas.
// hardCount: número de eventos EVENT_MOVEMENT_HARD o EVENT_IMPACT_DETECTED
//   detectados desde el último reset. Proxy de maniobras bruscas.
// softCount: número de eventos EVENT_MOVEMENT_SOFT desde el último reset.
//
// CONCURRENCIA: mismo riesgo que lastMovementTimestamp — float de 32 bits
//   es atómico en ARM para lecturas simples, pero hardCount/softCount son
//   uint16_t incrementados con ++ que no es atómico. En la práctica, el
//   tamaño de la ventana (30s) hace que una corrupción de un contador
//   sea imperceptible en el score de conducción.
typedef struct {
    volatile float    peakAccelDev;
    volatile float    peakGyroMag;
    volatile uint16_t hardCount;
    volatile uint16_t softCount;
} DriveMetrics_t;

// Instancia global. Definida en sensor_task.cpp (escritor principal).
// commTask la lee y resetea con memset(&g_driveMetrics, 0, sizeof(g_driveMetrics))
// tras enviar el frame DRIVE.
extern DriveMetrics_t g_driveMetrics;


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/core/include/system_flags.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo es como un "tablero de estado" compartido entre todos los
   módulos del firmware. Cualquier parte del código puede leer estas variables
   para saber qué está pasando ahora mismo en el sistema: ¿está la moto armada?
   ¿está en modo persecución? ¿cuándo fue el último movimiento? ¿qué tan sensible
   es el sensor?

   A diferencia de los handles de FreeRTOS (colas, mutexes, semáforos), estas
   variables son accesibles directamente. La palabra volatile le dice al compilador
   que no optimice las lecturas porque otro hilo puede cambiarlas en cualquier momento.

   TABLA DE VARIABLES:
   Variable            | Tipo                 | Escribe          | Lee
   --------------------|----------------------|------------------|------------------
   systemArmed         | volatile bool        | StateMachine     | sensor/mosfet/comm
   remoteAlert         | volatile bool        | StateMachine     | control/mosfet
   currentSystemState  | volatile SystemState_t| StateMachine    | todos
   lastMovementTimestamp| volatile uint64_t   | sensorTask       | commTask
   planType            | volatile PlanType_t  | commTask (boot)  | commTask
   motionSensitivity   | volatile SensitivityLevel_t| bleTask/commTask| sensorTask

   MAPA DE SENSIBILIDAD BLE ↔ NVS ↔ firmware:
   App móvil (BLE)   → byte 0x10-0x14
   bleTask           → setSensitivityLevel(0-4)
   NVS "argus_cfg/sensitivity" ← persiste el valor
   sensorTask        → classifyMovement() usa SENSITIVITY_TABLE[motionSensitivity]

   DEUDA TÉCNICA:
   1. lastMovementTimestamp es uint64_t volatile pero su escritura no es atómica
      en ARM (dos instrucciones). Debería protegerse con un mutex o usar un
      valor de 32 bits que sí sea atómico.
   2. No hay API tipo "setSystemState()" que tome el mutex automáticamente:
      los callers deben recordar tomar xStateMutex manualmente.
   3. planType no cambia en runtime — podría ser una constante (const en .h,
      definida según el dispositivo) en lugar de una variable global mutable.

   ═══════════════════════════════════════════════════════════ */
