#pragma once

/**
 * @file system_events.h
 * @brief Tipos de datos compartidos, eventos, estados y handles de comunicación
 *        inter-tarea para el firmware Argus Secure.
 *
 * PROPÓSITO:
 *   Ser el contrato entre todas las tareas del sistema: define los tipos que cruzan
 *   límites de tarea vía colas FreeRTOS (EventMessage_t, GpsData_t, SensorData_t),
 *   los estados posibles de la máquina de estados (SystemState_t), y los handles
 *   de las primitivas de comunicación inter-tarea.
 *
 *   Este archivo lo incluyen TODOS los módulos. Cualquier cambio aquí recompila
 *   todo el proyecto. Mantenerlo estable y coherente es crítico.
 *
 * ARQUITECTURA DE COMUNICACIÓN INTER-TAREA:
 *
 *   sensorTask  ──EVENT_MOVEMENT/IMPACT──►  xEventQueue  ──► controlTask
 *   bleTask     ──EVENT_ARM/DISARM/ALERT──►  xEventQueue  ──► controlTask
 *   commTask    ──EVENT_GPS_FIX──────────►  xEventQueue  ──► controlTask
 *   commTask    ──GpsData_t──────────────►  xGpsDataQueue (cap. 1, overwrite)
 *
 *   xSystemFlags (EventGroup): flags de estado consultables sin bloqueo.
 *   xStateMutex: protege lecturas/escrituras atómicas de currentSystemState.
 *   xModemMutex: serializa todas las sesiones AT del módulo A7670.
 *
 * ESTADOS vs MODOS vs FLAGS:
 *   - Estado (SystemState_t): "qué le está pasando a la moto" — cambia por eventos.
 *     IDLE → MOVING → ALERT → PURSUIT (transiciones explícitas, no arbitrarias).
 *   - Modo (SystemMode_t): "cómo se comporta el sistema" — derivado del estado + flags.
 *     No se almacena, se calcula cuando se necesita (StateMachine::currentMode()).
 *   - Flags (xSystemFlags EventGroup): bits de estado consultables sin bloqueo.
 *     Permiten que una tarea verifique rápidamente "¿está el sistema armado?"
 *     sin tomar el mutex. Deben sincronizarse con systemArmed (system_flags.h).
 *
 * VARIABLES CRÍTICAS:
 *   - xEventQueue: si se llena (capacidad 10), los eventos de sensor se descartan.
 *     Cada evento descartado es un movimiento potencial que el sistema no procesa.
 *   - xGpsDataQueue: capacidad 1. xQueueOverwrite() garantiza que siempre tenga
 *     el dato GPS más reciente, descartando el anterior si no fue leído.
 *   - xModemMutex: mutex RECURSIVO para que A7670Driver pueda hacer lock anidado
 *     dentro de sus propios métodos privados sin deadlock.
 *
 * CONCURRENCIA:
 *   - EventMessage_t y GpsData_t son copiadas por valor en las colas FreeRTOS.
 *     No hay punteros que cruzar entre tareas — eliminando dangling pointer risks.
 *   - La union en EventMessage_t.data (sensor | gps) ahorra memoria pero significa
 *     que solo el field correcto está inicializado. El receiver debe saber por
 *     el campo event cuál field leer.
 *
 * @module components/core/system_events
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

// ─── Estados del sistema ──────────────────────────────────────────────────────
//
// Los cuatro estados posibles de la moto. Solo StateMachine puede transicionar
// entre ellos. No hay transiciones implícitas ni automáticas, excepto por timeouts
// (EVENT_MOVING_TIMEOUT, EVENT_ALERT_TIMEOUT) que dispara control_task.
//
// Flujo principal (con systemArmed=true):
//   IDLE → MOVING → ALERT → PURSUIT
//          ↑           ↓
//      timeout      timeout (sin confirmación)
//
// Salida desde cualquier estado: EVENT_DISARM_CMD → IDLE

typedef enum {
    // La moto está quieta. Con systemArmed=true → modo vigilancia activa.
    // Con systemArmed=false → sistema pasivo (solo telemetría si PREMIUM).
    // Timer: ninguno. Puede permanecer indefinidamente.
    STATE_IDLE    = 0,

    // Se detectó movimiento pero no está confirmado como amenaza.
    // Fase de filtrado para distinguir ruido/viento de manipulación real.
    // Timer en control_task: 15s. Si expira → EVENT_MOVING_TIMEOUT → IDLE.
    // Si llega MOVEMENT_HARD o IMPACT → escala a ALERT.
    STATE_MOVING  = 1,

    // Movimiento confirmado como sospechoso. Buzzer activo. Backend notificado.
    // NO hay escalada automática a PURSUIT: requiere confirmación explícita.
    // Timer en control_task: 30s de quietud → EVENT_ALERT_TIMEOUT → IDLE (armado).
    STATE_ALERT   = 2,

    // Robo confirmado por usuario/operador. Máxima intensidad de respuesta.
    // Motor cortado. GPS + 4G continuos. Solo EVENT_DISARM_CMD puede salir.
    // Sin timer: permanece hasta intervención manual.
    STATE_PURSUIT = 3,
} SystemState_t;

// ─── Modos operativos ─────────────────────────────────────────────────────────
//
// El modo se DERIVA del estado + flags (StateMachine::currentMode()).
// No se almacena en ninguna variable; se calcula en tiempo de ejecución.
// El modo determina el comportamiento de comm_task (intervalos de telemetría)
// y de applyLedPattern() en mosfet_control.

typedef enum {
    // Bajo consumo, vigilancia mínima. STATE_IDLE + systemArmed=false.
    MODE_IDLE       = 0,

    // Vigilancia activa. STATE_IDLE + systemArmed=true, o STATE_MOVING.
    // Telemetría: 30s (PREMIUM) / 1h (FREEMIUM).
    MODE_MONITORING = 1,

    // Reacción activa. STATE_ALERT. Comunicaciones + alarma activas.
    // Telemetría: 15s independientemente del plan.
    MODE_ALERT      = 2,

    // Seguimiento agresivo. STATE_PURSUIT. GPS + 4G continuos + corte de motor.
    // Telemetría: 10s independientemente del plan.
    MODE_PURSUIT    = 3,
} SystemMode_t;

// ─── Plan del dispositivo ─────────────────────────────────────────────────────
//
// El plan determina los intervalos de telemetría en STATE_IDLE/MOVING.
// En ALERT y PURSUIT, el plan es ignorado (emergencia tiene prioridad).
// El plan no cambia en runtime en la implementación actual — es una constante
// del dispositivo configurada en commTask al arranque.

typedef enum {
    // Heartbeat cada 1h. Sin GPS continuo. 3 wakes por día máximo.
    // Para reducir costos de datos en dispositivos con SIM básica.
    PLAN_FREEMIUM = 0,

    // Telemetría GPS cada 30s en modo normal. 10-15s en emergencias.
    // 4G continuo en PURSUIT. Features avanzadas habilitadas.
    PLAN_PREMIUM  = 1,
} PlanType_t;

// ─── Eventos del sistema ──────────────────────────────────────────────────────
//
// Un evento es la única manera legítima de cambiar el estado de la máquina
// de estados. Los eventos viajan en EventMessage_t a través de xEventQueue.
// Ninguna tarea puede cambiar directamente currentSystemState sin pasar
// por StateMachine.processEvent() en control_task.

typedef enum {
    EVENT_NONE = 0,

    // ── Eventos de sensor (sensorTask → xEventQueue → controlTask) ──────────
    // Generados cuando classifyMovement() clasifica datos del MPU6050.

    // Vibración leve, puede ser ruido o viento. Desencadena IDLE→MOVING si armado.
    // En MOVING: reinicia el timer de 15s. En ALERT: reinicia el timer de 30s.
    EVENT_MOVEMENT_SOFT,

    // Movimiento brusco, probable manipulación de la moto. Escala MOVING→ALERT.
    // Umbral: |accel - 1g| > accelHard según SENSITIVITY_TABLE.
    EVENT_MOVEMENT_HARD,

    // Impacto de alta aceleración detectado por el hardware del MPU6050.
    // Produce escalada directa a ALERT desde cualquier estado (incluso IDLE).
    // Umbral: magnitud del vector de aceleración > MPU6050_IMPACT_G_THRESHOLD (2.5g).
    EVENT_IMPACT_DETECTED,

    // ── Timers internos de fase (generados por callbacks de esp_timer) ───────

    // 15 segundos en STATE_MOVING sin escalada → probable falsa alarma → volver a IDLE.
    // Generado por movingTimerCallback() en control_task.cpp.
    EVENT_MOVING_TIMEOUT,

    // ── GPS (commTask → xEventQueue → controlTask) ───────────────────────────

    // GPS adquirió posición válida (gps.fix_valid=true tras haberlo perdido).
    // Lleva GpsData_t en msg.data.gps.
    EVENT_GPS_FIX_OK,

    // GPS perdió señal o fix (gps.fix_valid=false tras haberlo tenido).
    // controlTask podría usar este evento para pausar telemetría por GPS (no implementado).
    EVENT_GPS_FIX_LOST,

    // ── Comandos del usuario/operador (BLE o backend TCP) ────────────────────

    // Usuario armó el sistema via BLE (0x01) o comando TCP "ARM".
    // En StateMachine: IDLE + systemArmed=false → systemArmed=true.
    EVENT_ARM_CMD,

    // Usuario desarmó el sistema via BLE (0x02) o comando TCP "DISARM".
    // Cancela cualquier estado activo → STATE_IDLE + apaga buzzer + desactiva ENGINE_CUT.
    EVENT_DISARM_CMD,

    // Operador forzó ALERT manualmente via BLE (0x03) o TCP "ALERT".
    // Permite activar la alarma sin necesitar movimiento físico real.
    EVENT_TRIGGER_ALERT_CMD,

    // Usuario/operador confirmó robo → activar STATE_PURSUIT.
    // NEVER se genera automáticamente: requiere confirmación explícita humana.
    // Generado por: comm_task al recibir CMD|ENGINE_CUT o CMD|PURSUIT_CONFIRM.
    EVENT_PURSUIT_CONFIRM,

    // Operador restaura el motor sin desarmar el sistema.
    // STATE_PURSUIT → STATE_IDLE (motor libre, sistema sigue armado).
    // Generado por: comm_task al recibir CMD|ENGINE_RESTORE.
    // Solo válido en STATE_PURSUIT. En otros estados es ignorado.
    EVENT_ENGINE_RESTORE,

    // ── Timers de estado ─────────────────────────────────────────────────────

    // 30 segundos en STATE_ALERT sin confirmación ni movimiento adicional.
    // El operador no respondió → probable falsa alarma → volver a IDLE (armado).
    // Generado por alertTimerCallback() en control_task.cpp.
    EVENT_ALERT_TIMEOUT,

    // ── Comunicaciones (commTask → xEventQueue → controlTask) ────────────────
    // Generados por commTask cuando SyncManager reporta éxito o fallo acumulado.

    // Reporte enviado correctamente al backend. controlTask podría usar esto para
    // indicar conectividad (no implementado actualmente en la máquina de estados).
    EVENT_COMMS_SUCCESS,

    // Múltiples fallos de envío (consecutiveFailures > 2). controlTask podría
    // usar esto para mostrar indicación de falta de conectividad (no implementado).
    EVENT_COMMS_FAILURE,
} SystemEvent_t;

// ─── Datos del sensor MPU6050 ─────────────────────────────────────────────────
//
// Llenada por MPU6050Driver::readSensorData() y enviada en EventMessage_t.data.sensor
// cuando sensorTask detecta y clasifica un evento de movimiento.
//
// Unidades: accel en g (1g = 9.81 m/s²), gyro en °/s.
// En reposo con el sensor horizontal: accel_z ≈ 1g, accel_x/y ≈ 0g.

typedef struct {
    float accel_x;          // Aceleración eje X en g (positivo = hacia un lado del chip)
    float accel_y;          // Aceleración eje Y en g
    float accel_z;          // Aceleración eje Z en g (≈1g en reposo por gravedad)
    float gyro_x;           // Velocidad angular eje X en °/s
    float gyro_y;           // Velocidad angular eje Y en °/s
    float gyro_z;           // Velocidad angular eje Z en °/s
    bool  motion_detected;  // true si el registro hardware INT_STATUS del MPU6050 indica movimiento
    bool  impact_detected;  // true si sqrt(ax²+ay²+az²) > MPU6050_IMPACT_G_THRESHOLD (2.5g)
} SensorData_t;

// ─── Datos de localización GPS ────────────────────────────────────────────────
//
// Llenada por A7670Driver::getGpsPosition() vía parseGNSINF().
// Se envía a xGpsDataQueue (capacidad 1, overwrite) y en EventMessage_t.data.gps
// cuando hay cambio de estado del fix (fix_ok / fix_lost).
//
// Campos de navegación en unidades SI directamente utilizables:
// latitude/longitude en grados decimales, speed_kmh en km/h, altitude_m en metros.

typedef struct {
    float    latitude;       // Latitud WGS-84 en grados decimales (+ = Norte, - = Sur)
    float    longitude;      // Longitud WGS-84 en grados decimales (+ = Este, - = Oeste)
    float    speed_kmh;      // Velocidad sobre el suelo en km/h (Ground Speed)
    float    altitude_m;     // Altitud sobre el nivel del mar en metros (MSL)
    uint8_t  satellites;     // Número de satélites utilizados en la solución de navegación
    bool     fix_valid;      // true si los datos son confiables (HDOP razonable, min satélites)
    char     timestamp[24];  // Fecha y hora UTC del fix GPS: "YYYYMMDD HHMMSS.mmm"
} GpsData_t;

// ─── Mensaje de evento para xEventQueue ──────────────────────────────────────
//
// Tipo de datos que viaja a través de xEventQueue desde sensorTask, commTask,
// y bleTask hacia controlTask. Tamaño: sizeof(SystemEvent_t) + sizeof(union).
//
// La union data ahorra memoria: un EventMessage_t no necesita espacio para
// GpsData_t cuando es un evento de sensor y viceversa. El campo event indica
// cuál union member leer (convención, no enforceada en compilación).
//
// IMPORTANTE: solo llenar el union member correspondiente al tipo de evento.
// El otro member contiene basura (datos no inicializados). El receiver
// decide qué leer basándose en msg.event.

typedef struct {
    SystemEvent_t event;
    union {
        SensorData_t sensor;  // Válido para: EVENT_MOVEMENT_SOFT/HARD, EVENT_IMPACT_DETECTED
        GpsData_t    gps;     // Válido para: EVENT_GPS_FIX_OK, EVENT_GPS_FIX_LOST
    } data;
} EventMessage_t;

// ─── Handles globales de comunicación inter-tarea ────────────────────────────
//
// DEFINIDOS en main.cpp (una sola definición en todo el proyecto).
// DECLARADOS como extern aquí para que cualquier módulo pueda usarlos.
// Inicializados a nullptr — assertHandle() en main.cpp verifica que no lo sean.
//
// NO usar estos handles antes de que main.cpp los inicialice (durante el arranque).
// Todas las tareas arrancan después de que main.cpp los crea, así que en práctica
// esto no es un problema en el flujo de arranque normal.

extern QueueHandle_t      xEventQueue;    // Cola de eventos (cap. 10) → controlTask
extern QueueHandle_t      xGpsDataQueue;  // Dato GPS más reciente (cap. 1, overwrite)
extern EventGroupHandle_t xSystemFlags;   // Flags de estado global (ver bits abajo)
extern SemaphoreHandle_t  xStateMutex;    // Mutex para currentSystemState
extern SemaphoreHandle_t  xModemMutex;    // Mutex RECURSIVO para el módem A7670

// ─── Bits del EventGroup xSystemFlags ────────────────────────────────────────
//
// El EventGroup permite que las tareas esperen o verifiquen flags de estado sin bloqueo.
// Uso: xEventGroupGetBits(xSystemFlags) & FLAG_SYSTEM_ARMED para verificar.
// Uso: xEventGroupWaitBits(xSystemFlags, FLAG_GPS_VALID, pdFALSE, pdTRUE, timeout) para esperar.
//
// Los flags deben mantenerse sincronizados con las variables en system_flags.h:
// FLAG_SYSTEM_ARMED debe ser coherente con systemArmed (bool), etc.

#define FLAG_SYSTEM_ARMED    (1 << 0)  // Sistema armado (sync con systemArmed)
#define FLAG_REMOTE_ALERT    (1 << 1)  // Alerta disparada remotamente (sync con remoteAlert)
#define FLAG_GPS_VALID       (1 << 2)  // GPS tiene fix válido actualmente
#define FLAG_COMMS_BUSY      (1 << 3)  // Módulo A7670 ocupado enviando datos
#define FLAG_SENSOR_MOTION   (1 << 4)  // Movimiento activo en este instante (pulso, 200ms)
#define FLAG_HIGH_FREQ_MODE  (1 << 5)  // Telemetría de alta frecuencia activa (ALERT/PURSUIT)


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/core/include/system_events.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo define el "idioma" que usan todas las tareas del firmware para
   comunicarse entre sí. Define:
   1. Los estados posibles de la moto (IDLE/MOVING/ALERT/PURSUIT)
   2. Los mensajes que las tareas se envían entre sí (EVENT_*)
   3. La estructura de los datos de sensor (SensorData_t) y GPS (GpsData_t)
   4. Los canales de comunicación inter-tarea (handles de FreeRTOS)

   Es el "contrato" del sistema: si se cambia algo aquí, todos los módulos se
   ven afectados y deben ser revisados.

   DIAGRAMA DE FLUJO DE EVENTOS:
   sensorTask detects motion
     ↓ classifyMovement() → EVENT_MOVEMENT_SOFT/HARD/IMPACT
     ↓ xQueueSend(xEventQueue, EventMessage_t)
   controlTask receives
     ↓ stateMachine.processEvent(msg)
     ↓ transition: IDLE → MOVING (if systemArmed)
     ↓ applyStateEffects() → MOSFETs, LEDs, timers
   movingTimer expires (15s)
     ↓ movingTimerCallback() → EVENT_MOVING_TIMEOUT → xEventQueue
   controlTask receives
     ↓ stateMachine.processEvent() → MOVING → IDLE

   TAMAÑOS DE LOS TIPOS (referencia para dimensionar colas):
   - EventMessage_t: sizeof(SystemEvent_t) + sizeof(union max) ≈ 4 + max(≈36, ≈44) ≈ 48 bytes
   - GpsData_t: float×4 + uint8_t + bool + char[24] ≈ 44 bytes
   - SensorData_t: float×6 + bool×2 ≈ 26 bytes

   CAPACIDADES DE COLA:
   - xEventQueue: cap. 10 × 48 bytes = 480 bytes en stack FreeRTOS
   - xGpsDataQueue: cap. 1 × 44 bytes = 44 bytes (siempre tiene el más reciente)

   DEUDA TÉCNICA:
   1. EVENT_PURSUIT_CONFIRM no tiene flujo de generación implementado:
      ¿quién confirma el robo? ¿BLE? ¿backend TCP? No está definido.
   2. EVENT_COMMS_SUCCESS / EVENT_COMMS_FAILURE están en el enum pero no
      tienen handler en StateMachine (retorna false para ellos).
   3. La union data en EventMessage_t no tiene un tag de discriminación seguro
      (type-checked union). Se asume que el receiver conoce el tipo por event.

   ═══════════════════════════════════════════════════════════ */
