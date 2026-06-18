#pragma once

/**
 * @file control_task.h
 * @brief Interfaz de la tarea FreeRTOS del controlador central del sistema Argus.
 *
 * PROPÓSITO:
 *   Declarar la tarea de control y su handle para que main.cpp pueda crear la
 *   tarea y, si fuera necesario, otras tareas puedan suspenderla o notificarla.
 *
 * ROL EN LA ARQUITECTURA:
 *   controlTask es el "árbitro de efectos físicos" del sistema. Recibe todos los
 *   eventos de xEventQueue, los pasa a StateMachine (que decide el estado) y luego
 *   aplica las consecuencias físicas reales: activar buzzer, cortar motor, gestionar
 *   timers de timeout, y controlar los patrones de LEDs.
 *
 *   Jerarquía de tareas (mayor → menor prioridad):
 *     sensorTask  (prio 5) → detecta movimiento
 *     controlTask (prio 4) → decide y actúa   ← ESTE ARCHIVO
 *     commTask    (prio 3) → reporta telemetría
 *     bleTask     (prio 2) → escucha comandos BLE
 *
 *   Justificación de prioridad 4:
 *     Debe responder a eventos más rápido que comm y BLE (que pueden estar
 *     bloqueados en I/O largo), pero cede ante sensorTask para no retrasar
 *     la detección de movimiento.
 *
 * RESPONSABILIDADES INTERNAS (no visibles en este .h):
 *   1. Instanciar StateMachine y llamar processEvent() por cada evento.
 *   2. Llamar applyStateEffects() cuando hay un cambio de estado.
 *   3. Gestionar dos timers ESP:
 *        movingTimer (15s) → EVENT_MOVING_TIMEOUT si no hay movimiento brusco
 *        alertTimer  (30s) → EVENT_ALERT_TIMEOUT si no hay confirmación de persecución
 *   4. Tick de LEDs cada 250ms llamando MosfetControl::applyLedPattern(ledTick).
 *   5. Patrón de beeps en ALERT suave: 2 beeps de 250ms cada 2s.
 *
 * EFECTOS FÍSICOS POR ESTADO:
 *   STATE_IDLE (desarmado)  → Buzzer OFF, Motor ON, LEDs: latido lento
 *   STATE_IDLE (armado)     → Buzzer OFF, Motor ON, LEDs: fijo (vigilando)
 *   STATE_MOVING            → Buzzer OFF, Motor ON, LEDs: parpadeo rápido, movingTimer activo
 *   STATE_ALERT (suave)     → Buzzer: patrón 2 beeps/2s, LEDs: alternado, alertTimer activo
 *   STATE_ALERT (fuerte)    → Buzzer ON continuo, LEDs: alternado, alertTimer activo
 *   STATE_PURSUIT           → Buzzer ON, Motor CORTADO, LEDs: sirena, sin timers
 *
 * STACK: 4096 bytes
 *   Incluye: StateMachine (datos en pila), esp_timer callbacks (registradas en heap),
 *   MosfetControl (static, no stack), xQueueReceive (datos EventMessage_t).
 *   4096 bytes es suficiente con margen razonable.
 *
 * CORE: 0 (default en ESP-IDF single-core scheduler)
 *
 * VARIABLES CRÍTICAS DEFINIDAS EN control_task.cpp:
 *   - movingTimer:  handle del timer de 15s para STATE_MOVING.
 *   - alertTimer:   handle del timer de 30s para STATE_ALERT.
 *   - alertIsHard:  bool que distingue entre ALERT suave (patrón de beeps)
 *                   y ALERT fuerte (sirena continua). Se evalúa en cada tick.
 *   - ledTick:      contador de ticks (250ms cada uno). Gobierna todos los
 *                   patrones visuales y de buzzer en el loop principal.
 *
 * CONCURRENCIA:
 *   - movingTimerCallback y alertTimerCallback se ejecutan en el timer task
 *     de ESP-IDF (prioridad configurable, normalmente 1). Usan xQueueSend con
 *     timeout 0 para enviar eventos sin bloquear. Si la cola está llena, el
 *     timeout se pierde silenciosamente — aceptable porque el sistema en STATE_IDLE
 *     (que sería el estado al que se transiciona) es el estado seguro por defecto.
 *   - alertIsHard es leído y escrito solo por controlTask → sin races.
 *   - ledTick es local a controlTask → sin races.
 *
 * @module components/services/control_task
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Handle de la tarea. Definido en control_task.cpp, inicializado por xTaskCreate() en main.cpp.
// Exportado para que otras tareas puedan suspender/reanudar controlTask si fuera necesario.
extern TaskHandle_t xControlTaskHandle;

/**
 * @brief Función principal de la tarea FreeRTOS del controlador central.
 *
 * PROPÓSITO:
 *   Actuar como orquestador de efectos físicos. Recibe eventos de xEventQueue,
 *   los delega a StateMachine, e implementa los cambios de hardware resultado
 *   de cada transición de estado. También gobierna los patrones de LEDs y buzzer
 *   mediante un tick de 250ms.
 *
 * FLUJO INTERNO:
 *   1. MosfetControl::init() — inicializar GPIOs de hardware.
 *   2. StateMachine::init()  — resetear flags y estado a IDLE desarmado.
 *   3. Loop con timeout de 250ms (LED_TICK_MS):
 *      a. xQueueReceive(xEventQueue, 250ms):
 *         - Si recibe evento: stateMachine.processEvent() → applyStateEffects().
 *         - Si MOVING + MOVEMENT_SOFT: reiniciar movingTimer.
 *         - Si ALERT + movimiento: reiniciar alertTimer.
 *      b. MosfetControl::applyLedPattern(ledTick) — actualizar patrón de LEDs.
 *      c. Si ALERT suave: setBuzzer((ledTick % 8) < 2) — patrón de beeps.
 *      d. ledTick++
 *
 * MANEJO DE ERRORES:
 *   - Si xQueueReceive no recibe nada (timeout 250ms): el tick de LEDs corre igual.
 *   - Si la cola está llena en los timer callbacks: el evento timeout se descarta
 *     (no es catastrófico — el sistema queda en el estado actual hasta el siguiente
 *     evento o hasta que el usuario desarme manualmente).
 *
 * NOTA DE DISEÑO — SEPARACIÓN DE RESPONSABILIDADES:
 *   StateMachine NO toca hardware. applyStateEffects() en este archivo ES
 *   la única función que activa buzzer, corta motor, y gestiona timers.
 *   Esto permite testear la lógica de estados sin hardware real.
 *
 * @param pvParameters No usado (nullptr). Requerido por la firma de tarea FreeRTOS.
 */
void controlTask(void* pvParameters);


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/include/control_task.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   control_task.h declara la tarea que "hace cosas" en el mundo físico.
   Si state_machine.h es el cerebro que decide, controlTask es el cuerpo
   que ejecuta: apaga el buzzer, prende los LEDs, corta el motor, inicia
   y detiene los timers de timeout.

   La tarea corre en un ciclo de 250ms (el "tick"). Si no llega ningún
   evento en ese tiempo, igual ejecuta el tick de LEDs y el patrón de beeps
   del buzzer. Esto garantiza que los efectos visuales y sonoros sean suaves
   y continuos, no abruptos.

   FUNCIONES PÚBLICAS:
   controlTask()  → tarea FreeRTOS (arrancada por main.cpp)

   DIAGRAMA DE FLUJO SIMPLIFICADO:
   xEventQueue → processEvent() → applyStateEffects()
                                        ↓
                              IDLE:  todo OFF
                              MOVING: movingTimer, LEDs rápidos
                              ALERT:  alertTimer, buzzer (beep o sirena)
                              PURSUIT: motor cortado, sirena, sin timers

   Cada 250ms (con o sin evento):
   MosfetControl::applyLedPattern(ledTick) + patrón buzzer

   ═══════════════════════════════════════════════════════════ */
