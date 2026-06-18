#pragma once

/**
 * @file state_machine.h
 * @brief Interfaz de la máquina de estados del sistema Argus Secure.
 *
 * PROPÓSITO:
 *   Definir la clase StateMachine que encapsula toda la lógica de transición
 *   de estados del sistema. Es la fuente de verdad sobre qué eventos pueden
 *   cambiar el estado del sistema y bajo qué condiciones.
 *
 * DISEÑO — SEPARACIÓN DE RESPONSABILIDADES:
 *   StateMachine es una clase PURA DE LÓGICA: NO toca ningún periférico,
 *   NO llama a MosfetControl, NO inicia timers, NO escribe en colas.
 *   Solo actualiza systemArmed, remoteAlert y currentSystemState, y
 *   retorna true/false para indicar si hubo un cambio relevante.
 *
 *   El caller (control_task) interpreta el valor de retorno y aplica
 *   los efectos físicos correspondientes mediante applyStateEffects().
 *   Esta separación permite testear la lógica de estados sin hardware.
 *
 * DIAGRAMA DE ESTADOS:
 *
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │                            IDLE                                  │
 *   │                     (systemArmed: bool)                         │
 *   └──────────────────────────────────────────────────────────────────┘
 *     │ ARM_CMD                         │ DISARM_CMD (desde cualquier estado)
 *     │ → systemArmed=true              │ → systemArmed=false → IDLE
 *     ▼
 *   IDLE (armado)
 *     │ MOVEMENT_SOFT/HARD (si armado) │ IMPACT (si armado)
 *     ▼                                │
 *   MOVING ◄────── reinicia timer ─────┘
 *     │ MOVING_TIMEOUT (15s sin escalar)
 *     ▼
 *   IDLE (armado)     ─── MOVEMENT_HARD / IMPACT ──►  ALERT
 *                                                        │
 *                                                        │ ALERT_TIMEOUT (30s sin confirm)
 *                                                        ▼
 *                                                      IDLE (armado)
 *                                                        │
 *                                                        │ PURSUIT_CONFIRM
 *                                                        ▼
 *                                                      PURSUIT
 *                                                        │
 *                                                        │ DISARM_CMD → IDLE
 *
 *   TRIGGER_ALERT_CMD puede llevar a ALERT desde IDLE o MOVING directamente.
 *   PURSUIT solo se activa por EVENT_PURSUIT_CONFIRM — nunca automáticamente.
 *
 * INVARIANTES:
 *   1. Solo StateMachine escribe currentSystemState (a través de transitionTo()).
 *   2. processEvent() es el único punto de entrada para cambios de estado.
 *   3. Ninguna transición ocurre sin un evento explícito en xEventQueue.
 *   4. PURSUIT solo se alcanza por confirmación explícita humana.
 *
 * VARIABLES CRÍTICAS:
 *   - currentState: copia local de la máquina de estados. Se sincroniza con
 *     currentSystemState (global) en transitionTo().
 *   - systemArmed (system_flags.h): si es false, todos los eventos de movimiento
 *     son ignorados en handleIdle().
 *
 * @module components/core/state_machine
 */

#include "system_events.h"

/**
 * @brief Máquina de estados del sistema Argus. Clase pura de lógica sin efectos físicos.
 *
 * PROPÓSITO:
 *   Encapsular las reglas de transición de estado. control_task crea una instancia
 *   estática de esta clase y la usa para procesar todos los eventos del sistema.
 *
 * ESTADO INTERNO:
 *   - currentState: el estado actual de la máquina (SystemState_t).
 *     Se mantiene en sync con currentSystemState (global) a través de transitionTo().
 *     Son dos copias del mismo valor para permitir que handleXxx() compare
 *     el estado anterior con el nuevo dentro de la misma transición.
 *
 * USO TÍPICO (dentro de control_task):
 *   StateMachine sm;
 *   sm.init();
 *   // En el loop:
 *   EventMessage_t msg;
 *   if (xQueueReceive(xEventQueue, &msg, timeout)) {
 *     bool changed = sm.processEvent(msg);
 *     if (changed) applyStateEffects(sm.getState(), ...);
 *   }
 */
class StateMachine {
public:
    StateMachine();

    /**
     * @brief Inicializa la máquina de estados al estado inicial del sistema.
     *
     * PROPÓSITO:
     *   Resetear todos los flags globales a su estado de arranque: STATE_IDLE,
     *   systemArmed=false, remoteAlert=false. Debe llamarse UNA sola vez al
     *   arrancar control_task, antes de comenzar a recibir eventos.
     *
     * EFECTOS:
     *   - currentState = STATE_IDLE
     *   - systemArmed = false (en system_flags.h)
     *   - remoteAlert = false
     *   - currentSystemState = STATE_IDLE
     */
    void init();

    /**
     * @brief Procesa un evento y ejecuta la transición de estado correspondiente.
     *
     * PROPÓSITO:
     *   Punto de entrada principal de la máquina de estados. Delega al handler
     *   del estado actual (handleIdle, handleMoving, handleAlert, handlePursuit)
     *   y retorna true si hubo un cambio relevante que requiere actualizar
     *   los efectos físicos en control_task.
     *
     * FLUJO LÓGICO:
     *   1. Despachar al handler del estado actual.
     *   2. El handler evalúa el evento y puede:
     *      a. Llamar transitionTo(newState) → actualiza currentState + currentSystemState.
     *      b. Actualizar flags (systemArmed, remoteAlert, xSystemFlags).
     *      c. Retornar true (cambio relevante) o false (evento ignorado).
     *   3. Retornar el resultado del handler al caller.
     *
     * VALOR DE RETORNO:
     *   true: hubo cambio de estado O cambio de flag crítico (ARM/DISARM/ALERT).
     *         El caller (control_task) debe llamar applyStateEffects().
     *   false: el evento fue ignorado o no requiere cambio de efectos físicos.
     *          El caller puede continuar sin hacer nada adicional.
     *
     * NOTA: EVENT_MOVEMENT_SOFT en STATE_MOVING retorna true para indicar que
     *   control_task debe reiniciar el timer de 15s, aunque no haya cambio de estado.
     *
     * @param msg Mensaje de evento recibido de xEventQueue.
     * @return true si hay un cambio que requiere actualizar efectos físicos.
     */
    bool processEvent(const EventMessage_t& msg);

    /**
     * @brief Retorna el estado actual de la máquina de estados.
     * No toma ningún mutex — retorna currentState (variable miembro local).
     * @return El estado actual (STATE_IDLE, MOVING, ALERT, o PURSUIT).
     */
    SystemState_t getState() const;

    /**
     * @brief Calcula el modo operativo actual a partir del estado y los flags globales.
     *
     * PROPÓSITO:
     *   Proporcionar una vista de alto nivel del comportamiento actual del sistema.
     *   commTask usa este método para decidir el intervalo de telemetría.
     *
     * LÓGICA:
     *   STATE_PURSUIT → MODE_PURSUIT
     *   STATE_ALERT   → MODE_ALERT
     *   STATE_MOVING  → MODE_MONITORING
     *   STATE_IDLE    → MODE_MONITORING si systemArmed, MODE_IDLE si !systemArmed
     *
     * @return El modo operativo actual (SystemMode_t).
     */
    static SystemMode_t currentMode();

    /**
     * @brief Retorna el nombre legible de un estado para logging.
     * @param state El estado a convertir.
     * @return String literal: "IDLE", "MOVING", "ALERT", "PURSUIT", o "UNKNOWN".
     */
    static const char* stateName(SystemState_t state);

    /**
     * @brief Retorna el nombre legible de un modo para logging.
     * @param mode El modo a convertir.
     * @return String literal: "MODE_IDLE", "MODE_MONITORING", "MODE_ALERT", "MODE_PURSUIT".
     */
    static const char* modeName(SystemMode_t mode);

private:
    // Estado actual de la máquina. Copia local de currentSystemState (global).
    // Siempre deben estar sincronizados — transitionTo() actualiza ambos.
    SystemState_t currentState;

    /**
     * @brief Handler de eventos en STATE_IDLE.
     *
     * PROPÓSITO:
     *   Procesar eventos cuando la moto está quieta. Acepta: ARM/DISARM,
     *   MOVEMENT_SOFT/HARD (si armado) → MOVING, IMPACT (si armado) → ALERT directo,
     *   TRIGGER_ALERT_CMD → ALERT.
     *
     * NOTA: MOVEMENT_SOFT/HARD mientras !systemArmed retorna false (ignorado).
     *       Esto es intencional: el sensor sigue detectando movimiento pero el
     *       sistema en modo desarmado no reacciona.
     */
    bool handleIdle(const EventMessage_t& msg);

    /**
     * @brief Handler de eventos en STATE_MOVING.
     *
     * PROPÓSITO:
     *   Fase de filtrado. Acepta: DISARM → IDLE, MOVEMENT_SOFT (reiniciar timer),
     *   MOVEMENT_HARD/IMPACT → ALERT, MOVING_TIMEOUT → IDLE, TRIGGER_ALERT_CMD → ALERT.
     *
     * NOTA: EVENT_MOVEMENT_SOFT retorna true para que control_task reinicie el timer,
     *       aunque no haya transición de estado. Es el único caso donde se retorna
     *       true sin llamar transitionTo().
     */
    bool handleMoving(const EventMessage_t& msg);

    /**
     * @brief Handler de eventos en STATE_ALERT.
     *
     * PROPÓSITO:
     *   Alerta activa esperando confirmación. Acepta: DISARM → IDLE,
     *   PURSUIT_CONFIRM → PURSUIT, MOVEMENT_* (reiniciar timer de 30s),
     *   ALERT_TIMEOUT → IDLE (armado).
     *
     * NOTA: No hay escalada automática a PURSUIT. Solo EVENT_PURSUIT_CONFIRM lo hace.
     *       El timer de 30s es el único mecanismo de escape automático hacia IDLE.
     */
    bool handleAlert(const EventMessage_t& msg);

    /**
     * @brief Handler de eventos en STATE_PURSUIT.
     *
     * PROPÓSITO:
     *   Estado de persecución activa. Acepta:
     *   - DISARM → STATE_IDLE (desarmado, motor libre).
     *   - ENGINE_RESTORE → STATE_IDLE (motor libre, sistema SIGUE armado).
     *   Todos los demás eventos son ignorados para no interrumpir la
     *   telemetría de emergencia continua.
     *
     * NOTA: ENGINE_RESTORE es la única forma de liberar el motor sin desarmar.
     *       Conserva FLAG_SYSTEM_ARMED. DISARM en cambio borra todo.
     */
    bool handlePursuit(const EventMessage_t& msg);

    /**
     * @brief Ejecuta la transición al nuevo estado y sincroniza los globales.
     *
     * PROPÓSITO:
     *   Actualizar currentState (local) y currentSystemState (global) de forma
     *   atómica desde el punto de vista de la lógica de la máquina de estados.
     *   Registra la transición en el log para trazabilidad.
     *
     * NOTA: No toma xStateMutex — la asunción es que solo control_task llama
     *       a processEvent(), así que no hay concurrencia en la escritura de
     *       currentSystemState desde la perspectiva de transitionTo().
     *
     * @param newState El estado destino de la transición.
     */
    void transitionTo(SystemState_t newState);
};


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/core/include/state_machine.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo define la "mente" del sistema Argus: la máquina de estados.
   La máquina de estados es como un árbitro que decide qué pasa cuando ocurre
   un evento. Por ejemplo: "si la moto está en modo vigilancia (IDLE armado)
   y alguien la mueve (MOVEMENT_SOFT), pasar a MOVING para observar si es
   un ladrón o simplemente una persona que se apoyó en la moto".

   La máquina de estados NO hace nada físico (no enciende el buzzer, no corta
   el motor). Solo decide qué estado es el correcto. control_task recibe la
   decisión y hace los efectos físicos.

   TABLA DE TRANSICIONES:
   Estado origen | Evento                   | Condición      | Estado destino
   --------------|--------------------------|----------------|---------------
   IDLE          | ARM_CMD                  | -              | IDLE (armado)
   IDLE          | MOVEMENT_SOFT/HARD       | armado=true    | MOVING
   IDLE          | IMPACT_DETECTED          | armado=true    | ALERT
   IDLE          | TRIGGER_ALERT_CMD        | -              | ALERT
   MOVING        | MOVEMENT_SOFT            | -              | MOVING (timer reset)
   MOVING        | MOVEMENT_HARD/IMPACT     | -              | ALERT
   MOVING        | MOVING_TIMEOUT (15s)     | -              | IDLE
   ALERT         | PURSUIT_CONFIRM          | -              | PURSUIT
   ALERT         | ALERT_TIMEOUT (30s)      | -              | IDLE
   ALERT/PURSUIT | DISARM_CMD              | -              | IDLE
   Cualquiera    | DISARM_CMD              | -              | IDLE

   DEUDA TÉCNICA:
   1. EVENT_PURSUIT_CONFIRM no tiene fuente de generación en el firmware actual.
   2. transitionTo() no toma xStateMutex — aceptable si solo control_task
      llama processEvent(), pero podría ser un problema en el futuro.
   3. Los handlers privados son funciones miembro, no funciones de tabla, lo
      que hace que el despacho sea un switch en processEvent() en lugar de
      una tabla de punteros a función (más extensible pero más verboso).

   ═══════════════════════════════════════════════════════════ */
