/**
 * @file state_machine.cpp
 * @brief Implementación de la máquina de estados del sistema Argus Secure.
 *
 * PROPÓSITO:
 *   Implementar toda la lógica de transición de estados del sistema. Este archivo
 *   es la implementación de state_machine.h — no toca hardware, no bloquea, no tiene
 *   efectos secundarios fuera de actualizar las variables de estado global.
 *
 * DISEÑO — LÓGICA PURA:
 *   Cada método handler (handleIdle, handleMoving, handleAlert, handlePursuit)
 *   implementa las reglas de transición para su estado. No hay lógica de negocio
 *   mezclada con efectos físicos. Esto es intencional para facilitar testing
 *   y razonamiento sobre el comportamiento del sistema.
 *
 *   Los únicos efectos secundarios permitidos aquí son:
 *   1. Actualizar systemArmed (volatile bool en system_flags.h)
 *   2. Actualizar remoteAlert (volatile bool en system_flags.h)
 *   3. Actualizar bits de xSystemFlags (EventGroup FreeRTOS)
 *   4. Llamar transitionTo() para actualizar currentState y currentSystemState
 *   Cualquier otro efecto (buzzer, LEDs, timers) pertenece en control_task.
 *
 * FLUJO PRINCIPAL (caso de robo típico):
 *   Boot → IDLE (desarmado)
 *   Usuario pone app: EVENT_ARM_CMD → IDLE (armado, systemArmed=true)
 *   Ladrón toca la moto: EVENT_MOVEMENT_SOFT → MOVING (timer 15s inicia)
 *   Ladrón intenta robarla: EVENT_MOVEMENT_HARD → ALERT (buzzer, timer 30s)
 *   Operador confirma: EVENT_PURSUIT_CONFIRM → PURSUIT (GPS continuo, 10s)
 *     ↳ Sin sirena automática: CMD|SIREN_ON explícito (evita alertar al ladrón → tira GPS)
 *     ↳ Sin corte de motor automático: CMD|ENGINE_CUT diferido hasta quietud (evita accidente)
 *   Moto recuperada: EVENT_DISARM_CMD → IDLE (desarmado, motor libre)
 *
 * VARIABLES CRÍTICAS:
 *   - currentState: estado actual de la máquina de estados (copia local).
 *   - systemArmed (system_flags.h): el flag más crítico — si es false, todos
 *     los eventos de movimiento son ignorados silenciosamente.
 *   - xSystemFlags (system_events.h): bits que other tareas pueden leer
 *     sin bloqueo para verificar el estado actual del sistema.
 *
 * CONCURRENCIA:
 *   - processEvent() solo debe llamarse desde control_task (única escritora).
 *   - transitionTo() no toma xStateMutex porque asumimos que solo control_task
 *     llama processEvent(). Si en el futuro otra tarea llamara processEvent()
 *     directamente (sin pasar por xEventQueue), habría una race condition en
 *     currentSystemState.
 *   - Las escrituras a systemArmed y remoteAlert (bool volatile) son atómicas
 *     en ARM Cortex-M para tipos de un byte o palabra. No requieren mutex.
 *
 * @module components/core/state_machine
 */

#include "state_machine.h"
#include "system_flags.h"
#include "esp_log.h"

static const char* TAG = "StateMachine";

// ─── Constructor ──────────────────────────────────────────────────────────────

/**
 * @brief Inicializa currentState a STATE_IDLE.
 *
 * No inicializa las variables globales (systemArmed, etc.) aquí porque
 * el constructor se llama antes de que FreeRTOS esté completamente inicializado.
 * La inicialización real de los globales ocurre en init().
 */
StateMachine::StateMachine() : currentState(STATE_IDLE) {}

// ─── Inicialización ───────────────────────────────────────────────────────────

/**
 * @brief Resetea la máquina de estados y todos los flags globales al estado inicial.
 *
 * PROPÓSITO:
 *   Garantizar que el sistema arranque en un estado conocido y seguro:
 *   IDLE, desarmado, sin alerta remota. Se llama una sola vez desde control_task
 *   al inicio de la tarea, antes de comenzar a recibir eventos.
 *
 * ORDEN DE OPERACIONES:
 *   Primero currentState y currentSystemState (la fuente de verdad del estado),
 *   luego los flags adicionales (systemArmed, remoteAlert).
 *   El log es el último paso para confirmar que todo fue inicializado.
 */
void StateMachine::init() {
    currentState       = STATE_IDLE;
    systemArmed        = false;
    remoteAlert        = false;
    currentSystemState = STATE_IDLE;
    ESP_LOGI(TAG, "Máquina de estados inicializada → IDLE (desarmado)");
}

// ─── Despacho de eventos ──────────────────────────────────────────────────────

/**
 * @brief Despacha el evento al handler del estado actual.
 *
 * PROPÓSITO:
 *   Implementar el patrón State: cada estado tiene su propio handler que sabe
 *   qué eventos acepta y cómo reaccionar. processEvent() solo hace el dispatch.
 *
 * FLUJO:
 *   Leer currentState → despachar al handler correspondiente → retornar resultado.
 *
 * NOTA: El default case retorna false para eventos en estados desconocidos.
 *   Nunca debería ejecutarse en código correcto.
 */
bool StateMachine::processEvent(const EventMessage_t& msg) {
    switch (currentState) {
        case STATE_IDLE:    return handleIdle(msg);
        case STATE_MOVING:  return handleMoving(msg);
        case STATE_ALERT:   return handleAlert(msg);
        case STATE_PURSUIT: return handlePursuit(msg);
        default:            return false;
    }
}

SystemState_t StateMachine::getState() const {
    return currentState;
}

// ─── Modo operativo derivado ──────────────────────────────────────────────────

/**
 * @brief Calcula el modo operativo actual a partir del estado y los flags.
 *
 * PROPÓSITO:
 *   Proporcionar una abstracción de alto nivel del comportamiento del sistema.
 *   No almacena el modo — lo calcula en el momento de la consulta.
 *
 * LÓGICA:
 *   Los estados de emergencia (PURSUIT, ALERT) siempre tienen su propio modo.
 *   MOVING implica vigilancia activa (MODE_MONITORING).
 *   IDLE es MODE_MONITORING si armado, MODE_IDLE si desarmado.
 *   El modo MODE_IDLE representa el consumo más bajo del sistema.
 */
SystemMode_t StateMachine::currentMode() {
    switch (currentSystemState) {
        case STATE_PURSUIT: return MODE_PURSUIT;
        case STATE_ALERT:   return MODE_ALERT;
        case STATE_MOVING:  return MODE_MONITORING;
        case STATE_IDLE:
        default:
            // La distinción entre MONITORING e IDLE en STATE_IDLE depende de systemArmed.
            // Si armado, el MPU6050 está procesando interrupciones (modo vigilancia activa).
            // Si desarmado, el sistema está en reposo, solo enviando heartbeats.
            return systemArmed ? MODE_MONITORING : MODE_IDLE;
    }
}

// ─── Nombres legibles (para logging) ─────────────────────────────────────────

const char* StateMachine::stateName(SystemState_t state) {
    switch (state) {
        case STATE_IDLE:    return "IDLE";
        case STATE_MOVING:  return "MOVING";
        case STATE_ALERT:   return "ALERT";
        case STATE_PURSUIT: return "PURSUIT";
        default:            return "UNKNOWN";
    }
}

const char* StateMachine::modeName(SystemMode_t mode) {
    switch (mode) {
        case MODE_IDLE:       return "MODE_IDLE";
        case MODE_MONITORING: return "MODE_MONITORING";
        case MODE_ALERT:      return "MODE_ALERT";
        case MODE_PURSUIT:    return "MODE_PURSUIT";
        default:              return "UNKNOWN";
    }
}

// ─── Handler: IDLE ───────────────────────────────────────────────────────────

/**
 * @brief Procesa eventos cuando la moto está quieta.
 *
 * PROPÓSITO:
 *   Gestionar el estado de reposo del sistema. En IDLE, el sistema puede:
 *   armarse/desarmarse, detectar movimiento (si armado), o recibir un override
 *   manual del operador para activar ALERT directamente.
 *
 * FLUJO DE DECISIÓN:
 *   ARM_CMD: si ya armado → ignorar. Si no → armar y retornar true.
 *   DISARM_CMD: si ya desarmado → ignorar. Si no → desarmar y retornar true.
 *   MOVEMENT_SOFT/HARD: si !armado → ignorar. Si armado → MOVING.
 *   IMPACT_DETECTED: si !armado → ignorar. Si armado → ALERT directamente
 *     (impacto directo en IDLE armado es más severo que movimiento normal —
 *      no tiene sentido esperar la fase de filtrado MOVING para un impacto).
 *   TRIGGER_ALERT_CMD: override del operador → ALERT independientemente del estado.
 *
 * NOTA SOBRE ARM/DISARM:
 *   Si systemArmed ya tiene el valor del comando (ya armado cuando llega ARM_CMD),
 *   retornamos false para no despertar a control_task innecesariamente y no
 *   re-aplicar efectos físicos que ya están aplicados.
 */
bool StateMachine::handleIdle(const EventMessage_t& msg) {
    switch (msg.event) {

        case EVENT_ARM_CMD:
            if (systemArmed) return false;  // Idempotente: ya estaba armado
            systemArmed = true;
            // Sincronizar el EventGroup para que otras tareas puedan leer el estado sin mutex
            xEventGroupSetBits(xSystemFlags, FLAG_SYSTEM_ARMED);
            ESP_LOGI(TAG, "Sistema ARMADO → modo vigilancia activo");
            return true;  // control_task debe actualizar LEDs (IDLE fijo en lugar de latido)

        case EVENT_DISARM_CMD:
            if (!systemArmed) return false;  // Idempotente: ya estaba desarmado
            systemArmed = false;
            // Limpiar FLAG_SYSTEM_ARMED y FLAG_REMOTE_ALERT al desarmar
            xEventGroupClearBits(xSystemFlags, FLAG_SYSTEM_ARMED | FLAG_REMOTE_ALERT);
            ESP_LOGI(TAG, "Sistema DESARMADO");
            return true;

        case EVENT_MOVEMENT_SOFT:
        case EVENT_MOVEMENT_HARD:
            // Movimiento detectado pero solo reaccionar si el sistema está armado.
            // Si !armado, el sensor sigue funcionando (para telemetría de inactividad)
            // pero la máquina de estados lo ignora — no es una amenaza en modo desarmado.
            if (!systemArmed) return false;
            transitionTo(STATE_MOVING);
            return true;

        case EVENT_IMPACT_DETECTED:
            // Un impacto en IDLE armado escalamos directamente a ALERT sin pasar por MOVING.
            // Razón: un impacto es un evento de alta severidad que no requiere filtrado.
            // Si fuera falso positivo, el timer de ALERT (30s) retornará a IDLE.
            if (!systemArmed) return false;
            ESP_LOGE(TAG, "Impacto en IDLE → escalando directamente a ALERT");
            transitionTo(STATE_ALERT);
            return true;

        case EVENT_TRIGGER_ALERT_CMD:
            // Override manual del operador: activar alerta independientemente del estado de armado.
            // remoteAlert = true para que control_task sepa que fue activado remotamente.
            remoteAlert = true;
            xEventGroupSetBits(xSystemFlags, FLAG_REMOTE_ALERT);
            transitionTo(STATE_ALERT);
            return true;

        case EVENT_PURSUIT_CONFIRM:
            // Robo confirmado por propietario u operador → activar rastreo silencioso.
            // Se acepta en IDLE (robo sin alarma previa: dispositivo desarmado o ladrón
            // rápido) o desde STATE_ALERT (escalada desde alerta activa).
            // NO activa sirena ni corte de motor — ambas son decisiones explícitas del
            // operador/propietario para no alertar al ladrón de que tiene GPS.
            ESP_LOGE(TAG, "Robo CONFIRMADO desde IDLE → activando PURSUIT (GPS 10s, sin sirena ni motor)");
            transitionTo(STATE_PURSUIT);
            return true;

        default:
            // Todos los demás eventos (GPS, timers, COMMS_*) se ignoran en IDLE
            return false;
    }
}

// ─── Handler: MOVING ─────────────────────────────────────────────────────────

/**
 * @brief Procesa eventos durante la fase de filtrado de movimiento.
 *
 * PROPÓSITO:
 *   Gestionar el estado de evaluación: se detectó movimiento pero no está
 *   confirmado como amenaza. El timer de 15s (en control_task) decide el destino.
 *
 * FLUJO DE DECISIÓN:
 *   DISARM_CMD: cancelar evaluación → IDLE desarmado.
 *   MOVEMENT_SOFT: movimiento suave continuo → reiniciar el timer (control_task lo hace)
 *     → retornar true para señalizar que control_task debe reiniciar el timer.
 *   MOVEMENT_HARD / IMPACT: movimiento confirmado como severo → ALERT.
 *   MOVING_TIMEOUT: timer de 15s expiró sin escalada → probable falsa alarma → IDLE.
 *   TRIGGER_ALERT_CMD: override del operador → ALERT.
 *
 * NOTA SOBRE MOVEMENT_SOFT EN MOVING:
 *   Retorna true aunque no haya transición de estado. Este es el único caso
 *   donde true no implica transitionTo(). control_task interpreta true en
 *   STATE_MOVING + EVENT_MOVEMENT_SOFT como "reiniciar el timer de 15s".
 *   Si no se reiniciara el timer, un ladrón podría mover la moto suavemente por
 *   15 segundos y luego detenerse para que el sistema vuelva a IDLE.
 */
bool StateMachine::handleMoving(const EventMessage_t& msg) {
    switch (msg.event) {

        case EVENT_DISARM_CMD:
            // Propietario llegó o falsa alarma. Desarmar y cancelar la fase de filtrado.
            systemArmed = false;
            xEventGroupClearBits(xSystemFlags, FLAG_SYSTEM_ARMED | FLAG_REMOTE_ALERT);
            transitionTo(STATE_IDLE);
            return true;

        case EVENT_MOVEMENT_SOFT:
            // Movimiento suave sostenido: reiniciar el timer de 15s en control_task.
            // No escala automáticamente — el movimiento suave solo extiende la observación.
            // Un ladrón que mueve la moto suavemente seguirá en MOVING hasta que
            // el movimiento sea más brusco o el timer expire.
            ESP_LOGD(TAG, "Movimiento suave sostenido en MOVING (timer reiniciado)");
            return true;  // control_task debe reiniciar el timer de MOVING

        case EVENT_MOVEMENT_HARD:
            // Movimiento brusco confirmado: la evaluación terminó, hay una amenaza real.
            ESP_LOGW(TAG, "Movimiento DURO en MOVING → escalando a ALERT");
            transitionTo(STATE_ALERT);
            return true;

        case EVENT_IMPACT_DETECTED:
            // Impacto mientras ya estábamos evaluando: alerta inmediata, no esperar más.
            ESP_LOGE(TAG, "Impacto en MOVING → escalando a ALERT");
            transitionTo(STATE_ALERT);
            return true;

        case EVENT_MOVING_TIMEOUT:
            // 15 segundos sin escalada: probablemente fue ruido, viento, o alguien
            // que se apoyó en la moto accidentalmente. Volver a vigilancia silenciosa.
            ESP_LOGI(TAG, "Timeout MOVING → ningún movimiento confirmado, volviendo a IDLE");
            transitionTo(STATE_IDLE);
            return true;

        case EVENT_TRIGGER_ALERT_CMD:
            // Override del operador: tiene información que el sensor no tiene.
            remoteAlert = true;
            xEventGroupSetBits(xSystemFlags, FLAG_REMOTE_ALERT);
            transitionTo(STATE_ALERT);
            return true;

        case EVENT_PURSUIT_CONFIRM:
            // Corte de motor manual mientras el sistema ya estaba evaluando movimiento.
            // El propietario vio el aviso de movimiento y decidió cortar el motor
            // sin esperar a que escale a ALERT. Transición directa a PURSUIT.
            ESP_LOGE(TAG, "Corte de motor manual en MOVING → activando PURSUIT");
            transitionTo(STATE_PURSUIT);
            return true;

        default:
            return false;
    }
}

// ─── Handler: ALERT ──────────────────────────────────────────────────────────

/**
 * @brief Procesa eventos durante la alerta activa.
 *
 * PROPÓSITO:
 *   Gestionar el estado de alarma: el buzzer está sonando, el backend fue
 *   notificado, y el sistema espera confirmación del propietario o del operador.
 *   El timer de 30s (en control_task) retorna a IDLE si no hay confirmación.
 *
 * FLUJO DE DECISIÓN:
 *   DISARM_CMD: propietario llegó o falsa alarma → IDLE desarmado.
 *   PURSUIT_CONFIRM: robo confirmado → PURSUIT (motor cortado, GPS continuo).
 *   MOVEMENT_*: más movimiento → retornar true para que control_task reinicie el timer.
 *   ALERT_TIMEOUT: 30s de quietud → probable falsa alarma → IDLE (armado).
 *
 * DISEÑO — NO HAY ESCALADA AUTOMÁTICA:
 *   No hay escalada automática ALERT → PURSUIT. Esta decisión es intencional:
 *   activar el corte de motor automáticamente sin confirmación humana podría
 *   cortar el motor de la moto mientras está en movimiento, causando un accidente.
 *   PURSUIT solo se activa por EVENT_PURSUIT_CONFIRM (confirmación explícita).
 *
 * NOTA SOBRE ALERT_TIMEOUT:
 *   Al retornar a IDLE desde ALERT, el sistema queda ARMADO (systemArmed=true).
 *   Esto es correcto: una falsa alarma no debe dejar la moto desprotegida.
 */
bool StateMachine::handleAlert(const EventMessage_t& msg) {
    switch (msg.event) {

        case EVENT_DISARM_CMD:
            // El propietario canceló la alerta (llegó físicamente o fue falsa alarma).
            // Limpiar TODOS los flags de alerta y desarmar.
            systemArmed = false;
            remoteAlert = false;
            xEventGroupClearBits(xSystemFlags,
                                 FLAG_SYSTEM_ARMED | FLAG_REMOTE_ALERT | FLAG_HIGH_FREQ_MODE);
            transitionTo(STATE_IDLE);
            return true;

        case EVENT_PURSUIT_CONFIRM:
            // Confirmación explícita de robo por usuario o operador.
            // Es la única forma de activar PURSUIT — no hay escalada automática.
            ESP_LOGE(TAG, "Robo CONFIRMADO → activando PURSUIT");
            transitionTo(STATE_PURSUIT);
            return true;

        case EVENT_MOVEMENT_SOFT:
        case EVENT_MOVEMENT_HARD:
        case EVENT_IMPACT_DETECTED:
            // Más movimiento durante la alerta: reiniciar el timer de 30s.
            // El sistema sigue en ALERT — el movimiento adicional no cambia el estado,
            // solo extiende el tiempo de la alerta activa.
            // control_task interpreta true + EVENT_MOVEMENT_* en STATE_ALERT como
            // "reiniciar el timer de ALERT".
            ESP_LOGW(TAG, "Movimiento adicional en ALERT (timer reiniciado)");
            return true;

        case EVENT_ALERT_TIMEOUT:
            // 30 segundos de quietud: probable falsa alarma.
            // Limpiar flags de alerta pero mantener el sistema ARMADO.
            ESP_LOGI(TAG, "Timeout ALERT sin confirmación → volviendo a IDLE (armado)");
            remoteAlert = false;
            xEventGroupClearBits(xSystemFlags, FLAG_REMOTE_ALERT | FLAG_HIGH_FREQ_MODE);
            transitionTo(STATE_IDLE);
            return true;

        default:
            return false;
    }
}

// ─── Handler: PURSUIT ────────────────────────────────────────────────────────

/**
 * @brief Procesa eventos durante la persecución activa.
 *
 * PROPÓSITO:
 *   Gestionar el estado de máxima prioridad. En PURSUIT, la telemetría GPS corre
 *   a 10s de intervalo (máxima frecuencia). La sirena y el corte de motor son
 *   comandos EXPLÍCITOS — no se activan automáticamente al entrar en PURSUIT.
 *   Solo un DISARM_CMD explícito puede salir de este estado.
 *
 * DISEÑO — IGNORAR TODO EXCEPTO DISARM:
 *   En PURSUIT ignoramos todos los eventos excepto DISARM_CMD. Razón:
 *   durante una persecución, el firmware debe concentrarse en enviar la
 *   posición GPS al backend. Cualquier procesamiento adicional de eventos
 *   (movimiento, GPS_FIX, etc.) es secundario y no cambia el comportamiento
 *   de PURSUIT.
 *
 *   Si se necesitara un timeout de PURSUIT (ej: si nadie confirmó la recuperación
 *   en 24h), habría que agregar un timer de larga duración en control_task y un
 *   nuevo evento EVENT_PURSUIT_TIMEOUT.
 */
bool StateMachine::handlePursuit(const EventMessage_t& msg) {
    switch (msg.event) {

        case EVENT_DISARM_CMD:
            // Operador o propietario recuperó la moto y cancela la persecución.
            // Limpiar TODOS los flags y volver a IDLE desarmado.
            systemArmed = false;
            remoteAlert = false;
            xEventGroupClearBits(xSystemFlags,
                                 FLAG_SYSTEM_ARMED | FLAG_REMOTE_ALERT | FLAG_HIGH_FREQ_MODE);
            transitionTo(STATE_IDLE);
            return true;

        case EVENT_ENGINE_RESTORE:
            // Operador restaura el motor sin desarmar el sistema.
            // A diferencia de DISARM, systemArmed y FLAG_SYSTEM_ARMED se mantienen:
            // la alarma sigue activa, solo se libera el relé de corte de motor.
            // applyStateEffects(STATE_IDLE) llamará setEngineCut(false).
            remoteAlert = false;
            xEventGroupClearBits(xSystemFlags, FLAG_REMOTE_ALERT | FLAG_HIGH_FREQ_MODE);
            transitionTo(STATE_IDLE);
            return true;

        default:
            // En PURSUIT, todo lo que no es DISARM o ENGINE_RESTORE se ignora.
            // El sistema está en modo de emergencia y no debe distraerse.
            return false;
    }
}

// ─── Transición de estado ─────────────────────────────────────────────────────

/**
 * @brief Actualiza currentState y currentSystemState al nuevo estado.
 *
 * PROPÓSITO:
 *   Ser el ÚNICO punto donde currentState y currentSystemState se actualizan.
 *   Garantiza que ambas variables siempre estén sincronizadas y que cada
 *   transición quede registrada en el log.
 *
 * ORDEN DE ACTUALIZACIÓN:
 *   Primero currentState (local), luego currentSystemState (global).
 *   El log se ejecuta con los valores ya actualizados para reflejar el
 *   estado final en el mensaje.
 *
 * NOTA: No notifica a xSystemFlags sobre el cambio de estado — los bits de
 *   FLAG_SYSTEM_ARMED, FLAG_REMOTE_ALERT, etc. son actualizados por los handlers
 *   de cada estado según el caso específico. No hay un FLAG_CURRENT_STATE
 *   porque los EventGroup bits no son suficientes para codificar 4 estados.
 */
void StateMachine::transitionTo(SystemState_t newState) {
    ESP_LOGI(TAG, "Transición: %s → %s  [modo: %s]",
             stateName(currentState),
             stateName(newState),
             modeName(currentMode()));

    currentState       = newState;
    currentSystemState = newState;  // Sincronizar la variable global
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/core/state_machine.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo implementa las reglas de qué hace el sistema Argus cuando ocurre
   algo. Por ejemplo: "si la moto está en ALERTA y llega una confirmación de robo,
   pasar a PERSECUCIÓN y cortar el motor". Cada función handle*() implementa las
   reglas para un estado específico.

   Lo importante de este diseño es que la máquina de estados SOLO decide qué
   estado es el correcto — no hace nada físico. El buzzer, los LEDs y el corte
   de motor son responsabilidad de control_task, que llama a MosfetControl
   después de que la máquina de estados toma su decisión.

   PSEUDOCÓDIGO DE CADA HANDLER:
   handleIdle(event):
     ARM_CMD     → systemArmed=true, retornar true
     DISARM_CMD  → systemArmed=false, retornar true
     MOVEMENT_*  → si armado: MOVING, sino ignorar
     IMPACT      → si armado: ALERT directo
     TRIGGER_ALERT_CMD → ALERT (override manual)

   handleMoving(event):
     DISARM_CMD    → IDLE desarmado
     MOVEMENT_SOFT → retornar true (reiniciar timer)
     MOVEMENT_HARD → ALERT
     IMPACT        → ALERT
     MOVING_TIMEOUT → IDLE (armado)

   handleAlert(event):
     DISARM_CMD    → IDLE desarmado
     PURSUIT_CONFIRM → PURSUIT (confirmación de robo)
     MOVEMENT_*    → retornar true (reiniciar timer)
     ALERT_TIMEOUT → IDLE (armado, falsa alarma)

   handlePursuit(event):
     DISARM_CMD      → IDLE desarmado
     ENGINE_RESTORE  → IDLE (motor libre, sigue armado)
     todo lo demás  → ignorar

   NOTA DE DISEÑO PURSUIT:
     La sirena y el corte de motor NO se activan automáticamente al entrar en
     PURSUIT (a diferencia de versiones anteriores). control_task.cpp tampoco
     los activa en applyStateEffects(STATE_PURSUIT). Razones:
     - Sirena automática: alerta al ladrón → puede tirar el GPS antes de ser encontrado.
     - Corte de motor automático en movimiento: riesgo de accidente mortal.
     → Usar CMD|SIREN_ON y CMD|ENGINE_CUT (diferido) de forma explícita.

   VARIABLES ESCRITAS:
   - currentState (local): estado de la instancia de StateMachine
   - currentSystemState (global): sincronización con el resto del sistema
   - systemArmed (global): true/false según ARM/DISARM
   - remoteAlert (global): true cuando el operador forzó ALERT/PURSUIT
   - xSystemFlags (EventGroup): bits FLAG_SYSTEM_ARMED, FLAG_REMOTE_ALERT, FLAG_HIGH_FREQ_MODE

   DEUDA TÉCNICA:
   1. EVENT_PURSUIT_CONFIRM no tiene implementación en ninguna tarea — no hay
      flujo de confirmación de robo en el firmware actual.
   2. transitionTo() no toma xStateMutex — safe solo porque control_task es la
      única que llama processEvent(). Fragilidad si el diseño cambia.
   3. No hay timeout para PURSUIT — puede quedar en ese estado indefinidamente.

   ═══════════════════════════════════════════════════════════ */
