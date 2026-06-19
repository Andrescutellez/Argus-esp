/**
 * @file control_task.cpp
 * @brief Tarea FreeRTOS del controlador central del sistema Argus.
 *
 * PROPÓSITO:
 *   Orquestar la máquina de estados y aplicar los efectos físicos correspondientes
 *   a cada transición de estado. Es el único archivo que activa/desactiva el buzzer,
 *   corta el motor (MOSFET), gestiona los timers de timeout y controla los patrones
 *   de LEDs en tiempo real.
 *
 * ARQUITECTURA DE CONTROL:
 *
 *   xEventQueue (cap. 10)
 *     → xQueueReceive (timeout 250ms)
 *     → stateMachine.processEvent()     ← lógica pura, sin hardware
 *     → applyStateEffects()             ← hardware: buzzer, motor, timers
 *     → MosfetControl::applyLedPattern() ← LEDs (siempre, cada 250ms)
 *     → Patrón buzzer en ALERT suave    ← beeps (siempre en ALERT, cada 250ms)
 *
 *   Timers ESP-IDF (no FreeRTOS):
 *     movingTimer (15s one-shot) → inyecta EVENT_MOVING_TIMEOUT en xEventQueue
 *     alertTimer  (30s one-shot) → inyecta EVENT_ALERT_TIMEOUT en xEventQueue
 *     Los timers se reinician con cada evento de movimiento adicional en MOVING/ALERT.
 *
 * TABLA DE EFECTOS POR ESTADO:
 *   Estado     | Armado | Buzzer           | Motor  | LEDs            | Timers
 *   -----------|--------|------------------|--------|-----------------|------------------
 *   IDLE       | NO     | OFF              | ON     | Latido lento    | Todos detenidos
 *   IDLE       | SÍ     | OFF              | ON     | Fijo encendido  | Todos detenidos
 *   MOVING     | SÍ     | OFF              | ON     | Parpadeo rápido | movingTimer (15s)
 *   ALERT soft | SÍ     | 2 beeps cada 2s  | ON     | Alternado       | alertTimer (30s)
 *   ALERT hard | SÍ     | Sirena continua  | ON     | Alternado       | alertTimer (30s)
 *   PURSUIT    | SÍ     | ON continuo      | CORTADO| Sirena          | Todos detenidos
 *
 *   NOTA: En ALERT el motor NO se corta. Puede ser una falsa alarma. Solo en PURSUIT
 *   se corta el motor definitivamente (confirmación humana requerida para llegar ahí).
 *
 * BUZZER EN ALERT:
 *   alertIsHard=false: patrón de beeps — 2 ticks ON (500ms) de cada 8 (2s).
 *                      Tick%8 < 2 → ON. Tick%8 >= 2 → OFF.
 *   alertIsHard=true:  sirena continua — setBuzzer(true) al entrar, sin lógica de tick.
 *
 *   alertIsHard se determina por el evento que disparó la alerta:
 *     EVENT_MOVEMENT_HARD | EVENT_IMPACT_DETECTED | EVENT_TRIGGER_ALERT_CMD → hard
 *     EVENT_MOVEMENT_SOFT (vía MOVING → ALERT) → soft
 *
 * TIMERS:
 *   Los timers se crean en la primera llamada a start*Timer() para evitar crear
 *   handles en el constructor (la tarea no existe aún). Una vez creados, se reusan.
 *   esp_timer_stop() es seguro si el timer ya estaba detenido (devuelve ESP_ERR_INVALID_STATE
 *   que aquí se ignora). Se usa start_once (one-shot) porque los timeouts de MOVING y ALERT
 *   no deben repetirse automáticamente — el nuevo evento de movimiento los reinicia.
 *
 * DEPENDENCIAS:
 *   - system_events.h: EventMessage_t, SystemState_t, xEventQueue, xSystemFlags
 *   - system_flags.h:  systemArmed, FLAG_* bits
 *   - state_machine.h: StateMachine (lógica pura de transiciones)
 *   - mosfet_control.h: MosfetControl (buzzer, motor, LEDs)
 *   - esp_timer.h:      timers de alta resolución del ESP-IDF
 *
 * VARIABLES CRÍTICAS:
 *   - movingTimer / alertTimer: handles de timers. nullptr hasta la primera llamada.
 *   - alertIsHard:  bool que determina el patrón del buzzer en ALERT.
 *   - ledTick:      contador de ticks (250ms/tick). Controla todos los patrones.
 *   - xEventQueue:  cola global (cap. 10) de donde se consumen los eventos.
 *   - xSystemFlags: EventGroup global — se escriben FLAG_HIGH_FREQ_MODE, FLAG_REMOTE_ALERT.
 *
 * CONCURRENCIA:
 *   - movingTimerCallback y alertTimerCallback se ejecutan en el timer service task
 *     de ESP-IDF (contexto distinto a controlTask). Usan xQueueSend con timeout=0:
 *     si la cola está llena, el evento se pierde silenciosamente. Esto es aceptable
 *     porque si la cola está llena es porque hay actividad reciente que impedirá
 *     el timeout de todas formas.
 *   - alertIsHard: solo escrito en applyStateEffects() (contexto controlTask) y leído
 *     en el tick del loop (mismo contexto). Sin races.
 *   - MosfetControl: métodos estáticos, acceden a GPIOs que solo escribe controlTask.
 *     Sin races desde la perspectiva de controlTask.
 *
 * @module components/services/control_task
 */

#include "control_task.h"
#include "system_events.h"
#include "system_flags.h"
#include "state_machine.h"
#include "mosfet_control.h"
#include "ble_task.h"
#include "esp_log.h"
#include "esp_timer.h"

// TAG de logging para todos los mensajes de esta tarea.
static const char* TAG = "ControlTask";

// Handle de la tarea. Asignado por xTaskCreate() en main.cpp.
// nullptr hasta que main.cpp llame xTaskCreate(controlTask, ..., &xControlTaskHandle).
TaskHandle_t xControlTaskHandle = nullptr;

/**
 * Duración del timeout de STATE_MOVING antes de volver a IDLE.
 * 15 segundos es tiempo suficiente para distinguir movimiento incidental
 * (persona que roza la moto) de robo real (que escalaría a ALERT antes de que expire).
 */
static const uint32_t MOVING_TIMEOUT_MS = 15000;

/**
 * Duración del timeout de STATE_ALERT antes de volver a IDLE (armado).
 * 30 segundos da tiempo al dueño para ver la notificación y confirmar persecución.
 * Si no confirma, asumimos falsa alarma y volvemos a vigilancia normal.
 */
static const uint32_t ALERT_TIMEOUT_MS  = 30000;

/**
 * Intervalo del tick principal del loop. Controla la granularidad de los patrones
 * visuales y sonoros. 250ms = 4 ticks/s = ciclo de 8 ticks = 2s por patrón completo.
 * También es el timeout de xQueueReceive — sin evento, igual corre el tick de LEDs.
 */
static const uint32_t LED_TICK_MS       = 250;

// ─── Handles de timers ───────────────────────────────────────────────────────────
// Se crean en la primera llamada a start*Timer() para que la tarea ya exista.
// nullptr indica que el timer aún no fue creado (no simplemente detenido).

static esp_timer_handle_t movingTimer = nullptr;
static esp_timer_handle_t alertTimer  = nullptr;

/**
 * @brief Callback del timer de MOVING — inyecta EVENT_MOVING_TIMEOUT en xEventQueue.
 *
 * PROPÓSITO:
 *   Generar el evento de expiración del período de observación de MOVING (15s).
 *   Si después de 15s de primer movimiento no hubo movimiento brusco, la moto
 *   vuelve a IDLE (el primer movimiento fue probablemente incidental).
 *
 * CONTEXTO DE EJECUCIÓN:
 *   Timer service task de ESP-IDF (no controlTask). No es ISR, pero tampoco es
 *   el contexto de controlTask. El xQueueSend con timeout=0 es seguro: si la cola
 *   está llena, el evento se descarta sin bloquear el timer service.
 *
 * FLUJO:
 *   esp_timer dispara → este callback → xQueueSend(EVENT_MOVING_TIMEOUT, timeout=0)
 *   → controlTask recibe en su próximo xQueueReceive → processEvent() → IDLE.
 */
static void movingTimerCallback(void*) {
    EventMessage_t msg = {};
    msg.event = EVENT_MOVING_TIMEOUT;
    // timeout=0: no bloquear si la cola está llena. Si se pierde, el sistema
    // permanece en MOVING hasta que llegue un evento de movimiento o se reintente.
    xQueueSend(xEventQueue, &msg, 0);
    ESP_LOGI(TAG, "Timer MOVING expirado → enviando MOVING_TIMEOUT");
}

/**
 * @brief Callback del timer de ALERT — inyecta EVENT_ALERT_TIMEOUT en xEventQueue.
 *
 * PROPÓSITO:
 *   Generar el evento de expiración del período de alerta (30s). Si el dueño no
 *   confirmó persecución en 30s, se asume que la alerta fue una falsa alarma
 *   o fue atendida, y el sistema vuelve a IDLE armado.
 *
 * CONTEXTO DE EJECUCIÓN:
 *   Timer service task de ESP-IDF. Mismo razonamiento que movingTimerCallback
 *   respecto al timeout=0 en xQueueSend.
 */
static void alertTimerCallback(void*) {
    EventMessage_t msg = {};
    msg.event = EVENT_ALERT_TIMEOUT;
    xQueueSend(xEventQueue, &msg, 0);
    ESP_LOGI(TAG, "Timer ALERT expirado → enviando ALERT_TIMEOUT");
}

/**
 * @brief Inicia (o reinicia) el timer de 15s para STATE_MOVING.
 *
 * PROPÓSITO:
 *   Crear el handle del timer si no existe aún (primera llamada), detener
 *   cualquier instancia previa, y arrancar un nuevo one-shot de 15s.
 *
 * FLUJO:
 *   1. Si movingTimer == nullptr → esp_timer_create() con movingTimerCallback.
 *   2. esp_timer_stop() — detener si estaba corriendo (sin error si ya estaba parado).
 *   3. esp_timer_start_once(15_000_000 µs) — arrancar one-shot de 15s.
 *
 * CUÁNDO SE LLAMA:
 *   - Al entrar en STATE_MOVING (transición desde IDLE).
 *   - Al recibir EVENT_MOVEMENT_SOFT adicional en STATE_MOVING (reinicio del timer).
 *
 * NOTA: La conversión de ms a µs usa (uint64_t) para evitar overflow de uint32_t.
 *   15000 ms × 1000 = 15_000_000 µs, que cabe en uint32_t (máx 4_294_967_295)
 *   pero la API espera uint64_t.
 */
static void startMovingTimer() {
    if (movingTimer == nullptr) {
        esp_timer_create_args_t args = {};
        args.callback = movingTimerCallback;
        args.name     = "moving_timer";
        esp_timer_create(&args, &movingTimer);
    }
    esp_timer_stop(movingTimer);
    esp_timer_start_once(movingTimer, (uint64_t)MOVING_TIMEOUT_MS * 1000);
    ESP_LOGI(TAG, "Timer MOVING iniciado (%lu s)", (unsigned long)(MOVING_TIMEOUT_MS / 1000));
}

/**
 * @brief Detiene el timer de MOVING si está corriendo.
 *
 * CUÁNDO SE LLAMA:
 *   - Al salir de STATE_MOVING (hacia IDLE o ALERT) en applyStateEffects().
 *   - En STATE_IDLE y STATE_PURSUIT como medida de seguridad.
 *
 * NOTA: esp_timer_stop() devuelve ESP_ERR_INVALID_STATE si el timer no estaba
 *   corriendo. Este error se ignora intencionalmente.
 */
static void stopMovingTimer() {
    if (movingTimer) esp_timer_stop(movingTimer);
}

/**
 * @brief Inicia (o reinicia) el timer de 30s para STATE_ALERT.
 *
 * PROPÓSITO:
 *   Crear el handle si no existe, y arrancar un one-shot de 30s. Si el dueño
 *   no confirma persecución en 30s, el sistema vuelve a IDLE.
 *
 * CUÁNDO SE LLAMA:
 *   - Al entrar en STATE_ALERT (transición desde MOVING o IDLE).
 *   - Al recibir movimiento adicional en STATE_ALERT (reinicio del timer).
 *     Esto evita que una alerta activa con movimiento continuo expire prematuramente.
 */
static void startAlertTimer() {
    if (alertTimer == nullptr) {
        esp_timer_create_args_t args = {};
        args.callback = alertTimerCallback;
        args.name     = "alert_timer";
        esp_timer_create(&args, &alertTimer);
    }
    esp_timer_stop(alertTimer);
    esp_timer_start_once(alertTimer, (uint64_t)ALERT_TIMEOUT_MS * 1000);
    ESP_LOGI(TAG, "Timer ALERT iniciado (%lu s)", (unsigned long)(ALERT_TIMEOUT_MS / 1000));
}

/**
 * @brief Detiene el timer de ALERT si está corriendo.
 *
 * CUÁNDO SE LLAMA:
 *   - Al salir de STATE_ALERT (hacia IDLE, PURSUIT) en applyStateEffects().
 *   - En STATE_IDLE como medida de seguridad (puede llegarse a IDLE desde ALERT).
 */
static void stopAlertTimer() {
    if (alertTimer) esp_timer_stop(alertTimer);
}

// ─── Flag de intensidad de buzzer ────────────────────────────────────────────────
//
// Separa el "modo de ALERT" del estado para poder aplicar dos patrones diferentes
// de buzzer dentro del mismo estado STATE_ALERT. Solo se lee/escribe desde controlTask.

/**
 * Indica si la alerta actual fue disparada por un evento severo.
 *   true  → buzzer ON continuo (sirena) al entrar en ALERT y se mantiene hasta salir.
 *   false → patrón de beeps: 2 ticks ON (500ms) de cada 8 ticks (2s de ciclo).
 *
 * Se asigna en applyStateEffects(STATE_ALERT) según el evento disparador.
 * Se resetea a false en applyStateEffects(STATE_IDLE).
 */
static bool alertIsHard = false;

// Corte de motor preventivo activo — independiente del estado de la state machine.
// Se activa con EVENT_ENGINE_CUT_SILENT o al entrar a STATE_PURSUIT.
// Se limpia con EVENT_ENGINE_RESTORE (llamada directa al relé + flag).
static bool s_motorManualCut = false;

// ─── Secuenciador de beep de confirmación ARM/DISARM ─────────────────────────
//
// Cuando el usuario arma o desarma, el sistema emite 1 ó 2 beeps cortos como
// confirmación auditiva, igual que los alarmas de moto convencionales.
//
// DISEÑO:
//   beepMax:  total de steps de la secuencia (0 = sin beep activo).
//   beepStep: step actual. Se incrementa en cada tick de 250ms.
//   Patrón: even step = buzzer ON, odd step = buzzer OFF (cada step = 250ms).
//   ARM    → 2 steps: ON+OFF = 1 beep × 250ms.
//   DISARM → 4 steps: ON+OFF+ON+OFF = 2 beeps × 250ms, separados 250ms.
//
// SEGURIDAD: solo activo en STATE_IDLE. En ALERT/PURSUIT el buzzer es
//   controlado por applyStateEffects() / ledTick — no se interfiere.

static uint8_t beepStep = 0;
static uint8_t beepMax  = 0;

// ─── Efectos físicos por estado ─────────────────────────────────────────────────

/**
 * @brief Aplica los efectos físicos correspondientes al nuevo estado del sistema.
 *
 * PROPÓSITO:
 *   Ser el único punto del firmware que activa/desactiva hardware en función del
 *   estado del sistema. La StateMachine NO toca hardware — solo llama processEvent().
 *   Esta función traduce "estado nuevo" en "acciones GPIO y timers".
 *
 * FLUJO LÓGICO POR ESTADO:
 *
 *   STATE_IDLE:
 *     → Buzzer OFF, Motor ON (engine cut OFF), timers detenidos.
 *     → Limpiar FLAG_REMOTE_ALERT y FLAG_HIGH_FREQ_MODE del EventGroup.
 *     → alertIsHard = false (reset para la próxima ALERT).
 *     → FLAG_SYSTEM_ARMED NO se toca aquí (lo gestiona StateMachine vía systemArmed).
 *
 *   STATE_MOVING:
 *     → Buzzer OFF (aún no sabemos si es robo).
 *     → Motor ON (no cortar por simple movimiento).
 *     → Detener alertTimer (si venimos de ALERT, lo limpiamos).
 *     → Iniciar movingTimer (15s de ventana de observación).
 *     → Sin FLAG_HIGH_FREQ_MODE: la telemetría sigue en modo normal.
 *
 *   STATE_ALERT:
 *     → Motor ON (puede ser falsa alarma — solo PURSUIT corta el motor).
 *     → Detener movingTimer (ya no estamos en MOVING).
 *     → Iniciar alertTimer SOLO si venimos de un estado distinto a ALERT
 *       (evita reiniciar el timer si seguimos en ALERT por nuevo movimiento —
 *        el reinicio lo hace el loop principal por separado).
 *     → FLAG_HIGH_FREQ_MODE: commTask aumenta frecuencia de telemetría.
 *     → alertIsHard según el evento: HARD/IMPACT/CMD → sirena; SOFT → beeps.
 *
 *   STATE_PURSUIT:
 *     → Buzzer ON (sirena continua).
 *     → Motor CORTADO (ENGINE_CUT activo — medida de seguridad máxima).
 *     → Todos los timers detenidos (PURSUIT no tiene timeout automático).
 *     → Todos los flags activados: FLAG_SYSTEM_ARMED, FLAG_REMOTE_ALERT, FLAG_HIGH_FREQ_MODE.
 *     → Solo DISARM_CMD puede sacar el sistema de PURSUIT.
 *
 * NOTA SOBRE prevState == STATE_ALERT:
 *   La condición `if (prevState != STATE_ALERT)` antes de startAlertTimer() evita
 *   que una transición de ALERT → ALERT (por movimiento en ALERT) reinicie el timer
 *   dos veces. El reinicio real por movimiento adicional ocurre en el loop de controlTask
 *   mediante una verificación separada.
 *
 * @param newState   El estado al que transitó la máquina de estados.
 * @param prevState  El estado previo (para detectar transiciones dentro del mismo estado).
 * @param trigger    El evento que disparó la transición (para determinar alertIsHard).
 */
static void applyStateEffects(SystemState_t newState, SystemState_t prevState,
                               const EventMessage_t& trigger) {
    switch (newState) {

        case STATE_IDLE:
            MosfetControl::setBuzzer(false);
            // NO limpiar s_motorManualCut aquí: el app controla cuándo restaurar el motor
            // vía CMD|ENGINE_RESTORE. Si limpiáramos en DISARM, el motor se restauraría
            // aunque el usuario tuviera "restaurar al desarmar" desactivado.
            MosfetControl::setEngineCut(s_motorManualCut);
            stopMovingTimer();
            stopAlertTimer();
            alertIsHard = false;
            // Limpiar los flags de alerta del EventGroup para que commTask
            // y bleTask dejen de estar en modo de alta frecuencia.
            xEventGroupClearBits(xSystemFlags,
                                 FLAG_REMOTE_ALERT | FLAG_HIGH_FREQ_MODE);
            // FLAG_SYSTEM_ARMED se mantiene: systemArmed (manejado por StateMachine)
            // es la fuente de verdad. El EventGroup refleja su valor pero no se toca aquí.
            ESP_LOGI(TAG, "Efectos IDLE aplicados (armed=%d)", (int)systemArmed);
            break;

        case STATE_MOVING:
            MosfetControl::setBuzzer(false);
            MosfetControl::setEngineCut(s_motorManualCut);  // conservar corte preventivo si activo
            stopAlertTimer();  // Por si venimos de ALERT (raro pero posible con DISARM+rearm)
            startMovingTimer();
            // Sin FLAG_HIGH_FREQ_MODE todavía: aún no es alerta confirmada.
            // commTask sigue a frecuencia normal (heartbeat o 30s según plan).
            ESP_LOGI(TAG, "Efectos MOVING aplicados — filtrando movimiento");
            break;

        case STATE_ALERT:
            MosfetControl::setEngineCut(s_motorManualCut);  // respetar corte preventivo si activo
            stopMovingTimer();
            if (prevState != STATE_ALERT) {
                // Solo iniciar el timer de 30s si es una transición nueva a ALERT.
                // Si ya estábamos en ALERT y llegó otro movimiento, el loop principal
                // reinicia el timer con startAlertTimer() de forma independiente.
                startAlertTimer();
            }
            xEventGroupSetBits(xSystemFlags, FLAG_HIGH_FREQ_MODE);

            // Determinar intensidad del buzzer según el evento que disparó la alerta.
            // Eventos severos → sirena inmediata.
            // Movimiento suave que escaló → patrón de beeps (puede ser falsa alarma).
            alertIsHard = (trigger.event == EVENT_MOVEMENT_HARD  ||
                           trigger.event == EVENT_IMPACT_DETECTED ||
                           trigger.event == EVENT_TRIGGER_ALERT_CMD);

            if (alertIsHard) {
                MosfetControl::setBuzzer(true);  // Sirena continua al instante
                ESP_LOGW(TAG, "ALERT fuerte → sirena continua activada");
            } else {
                MosfetControl::setBuzzer(false);  // El tick de LEDs aplicará el patrón de beeps
                ESP_LOGW(TAG, "ALERT suave → patrón de beeps activado");
            }
            break;

        case STATE_PURSUIT:
            MosfetControl::setBuzzer(true);
            MosfetControl::setEngineCut(true);  // CORTE DE MOTOR — acción crítica e irreversible hasta DISARM
            s_motorManualCut = true;             // sincronizar flag: PURSUIT implica motor siempre cortado
            stopMovingTimer();
            stopAlertTimer();
            alertIsHard = true;  // Para que el tick no interfiera con el buzzer
            // Activar todos los flags de máxima urgencia:
            // FLAG_SYSTEM_ARMED: el sistema sigue armado.
            // FLAG_REMOTE_ALERT: commTask envía con máxima prioridad.
            // FLAG_HIGH_FREQ_MODE: telemetría cada 10s.
            xEventGroupSetBits(xSystemFlags,
                               FLAG_SYSTEM_ARMED | FLAG_REMOTE_ALERT | FLAG_HIGH_FREQ_MODE);
            ESP_LOGE(TAG, "Efectos PURSUIT aplicados — CORTE DE MOTOR ACTIVO");
            break;
    }
}

// ─── Tarea principal ─────────────────────────────────────────────────────────────

/**
 * @brief Función principal de la tarea FreeRTOS del controlador central.
 *
 * PROPÓSITO:
 *   Ejecutar el loop principal de control: consumir eventos de xEventQueue,
 *   procesarlos con StateMachine, aplicar efectos físicos, y actualizar los
 *   patrones de LEDs y buzzer en cada tick de 250ms.
 *
 * FLUJO DE ARRANQUE:
 *   1. MosfetControl::init() → inicializar GPIOs (buzzer=OFF, motor=ON, LEDs=OFF).
 *   2. stateMachine.init()   → systemArmed=false, currentSystemState=IDLE.
 *   3. prevState = IDLE, ledTick = 0.
 *
 * LOOP (tick = 250ms):
 *   A. xQueueReceive(xEventQueue, 250ms timeout):
 *      - Si recibe mensaje:
 *        1. Guardar prevState.
 *        2. processEvent(msg) → retorna true si hubo cambio relevante.
 *        3. Si changed: applyStateEffects(newState, prevState, msg).
 *        4. Si MOVING + MOVEMENT_SOFT: reiniciar movingTimer (observación continua).
 *        5. Si ALERT + movimiento: reiniciar alertTimer (la moto sigue en movimiento).
 *      - Si timeout (250ms sin evento): solo ejecutar el tick de LEDs/buzzer.
 *   B. MosfetControl::applyLedPattern(ledTick) → actualizar patrón de LEDs.
 *   C. Si ALERT suave: setBuzzer((ledTick % 8) < 2) → beeps de 500ms cada 2s.
 *   D. ledTick++
 *
 * NOTA SOBRE EL TIMER REINICIO EN ALERT:
 *   La condición `if (STATE_ALERT && (SOFT|HARD|IMPACT))` en el loop reinicia
 *   el alertTimer de 30s cada vez que llega movimiento en ALERT. Esto evita
 *   que la alerta expire mientras la moto está en movimiento activo. Sin este
 *   reinicio, el ladrón podría esperar los 30s para que la alerta se apague.
 *
 * NOTA SOBRE EL PATRÓN DE BEEPS:
 *   Se implementa directamente en el loop del tick en lugar de en applyStateEffects()
 *   porque es un comportamiento continuo (patrón repetitivo), no un efecto de una sola vez.
 *   La condición `!alertIsHard` evita que el patrón de beeps interfiera con la sirena.
 *
 * @param pvParameters No usado. Requerido por la firma de tarea FreeRTOS.
 */
void controlTask(void* pvParameters) {
    ESP_LOGI(TAG, "Control task iniciada");

    // Inicializar GPIOs de hardware. Si falla (improbable con GPIO), el sistema
    // queda en estado indefinido — no hay mecanismo de recuperación desde aquí.
    MosfetControl::init();

    StateMachine stateMachine;
    stateMachine.init();

    SystemState_t prevState = STATE_IDLE;
    EventMessage_t msg;
    uint32_t ledTick = 0;

    // QUEUE_TIMEOUT define el tick del loop. No es solo un timeout de cola:
    // es el ritmo de actualización de los patrones de LEDs y buzzer.
    const TickType_t QUEUE_TIMEOUT = pdMS_TO_TICKS(LED_TICK_MS);

    ESP_LOGI(TAG, "Sistema listo. Estado: IDLE | Armado: NO");

    while (true) {
        // Esperar un evento o expirar el timeout de 250ms.
        // Si hay evento: procesarlo. Si no: solo hacer el tick de LEDs.
        BaseType_t received = xQueueReceive(xEventQueue, &msg, QUEUE_TIMEOUT);

        if (received == pdTRUE) {
            // ── Corte preventivo de motor (bypass de la state machine) ─────────────
            // EVENT_ENGINE_CUT_SILENT corta el relé sin cambiar estado ni activar
            // sirena — es preventivo. El flag s_motorManualCut persiste a través de
            // todas las transiciones hasta EVENT_ENGINE_RESTORE o DISARM.
            if (msg.event == EVENT_ENGINE_CUT_SILENT) {
                s_motorManualCut = true;
                MosfetControl::setEngineCut(true);
                ESP_LOGI(TAG, "[MOTOR] Corte preventivo activado (estado: %s)",
                         StateMachine::stateName(stateMachine.getState()));

            // ── Sirena manual (bypass de la state machine) ───────────────────────
            // Solo enciende/apaga el buzzer sin afectar estado, timers ni motorCut.
            // Uso: localizar la moto a distancia (bocina de búsqueda).
            } else if (msg.event == EVENT_SIREN_ON) {
                MosfetControl::setBuzzer(true);
                ESP_LOGI(TAG, "[SIREN] Sirena manual ON (estado: %s)",
                         StateMachine::stateName(stateMachine.getState()));
            } else if (msg.event == EVENT_SIREN_OFF) {
                MosfetControl::setBuzzer(false);
                ESP_LOGI(TAG, "[SIREN] Sirena manual OFF (estado: %s)",
                         StateMachine::stateName(stateMachine.getState()));
            } else {
                if (msg.event == EVENT_ENGINE_RESTORE) {
                    // Restaurar relé directamente sin importar el estado actual.
                    // Antes solo limpiábamos el flag y esperábamos que applyStateEffects
                    // lo propagara, pero si el estado no cambia (STATE_IDLE+RESTORE),
                    // processEvent retorna false → applyStateEffects nunca se llama → relé queda cortado.
                    s_motorManualCut = false;
                    MosfetControl::setEngineCut(false);
                    ESP_LOGI(TAG, "[MOTOR] Motor restaurado directamente (estado: %s)",
                             StateMachine::stateName(stateMachine.getState()));
                }

                prevState = stateMachine.getState();
                bool changed = stateMachine.processEvent(msg);

                if (changed) {
                    SystemState_t newState = stateMachine.getState();

                    // Log solo cuando el estado cambió realmente (no cuando changed=true
                    // por reinicio de timer en MOVING+SOFT que no cambia el estado).
                    if (newState != prevState) {
                        ESP_LOGI(TAG, "Estado: %s → %s  [%s]",
                                 StateMachine::stateName(prevState),
                                 StateMachine::stateName(newState),
                                 StateMachine::modeName(StateMachine::currentMode()));
                    }

                    applyStateEffects(newState, prevState, msg);

                    // Notificar estado ARM/DISARM a la app via BLE.
                    if (msg.event == EVENT_ARM_CMD || msg.event == EVENT_DISARM_CMD) {
                        bleNotifyState(systemArmed);

                        // Confirmación auditiva: 1 beep al armar, 2 beeps al desarmar.
                        // Dispara la secuencia; el tick de 250ms la ejecuta paso a paso.
                        // No bloqueante: el loop principal continúa sin delay.
                        beepStep = 0;
                        beepMax  = (msg.event == EVENT_ARM_CMD) ? 2U : 4U;
                        ESP_LOGI(TAG, "Beep ARM/DISARM: %u steps programados", beepMax);
                    }
                }

                // Reiniciar timer de MOVING cuando llega movimiento suave adicional.
                // La StateMachine ya retornó changed=true y NO transicionó (seguimos en MOVING).
                // Esto extiende la ventana de observación de 15s con cada movimiento suave.
                if (stateMachine.getState() == STATE_MOVING &&
                    msg.event == EVENT_MOVEMENT_SOFT) {
                    startMovingTimer();
                }

                // Reiniciar timer de ALERT cuando llega movimiento en ALERT.
                // Previene que la alerta expire mientras la moto sigue en movimiento.
                // La alerta solo expira si hay 30s consecutivos SIN ningún movimiento.
                if (stateMachine.getState() == STATE_ALERT &&
                    (msg.event == EVENT_MOVEMENT_SOFT ||
                     msg.event == EVENT_MOVEMENT_HARD ||
                     msg.event == EVENT_IMPACT_DETECTED)) {
                    startAlertTimer();
                }
            }
        }

        // ── Auto-ARM por inactividad (solo PLAN_PREMIUM) ──────────────────────────
        // Cada 250ms se evalúa si la moto lleva >autoArmDelayMs quieta y sin armar.
        // Solo actúa en STATE_IDLE para no interferir con MOVING/ALERT/PURSUIT.
        // lastAutoArmFiredUs evita re-armado inmediato tras DISARM sin movimiento.
        if (planType == PLAN_PREMIUM && autoArmEnabled &&
            !systemArmed && stateMachine.getState() == STATE_IDLE) {

            uint64_t nowUs = esp_timer_get_time();
            static uint64_t lastAutoArmFiredUs = 0;
            uint64_t delayUs = (uint64_t)autoArmDelayMs * 1000ULL;

            bool idleEnough   = (nowUs - lastMovementTimestamp) >= delayUs;
            bool cooldownOk   = (nowUs - lastAutoArmFiredUs)    >= delayUs;

            if (idleEnough && cooldownOk) {
                EventMessage_t autoArmMsg = {};
                autoArmMsg.event = EVENT_ARM_CMD;
                if (xQueueSend(xEventQueue, &autoArmMsg, 0) == pdTRUE) {
                    lastAutoArmFiredUs = nowUs;
                    ESP_LOGI(TAG, "[AUTO-ARM] Armando por inactividad (>%lus quieto)",
                             (unsigned long)(autoArmDelayMs / 1000UL));
                }
            }
        }

        // ── Tick de LEDs (siempre, cada 250ms) ───────────────────────────────────
        // applyLedPattern usa ledTick para calcular el patrón visual actual.
        // Funciona para todos los estados: latido, fijo, parpadeo, alternado, sirena.
        MosfetControl::applyLedPattern(ledTick);

        // ── Patrón de beeps en ALERT suave (siempre, si aplica) ──────────────────
        // 2 beeps cortos (500ms = 2 ticks) cada 2s (8 ticks × 250ms).
        // Ticks 0-1 del ciclo: ON. Ticks 2-7: OFF.
        // Si alertIsHard=true, el buzzer ya está ON continuo desde applyStateEffects
        // y esta condición es false — no interferimos con la sirena.
        if (stateMachine.getState() == STATE_ALERT && !alertIsHard) {
            MosfetControl::setBuzzer((ledTick % 8) < 2);
        }

        // ── Beep de confirmación ARM/DISARM ───────────────────────────────────
        // Solo activo en STATE_IDLE: en ALERT/PURSUIT el buzzer ya tiene su lógica.
        // Even step → ON, odd step → OFF. Cada step dura 1 tick = 250ms.
        // ARM:    2 steps → 1 beep (250ms ON + 250ms OFF).
        // DISARM: 4 steps → 2 beeps (ON+OFF+ON+OFF, 1s total).
        if (beepMax > 0 && stateMachine.getState() == STATE_IDLE) {
            MosfetControl::setBuzzer((beepStep % 2U) == 0U);
            beepStep++;
            if (beepStep >= beepMax) {
                MosfetControl::setBuzzer(false);
                beepMax = 0;  // Secuencia completada — buzzer queda apagado
            }
        }

        ledTick++;
    }
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/control_task.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo es el "cuerpo" del sistema Argus. Mientras state_machine.cpp
   decide QUÉ estado tiene el sistema, control_task.cpp decide QUÉ hace el
   hardware en respuesta a ese estado.

   Hay tres capas de lógica:
   1. Procesamiento de eventos: recibir eventos de la cola y pasarlos a la
      máquina de estados.
   2. Aplicación de efectos: activar/desactivar buzzer, cortar motor, gestionar
      timers de timeout.
   3. Tick de LEDs/buzzer: cada 250ms, actualizar los patrones visuales y sonoros
      independientemente de si llegó un evento o no.

   PSEUDOCÓDIGO DEL LOOP PRINCIPAL:
   ```
   init MosfetControl, StateMachine
   loop:
     msg = xQueueReceive(timeout=250ms)
     if msg:
       changed = stateMachine.processEvent(msg)
       if changed:
         applyStateEffects(newState, prevState, msg)
       if MOVING + MOVEMENT_SOFT:  startMovingTimer()   // reiniciar ventana
       if ALERT + movimiento:       startAlertTimer()   // reiniciar timeout
     applyLedPattern(ledTick)                           // siempre
     if ALERT_suave: setBuzzer(ledTick%8 < 2)           // beeps
     ledTick++
   ```

   TABLA DE TIMERS:
   Timer          | Duración | Dispara             | Reiniciado por
   ---------------|----------|---------------------|------------------------
   movingTimer    | 15s      | EVENT_MOVING_TIMEOUT | EVENT_MOVEMENT_SOFT en MOVING
   alertTimer     | 30s      | EVENT_ALERT_TIMEOUT  | Cualquier movimiento en ALERT

   DEUDA TÉCNICA:
   1. applyStateEffects() con ALERT no reinicia el timer si prevState == STATE_ALERT.
      El reinicio lo hace el loop principal por separado. Esto duplica lógica.
   2. esp_timer_stop() ignorado si retorna ESP_ERR_INVALID_STATE — aceptable pero
      sería más correcto verificar esp_timer_is_active() antes de stop.
   3. Si la cola xEventQueue está llena al expirar movingTimer o alertTimer,
      el timeout se pierde silenciosamente. No hay reintento.
   4. No hay registro de cuántas veces se reinició el movingTimer o alertTimer,
      lo cual dificultaría depurar "¿por qué la alerta no expiró?".

   ═══════════════════════════════════════════════════════════ */
