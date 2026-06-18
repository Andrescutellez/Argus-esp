/**
 * @file sync_manager.cpp
 * @brief Implementación del gestor de sincronización TCP con backoff y watchdog.
 *
 * PROPÓSITO:
 *   Implementar el ciclo de drenado de EventQueue hacia el backend TCP con las
 *   siguientes garantías de confiabilidad:
 *   - Rate-limiting: máximo 5 eventos por tick (protege el modem y la CPU).
 *   - Backoff exponencial: en fallo, esperar progresivamente más antes de reintentar.
 *   - Watchdog: si hay eventos pendientes y no hay éxito en 15 min → recover().
 *   - Métricas: totalSent/totalFailed/backoffLevel para diagnóstico desde commTask.
 *
 * CICLO DE VIDA TÍPICO (escenario exitoso):
 *
 *   1. commTask llama submitGps() → EventQueue.enqueue(ev, NORMAL).
 *   2. commTask llama tick(now):
 *      a. checkWatchdog() → sin timeout.
 *      b. isInBackoff() → false (primer tick).
 *      c. queue.dequeue(ev) → obtener evento GPS.
 *      d. trySend(ev) → drv.tcpSend(ev.payload) → ESP_OK.
 *      e. onSuccess() → metrics.totalSent++, backoffLevel=0, lastSuccessAt=now.
 *   3. tick() retorna. commTask sigue su loop con vTaskDelay(1s).
 *
 * CICLO DE VIDA TÍPICO (escenario de fallo + backoff):
 *
 *   1. trySend() → drv.tcpSend() → ESP_FAIL (sin red).
 *   2. onFailure():
 *      a. metrics.totalFailed++, consecutiveFailures++.
 *      b. queue.requeue(ev) → ev.retryCount=1, vuelve a cola.
 *      c. scheduleBackoff() → backoffLevel=1, backoffUntilUs = now + 2000ms.
 *   3. Siguiente tick (1s después):
 *      a. isInBackoff() → true (faltan ~1s). tick() retorna sin enviar.
 *   4. Siguiente tick (2s después):
 *      a. isInBackoff() → false. Intenta de nuevo.
 *      b. Si falla de nuevo: backoffLevel=2, esperar 5s. Y así sucesivamente.
 *   5. Cuando tcpSend() tiene éxito:
 *      a. onSuccess() → consecutiveFailures=0, backoffLevel=0, backoffUntilUs=0.
 *
 * CICLO DE WATCHDOG (modem zombie):
 *
 *   Situación: cola tiene eventos, pero drv.tcpSend() falla siempre.
 *   Después de 15 minutos (SYNC_WATCHDOG_TIMEOUT_US):
 *   1. checkWatchdog() detecta sinceLastSuccess >= 15min.
 *   2. metrics.state = SYNC_RECOVERING. drv.recover().
 *   3. resetBackoff() → siguiente tick intenta de nuevo.
 *   4. metrics.lastSuccessAt = now (para no disparar inmediatamente de nuevo).
 *
 * DEPENDENCIAS:
 *   - sync_manager.h: definiciones, constantes, SyncMetrics_t, SyncState_t
 *   - event_queue.h:  EventQueue (dequeue, requeue, markSent)
 *   - a7670_driver.h: A7670Driver (tcpSend, recover)
 *   - esp_timer.h:    esp_timer_get_time() en onSuccess() e isHealthy()
 *   - freertos/task.h: vTaskDelay() entre envíos del mismo tick
 *
 * VARIABLES CRÍTICAS:
 *   - metrics:         SyncMetrics_t completo. Único estado mutable del manager.
 *   - backoffUntilUs:  Timestamp hasta el que no se enviará. 0 = sin backoff activo.
 *   - tickCounter:     Contador de ticks para log periódico. Nunca se resetea.
 *   - deviceId[20]:    Para logging. Sin uso en la lógica de envío.
 *
 * CONCURRENCIA:
 *   SyncManager NO es thread-safe. Solo commTask llama sus métodos.
 *   La EventQueue inyectada tiene su propio mutex interno.
 *   A7670Driver: ver driver — tcpSend() puede bloquear hasta que UART responde.
 *
 * @module components/services/sync_manager
 */

#include "sync_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// TAG de logging para todos los mensajes del SyncManager.
static const char* TAG = "SyncMgr";

// ─── Constructor / init() ────────────────────────────────────────────────────

/**
 * @brief Constructor — inicializa métricas a cero y almacena referencias.
 *
 * PROPÓSITO:
 *   Configurar el estado inicial del SyncManager. Las referencias a driver y queue
 *   se almacenan para uso en tick() y submit*(). Las métricas se inicializan a cero.
 *
 * NOTA: backoffUntilUs=0 → sin backoff activo desde el inicio.
 *   metrics.state=SYNC_IDLE → estado de reposo hasta que haya eventos.
 *   lastSuccessAt se inicializa en init() para tener una referencia de tiempo válida.
 */
SyncManager::SyncManager(A7670Driver& drv, EventQueue& queue)
    : drv(drv),
      queue(queue),
      backoffUntilUs(0),
      tickCounter(0)
{
    memset(&metrics, 0, sizeof(metrics));
    memset(deviceId, 0, sizeof(deviceId));
    metrics.state = SYNC_IDLE;
}

/**
 * @brief Inicializa el manager con el ID del dispositivo.
 *
 * PROPÓSITO:
 *   Almacenar deviceId para logging y establecer lastSuccessAt al tiempo actual.
 *   Si lastSuccessAt quedara en 0, checkWatchdog() dispararía en el primer tick
 *   si la cola tiene eventos (sinceLastSuccess sería ~boot_time_us).
 *
 * FLUJO:
 *   1. Copiar deviceId con strncpy (null-terminado garantizado).
 *   2. metrics.lastSuccessAt = esp_timer_get_time() → referencia de tiempo inicial.
 *   3. Log con configuración de parámetros.
 *
 * @param devId String "ARGUS-AABBCCDD". Si nullptr, deviceId queda vacío.
 * @return ESP_OK siempre (no hay operación que pueda fallar aquí).
 */
esp_err_t SyncManager::init(const char* devId) {
    if (devId != nullptr) {
        strncpy(deviceId, devId, sizeof(deviceId) - 1);
        deviceId[sizeof(deviceId) - 1] = '\0';
    }
    // Inicializar lastSuccessAt al tiempo actual para que el watchdog de 15min
    // cuente desde ahora, no desde el epoch del chip.
    metrics.lastSuccessAt = esp_timer_get_time();
    ESP_LOGI(TAG, "Inicializado (device=%s, watchdog=%llu min, max/tick=%d, backoff=%d niveles)",
             deviceId,
             (unsigned long long)(SYNC_WATCHDOG_TIMEOUT_US / 60000000ULL),
             SYNC_MAX_PER_TICK, BACKOFF_LEVELS);
    return ESP_OK;
}

// ─── tick() ──────────────────────────────────────────────────────────────────
//
// Punto de entrada del loop de commTask. Se llama cada ~1 segundo.
//
// Flujo:
//   1. Verificar watchdog de conectividad.
//   2. Si está en backoff → retornar inmediatamente (no bloquear).
//   3. Si hay eventos en cola → enviar hasta SYNC_MAX_PER_TICK.
//   4. En fallo → onFailure() + break (no enviar más en este tick).

/**
 * @brief Ciclo principal de sincronización. Llamar una vez por segundo desde commTask.
 *
 * PROPÓSITO:
 *   Drenar la EventQueue enviando eventos vía TCP, con rate-limiting y backoff.
 *   Si está en backoff, retorna inmediatamente sin bloquear.
 *
 * FLUJO:
 *   1. tickCounter++ → log periódico cada 60 ticks.
 *   2. checkWatchdog(nowUs) → detectar zombie modem.
 *   3. Si isInBackoff(nowUs): metrics.state=BACKOFF, retornar.
 *   4. Si queue.isEmpty(): metrics.state=IDLE, retornar.
 *   5. Loop de envío (hasta SYNC_MAX_PER_TICK):
 *      a. queue.dequeue(ev).
 *      b. trySend(ev).
 *      c. Si ok: onSuccess() + sent++.
 *      d. Si falla: onFailure() + break (el backoff activo bloqueará el próximo tick).
 *      e. Pausa de SYNC_INTER_SEND_MS=400ms entre envíos (si hay más por enviar).
 *   6. Actualizar state=IDLE si la cola quedó vacía.
 *
 * NOTA SOBRE LA PAUSA ENTRE ENVÍOS:
 *   vTaskDelay(400ms) entre envíos del mismo tick da tiempo al modem A7670 entre
 *   comandos AT. Sin esta pausa, el segundo tcpSend() puede llegar mientras el
 *   modem procesa la respuesta del primero, causando errores "BUSY".
 *   La pausa solo ocurre si hay más eventos y no se alcanzó SYNC_MAX_PER_TICK.
 *
 * @param nowUs Timestamp actual en µs (esp_timer_get_time() desde commTask).
 */
void SyncManager::tick(uint64_t nowUs) {
    tickCounter++;

    // Log de métricas periódico: cada SYNC_METRICS_LOG_PERIOD ticks (~60s).
    if (tickCounter % SYNC_METRICS_LOG_PERIOD == 0) {
        logStatus();
    }

    // Verificar si el modem está en estado zombie (red activa pero sin tráfico exitoso).
    checkWatchdog(nowUs);

    // Si estamos en backoff: actualizar estado y retornar sin enviar.
    // El backoff es no-bloqueante: solo comparamos timestamps.
    if (isInBackoff(nowUs)) {
        if (metrics.state != SYNC_BACKOFF) {
            metrics.state = SYNC_BACKOFF;
        }
        return;
    }

    if (queue.isEmpty()) {
        metrics.state = SYNC_IDLE;
        return;
    }

    // ── Enviar hasta SYNC_MAX_PER_TICK eventos ──────────────────────────────
    metrics.state = SYNC_SENDING;
    int sent = 0;

    while (sent < SYNC_MAX_PER_TICK && !queue.isEmpty()) {
        QueuedEvent_t ev;
        if (!queue.dequeue(ev)) break;  // Defensivo: isEmpty() ya verificó

        ESP_LOGD(TAG, "Enviando evento [tipo=%d prio=%d retry=%d] payload=%.40s...",
                 ev.type, ev.priority, ev.retryCount, ev.payload);

        bool ok = trySend(ev);

        if (ok) {
            onSuccess(ev);
            sent++;
        } else {
            // En fallo: reencolar con retryCount++ y aplicar backoff.
            // Romper el loop: no tiene sentido enviar más si el canal está caído.
            onFailure(ev);
            break;
        }

        // Pausa entre envíos consecutivos del mismo tick.
        // Solo si hay más eventos por enviar Y no alcanzamos el límite.
        if (!queue.isEmpty() && sent < SYNC_MAX_PER_TICK) {
            vTaskDelay(pdMS_TO_TICKS(SYNC_INTER_SEND_MS));
        }
    }

    if (queue.isEmpty()) {
        metrics.state = SYNC_IDLE;
    }
}

// ─── trySend() ───────────────────────────────────────────────────────────────
//
// Encapsula el envío TCP via A7670Driver.
// En la versión actual es una llamada directa a tcpSend().
// En versiones futuras podría agregar packaging MQTT o headers HTTP.

/**
 * @brief Intenta enviar el evento via drv.tcpSend().
 *
 * PROPÓSITO:
 *   Capa de abstracción sobre el envío TCP. Actualmente delega directamente
 *   a drv.tcpSend(ev.payload). Si en el futuro se cambia el protocolo de
 *   transporte (MQTT, HTTP), solo este método necesita cambiar.
 *
 * NOTA: drv.tcpSend() puede bloquearse hasta que UART recibe la respuesta del modem.
 *   Con el modem A7670 a 115200 baud, un frame de 200 bytes tarda ~20ms.
 *   La función puede tardar hasta varios segundos si el modem está ocupado.
 *
 * @param ev El evento a enviar. Solo se usa ev.payload (frame TCP completo).
 * @return true si drv.tcpSend() retornó ESP_OK, false en cualquier otro caso.
 */
bool SyncManager::trySend(QueuedEvent_t& ev) {
    esp_err_t ret = drv.tcpSend(ev.payload);
    return (ret == ESP_OK);
}

// ─── onSuccess() ─────────────────────────────────────────────────────────────

/**
 * @brief Actualiza métricas y resetea backoff tras un envío exitoso.
 *
 * PROPÓSITO:
 *   Registrar el éxito del envío: incrementar totalSent, actualizar lastSuccessAt
 *   (para que el watchdog sepa que el canal está activo), y resetear el backoff
 *   para que el próximo tick pueda enviar sin esperar.
 *
 * EFECTOS:
 *   - queue.markSent() → stats.totalSent++ en EventQueue.
 *   - metrics.totalSent++ → para detección de cambios en commTask.
 *   - consecutiveFailures = 0 → resetear contador de fallos.
 *   - lastSuccessAt = esp_timer_get_time() → timestamp del éxito.
 *   - backoffLevel = 0, backoffUntilUs = 0 → sin backoff.
 *   - state = SYNC_SENDING (seguimos enviando en este tick).
 *
 * @param ev El evento que fue enviado exitosamente (para logging).
 */
void SyncManager::onSuccess(const QueuedEvent_t& ev) {
    queue.markSent();
    metrics.totalSent++;
    metrics.consecutiveFailures = 0;
    metrics.lastSuccessAt       = esp_timer_get_time();
    metrics.backoffLevel        = 0;
    backoffUntilUs              = 0;
    metrics.state               = SYNC_SENDING;

    ESP_LOGI(TAG, "OK [tipo=%d prio=%d retry=%d] — cola=%lu",
             ev.type, ev.priority, ev.retryCount, (unsigned long)queue.size());
}

// ─── onFailure() ─────────────────────────────────────────────────────────────
//
// Gestiona el fallo de un envío:
//   - Incrementa contadores de fallo.
//   - Re-encola el evento si no excedió retries.
//   - Aplica backoff exponencial para el próximo intento.

/**
 * @brief Gestiona el fallo de un envío: actualiza métricas, reencola, aplica backoff.
 *
 * PROPÓSITO:
 *   Registrar el fallo, dar otra oportunidad al evento (si tiene retries restantes),
 *   y programar el backoff para no bombardear el servidor con reintentos inmediatos.
 *
 * FLUJO:
 *   1. metrics.totalFailed++, consecutiveFailures++.
 *   2. Log con estado actual.
 *   3. queue.requeue(ev):
 *      - Incrementa ev.retryCount.
 *      - Si retryCount > maxRetries (5 o 10 para CRITICAL): descarta el evento.
 *      - Si no: enqueue(ev) → vuelve a la cola con nueva oportunidad.
 *   4. scheduleBackoff(now) → calcula el próximo tiempo permitido de envío.
 *
 * NOTA: No cambia metrics.state aquí — tick() lo cambia a BACKOFF después de retornar.
 *
 * @param ev El evento que falló. Se modifica: ev.retryCount++ en queue.requeue().
 */
void SyncManager::onFailure(QueuedEvent_t& ev) {
    metrics.totalFailed++;
    metrics.consecutiveFailures++;

    ESP_LOGW(TAG, "FALLO [tipo=%d prio=%d retry=%d] consecutivos=%lu",
             ev.type, ev.priority, ev.retryCount,
             (unsigned long)metrics.consecutiveFailures);

    // Re-encolar para reintento posterior (el backoff garantiza que no se reintente
    // inmediatamente). Si se agotaron los retries, requeue() descarta el evento.
    if (!queue.requeue(ev)) {
        ESP_LOGW(TAG, "Evento descartado definitivamente (tipo=%d prio=%d)",
                 ev.type, ev.priority);
    }

    // Aplicar backoff exponencial: el próximo tick verificará isInBackoff()
    // y no enviará hasta que expire el período de espera.
    scheduleBackoff(esp_timer_get_time());
}

// ─── checkWatchdog() ─────────────────────────────────────────────────────────
//
// Detecta el estado "zombie": el módem responde AT pero no logra transmitir
// datos TCP por un tiempo prolongado. Diferente del backoff: el watchdog activa
// recover() del driver, no solo espera.

/**
 * @brief Verifica el watchdog de conectividad. Fuerza recover() si es necesario.
 *
 * PROPÓSITO:
 *   Detectar y recuperar el modem cuando está en estado zombie: aparentemente
 *   funcional (AT responde) pero no puede establecer o mantener la conexión TCP.
 *
 * FLUJO:
 *   1. Si queue.isEmpty():
 *      - resetear lastSuccessAt = now (evitar falso positivo en reposo legítimo).
 *      - Retornar.
 *   2. Calcular sinceLastSuccess = now - metrics.lastSuccessAt.
 *   3. Si sinceLastSuccess >= 15 min:
 *      a. metrics.totalWatchdogFires++.
 *      b. Log con tiempo transcurrido y número de disparo.
 *      c. metrics.state = SYNC_RECOVERING.
 *      d. drv.recover() → reiniciar pila TCP del modem.
 *      e. resetBackoff() → el próximo tick intentará sin espera.
 *      f. metrics.lastSuccessAt = now → evitar re-disparo inmediato.
 *
 * NOTA SOBRE EL RESET DE lastSuccessAt CUANDO queue.isEmpty():
 *   Si la cola está vacía legítimamente (sistema en reposo), no hay fallos de red.
 *   Actualizar lastSuccessAt en ese caso previene que el watchdog dispare tras
 *   un período largo de inactividad seguido por un evento que falla por primera vez.
 *
 * @param nowUs Timestamp actual en µs.
 */
void SyncManager::checkWatchdog(uint64_t nowUs) {
    if (queue.isEmpty()) {
        // En reposo legítimo: resetear el reloj para evitar falsos positivos.
        metrics.lastSuccessAt = nowUs;
        return;
    }

    uint64_t sinceLastSuccess = nowUs - metrics.lastSuccessAt;

    if (sinceLastSuccess >= SYNC_WATCHDOG_TIMEOUT_US) {
        metrics.totalWatchdogFires++;
        ESP_LOGE(TAG, "WATCHDOG: %llu min sin envío exitoso — forzando recovery (disparo #%lu)",
                 (unsigned long long)(sinceLastSuccess / 60000000ULL),
                 (unsigned long)metrics.totalWatchdogFires);

        metrics.state = SYNC_RECOVERING;

        // recover() reinicia la pila TCP del modem: cierra y reabre la conexión,
        // verifica AT, restablece el contexto de red.
        drv.recover();

        // Resetear backoff para que el siguiente tick intente sin espera.
        resetBackoff();
        // Actualizar lastSuccessAt para evitar re-disparo en el próximo tick.
        metrics.lastSuccessAt = nowUs;
    }
}

// ─── isInBackoff() ───────────────────────────────────────────────────────────

/**
 * @brief Retorna true si el backoff exponencial está activo.
 *
 * IMPLEMENTACIÓN:
 *   Comparación simple de timestamps. No bloquea, no consume CPU.
 *   backoffUntilUs = 0 significa sin backoff (se inicializa a 0 en el constructor).
 *   nowUs < 0 es imposible (uint64_t), así que si backoffUntilUs=0, retorna false.
 *
 * @param nowUs Timestamp actual en µs.
 * @return true si aún no ha llegado el momento de reintentar.
 */
bool SyncManager::isInBackoff(uint64_t nowUs) const {
    return (nowUs < backoffUntilUs);
}

// ─── scheduleBackoff() ───────────────────────────────────────────────────────
//
// Programa el próximo intento según la tabla de backoff exponencial.
// El nivel avanza con cada llamada y topa en BACKOFF_LEVELS-1 (60s).

/**
 * @brief Programa el próximo reintento según la tabla de backoff exponencial.
 *
 * PROPÓSITO:
 *   Calcular cuándo puede enviarse el próximo evento tras un fallo.
 *   El nivel de backoff aumenta con cada fallo consecutivo, haciendo que las
 *   esperas sean progresivamente más largas.
 *
 * FLUJO:
 *   1. Si backoffLevel < BACKOFF_LEVELS-1: backoffLevel++.
 *      (El nivel nunca supera el índice máximo de BACKOFF_MS[].)
 *   2. delayMs = BACKOFF_MS[backoffLevel].
 *   3. backoffUntilUs = nowUs + delayMs × 1000.
 *
 * NOTA: backoffLevel ya está en 0 tras un éxito (resetBackoff lo resetea).
 *   Primer fallo: 0 → 1 → BACKOFF_MS[1] = 2000ms.
 *   Segundo fallo: 1 → 2 → BACKOFF_MS[2] = 5000ms.
 *   ... hasta el máximo BACKOFF_MS[4] = 60000ms.
 *
 * @param nowUs Timestamp actual en µs (punto de partida del período de backoff).
 */
void SyncManager::scheduleBackoff(uint64_t nowUs) {
    if (metrics.backoffLevel < (uint32_t)(BACKOFF_LEVELS - 1)) {
        metrics.backoffLevel++;
    }
    uint32_t delayMs = BACKOFF_MS[metrics.backoffLevel];
    backoffUntilUs   = nowUs + (uint64_t)delayMs * 1000ULL;

    ESP_LOGW(TAG, "Backoff nivel %lu → espera %lu ms (próximo intento en ~%lu s)",
             (unsigned long)metrics.backoffLevel,
             (unsigned long)delayMs,
             (unsigned long)(delayMs / 1000));
}

// ─── resetBackoff() ──────────────────────────────────────────────────────────

/**
 * @brief Resetea el backoff a nivel 0 (sin espera activa).
 *
 * CUÁNDO SE LLAMA:
 *   - onSuccess(): cada envío exitoso resetea el backoff.
 *   - checkWatchdog(): tras drv.recover() para que el siguiente tick intente de nuevo.
 *
 * EFECTO:
 *   backoffLevel = 0 → el próximo fallo empezará con la espera mínima (2s).
 *   backoffUntilUs = 0 → isInBackoff() retorna false de inmediato.
 */
void SyncManager::resetBackoff() {
    metrics.backoffLevel = 0;
    backoffUntilUs       = 0;
}

// ─── submit*() ───────────────────────────────────────────────────────────────
//
// Todos los métodos submit*() construyen un QueuedEvent_t con los metadatos
// correspondientes y delegan a queue.enqueue(). La diferencia entre ellos es
// la prioridad y el tipo de evento.

/**
 * @brief Encola un reporte GPS regular (prioridad NORMAL).
 *
 * PROPÓSITO:
 *   Encolar el reporte GPS estándar de commTask. La prioridad NORMAL significa que
 *   puede ser desalojado de la cola si llegan eventos HIGH/CRITICAL.
 *
 * @param json  Frame TCP del protocolo Argus (con '\n' al final).
 * @param nowUs Timestamp de enqueue (para FIFO dentro del mismo nivel).
 * @return true si fue encolado, false si rechazado.
 */
bool SyncManager::submitGps(const char* json, uint64_t nowUs) {
    QueuedEvent_t ev = {};
    ev.timestamp  = nowUs;
    ev.type       = ETYPE_GPS_REPORT;
    ev.priority   = EPRIO_NORMAL;
    ev.retryCount = 0;
    strncpy(ev.payload, json, EQ_PAYLOAD_MAX_LEN - 1);
    ev.payload[EQ_PAYLOAD_MAX_LEN - 1] = '\0';  // Null-terminator garantizado

    bool ok = queue.enqueue(ev);
    if (!ok) ESP_LOGW(TAG, "submitGps: cola llena — dato descartado");
    return ok;
}

/**
 * @brief Encola un reporte GPS de alerta activa (prioridad HIGH).
 *
 * PROPÓSITO:
 *   Encolar reportes durante STATE_ALERT con mayor urgencia que GPS normal.
 *   Puede desalojar eventos NORMAL y LOW si la cola está llena.
 *
 * @param json  Frame TCP del protocolo Argus.
 * @param nowUs Timestamp de enqueue.
 * @return true si fue encolado, false si rechazado.
 */
bool SyncManager::submitAlertGps(const char* json, uint64_t nowUs) {
    QueuedEvent_t ev = {};
    ev.timestamp  = nowUs;
    ev.type       = ETYPE_ALERT;
    ev.priority   = EPRIO_HIGH;
    ev.retryCount = 0;
    strncpy(ev.payload, json, EQ_PAYLOAD_MAX_LEN - 1);
    ev.payload[EQ_PAYLOAD_MAX_LEN - 1] = '\0';

    bool ok = queue.enqueue(ev);
    if (!ok) ESP_LOGW(TAG, "submitAlertGps: rechazado (cola llena con críticos)");
    return ok;
}

/**
 * @brief Encola una alerta crítica de persecución (prioridad CRITICAL + NVS inmediato).
 *
 * PROPÓSITO:
 *   Encolar reportes de STATE_PURSUIT con máxima prioridad. Persiste en NVS
 *   inmediatamente para garantizar durabilidad ante reboot del ESP32.
 *
 * NOTA: La llamada a queue.saveToNVS() después del enqueue introduce latencia
 *   de flash (~10-50ms) en commTask. Aceptable porque PURSUIT es poco frecuente
 *   y la durabilidad es crítica en ese estado.
 *
 * @param json  Frame TCP del protocolo Argus.
 * @param nowUs Timestamp de enqueue.
 * @return true si fue encolado, false si rechazado.
 */
bool SyncManager::submitCriticalAlert(const char* json, uint64_t nowUs) {
    QueuedEvent_t ev = {};
    ev.timestamp  = nowUs;
    ev.type       = ETYPE_ALERT;
    ev.priority   = EPRIO_CRITICAL;
    ev.retryCount = 0;
    strncpy(ev.payload, json, EQ_PAYLOAD_MAX_LEN - 1);
    ev.payload[EQ_PAYLOAD_MAX_LEN - 1] = '\0';

    bool ok = queue.enqueue(ev);

    // Persistir inmediatamente en NVS para que el evento sobreviva a un reboot.
    // Los eventos CRITICAL representan la última posición conocida del ladrón.
    if (ok) {
        esp_err_t nvs = queue.saveToNVS();
        if (nvs != ESP_OK) {
            ESP_LOGW(TAG, "submitCriticalAlert: NVS save falló (%s)", esp_err_to_name(nvs));
        }
    }
    return ok;
}

/**
 * @brief Encola un heartbeat de keepalive (prioridad LOW).
 *
 * PROPÓSITO:
 *   Mantener la sesión TCP activa cuando el sistema está en reposo.
 *   Los heartbeats son opcionales — se descartan silenciosamente si la cola está llena.
 *
 * NOTA: No logueamos warning si se rechaza. Un heartbeat perdido no es un problema:
 *   el próximo ciclo de telemetría intentará otro heartbeat.
 *
 * @param json  Frame TCP del protocolo Argus (con lastKnownGps y epoch=0).
 * @param nowUs Timestamp de enqueue.
 * @return true si fue encolado, false si rechazado (silencioso).
 */
bool SyncManager::submitHeartbeat(const char* json, uint64_t nowUs) {
    QueuedEvent_t ev = {};
    ev.timestamp  = nowUs;
    ev.type       = ETYPE_HEARTBEAT;
    ev.priority   = EPRIO_LOW;
    ev.retryCount = 0;
    strncpy(ev.payload, json, EQ_PAYLOAD_MAX_LEN - 1);
    ev.payload[EQ_PAYLOAD_MAX_LEN - 1] = '\0';

    bool ok = queue.enqueue(ev);
    // Sin warning si se rechaza: heartbeats son descartables por diseño.
    return ok;
}

// ─── isHealthy() ─────────────────────────────────────────────────────────────

/**
 * @brief Indica si el canal de comunicación está en estado saludable.
 *
 * PROPÓSITO:
 *   Proporcionar una evaluación rápida del estado del canal para que commTask
 *   pueda decidir si mostrar un indicador de error en el sistema.
 *
 * LÓGICA:
 *   Saludable = (tiempo desde último éxito < 7.5 min) Y (consecutiveFailures < 5).
 *   7.5 min = mitad del watchdog (15 min): señal de advertencia temprana.
 *   consecutiveFailures < 5: el sistema está en fallo crónico si falla 5 veces seguidas.
 *
 * NOTA: esp_timer_get_time() dentro de isHealthy() — no usa el nowUs del tick
 *   porque isHealthy() puede llamarse fuera del contexto de tick().
 *
 * @return true si el canal está operativo, false si hay problemas persistentes.
 */
bool SyncManager::isHealthy() const {
    uint64_t now = esp_timer_get_time();
    uint64_t sinceSuccess = now - metrics.lastSuccessAt;
    return (sinceSuccess < SYNC_WATCHDOG_TIMEOUT_US / 2) &&
           (metrics.consecutiveFailures < 5);
}

// ─── getMetrics() / logStatus() ─────────────────────────────────────────────

/**
 * @brief Copia el snapshot de métricas actuales al struct de salida.
 *
 * PROPÓSITO:
 *   Exponer métricas a commTask para detectar cambios en totalSent/totalFailed
 *   y generar eventos EVENT_COMMS_SUCCESS/FAILURE en xEventQueue.
 *
 * NOTA: Copia directa de metrics — sin mutex porque solo commTask llama esta función.
 *
 * @param out Destino donde se copian las métricas.
 */
void SyncManager::getMetrics(SyncMetrics_t& out) const {
    out = metrics;
}

/**
 * @brief Loguea el estado completo de métricas con ESP_LOGI.
 *
 * PROPÓSITO:
 *   Proporcionar visibilidad periódica del estado del canal de comunicación.
 *   Se llama automáticamente desde tick() cada SYNC_METRICS_LOG_PERIOD ticks.
 *
 * INCLUYE:
 *   - Estado actual (IDLE/SENDING/BACKOFF/RECOVERING).
 *   - Tamaño actual y high watermark de la cola.
 *   - Totales de envíos y fallos.
 *   - Disparos del watchdog.
 *   - Nivel de backoff y fallos consecutivos.
 *   - Tiempo desde el último envío exitoso.
 */
void SyncManager::logStatus() const {
    EQStats_t qs;
    queue.getStats(qs);

    uint64_t now = esp_timer_get_time();
    uint64_t secSinceOk = (now - metrics.lastSuccessAt) / 1000000ULL;

    ESP_LOGI(TAG,
             "MÉTRICAS — estado=%d | cola=%lu/HWM=%lu | "
             "enviados=%lu fallos=%lu watchdog=%lu | "
             "backoff_lvl=%lu consec_fail=%lu | "
             "último_ok=hace %llus",
             metrics.state,
             (unsigned long)qs.currentSize,
             (unsigned long)qs.highWaterMark,
             (unsigned long)metrics.totalSent,
             (unsigned long)metrics.totalFailed,
             (unsigned long)metrics.totalWatchdogFires,
             (unsigned long)metrics.backoffLevel,
             (unsigned long)metrics.consecutiveFailures,
             (unsigned long long)secSinceOk);
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/sync_manager.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   SyncManager es el "motor de entrega" del sistema de telemetría. Cada segundo,
   commTask le dice "ve a ver si hay algo que enviar". SyncManager saca el evento
   más urgente de la cola, lo envía, y decide si fue exitoso o falló.

   Si falla, espera antes de reintentar (backoff). Si lleva 15 minutos fallando,
   reinicia el modem (watchdog). Si tiene éxito, resetea el backoff y está
   listo para el próximo envío.

   PSEUDOCÓDIGO:
   ```
   tick(now):
     si hay_log_periódico: logStatus()
     checkWatchdog(now)        ← watchdog 15 min
     si en_backoff: retornar   ← no bloqueante
     mientras eventos_en_cola y enviados < 5:
       ev = cola.dequeue()
       si tcpSend(ev.payload) == OK:
         onSuccess()           ← resetear backoff, stats++
         enviados++
         esperar 400ms         ← dar tiempo al modem
       sino:
         onFailure()           ← requeue + backoff++
         break                 ← no enviar más en este tick
   ```

   TABLA DE ESTADOS:
   Estado         | Cola   | Backoff | Descripción
   ---------------|--------|---------|------------------------
   SYNC_IDLE      | Vacía  | No      | En reposo
   SYNC_SENDING   | Activa | No      | Enviando activamente
   SYNC_BACKOFF   | Activa | Sí      | Esperando antes de reintentar
   SYNC_RECOVERING| Activa | No      | drv.recover() en progreso

   DEUDA TÉCNICA:
   1. trySend() no distingue entre errores transitorios (sin red) y permanentes
      (payload mal formado). Ambos causan backoff aunque un payload malo nunca se
      enviará exitosamente en ningún reintento.
   2. No hay circuit breaker: si consecutiveFailures > 10, podría tener sentido
      detener los reintentos y esperar un evento externo (ej. GPS fix) antes de
      continuar.
   3. deviceId[20] se usa solo para logging — no se incluye en el payload (ya
      está en el frame Argus TCP construido por commTask).

   ═══════════════════════════════════════════════════════════ */
