#pragma once

/**
 * @file sync_manager.h
 * @brief Gestor de sincronización con backoff exponencial y watchdog para el sistema Argus.
 *
 * PROPÓSITO:
 *   Orquestar el drenado de la EventQueue hacia el backend TCP via A7670Driver,
 *   gestionando automáticamente:
 *   1. Rate-limiting: máximo SYNC_MAX_PER_TICK envíos por tick (cada ~1s).
 *   2. Backoff exponencial: en caso de fallo, esperar antes de reintentar.
 *   3. Watchdog de conectividad: si no hay éxito en 15 minutos con cola activa,
 *      forzar recover() del driver A7670.
 *   4. Métricas exportables: totalSent/totalFailed/backoffLevel/estado actual.
 *
 * ROL EN LA ARQUITECTURA:
 *   SyncManager es la "capa de confiabilidad" del canal TCP:
 *
 *   commTask → submitGps/Alert/Heartbeat() → EventQueue.enqueue()
 *   commTask → tick() cada 1s → SyncManager → EventQueue.dequeue() → A7670Driver.tcpSend()
 *
 *   En caso de fallo de red:
 *   tcpSend() falla → onFailure() → queue.requeue() + scheduleBackoff()
 *   tick() en backoff → isInBackoff() == true → no enviar hasta que expire el backoff
 *   tick() post-backoff → intenta de nuevo → si falla → backoffLevel++
 *
 * TABLA DE BACKOFF EXPONENCIAL:
 *   Nivel | Espera   | Descripción
 *   ------|----------|------------------------------------------
 *     0   |    0 ms  | (sin backoff — estado inicial)
 *     1   |  2,000ms | Primer fallo: esperar 2s
 *     2   |  5,000ms | Segundo fallo: esperar 5s
 *     3   | 15,000ms | Tercer fallo: esperar 15s
 *     4   | 30,000ms | Cuarto fallo: esperar 30s
 *     5   | 60,000ms | Quinto+ fallo: esperar 60s (máximo)
 *
 *   El nivel no baja hasta que hay un envío exitoso (onSuccess() → backoffLevel=0).
 *   Con 5 retries y 60s máximo de espera, un evento puede tardar ~2.5 minutos
 *   antes de ser descartado en condiciones de red persistentemente caída.
 *
 * WATCHDOG DE CONECTIVIDAD (15 minutos):
 *   Si la cola tiene eventos pendientes pero no hay un envío exitoso en 15 minutos,
 *   se asume que el modem está en estado zombie (responde AT pero no transmite).
 *   Acción: drv.recover() + resetBackoff(). El watchdog NO se activa cuando la
 *   cola está vacía (el sistema puede estar genuinamente inactivo).
 *
 * ESTADOS (SyncState_t):
 *   SYNC_IDLE:       Cola vacía o sin red — no hay nada que enviar.
 *   SYNC_SENDING:    Enviando activamente (dentro de un tick, en la ventana de envío).
 *   SYNC_BACKOFF:    Esperando antes de reintentar (backoff exponencial activo).
 *   SYNC_RECOVERING: drv.recover() fue llamado por el watchdog.
 *
 * CONFIGURACIÓN:
 *   SYNC_MAX_PER_TICK      = 5    eventos por llamada a tick()
 *   SYNC_INTER_SEND_MS     = 400  ms de pausa entre envíos del mismo tick
 *   SYNC_WATCHDOG_TIMEOUT  = 15min (en µs)
 *   SYNC_METRICS_LOG_PERIOD = 60 ticks (~60s)
 *   BACKOFF_MS[]           = {2000, 5000, 15000, 30000, 60000} ms
 *   BACKOFF_LEVELS         = 5
 *
 * CONCURRENCIA:
 *   SyncManager es instanciado y usado SOLO desde commTask. No es thread-safe.
 *   No necesita mutex porque ninguna otra tarea lo llama directamente.
 *   EventQueue (inyectada por referencia) SÍ tiene su propio mutex interno.
 *
 * DEPENDENCIAS:
 *   - event_queue.h:  EventQueue (cola con prioridad)
 *   - a7670_driver.h: A7670Driver (TCP send, recover)
 *   - esp_err.h:      esp_err_t para init()
 *
 * @module components/services/sync_manager
 */

#include "event_queue.h"
#include "a7670_driver.h"
#include "esp_err.h"
#include <stdint.h>

// ─── Configuración ────────────────────────────────────────────────────────────

/**
 * Máximo de eventos enviados por llamada a tick().
 * Evita que un replay masivo (tras reconexión) monopolice la CPU durante segundos
 * bloqueando otras tareas. Con SYNC_INTER_SEND_MS=400ms, 5 eventos = ~2s de envío.
 */
#define SYNC_MAX_PER_TICK          5

/**
 * Pausa entre envíos consecutivos dentro del mismo tick (ms).
 * Da tiempo al modem A7670 entre comandos AT TCP. Sin esta pausa, el modem puede
 * reportar "BUSY" al procesar el segundo envío inmediatamente después del primero.
 */
#define SYNC_INTER_SEND_MS         400

/**
 * Timeout del watchdog de conectividad.
 * Si han pasado 15 minutos desde el último envío exitoso con cola activa,
 * se asume que el modem está en estado zombie y se fuerza drv.recover().
 * 15 minutos = 15 × 60 × 1_000_000 µs.
 */
#define SYNC_WATCHDOG_TIMEOUT_US   (15ULL * 60ULL * 1000000ULL)

/**
 * Frecuencia de logging de métricas en ticks.
 * Con un tick por segundo (vTaskDelay(1000ms) en commTask), log cada 60 ticks ≈ 60s.
 */
#define SYNC_METRICS_LOG_PERIOD    60

// ─── Tabla de backoff exponencial ────────────────────────────────────────────
//
// Tiempos de espera entre reintentos en caso de fallo TCP.
// El nivel aumenta con cada fallo y se resetea con cada éxito.
// Índice 0 = primer fallo, índice 4 = fallos repetidos (máximo).
//
// Con esta tabla:
//   Fallo 1 → esperar 2s
//   Fallo 2 → esperar 5s
//   Fallo 3 → esperar 15s
//   Fallo 4 → esperar 30s
//   Fallo 5+ → esperar 60s (máximo, se repite en cada fallo adicional)
//
// constexpr → valores en ROM, no en RAM. static → visibilidad de módulo.

/** Tabla de delays de backoff en milisegundos. Acceso por BACKOFF_MS[backoffLevel]. */
static constexpr uint32_t BACKOFF_MS[]    = {2000, 5000, 15000, 30000, 60000};

/** Número de entradas en BACKOFF_MS[]. Índice máximo = BACKOFF_LEVELS - 1 = 4. */
static constexpr int      BACKOFF_LEVELS  = 5;

// ─── Estado interno del SyncManager ─────────────────────────────────────────

/**
 * @brief Estado operativo del SyncManager.
 *
 * Refleja qué está haciendo el manager en el ciclo actual.
 * Exportado en SyncMetrics_t para diagnóstico desde commTask.
 *
 * SYNC_IDLE:       Cola vacía o sin nada que hacer — estado en reposo.
 * SYNC_SENDING:    Dentro de un ciclo de envío (entre dequeue y success/failure).
 * SYNC_BACKOFF:    Esperando el tiempo de backoff — no se intenta enviar.
 * SYNC_RECOVERING: drv.recover() fue llamado — esperando que el modem se reinicie.
 */
typedef enum : uint8_t {
    SYNC_IDLE       = 0,   // Sin eventos pendientes o sin red activa
    SYNC_SENDING    = 1,   // Enviando evento activo vía TCP
    SYNC_BACKOFF    = 2,   // En espera de backoff exponencial
    SYNC_RECOVERING = 3,   // Recovery del driver A7670 en progreso
} SyncState_t;

// ─── Métricas exportables ────────────────────────────────────────────────────

/**
 * @brief Snapshot de métricas del SyncManager exportables a commTask.
 *
 * commTask monitorea totalSent y totalFailed para detectar cambios y generar
 * EVENT_COMMS_SUCCESS / EVENT_COMMS_FAILURE en xEventQueue hacia controlTask.
 *
 * CAMPOS:
 *   lastSuccessAt:         µs timestamp del último envío exitoso (desde boot).
 *                          Se usa para calcular cuánto tiempo lleva sin éxito.
 *   consecutiveFailures:   Fallos seguidos sin éxito intercalado. Se resetea a 0
 *                          en cada onSuccess(). Si > 4 → isHealthy() retorna false.
 *   backoffLevel:          Índice actual en BACKOFF_MS[]. 0=sin backoff, 4=60s max.
 *   totalSent:             Total de envíos exitosos desde el boot.
 *   totalFailed:           Total de intentos fallidos desde el boot.
 *   totalWatchdogFires:    Número de veces que el watchdog forzó recover().
 *   state:                 Estado actual del SyncManager (SyncState_t).
 */
typedef struct {
    uint64_t    lastSuccessAt;         // µs del último envío exitoso
    uint32_t    consecutiveFailures;   // Fallos consecutivos sin éxito
    uint32_t    backoffLevel;          // Nivel de backoff actual (0-4)
    uint32_t    totalSent;             // Envíos exitosos acumulados (sesión)
    uint32_t    totalFailed;           // Envíos fallidos acumulados (sesión)
    uint32_t    totalWatchdogFires;    // Disparos del watchdog de 15 minutos
    SyncState_t state;                 // Estado operativo actual
} SyncMetrics_t;

// ─── SyncManager ─────────────────────────────────────────────────────────────

/**
 * @brief Gestor de sincronización TCP con backoff exponencial y watchdog.
 *
 * PROPÓSITO:
 *   Encapsular toda la lógica de confiabilidad del canal de comunicación:
 *   rate-limiting, backoff, watchdog, y métricas. commTask solo llama tick()
 *   cada segundo y submit*() para agregar eventos — la complejidad de recuperación
 *   de red está oculta dentro de SyncManager.
 *
 * DISEÑO:
 *   - No es thread-safe: solo commTask accede a SyncManager.
 *   - Recibe A7670Driver y EventQueue por referencia (inyección de dependencias).
 *     Esto facilita testear con mocks de driver y cola si fuera necesario.
 *   - No gestiona el ciclo de vida del driver ni la cola — solo los usa.
 *
 * USO TÍPICO (dentro de commTask):
 *   SyncManager sync(a7670, eventQueue);
 *   sync.init("ARGUS-AABBCCDD");
 *   // En el loop de commTask:
 *   sync.tick(esp_timer_get_time());
 *   // Para encolar un nuevo reporte:
 *   sync.submitGps(tcpPacket, now);
 */
class SyncManager {
public:
    /**
     * @brief Constructor. Inicializa métricas a cero y almacena referencias.
     *
     * NOTA: No hace nada con el driver o la cola todavía. init() configura el
     *   estado inicial (deviceId, lastSuccessAt).
     *
     * @param drv   Referencia al driver A7670 para tcpSend() y recover().
     * @param queue Referencia a la EventQueue para dequeue/requeue/markSent.
     */
    SyncManager(A7670Driver& drv, EventQueue& queue);

    /**
     * @brief Inicializa el manager con el ID del dispositivo.
     *
     * PROPÓSITO:
     *   Almacenar el deviceId (para logging) e inicializar lastSuccessAt al tiempo
     *   actual para evitar que el watchdog dispare en el primer ciclo.
     *
     * @param deviceId String "ARGUS-AABBCCDD" derivado de la MAC del ESP32.
     * @return ESP_OK siempre (init no puede fallar en la implementación actual).
     */
    esp_err_t init(const char* deviceId);

    /**
     * @brief Punto de entrada del loop de commTask. Llamar una vez por segundo.
     *
     * PROPÓSITO:
     *   Ejecutar un ciclo completo de sincronización:
     *   1. Log de métricas periódico (cada 60 ticks).
     *   2. Verificar watchdog de conectividad.
     *   3. Si en backoff: retornar sin enviar.
     *   4. Si hay eventos: enviar hasta SYNC_MAX_PER_TICK con pausa entre envíos.
     *
     * NO BLOQUEANTE en backoff:
     *   Si isInBackoff() == true, tick() retorna inmediatamente sin esperar.
     *   El backoff se implementa comparando nowUs con backoffUntilUs (timestamp).
     *   Esto permite que commTask siga corriendo su loop (maintain, NVS, etc.)
     *   sin quedar bloqueado durante el período de backoff.
     *
     * @param nowUs Timestamp actual en µs (esp_timer_get_time()). Pasado explícitamente
     *              para evitar llamadas duplicadas al timer dentro de tick().
     */
    void tick(uint64_t nowUs);

    // ── Métodos para encolar eventos ──────────────────────────────────────────

    /**
     * @brief Encola un reporte GPS regular (prioridad NORMAL).
     *
     * PROPÓSITO:
     *   Agregar un reporte GPS periódico a la cola para envío en el próximo tick.
     *   Usado en STATE_MOVING y STATE_IDLE armado.
     *
     * @param json  Frame TCP del protocolo Argus (terminado en '\n').
     * @param nowUs Timestamp de creación del evento (µs).
     * @return true si fue encolado, false si fue rechazado (cola llena de CRITICAL).
     */
    bool submitGps(const char* json, uint64_t nowUs);

    /**
     * @brief Encola un reporte GPS de alerta activa (prioridad HIGH).
     *
     * PROPÓSITO:
     *   Agregar un reporte GPS con alta urgencia a la cola.
     *   Usado en STATE_ALERT (telemetría cada 15s con alta prioridad).
     *
     * @param json  Frame TCP del protocolo Argus.
     * @param nowUs Timestamp de creación del evento.
     * @return true si fue encolado, false si fue rechazado.
     */
    bool submitAlertGps(const char* json, uint64_t nowUs);

    /**
     * @brief Encola una alerta crítica de persecución (prioridad CRITICAL + NVS).
     *
     * PROPÓSITO:
     *   Agregar un reporte GPS de persecución activa a la cola con la máxima
     *   prioridad. Persiste inmediatamente en NVS para sobrevivir a reboots.
     *   Usado en STATE_PURSUIT (telemetría cada 10s, nunca se pierde).
     *
     * NOTA SOBRE NVS INMEDIATO:
     *   submitCriticalAlert() llama queue.saveToNVS() inmediatamente tras encolar.
     *   Esto introduce latencia de flash (10-50ms) en el hilo de commTask.
     *   Es aceptable porque STATE_PURSUIT ocurre raramente y la durabilidad del
     *   dato es más importante que la latencia en ese momento.
     *
     * @param json  Frame TCP del protocolo Argus.
     * @param nowUs Timestamp de creación del evento.
     * @return true si fue encolado, false si fue rechazado.
     */
    bool submitCriticalAlert(const char* json, uint64_t nowUs);

    /**
     * @brief Encola un heartbeat de keepalive (prioridad LOW).
     *
     * PROPÓSITO:
     *   Mantener la sesión TCP activa cuando el sistema está inactivo.
     *   El heartbeat es el primer evento en ser desalojado si la cola está llena.
     *   Si se rechaza, no se loguea warning (es aceptable perder heartbeats).
     *
     * @param json  Frame TCP del protocolo Argus (con posición de lastKnownGps).
     * @param nowUs Timestamp de creación del evento.
     * @return true si fue encolado, false si fue rechazado (silencioso).
     */
    bool submitHeartbeat(const char* json, uint64_t nowUs);

    // ── Estado y métricas ─────────────────────────────────────────────────────

    /**
     * @brief Retorna el estado operativo actual.
     * Sin mutex — solo commTask llama a SyncManager.
     * @return Estado actual (SYNC_IDLE, SENDING, BACKOFF, RECOVERING).
     */
    SyncState_t getState()  const { return metrics.state; }

    /**
     * @brief Indica si el canal de comunicación está saludable.
     *
     * LÓGICA:
     *   Saludable = (tiempo desde último éxito < 7.5 min) Y (consecutiveFailures < 5).
     *   El umbral de 7.5 min es la mitad del watchdog (15 min), como señal de advertencia.
     *
     * @return true si el canal está saludable, false si hay problemas de conectividad.
     */
    bool isHealthy() const;

    /**
     * @brief Copia las métricas actuales al struct de salida.
     * commTask llama getMetrics() para detectar cambios en totalSent/totalFailed.
     * @param out Destino donde se copian las métricas.
     */
    void getMetrics(SyncMetrics_t& out) const;

    /**
     * @brief Loguea el estado completo de métricas (ESP_LOGI).
     * Llamado internamente desde tick() cada SYNC_METRICS_LOG_PERIOD ticks.
     */
    void logStatus() const;

private:
    A7670Driver&   drv;             // Driver del modem A7670 (TCP send, recover)
    EventQueue&    queue;           // Cola de eventos con prioridad
    SyncMetrics_t  metrics;         // Métricas internas (exportables via getMetrics())
    uint64_t       backoffUntilUs;  // Timestamp hasta el que el backoff está activo
    uint32_t       tickCounter;     // Contador de ticks para el log periódico
    char           deviceId[20];    // "ARGUS-AABBCCDD" — para logging

    /**
     * @brief Intenta enviar un evento via drv.tcpSend(ev.payload).
     *
     * PROPÓSITO:
     *   Encapsular el envío TCP simple. En la versión actual, solo llama tcpSend().
     *   En versiones futuras podría agregar headers HTTP, MQTT packaging, etc.
     *
     * @param ev El evento a enviar (payload ya construido).
     * @return true si drv.tcpSend() retornó ESP_OK, false si hubo error.
     */
    bool trySend(QueuedEvent_t& ev);

    /**
     * @brief Maneja el éxito de un envío: actualiza métricas y resetea backoff.
     *
     * EFECTOS:
     *   - queue.markSent() → stats.totalSent++.
     *   - metrics.consecutiveFailures = 0.
     *   - metrics.lastSuccessAt = ahora.
     *   - metrics.backoffLevel = 0 → resetBackoff().
     *
     * @param ev El evento que fue enviado exitosamente.
     */
    void onSuccess(const QueuedEvent_t& ev);

    /**
     * @brief Maneja el fallo de un envío: reencola y aplica backoff exponencial.
     *
     * EFECTOS:
     *   - metrics.totalFailed++, consecutiveFailures++.
     *   - queue.requeue(ev) → si retries agotados, el evento se descarta.
     *   - scheduleBackoff() → backoffLevel++ + calcula backoffUntilUs.
     *
     * @param ev El evento fallido (se modifica: retryCount++ en requeue).
     */
    void onFailure(QueuedEvent_t& ev);

    /**
     * @brief Verifica el watchdog de conectividad. Llama recover() si es necesario.
     *
     * PROPÓSITO:
     *   Detectar el estado zombie: cola activa pero sin éxito en 15 minutos.
     *   Resetea lastSuccessAt si la cola está vacía (evita falsos positivos en reposo).
     *
     * @param nowUs Timestamp actual en µs.
     */
    void checkWatchdog(uint64_t nowUs);

    /**
     * @brief Retorna true si el backoff exponencial está activo.
     * @param nowUs Timestamp actual en µs.
     * @return true si nowUs < backoffUntilUs (aún en período de espera).
     */
    bool isInBackoff(uint64_t nowUs) const;

    /**
     * @brief Programa el próximo reintento según la tabla de backoff exponencial.
     *
     * PROPÓSITO:
     *   Avanzar el nivel de backoff (hasta el máximo BACKOFF_LEVELS-1) y calcular
     *   el timestamp hasta el que no se enviará nada.
     *
     * @param nowUs Timestamp actual en µs (punto de partida del backoff).
     */
    void scheduleBackoff(uint64_t nowUs);

    /**
     * @brief Resetea el backoff a nivel 0 (sin espera).
     * Llamado por onSuccess() y por checkWatchdog() tras drv.recover().
     */
    void resetBackoff();
};


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/include/sync_manager.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   SyncManager es el "cartero inteligente" del sistema. Toma los mensajes GPS
   de la cola (EventQueue) y los entrega al servidor TCP. Si la entrega falla
   (sin red, modem caído), el cartero espera antes de reintentar, aumentando
   la espera cada vez (backoff exponencial). Si lleva 15 minutos sin entregar
   nada, reinicia el modem (watchdog) para recuperar la conectividad.

   El backoff protege al sistema de bombardear el servidor con reintentos cuando
   la red está caída — si el servidor está sobrecargado o el modem tiene problemas,
   las ráfagas de reintentos solo empeorarían la situación.

   TABLA BACKOFF:
   Fallo 1→ 2s | Fallo 2→ 5s | Fallo 3→ 15s | Fallo 4→ 30s | Fallo 5+→ 60s

   MÉTRICAS MONITOREADAS:
   Métrica              | Descripción
   ---------------------|----------------------------------------------
   totalSent            | Envíos exitosos (commTask detecta cambios)
   totalFailed          | Envíos fallidos (commTask detecta cambios)
   consecutiveFailures  | Fallos seguidos (isHealthy() usa este valor)
   backoffLevel         | Nivel actual de espera (0-4)
   totalWatchdogFires   | Reinicios del modem por timeout

   ═══════════════════════════════════════════════════════════ */
