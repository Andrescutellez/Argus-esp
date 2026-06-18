#pragma once

/**
 * @file event_queue.h
 * @brief Cola de telemetría con prioridad, evicción y persistencia NVS para el sistema Argus.
 *
 * PROPÓSITO:
 *   Proporcionar una cola de eventos de telemetría que:
 *   1. Permite encolar reportes GPS/alertas con 4 niveles de prioridad.
 *   2. Gestiona overflow inteligentemente: descarta el evento de menor prioridad
 *      cuando la cola está llena (en lugar de rechazar siempre el nuevo).
 *   3. Persiste eventos CRITICAL en NVS para que sobrevivan a reboots del ESP32.
 *   4. Expone estadísticas de telemetría (encolados, enviados, descartados).
 *
 * ROL EN LA ARQUITECTURA:
 *   EventQueue se ubica entre commTask (productor) y SyncManager (consumidor):
 *
 *   commTask → submitReport() → syncManager.submit*() → eventQueue.enqueue()
 *   SyncManager → tick() → eventQueue.dequeue() → a7670.tcpSend() → backend
 *
 *   Si tcpSend() falla: syncManager llama requeue() con el evento fallido.
 *   Si la red sigue caída: el evento acumula retryCount hasta EQ_MAX_RETRIES.
 *   Si retryCount se agota: el evento se descarta (excepto CRITICAL que tiene 2×retries).
 *
 * ESTRUCTURA DE DATOS:
 *   Array circular de tamaño fijo buf[EQ_MAX_SIZE] con búsqueda lineal O(n).
 *   No es una heap/priority queue porque n=100 es pequeño y la búsqueda lineal
 *   es más simple de auditar y no fragmenta el heap. El mutex protege todas las
 *   operaciones de lectura/escritura.
 *
 * POLÍTICA DE OVERFLOW (cola llena):
 *   Al enqueue() con cola llena:
 *   1. findEvictionCandidate_locked() → buscar el evento de menor prioridad + más antiguo.
 *   2. Si ese evento tiene MENOR prioridad que el nuevo:
 *      → Reemplazar ese evento con el nuevo (evicción). totalDropped++.
 *   3. Si todos los eventos tienen IGUAL o MAYOR prioridad:
 *      → Rechazar el nuevo. totalDropped++.
 *   Esta política garantiza que eventos CRITICAL nunca son desplazados por GPS periódico.
 *
 * POLÍTICA DE DEQUEUE:
 *   Búsqueda lineal del evento con mayor prioridad (menor valor EPrio_t).
 *   Dentro de la misma prioridad: el más antiguo (menor timestamp) sale primero (FIFO).
 *   Al extraer: el último elemento del array llena el hueco (swap with last, O(1) compaction).
 *
 * POLÍTICA DE REQUEUE:
 *   Incrementa retryCount. Si retryCount > EQ_MAX_RETRIES (o × 2 para CRITICAL):
 *   el evento se descarta. Si no: se vuelve a encolar (puede ser desalojado de nuevo
 *   si la cola está llena de CRITICAL mientras tanto).
 *
 * PERSISTENCIA NVS:
 *   saveToNVS(): serializa hasta EQ_NVS_MAX_PERSIST eventos CRITICAL en un blob NVS.
 *   loadFromNVS(): deserializa al arrancar y los reencola.
 *   Formato blob: [uint8_t count][QueuedEvent_t ev0]...[QueuedEvent_t evN-1]
 *   Namespace: "argus_eq", key: "critical".
 *   saveToNVS() se llama desde commTask cada 5 minutos.
 *
 * CONFIGURACIÓN:
 *   EQ_MAX_SIZE        = 100  eventos en memoria
 *   EQ_PAYLOAD_MAX_LEN = 300  bytes por payload (frame Argus TCP ~150 chars)
 *   EQ_MAX_RETRIES     = 5    intentos normales (×2 = 10 para CRITICAL)
 *   EQ_NVS_MAX_PERSIST = 8    eventos CRITICAL guardados en NVS
 *
 * CONCURRENCIA:
 *   Todos los métodos públicos (excepto isEmpty/isFull que llaman a size())
 *   toman el mutex interno xSemaphoreCreateMutex() antes de acceder a buf[].
 *   Uso típico: commTask como productor, SyncManager como consumidor.
 *   Ambos corren en tareas FreeRTOS distintas → el mutex es necesario.
 *
 * TAMAÑO EN MEMORIA:
 *   buf[100] × sizeof(QueuedEvent_t) ≈ 100 × (8+1+1+1+1+300) = ~31,200 bytes
 *   Se aloca estáticamente en la instancia (no heap). commTask tiene stack de 8192 bytes,
 *   pero EventQueue se instancia como static en comm_task.cpp → va al heap/BSS.
 *
 * @module components/services/event_queue
 */

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

// ─── Configuración ────────────────────────────────────────────────────────────

/** Máximo de eventos simultáneos en la cola. Array estático, no heap. */
#define EQ_MAX_SIZE          100

/**
 * Máximo de bytes por payload de evento. El frame Argus TCP tiene ~150 chars
 * en la práctica. 300 bytes da margen para payloads JSON más ricos en el futuro.
 */
#define EQ_PAYLOAD_MAX_LEN   300

/**
 * Número máximo de reintentos antes de descartar un evento fallido.
 * Los eventos CRITICAL tienen EQ_MAX_RETRIES × 2 = 10 intentos.
 * Con backoff exponencial, 5 reintentos pueden tardar hasta 60s antes de descartar.
 */
#define EQ_MAX_RETRIES       5

/**
 * Máximo de eventos CRITICAL que se persisten en NVS por saveToNVS().
 * 8 eventos CRITICAL × 312 bytes = ~2500 bytes de blob NVS (dentro del límite NVS).
 */
#define EQ_NVS_MAX_PERSIST   8

// ─── Tipo de evento ───────────────────────────────────────────────────────────

/**
 * @brief Clasificación semántica del evento de telemetría.
 *
 * Define qué tipo de dato representa el evento. En la versión actual, el payload
 * siempre es el frame Argus TCP (mismo formato para todos los tipos). El tipo
 * permite al backend clasificar el evento sin parsear el payload completo.
 */
typedef enum : uint8_t {
    ETYPE_GPS_REPORT  = 0,  // Reporte de posición GPS (normal o fallback)
    ETYPE_ALERT       = 1,  // Alerta o persecución activa (alta urgencia)
    ETYPE_HEARTBEAT   = 2,  // Keepalive / latido (mínima urgencia)
} EType_t;

// ─── Prioridad de envío ───────────────────────────────────────────────────────

/**
 * @brief Nivel de prioridad del evento. Menor valor = mayor prioridad.
 *
 * CRITICAL: Nunca desplazado en overflow. Persistido en NVS. 10 reintentos.
 *           Usado en STATE_PURSUIT: cada reporte puede ser la última posición conocida.
 *
 * HIGH:     Desplaza a NORMAL y LOW en overflow. No persistido en NVS. 5 reintentos.
 *           Usado en STATE_ALERT: alerta activa con telemetría frecuente.
 *
 * NORMAL:   Desplaza a LOW en overflow. 5 reintentos.
 *           Usado en STATE_MOVING e IDLE armado: monitoreo activo.
 *
 * LOW:      Primero en ser desplazado en overflow. 5 reintentos.
 *           Usado en IDLE desarmado: heartbeat de keepalive.
 *
 * El rango de valores (0-3) es deliberadamente compacto para que la comparación
 * de prioridades sea una resta simple (menor valor = mayor prioridad).
 */
typedef enum : uint8_t {
    EPRIO_CRITICAL = 0,  // Persecución: el ESP32 puede resetearse, no perder este dato
    EPRIO_HIGH     = 1,  // Alerta activa: alta urgencia, pero recuperable
    EPRIO_NORMAL   = 2,  // GPS periódico regular
    EPRIO_LOW      = 3,  // Heartbeat: se descarta primero en overflow
} EPrio_t;

// ─── Evento en cola ───────────────────────────────────────────────────────────

/**
 * @brief Estructura que representa un evento de telemetría en la cola.
 *
 * CAMPOS:
 *   timestamp:   µs desde boot (esp_timer_get_time()). Se usa para desempate FIFO
 *                dentro del mismo nivel de prioridad. Es el timestamp de enqueue(),
 *                no el timestamp del GPS (que va en el payload).
 *   type:        clasificación semántica del evento (GPS, ALERT, HEARTBEAT).
 *   priority:    nivel de prioridad para el orden de dequeue y la política de evicción.
 *   retryCount:  número de intentos fallidos. Incrementado por requeue().
 *                uint8_t: suficiente para EQ_MAX_RETRIES × 2 = 10.
 *   _reserved:   padding para alineación a 4 bytes. Sin uso actual.
 *   payload:     frame TCP completo del protocolo Argus, listo para enviar.
 *                Incluye el '\n' delimitador al final (ver buildArgusPacket() en comm_task.cpp).
 *
 * TAMAÑO: 8 (timestamp) + 1 (type) + 1 (priority) + 1 (retryCount) + 1 (_reserved) + 300 (payload)
 *         = 312 bytes por evento. buf[100] = 31,200 bytes total.
 */
typedef struct {
    uint64_t  timestamp;                    // µs desde boot (esp_timer_get_time())
    EType_t   type;                         // Tipo semántico del evento
    EPrio_t   priority;                     // Prioridad de envío (menor = más urgente)
    uint8_t   retryCount;                   // Reintentos fallidos acumulados
    uint8_t   _reserved;                    // Padding — sin uso actual
    char      payload[EQ_PAYLOAD_MAX_LEN];  // Frame TCP del protocolo Argus (con '\n')
} QueuedEvent_t;

// ─── Estadísticas internas ────────────────────────────────────────────────────

/**
 * @brief Estadísticas de telemetría de la EventQueue.
 *
 * Permiten a commTask detectar cambios en el estado de la cola (ej. nuevos éxitos
 * o fallos) y generar eventos EVENT_COMMS_SUCCESS/FAILURE para controlTask.
 *
 * CAMPOS:
 *   totalEnqueued:  total de eventos encolados, incluyendo los que desplazaron a otros.
 *   totalSent:      total de eventos enviados y confirmados por el backend.
 *   totalDropped:   total de eventos descartados (por overflow o retries agotados).
 *   currentSize:    eventos actualmente en cola (se actualiza en enqueue/dequeue).
 *   highWaterMark:  máximo histórico de currentSize (útil para diagnóstico de memoria).
 */
typedef struct {
    uint32_t totalEnqueued;    // Eventos encolados desde el arranque (incluyendo eviction)
    uint32_t totalSent;        // Enviados y confirmados exitosamente
    uint32_t totalDropped;     // Descartados por overflow o retries agotados
    uint32_t currentSize;      // Eventos actualmente en cola
    uint32_t highWaterMark;    // Máximo de eventos simultáneos observado
} EQStats_t;

// ─── EventQueue ──────────────────────────────────────────────────────────────

/**
 * @brief Cola de telemetría con prioridad, evicción y persistencia NVS.
 *
 * PROPÓSITO:
 *   Gestionar el buffer de eventos entre commTask (productor) y SyncManager (consumidor)
 *   con política de prioridad que garantiza que los eventos CRITICAL nunca se pierden
 *   a expensas de los heartbeats de baja prioridad.
 *
 * USO TÍPICO (dentro de SyncManager y commTask):
 *   EventQueue eq;
 *   eq.init();                         // crear mutex
 *   eq.loadFromNVS();                  // restaurar CRITICAL del boot anterior
 *   eq.enqueue(ev);                    // agregar un reporte GPS
 *   QueuedEvent_t out;
 *   if (eq.dequeue(out)) {             // obtener el más urgente
 *     if (send(out)) eq.markSent();   // marcar como enviado
 *     else           eq.requeue(out); // reintentar con backoff
 *   }
 *   eq.saveToNVS();                    // persistir CRITICAL cada 5 min
 *
 * THREAD SAFETY:
 *   Todos los métodos que acceden a buf[] toman el mutex interno.
 *   Es seguro llamar desde múltiples tareas FreeRTOS simultáneamente.
 *   EXCEPCIÓN: init() debe llamarse antes de que cualquier otra tarea acceda
 *   a la instancia (no es seguro llamar init() y enqueue() concurrentemente).
 */
class EventQueue {
public:
    /**
     * @brief Constructor. Inicializa buf[] y stats a cero. mtx queda nullptr hasta init().
     *
     * NOTA: No crear el mutex aquí porque el constructor puede llamarse antes de que
     *   FreeRTOS esté listo. El mutex se crea en init() que se llama desde la tarea.
     */
    EventQueue();

    /**
     * @brief Destructor. Libera el mutex si fue creado.
     *
     * NOTA: En el sistema Argus, EventQueue se instancia como static en comm_task.cpp
     *   y nunca se destruye en runtime. El destructor es seguridad defensiva.
     */
    ~EventQueue();

    /**
     * @brief Crea el mutex interno. Debe llamarse una vez antes de usar la cola.
     *
     * PROPÓSITO:
     *   Separar la creación del mutex de la construcción del objeto porque
     *   xSemaphoreCreateMutex() requiere que el scheduler de FreeRTOS esté activo,
     *   lo cual no está garantizado en el momento del constructor estático.
     *
     * @return ESP_OK si el mutex se creó. ESP_ERR_NO_MEM si malloc falló.
     */
    esp_err_t init();

    /**
     * @brief Encola un evento. Si la cola está llena, aplica política de evicción.
     *
     * PROPÓSITO:
     *   Agregar un nuevo evento de telemetría a la cola con manejo inteligente
     *   de overflow: en lugar de rechazar siempre el nuevo evento, se evalúa
     *   si existe un evento de menor prioridad que pueda ser desalojado.
     *
     * FLUJO:
     *   1. Tomar mutex.
     *   2. Si count < EQ_MAX_SIZE: agregar al final de buf[]. count++.
     *   3. Si cola llena:
     *      a. findEvictionCandidate_locked() → índice del evento peor.
     *      b. Si ev.priority < buf[victim].priority → reemplazar victim con ev.
     *      c. Si no → rechazar ev. Liberar mutex. Retornar false.
     *   4. Actualizar stats. Liberar mutex.
     *
     * NOTA: La comparación de prioridades es buf[victim].priority <= ev.priority:
     *   Si el candidato tiene la MISMA prioridad que el nuevo, NO se desaloja.
     *   Esto previene que dos eventos de la misma prioridad se desplacen mutuamente.
     *
     * @param ev Evento a encolar (se copia por valor).
     * @return true si fue encolado (incluso si desalojó a otro), false si rechazado.
     */
    bool enqueue(const QueuedEvent_t& ev);

    /**
     * @brief Extrae el evento de mayor prioridad de la cola.
     *
     * PROPÓSITO:
     *   Obtener el siguiente evento a enviar según el orden de prioridad.
     *   Dentro del mismo nivel de prioridad, aplica FIFO por timestamp.
     *
     * FLUJO:
     *   1. Tomar mutex.
     *   2. Búsqueda lineal del mejor evento (menor priority, menor timestamp en empate).
     *   3. Copiar a out.
     *   4. Compactar: buf[best] = buf[--count] (swap with last, O(1) sin memmove).
     *   5. Actualizar stats. Liberar mutex.
     *
     * NOTA SOBRE COMPACTACIÓN:
     *   El swap-with-last es O(1) pero cambia el orden relativo de los elementos
     *   en buf[]. Esto NO afecta la corrección de dequeue() porque la búsqueda
     *   es lineal y no depende del orden en buf[].
     *
     * @param out Buffer donde se escribe el evento extraído.
     * @return true si se extrajo un evento, false si la cola estaba vacía.
     */
    bool dequeue(QueuedEvent_t& out);

    /**
     * @brief Re-encola un evento fallido incrementando su retryCount.
     *
     * PROPÓSITO:
     *   Manejar los reintentos de envío cuando tcpSend() falla. Se llama desde
     *   SyncManager::onFailure() después de cada intento fallido.
     *
     * FLUJO:
     *   1. Calcular maxRetries: CRITICAL → EQ_MAX_RETRIES × 2; otros → EQ_MAX_RETRIES.
     *   2. ev.retryCount++.
     *   3. Si retryCount > maxRetries → log WARN + stats.totalDropped++ + retornar false.
     *   4. Si no → enqueue(ev) (puede fallar de nuevo si la cola está llena).
     *
     * NOTA: Los eventos CRITICAL tienen el doble de reintentos porque representan
     *   posiciones GPS durante una persecución activa — es crítico no perderlos
     *   aunque la red esté intermitente.
     *
     * @param ev Evento fallido (se modifica: retryCount++).
     * @return true si fue reencolado, false si fue descartado.
     */
    bool requeue(QueuedEvent_t& ev);

    /**
     * @brief Registra un envío exitoso en las estadísticas.
     *
     * PROPÓSITO:
     *   Incrementar stats.totalSent cuando SyncManager confirma que tcpSend()
     *   fue exitoso. Llamado por SyncManager::onSuccess().
     *
     * NOTA: No modifica buf[] — el evento ya fue extraído por dequeue() antes de
     *   intentar el envío. Esta función solo actualiza el contador de estadísticas.
     */
    void markSent();

    /**
     * @brief Retorna el número de eventos actualmente en la cola.
     * Toma el mutex para lectura atómica de count.
     * @return Número de eventos en cola (0 a EQ_MAX_SIZE).
     */
    uint32_t size() const;

    /**
     * @brief Retorna true si la cola está vacía.
     * Equivalente a size() == 0. Toma el mutex internamente.
     */
    bool isEmpty() const;

    /**
     * @brief Retorna true si la cola está llena.
     * Equivalente a size() >= EQ_MAX_SIZE. Toma el mutex internamente.
     */
    bool isFull() const;

    /**
     * @brief Copia las estadísticas actuales al struct out.
     * Toma el mutex para lectura atómica de stats.
     * @param out Destino donde se escriben las estadísticas.
     */
    void getStats(EQStats_t& out) const;

    /**
     * @brief Persiste eventos CRITICAL en NVS para sobrevivir a reboots.
     *
     * PROPÓSITO:
     *   Guardar hasta EQ_NVS_MAX_PERSIST eventos CRITICAL en NVS como blob binario.
     *   Si el ESP32 se reinicia (watchdog, panic, corte de energía), los eventos
     *   CRITICAL se restauran en el próximo arranque con loadFromNVS().
     *
     * FLUJO:
     *   1. nvs_open("argus_eq", READWRITE).
     *   2. Tomar mutex. Recolectar hasta 8 eventos CRITICAL de buf[]. Liberar mutex.
     *   3. malloc(1 + count × sizeof(QueuedEvent_t)) → blob.
     *   4. blob[0] = count. memcpy(blob+1, eventos).
     *   5. nvs_set_blob("critical", blob). nvs_commit(). nvs_close().
     *   6. free(blob).
     *
     * NOTA: La separación entre "tomar mutex → copiar → liberar mutex" y "escribir NVS"
     *   evita bloquear la cola durante la escritura NVS (que puede tardar decenas de ms).
     *
     * CUÁNDO LLAMAR:
     *   commTask la llama cada 5 minutos. También puede llamarse antes de un
     *   reinicio voluntario si se implementa ese mecanismo en el futuro.
     *
     * @return ESP_OK si se guardó correctamente, código de error de NVS si no.
     */
    esp_err_t saveToNVS();

    /**
     * @brief Restaura eventos CRITICAL guardados en NVS al arrancar.
     *
     * PROPÓSITO:
     *   Recuperar eventos CRITICAL pendientes del boot anterior. Llamar en
     *   commTask antes de comenzar a recibir nuevos eventos para que los
     *   eventos críticos se envíen primero (mayor prioridad).
     *
     * FLUJO:
     *   1. nvs_open("argus_eq", READONLY).
     *   2. nvs_get_blob con size=0 para consultar el tamaño del blob.
     *   3. malloc(blobSize) → leer blob.
     *   4. blob[0] = count. Validar que blobSize >= 1 + count × sizeof(QueuedEvent_t).
     *   5. Para cada evento: enqueue() (con la política de prioridad normal).
     *
     * RETORNO:
     *   ESP_OK incluso si no hay datos guardados (primera vez es normal).
     *   Solo retorna error en fallo real de NVS.
     *
     * @return ESP_OK si cargó correctamente (o no había datos), error NVS si falla.
     */
    esp_err_t loadFromNVS();

private:
    /** Buffer de eventos. Array estático de tamaño fijo EQ_MAX_SIZE. */
    QueuedEvent_t    buf[EQ_MAX_SIZE];

    /** Número de eventos actualmente en buf[]. Índice de escritura del próximo elemento. */
    uint32_t         count;

    /**
     * Mutex FreeRTOS que protege buf[] y count de accesos concurrentes.
     * nullptr hasta que init() lo crea con xSemaphoreCreateMutex().
     * Todos los métodos que leen/escriben buf[] deben tomarlo antes de acceder.
     */
    SemaphoreHandle_t mtx;

    /** Estadísticas internas. Protegidas por el mutex. */
    EQStats_t         stats;

    /**
     * @brief Busca el evento de menor prioridad (más antiguo en empate) en buf[].
     *
     * PROPÓSITO:
     *   Encontrar el candidato a desalojar cuando la cola está llena. El candidato
     *   es el evento "peor": menor prioridad (mayor valor EPrio_t) y, en caso de
     *   empate, el más antiguo (menor timestamp).
     *
     * PRECONDICIÓN:
     *   El caller DEBE tener el mutex tomado antes de llamar este método.
     *   (Nombre _locked indica que el mutex ya fue tomado.)
     *
     * COMPLEJIDAD: O(n) — búsqueda lineal sobre buf[count].
     *
     * @return Índice del candidato en buf[], o -1 si count == 0.
     */
    int findEvictionCandidate_locked() const;
};


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/include/event_queue.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   EventQueue es como la "bandeja de salida" de los correos GPS del sistema.
   Cuando commTask quiere enviar una posición GPS al servidor, la pone en esta
   cola. SyncManager la saca de la cola y la envía via TCP. Si el envío falla,
   SyncManager la devuelve a la cola con un contador de reintentos.

   La cola tiene prioridades: una alerta de persecución (CRITICAL) nunca se
   descarta aunque la cola esté llena — en cambio, se descarta el heartbeat
   de menor urgencia. Así nunca se pierde la última posición conocida del ladrón.

   TABLA DE PRIORIDADES:
   Prioridad | Valor | Usado en   | NVS | Reintentos
   ----------|-------|------------|-----|----------
   CRITICAL  |   0   | PURSUIT    | Sí  | 10
   HIGH      |   1   | ALERT      | No  | 5
   NORMAL    |   2   | MOVING/IDLE| No  | 5
   LOW       |   3   | Heartbeat  | No  | 5

   TABLA DE CONFIGURACIÓN:
   Constante          | Valor | Descripción
   -------------------|-------|-------------------------------------------
   EQ_MAX_SIZE        | 100   | Máximo eventos en cola (31,200 bytes RAM)
   EQ_PAYLOAD_MAX_LEN | 300   | Bytes máx por payload (frame Argus ~150)
   EQ_MAX_RETRIES     | 5     | Intentos antes de descartar (×2 si CRITICAL)
   EQ_NVS_MAX_PERSIST | 8     | Máx eventos CRITICAL en NVS (~2500 bytes)

   ═══════════════════════════════════════════════════════════ */
