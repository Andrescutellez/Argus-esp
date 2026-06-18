/**
 * @file event_queue.cpp
 * @brief Implementación de la cola de telemetría con prioridad y persistencia NVS.
 *
 * PROPÓSITO:
 *   Implementar la lógica de cola EventQueue: enqueue con evicción inteligente,
 *   dequeue con prioridad + FIFO, requeue con límite de reintentos, y persistencia
 *   de eventos CRITICAL en NVS para sobrevivir a reboots del ESP32.
 *
 * ESTRUCTURA DE DATOS:
 *   buf[EQ_MAX_SIZE]: array estático de QueuedEvent_t (31,200 bytes en BSS).
 *   count: número de elementos válidos en buf[0..count-1].
 *   El array NO está ordenado por prioridad. La búsqueda del mejor/peor elemento
 *   es O(n) lineal. Para n=100 esto es insignificante en CPU a 240MHz (<1µs).
 *
 *   Ventajas del array no-ordenado vs priority heap:
 *   - Inserción O(1) en el caso normal (cola no llena).
 *   - Compactación O(1) en dequeue (swap with last, sin shift).
 *   - No fragmenta el heap (array estático).
 *   - Más simple de auditar para bugs de concurrencia.
 *
 * CONCURRENCIA:
 *   Mutex interno (xSemaphoreCreateMutex) protege todas las operaciones en buf[].
 *   saveToNVS() libera el mutex antes de escribir en NVS para no bloquear
 *   enqueue/dequeue durante la escritura flash (que puede tardar 10-50ms).
 *
 * NVS:
 *   Namespace: "argus_eq" (distinto de "argus_cfg" usado por sensitivity).
 *   Key: "critical" → blob binario con los eventos CRITICAL.
 *   Formato: [uint8_t count][QueuedEvent_t ev0][QueuedEvent_t ev1]...
 *   El count en el byte 0 permite validar la integridad del blob.
 *
 * DEPENDENCIAS:
 *   - event_queue.h:  Definiciones de la clase, constantes, tipos
 *   - esp_log.h:      ESP_LOGI/LOGW para diagnóstico
 *   - nvs_flash.h + nvs.h: API NVS de ESP-IDF para persistencia
 *
 * VARIABLES CRÍTICAS:
 *   - buf[]:   array de eventos. El único estado persistente de la cola en RAM.
 *   - count:   siempre en rango [0, EQ_MAX_SIZE]. Invariante crítico.
 *   - mtx:     mutex. Si nullptr, las operaciones son undefined (crash probable).
 *
 * @module components/services/event_queue
 */

#include "event_queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

// TAG de logging para todos los mensajes de esta clase.
static const char* TAG           = "EventQueue";

// Namespace y key NVS para la persistencia de eventos CRITICAL.
// Separados de "argus_cfg" (sensibilidad) para independencia entre subsistemas.
static const char* NVS_NAMESPACE = "argus_eq";
static const char* NVS_KEY       = "critical";

// ─── Constructor / Destructor ─────────────────────────────────────────────────

/**
 * @brief Constructor — inicializa el estado interno a cero.
 *
 * PROPÓSITO:
 *   Inicializar count=0, mtx=nullptr, y buf[]/stats a cero.
 *   El mutex se crea en init() para garantizar que FreeRTOS esté activo.
 *
 * NOTA: Este constructor puede ejecutarse antes de que el scheduler de FreeRTOS
 *   esté corriendo (si EventQueue es una variable global o static). Por eso
 *   NO se crea el mutex aquí — xSemaphoreCreateMutex() requiere el scheduler.
 */
EventQueue::EventQueue() : count(0), mtx(nullptr) {
    memset(buf,    0, sizeof(buf));
    memset(&stats, 0, sizeof(stats));
}

/**
 * @brief Destructor — libera el mutex si fue creado.
 *
 * En el sistema Argus, EventQueue es static en comm_task.cpp y nunca se destruye.
 * El destructor es código defensivo para uso en tests o cambios futuros.
 */
EventQueue::~EventQueue() {
    if (mtx) vSemaphoreDelete(mtx);
}

/**
 * @brief Crea el mutex interno. Debe llamarse desde la tarea FreeRTOS antes de usar la cola.
 *
 * PROPÓSITO:
 *   Crear el mutex que protege buf[] y count de accesos concurrentes.
 *   Se llama desde commTask() después de que FreeRTOS está activo.
 *
 * FLUJO:
 *   xSemaphoreCreateMutex() → asigna a mtx. Si falla (malloc interno falla):
 *   retorna ESP_ERR_NO_MEM y las operaciones subsiguientes serán unsafe.
 *
 * @return ESP_OK si el mutex se creó correctamente. ESP_ERR_NO_MEM si malloc falló.
 */
esp_err_t EventQueue::init() {
    mtx = xSemaphoreCreateMutex();
    if (!mtx) {
        ESP_LOGE(TAG, "No se pudo crear mutex");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Inicializada (capacidad=%d, payload_max=%d B)",
             EQ_MAX_SIZE, EQ_PAYLOAD_MAX_LEN);
    return ESP_OK;
}

// ─── enqueue() ───────────────────────────────────────────────────────────────
//
// POLÍTICA DE OVERFLOW (cola llena):
//   1. Buscar el evento de menor prioridad (mayor EPrio_t) y más antiguo.
//   2. Si ese evento tiene MENOR prioridad que el entrante → descartar ese, aceptar el nuevo.
//      "Menor prioridad" = buf[victim].priority > ev.priority (mayor valor enum = menor prioridad).
//   3. Si todos los existentes tienen prioridad >= la del nuevo → rechazar el nuevo.
//      Esto protege alertas CRITICAL de ser desplazadas por GPS periódico.

/**
 * @brief Encola un evento. Si la cola está llena, aplica política de evicción.
 *
 * PROPÓSITO:
 *   Agregar ev a la cola gestionando el overflow de forma inteligente.
 *   Garantiza que eventos de alta prioridad entran aunque la cola esté llena.
 *
 * FLUJO:
 *   1. xSemaphoreTake(mtx, portMAX_DELAY) — esperar el mutex sin timeout.
 *   2. Si count < EQ_MAX_SIZE: buf[count++] = ev. Actualizar stats. Liberar. OK.
 *   3. Si cola llena: findEvictionCandidate_locked() → índice del peor evento.
 *      a. Si victim < 0 (imposible si count > 0, pero defensivo): rechazar.
 *      b. Si buf[victim].priority <= ev.priority: el candidato tiene igual o
 *         mayor prioridad → no desalojar → rechazar el nuevo. totalDropped++.
 *      c. Si buf[victim].priority > ev.priority: desalojar victim → buf[victim] = ev.
 *         count no cambia (misma cantidad de eventos). totalDropped++ (el victim se perdió).
 *   4. xSemaphoreGive(mtx).
 *
 * NOTA SOBRE LA COMPARACIÓN `buf[victim].priority <= ev.priority`:
 *   EPrio_t: CRITICAL=0 < HIGH=1 < NORMAL=2 < LOW=3.
 *   Si victim.priority <= ev.priority (ej. victim=HIGH=1, nuevo=NORMAL=2):
 *   victim NO tiene menor prioridad que el nuevo → no desalojar.
 *   Si victim.priority > ev.priority (ej. victim=LOW=3, nuevo=HIGH=1):
 *   victim SÍ tiene menor prioridad → desalojar.
 *
 * @param ev Evento a encolar (copia por valor).
 * @return true si fue aceptado, false si fue rechazado.
 */
bool EventQueue::enqueue(const QueuedEvent_t& ev) {
    xSemaphoreTake(mtx, portMAX_DELAY);

    if (count < EQ_MAX_SIZE) {
        // Caso normal: hay espacio en la cola.
        buf[count++] = ev;
        stats.totalEnqueued++;
        stats.currentSize = count;
        if (count > stats.highWaterMark) stats.highWaterMark = count;
        xSemaphoreGive(mtx);
        return true;
    }

    // Cola llena: buscar candidato a desalojar.
    int victim = findEvictionCandidate_locked();
    if (victim < 0 || buf[victim].priority <= ev.priority) {
        // No hay candidato válido o el candidato tiene igual/mayor prioridad
        // que el nuevo → el nuevo tiene menor urgencia, se descarta.
        stats.totalDropped++;
        xSemaphoreGive(mtx);
        ESP_LOGW(TAG, "Cola llena — evento descartado (prio=%d, tipo=%d)",
                 ev.priority, ev.type);
        return false;
    }

    // El candidato tiene menor prioridad que el nuevo → desalojarlo.
    // count no cambia (seguimos con EQ_MAX_SIZE eventos).
    ESP_LOGW(TAG, "Cola llena — desalojando evento (prio=%d) para aceptar (prio=%d)",
             buf[victim].priority, ev.priority);
    stats.totalDropped++;  // El evento desalojado se cuenta como descartado
    buf[victim] = ev;
    stats.totalEnqueued++;
    // count no cambia: misma cantidad de eventos

    xSemaphoreGive(mtx);
    return true;
}

// ─── dequeue() ───────────────────────────────────────────────────────────────
//
// Extrae el evento de mayor prioridad (menor valor de EPrio_t).
// Desempate FIFO: dentro de la misma prioridad, el más antiguo (menor timestamp).
// Compactación: swap with last para O(1) sin memmove.

/**
 * @brief Extrae el evento de mayor prioridad de la cola.
 *
 * PROPÓSITO:
 *   Obtener el próximo evento a enviar, respetando el orden de prioridad y FIFO.
 *
 * FLUJO:
 *   1. Tomar mutex.
 *   2. Si count == 0: liberar mutex, retornar false.
 *   3. Búsqueda lineal O(n) del "mejor" evento:
 *      - Preferir menor priority (mayor urgencia).
 *      - Empate de prioridad: preferir menor timestamp (el más antiguo = FIFO).
 *   4. out = buf[best]. Compactar: buf[best] = buf[--count].
 *   5. stats.currentSize = count. Liberar mutex. Retornar true.
 *
 * NOTA SOBRE COMPACTACIÓN:
 *   Copiar el último elemento al hueco es O(1) y preserva todos los elementos.
 *   El orden relativo en buf[] cambia pero eso no importa porque dequeue()
 *   siempre busca linealmente — no asume ningún orden.
 *
 * @param out Destino donde se escribe el evento extraído.
 * @return true si se extrajo un evento, false si la cola estaba vacía.
 */
bool EventQueue::dequeue(QueuedEvent_t& out) {
    xSemaphoreTake(mtx, portMAX_DELAY);

    if (count == 0) {
        xSemaphoreGive(mtx);
        return false;
    }

    // Buscar el evento con mayor urgencia (menor EPrio_t, mayor timestamp si empate).
    int best = 0;
    for (int i = 1; i < (int)count; i++) {
        if (buf[i].priority < buf[best].priority) {
            // Este evento es más urgente
            best = i;
        } else if (buf[i].priority == buf[best].priority &&
                   buf[i].timestamp < buf[best].timestamp) {
            // Misma prioridad → preferir el más antiguo (FIFO)
            best = i;
        }
    }

    out = buf[best];
    // Compactar moviendo el último elemento al hueco.
    // Si best == count-1, es un self-assign inofensivo.
    buf[best] = buf[--count];
    stats.currentSize = count;

    xSemaphoreGive(mtx);
    return true;
}

// ─── requeue() ───────────────────────────────────────────────────────────────
//
// Incrementa retryCount y vuelve a encolar. Si se excede EQ_MAX_RETRIES, el
// evento se descarta. Para eventos CRITICAL se duplica el límite de retries.

/**
 * @brief Re-encola un evento fallido incrementando su retryCount.
 *
 * PROPÓSITO:
 *   Gestionar los reintentos de envío cuando tcpSend() falla. SyncManager::onFailure()
 *   llama requeue() para dar al evento otra oportunidad con el próximo ciclo de backoff.
 *
 * FLUJO:
 *   1. Calcular maxRetries: CRITICAL → 10, otros → 5.
 *   2. ev.retryCount++.
 *   3. Si retryCount > maxRetries:
 *      - Tomar mutex. stats.totalDropped++. Liberar mutex.
 *      - Log WARN con el número de retries.
 *      - Retornar false (el evento se descarta).
 *   4. Si no: enqueue(ev) — puede fallar de nuevo si la cola está llena de CRITICAL.
 *
 * NOTA SOBRE CRITICAL × 2:
 *   Con backoff exponencial (2s, 5s, 15s, 30s, 60s...) y 10 reintentos,
 *   un evento CRITICAL puede permanecer en cola e intentarse durante varios minutos
 *   antes de descartarse. Esto es intencional para persecución activa.
 *
 * @param ev Evento fallido. Se modifica: ev.retryCount es incrementado.
 * @return true si fue reencolado, false si fue descartado (retries agotados).
 */
bool EventQueue::requeue(QueuedEvent_t& ev) {
    int maxRetries = (ev.priority == EPRIO_CRITICAL)
                   ? EQ_MAX_RETRIES * 2   // CRITICAL: 10 intentos
                   : EQ_MAX_RETRIES;       // Otros: 5 intentos

    ev.retryCount++;

    if (ev.retryCount > (uint8_t)maxRetries) {
        // Retries agotados: el evento se pierde definitivamente.
        xSemaphoreTake(mtx, portMAX_DELAY);
        stats.totalDropped++;
        xSemaphoreGive(mtx);
        ESP_LOGW(TAG, "Evento descartado tras %d retries (tipo=%d)",
                 ev.retryCount - 1, ev.type);
        return false;
    }

    // Volver a encolar con la política de prioridad normal.
    // Si la cola está llena de CRITICAL y este evento es NORMAL, puede ser rechazado.
    return enqueue(ev);
}

// ─── markSent() ──────────────────────────────────────────────────────────────

/**
 * @brief Registra un envío exitoso en las estadísticas.
 *
 * PROPÓSITO:
 *   Incrementar stats.totalSent para que commTask pueda detectar cambios y
 *   generar EVENT_COMMS_SUCCESS en xEventQueue hacia controlTask.
 *
 * NOTA: El evento ya fue extraído por dequeue() antes de intentar el envío.
 *   Esta función solo actualiza el contador — no modifica buf[].
 */
void EventQueue::markSent() {
    xSemaphoreTake(mtx, portMAX_DELAY);
    stats.totalSent++;
    xSemaphoreGive(mtx);
}

// ─── Accesores ────────────────────────────────────────────────────────────────

/**
 * @brief Retorna el número de eventos actualmente en la cola.
 *
 * NOTA: Toma el mutex para garantizar que count se lee de forma atómica.
 *   En ARM Cortex-M, la lectura de uint32_t es atómica si está alineada,
 *   pero tomar el mutex es más correcto formalmente.
 */
uint32_t EventQueue::size() const {
    xSemaphoreTake(mtx, portMAX_DELAY);
    uint32_t s = count;
    xSemaphoreGive(mtx);
    return s;
}

/** Llama a size() con overhead de mutex. Para código crítico en el tiempo,
 *  considerar cachear el valor o leer count directamente (con mutex tomado externamente). */
bool EventQueue::isEmpty() const { return size() == 0; }
bool EventQueue::isFull()  const { return size() >= EQ_MAX_SIZE; }

/**
 * @brief Copia las estadísticas actuales al struct de salida.
 * Toma el mutex para que la copia de todos los campos de stats sea atómica.
 */
void EventQueue::getStats(EQStats_t& out) const {
    xSemaphoreTake(mtx, portMAX_DELAY);
    out = stats;
    xSemaphoreGive(mtx);
}

// ─── findEvictionCandidate_locked() ──────────────────────────────────────────
//
// PRECONDICIÓN: el caller debe tener el mutex tomado.
// Busca el evento de "menor valor": mayor EPrio_t (menos urgente)
// y, en caso de empate, menor timestamp (más antiguo).
// El más antiguo es el candidato natural: lleva más tiempo en cola sin enviarse.

/**
 * @brief Busca el evento de menor prioridad + más antiguo en buf[].
 *
 * PROPÓSITO:
 *   Identificar el candidato a desalojar cuando la cola está llena.
 *   El candidato es el evento con mayor valor de EPrio_t (LOW=3 antes que NORMAL=2).
 *   En empate, el más antiguo (menor timestamp) es preferido para desalojar.
 *
 * PRECONDICIÓN:
 *   El caller debe tener el mutex tomado (indicado por el sufijo _locked).
 *   Si se llama sin el mutex, hay riesgo de race condition en la lectura de buf[].
 *
 * COMPLEJIDAD: O(n) — búsqueda lineal. Para n=100 es ~100 comparaciones, <1µs.
 *
 * @return Índice en buf[] del candidato a desalojar, o -1 si count == 0.
 */
int EventQueue::findEvictionCandidate_locked() const {
    if (count == 0) return -1;

    int worst = 0;
    for (int i = 1; i < (int)count; i++) {
        if (buf[i].priority > buf[worst].priority) {
            // Este evento es menos urgente (mayor valor EPrio_t)
            worst = i;
        } else if (buf[i].priority == buf[worst].priority &&
                   buf[i].timestamp < buf[worst].timestamp) {
            // Misma prioridad → preferir el más antiguo para desalojar
            // (el más reciente tiene más chance de enviarse pronto)
            worst = i;
        }
    }
    return worst;
}

// ─── saveToNVS() ─────────────────────────────────────────────────────────────
//
// Persiste eventos CRITICAL en NVS para sobrevivir reboots del ESP32.
// Formato del blob: [uint8_t count][QueuedEvent_t ev0]...[QueuedEvent_t evN]
// Solo se persisten hasta EQ_NVS_MAX_PERSIST eventos para no saturar NVS.

/**
 * @brief Persiste eventos CRITICAL en NVS como blob binario.
 *
 * PROPÓSITO:
 *   Garantizar que los eventos de persecución activa (CRITICAL) no se pierdan
 *   si el ESP32 se reinicia (watchdog, panic, corte de energía durante persecución).
 *
 * DISEÑO DE SERIALIZACIÓN:
 *   El blob es simplemente la representación binaria de los structs QueuedEvent_t.
 *   byte[0] = número de eventos. bytes[1..] = sizeof(QueuedEvent_t) × count.
 *   No hay versioning del formato — si QueuedEvent_t cambia, el blob es incompatible.
 *   ARQUITECTURA: Falta versioning del formato NVS. Si QueuedEvent_t cambia
 *      en una actualizacion de firmware, los datos del arranque anterior son corruptos.
 *
 * FLUJO:
 *   1. nvs_open("argus_eq", NVS_READWRITE) — abrir handle NVS.
 *   2. Tomar mutex. Copiar hasta 8 eventos CRITICAL de buf[]. Liberar mutex.
 *      (Se libera ANTES de escribir en NVS para no bloquear la cola durante flash I/O.)
 *   3. malloc(1 + critCount × sizeof(QueuedEvent_t)) — buffer temporal.
 *   4. blob[0] = critCount. memcpy(blob+1, critical).
 *   5. nvs_set_blob("critical", blob, blobSize). nvs_commit(). free(blob).
 *   6. nvs_close(handle).
 *
 * @return ESP_OK si guardó correctamente, código de error NVS si falla.
 */
esp_err_t EventQueue::saveToNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "saveToNVS: nvs_open falló (%s)", esp_err_to_name(ret));
        return ret;
    }

    // Copiar eventos CRITICAL bajo mutex, luego liberar para la escritura NVS.
    // La escritura NVS puede tardar 10-50ms — no bloquear la cola tanto tiempo.
    xSemaphoreTake(mtx, portMAX_DELAY);

    QueuedEvent_t critical[EQ_NVS_MAX_PERSIST];
    uint8_t critCount = 0;
    for (uint32_t i = 0; i < count && critCount < EQ_NVS_MAX_PERSIST; i++) {
        if (buf[i].priority == EPRIO_CRITICAL) {
            critical[critCount++] = buf[i];
        }
    }

    xSemaphoreGive(mtx);  // Liberar mutex ANTES de escribir en NVS

    // Construir el blob binario: [count][ev0][ev1]...
    size_t blobSize = sizeof(uint8_t) + critCount * sizeof(QueuedEvent_t);
    uint8_t* blob = (uint8_t*)malloc(blobSize);
    if (!blob) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    blob[0] = critCount;
    memcpy(blob + 1, critical, critCount * sizeof(QueuedEvent_t));

    ret = nvs_set_blob(handle, NVS_KEY, blob, blobSize);
    free(blob);

    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS: %d eventos críticos guardados", critCount);
    } else {
        ESP_LOGW(TAG, "saveToNVS: error al escribir (%s)", esp_err_to_name(ret));
    }
    return ret;
}

// ─── loadFromNVS() ───────────────────────────────────────────────────────────
//
// Restaura eventos CRITICAL guardados en un arranque anterior.
// Llamar en commTask() antes de empezar a recibir nuevos eventos.

/**
 * @brief Restaura eventos CRITICAL guardados en NVS al arrancar.
 *
 * PROPÓSITO:
 *   Recuperar los eventos de persecución activa que estaban en la cola cuando el
 *   ESP32 se reinició inesperadamente. Se llaman desde commTask() en el arranque.
 *
 * FLUJO:
 *   1. nvs_open("argus_eq", NVS_READONLY) — retorna ESP_ERR_NVS_NOT_FOUND si no existe.
 *   2. nvs_get_blob("critical", nullptr, &blobSize) — consultar tamaño sin leer.
 *   3. malloc(blobSize) + nvs_get_blob() → leer blob completo.
 *   4. Validar: blobSize >= 1 + savedCount × sizeof(QueuedEvent_t).
 *   5. Para cada evento: enqueue() con la política normal de prioridad.
 *   6. free(blob). nvs_close().
 *
 * NOTA SOBRE ESP_ERR_NVS_NOT_FOUND:
 *   Es el caso normal en el primer arranque del dispositivo — no es un error.
 *   También puede ocurrir si el firmware fue flasheado limpiamente (borrando NVS).
 *   Retornamos ESP_OK para que commTask no lo trate como error.
 *
 * NOTA SOBRE LA VALIDACIÓN DEL BLOB:
 *   Si el blob está corrupto (blobSize < expectedSize), se ignora silenciosamente
 *   para evitar crash. La validación savedCount <= EQ_NVS_MAX_PERSIST previene
 *   que un blob malformado cause loop de enqueue() infinito.
 *
 * @return ESP_OK si cargó correctamente (incluso si no había datos guardados).
 */
esp_err_t EventQueue::loadFromNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;  // Primera vez — normal, no es error
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "loadFromNVS: nvs_open falló (%s)", esp_err_to_name(ret));
        return ret;
    }

    // Consultar tamaño del blob sin leer los datos (paso necesario en la API NVS).
    size_t blobSize = 0;
    ret = nvs_get_blob(handle, NVS_KEY, nullptr, &blobSize);
    if (ret == ESP_ERR_NVS_NOT_FOUND || blobSize == 0) {
        nvs_close(handle);
        return ESP_OK;  // No hay datos guardados — normal
    }
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    uint8_t* blob = (uint8_t*)malloc(blobSize);
    if (!blob) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    ret = nvs_get_blob(handle, NVS_KEY, blob, &blobSize);
    nvs_close(handle);

    if (ret != ESP_OK) {
        free(blob);
        return ret;
    }

    uint8_t restored = 0;
    if (blobSize >= 1) {
        uint8_t savedCount = blob[0];
        // Validar que el blob tiene exactamente el tamaño esperado (integridad básica).
        size_t expectedSize = sizeof(uint8_t) + savedCount * sizeof(QueuedEvent_t);
        if (blobSize >= expectedSize && savedCount <= EQ_NVS_MAX_PERSIST) {
            QueuedEvent_t* events = (QueuedEvent_t*)(blob + 1);
            for (uint8_t i = 0; i < savedCount; i++) {
                if (enqueue(events[i])) restored++;
            }
        }
    }

    free(blob);
    ESP_LOGI(TAG, "NVS: %d eventos críticos restaurados", restored);
    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/event_queue.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   EventQueue implementa la "bandeja de salida con prioridad" del sistema.
   Cuando commTask quiere enviar una posición GPS, crea un evento y lo encola.
   SyncManager lo saca de la cola y lo envía. Si falla, lo devuelve con un
   contador de reintentos.

   Lo más importante es la política de overflow: cuando la cola tiene 100
   eventos y llega uno nuevo, no se rechaza automáticamente. Se busca el evento
   de menor urgencia existente (ej. un heartbeat LOW) y se descarta para hacer
   espacio al evento más importante. Así un GPS de persecución (CRITICAL) siempre
   entra aunque la cola esté llena de heartbeats.

   PSEUDOCÓDIGO DE enqueue():
   ```
   if cola_no_llena:
     buf[count++] = ev → retornar OK
   else:
     victim = buscar_peor_evento()
     if victim.prioridad < ev.prioridad:  # victim es menos urgente
       buf[victim] = ev → retornar OK    # desalojar victim
     else:
       rechazar ev → retornar false       # ev no es más urgente que nadie
   ```

   PSEUDOCÓDIGO DE dequeue():
   ```
   best = búsqueda_lineal(menor_prioridad_y_más_antiguo)
   out = buf[best]
   buf[best] = buf[--count]  # compactar O(1)
   retornar out
   ```

   FORMATO NVS:
   [uint8_t count] [QueuedEvent_t ev0] [QueuedEvent_t ev1] ... [QueuedEvent_t evN-1]
   Tamaño: 1 + N × 312 bytes. Máx: 1 + 8 × 312 = 2497 bytes.

   DEUDA TÉCNICA:
   1. Sin versioning del formato NVS: si QueuedEvent_t cambia en una actualización,
      el blob del boot anterior está corrupto. Añadir un byte de versión al inicio.
   2. La validación del blob en loadFromNVS() es básica (solo tamaño).
      Un blob con retryCount=255 o payload corrupto se encolaría sin sanitización.
   3. El mutex se toma en size()/isEmpty()/isFull() aunque podrían leer count
      directamente si el caller ya tiene el mutex. Overhead mínimo pero presente.

   ═══════════════════════════════════════════════════════════ */
