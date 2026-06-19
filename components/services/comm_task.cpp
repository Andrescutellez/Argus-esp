/**
 * @file comm_task.cpp
 * @brief Tarea FreeRTOS de comunicación GPS + telemetría del sistema Argus.
 *
 * PROPÓSITO:
 *   Ejecutar el ciclo de telemetría adaptativa del sistema. Combina tres
 *   subsistemas independientes (A7670Driver, EventQueue, SyncManager) para:
 *   1. Obtener posición GPS del modem A7670 cada N segundos.
 *   2. Construir el payload del protocolo Argus con firma CRC32.
 *   3. Encolarlo con prioridad según el estado del sistema.
 *   4. Drenar la cola y enviar al backend via TCP con backoff exponencial.
 *
 * ARQUITECTURA DE COMUNICACIÓN:
 *
 *   GPS polling (cada N segundos según estado/plan)
 *     → a7670.getGpsPosition(gpsData)
 *     → notifyGpsStatusChange()   ← actualiza xEventQueue (fix ok/lost) + FLAG_GPS_VALID
 *     → xQueueOverwrite(xGpsDataQueue, gpsData)  ← bleTask puede leer posición
 *     → buildArgusPacket()        ← "ARGUS|id|epoch|lat|lon|crc32\n"
 *     → submitReport()            ← eventQueue con prioridad según estado
 *     → syncManager.tick()        ← drena la cola, TCP send, backoff
 *     → backend TCP (34.69.219.193:9000)
 *
 *   Métricas de sync (cada iteración del loop):
 *     syncManager.getMetrics() → si nueva falla o éxito → EVENT_COMMS_SUCCESS/FAILURE
 *     → xEventQueue → controlTask (puede indicar estado de conectividad en LEDs)
 *
 * PROTOCOLO ARGUS TCP:
 *   Formato: "ARGUS|<device_id>|<epoch_ms>|<lat>|<lon>|<crc32_hex>\n"
 *   Ejemplo: "ARGUS|ARGUS-1A2B3C4D|1715027200000|19.432608|-99.133209|3F8A1B2C\n"
 *
 *   CRC32 se calcula sobre la cadena:
 *     "<device_id>|<epoch_ms>|<lat>|<lon>|<ARGUS_SIGNATURE_SECRET>"
 *   usando el algoritmo CRC32 Ethernet (polinomio 0xEDB88320, big-endian reflect).
 *   El backend recalcula el CRC con el mismo secreto para autenticar el frame.
 *
 * DEVICE ID:
 *   Derivado de los últimos 4 bytes de la MAC del ESP32 (eFuse).
 *   Formato: "ARGUS-AABBCCDD" (14 chars + null = 15 bytes, buf[20] tiene margen).
 *   La MAC viene de esp_efuse_mac_get_default() que retorna 6 bytes en orden MSB.
 *   Se usan mac[2..5] para mejor distribución entre dispositivos del mismo lote.
 *
 * INTERVALOS DE TELEMETRÍA:
 *   STATE_PURSUIT:                 10s  (emergencia — ignora plan)
 *   STATE_ALERT:                   15s  (alerta — ignora plan)
 *   STATE_MOVING / STATE_IDLE:     30s  (PREMIUM) / 3600s (FREEMIUM)
 *
 * LÓGICA DE INACTIVIDAD (5 minutos en IDLE):
 *   Si el sistema está en IDLE sin movimiento por >5min:
 *     - La telemetría GPS activa se pausa.
 *     - Si la cola está vacía y hay fix previo: envía keepalive con lastKnownGps.
 *     - a7670.isAlive() periódicamente para no perder la conexión TCP.
 *   Al volver a haber actividad: reanuda telemetría normal, verifica A7670.
 *   Esto conserva batería y cuota de datos del plan FREEMIUM.
 *
 * DEPENDENCIAS:
 *   - sync_manager.h:  SyncManager (cola con backoff y envío TCP)
 *   - event_queue.h:   EventQueue (cola con prioridad y NVS)
 *   - system_events.h: EventMessage_t, SystemState_t, xEventQueue, xGpsDataQueue
 *   - system_flags.h:  systemArmed, planType, lastMovementTimestamp, xSystemFlags
 *   - state_machine.h: StateMachine::stateName() para logging
 *   - a7670_driver.h:  A7670Driver (GPS + TCP)
 *   - esp_mac.h:       esp_efuse_mac_get_default()
 *   - esp_timer.h:     esp_timer_get_time() para timestamps de alta resolución
 *
 * VARIABLES CRÍTICAS:
 *   - g_deviceId:          ID del dispositivo. Inicializado una vez en initDeviceId().
 *   - lastKnownGps:        Último fix GPS válido. Permite reportar posición incluso
 *                          si el GNSS pierde señal temporalmente (ej. túnel).
 *   - hasEverHadFix:       Guarda si el dispositivo obtuvo al menos un fix en esta
 *                          sesión. Sin ningún fix, no tiene sentido enviar coords (0,0).
 *   - ARGUS_SIGNATURE_SECRET: Secreto CRC32. Hardcodeado en dev.
 *   - telemetriaPausada:   Flag local que evita log spam cuando se pausa por inactividad.
 *   - lastSendTime:        Timestamp del último ciclo de telemetría. Inicializado
 *                          con (now - interval) para enviar inmediatamente al arrancar.
 *   - lastNvsSave:         Timestamp del último guardado de NVS.
 *
 * CONCURRENCIA:
 *   - lastMovementTimestamp: escrito por sensorTask (cualquier evento). Ver system_flags.h.
 *   - lastHardMovementTimestamp: escrito por sensorTask (solo HARD/IMPACT). Gate de GPS sleep.
 *   - currentSystemState:    escrito por controlTask, leído aquí. Lectura sin mutex.
 *   - xGpsDataQueue (cap. 1): xQueueOverwrite es atómico. commTask escribe, bleTask lee.
 *   - FLAG_COMMS_BUSY: se activa antes de submitReport() y se limpia después.
 *     Ninguna otra tarea actúa sobre este flag actualmente — es informativo.
 *
 * ARQUITECTURA:
 *    ARGUS_SIGNATURE_SECRET esta hardcodeado como "argus-dev-secret".
 *    En produccion esto es una vulnerabilidad de seguridad: cualquiera con el firmware
 *    puede forjar frames validos. El secreto deberia venir de:
 *    1. Provisioning seguro en fabrica -> NVS con encriptacion flash.
 *    2. Mutual TLS en lugar de HMAC/CRC -- el certificado autentica el dispositivo.
 *    No se corrige aqui porque afecta tambien al backend. Reportar antes de produccion.
 *
 * @module components/services/comm_task
 */

#include "comm_task.h"
#include "sync_manager.h"
#include "event_queue.h"
#include "system_events.h"
#include "system_flags.h"
#include "state_machine.h"
#include "a7670_driver.h"
#include "sensor_task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

// TAG de logging para todos los mensajes de esta tarea.
static const char* TAG = "CommTask";

// Handle de la tarea. Asignado por xTaskCreate() en main.cpp.
TaskHandle_t xCommTaskHandle = nullptr;

// ─── Instancias del subsistema de comunicación ────────────────────────────────
//
// Se instancian como static (una sola instancia por proceso) porque:
// - La tarea commTask es la única que los usa.
// - No necesitan vivir más allá del ciclo de vida de la tarea.
// - Static garantiza inicialización en BSS antes de que la tarea arranque.
// SyncManager recibe referencias a a7670 y eventQueue (sin copia).

static A7670Driver  a7670;
static EventQueue   eventQueue;
static SyncManager  syncManager(a7670, eventQueue);

/**
 * Buffer para el Device ID del dispositivo.
 * Formato: "ARGUS-AABBCCDD" (14 chars + '\0'). buf[20] tiene margen.
 * Se llena una sola vez en initDeviceId() y luego es de solo lectura.
 */
static char g_deviceId[20] = {0};

// ─── Constantes de tiempo ─────────────────────────────────────────────────────

/**
 * Timeout de inactividad antes de pausar la telemetría GPS.
 * 5 minutos = 5 × 60 × 1_000_000 µs. Usa ULL para evitar overflow de uint32_t.
 * Solo aplica en STATE_IDLE (el sistema está quieto y desarmado o vigilando).
 */
static const uint64_t INACTIVITY_TIMEOUT_US = 5ULL * 60000000ULL;

/**
 * Intervalo de guardado periódico de la EventQueue en NVS.
 * 5 minutos: balance entre durabilidad (no perder eventos CRITICAL en crash)
 * y desgaste de flash (NVS wear leveling, pero las escrituras frecuentes lo reducen).
 */
static const uint64_t NVS_SAVE_INTERVAL_US  = 5ULL * 60000000ULL;

/**
 * Intervalo entre keepalives cuando la telemetría está pausada por inactividad.
 * 5 minutos: el servidor tiene INACTIVITY_TIMEOUT_MS = 600,000 ms (10 min),
 * así que un keepalive cada 5 min mantiene el socket abierto con margen.
 * Reduce los paquetes 4G de ~120/h a ~12/h mientras la moto está quieta.
 * Los comandos (ARM/DISARM) siguen llegando en ~1s vía URC del modem,
 * independientemente de este intervalo.
 */
static const uint64_t KEEPALIVE_INTERVAL_US = 5ULL * 60000000ULL;

/**
 * Secreto compartido para la firma CRC32 del protocolo Argus.
 * Se incluye en la cadena que se firma antes de calcular el CRC32.
 * ARQUITECTURA: En produccion debe venir de NVS con flash encryption.
 */
static constexpr char ARGUS_SIGNATURE_SECRET[] = "argus-dev-secret";

/**
 * Última posición GPS válida conocida.
 * Permite reportar posición al backend incluso si el GNSS pierde señal (ej. túnel,
 * parking subterráneo). El backend lo distingue por epoch=0 (timestamp inválido).
 */
static GpsData_t lastKnownGps   = {};

/**
 * Flag que indica si el dispositivo obtuvo al menos un fix GPS en esta sesión.
 * false → no enviar telemetría (coords (0,0) son ruido para el backend).
 * true  → puede usar lastKnownGps como fallback si el fix actual no está válido.
 */
static bool hasEverHadFix  = false;

// ─── Intervalo de telemetría según plan y estado ──────────────────────────────

/**
 * @brief Calcula el intervalo de telemetría activo según el estado y el plan.
 *
 * PROPÓSITO:
 *   Adaptar la frecuencia de reportes GPS al estado de urgencia del sistema.
 *   En estados de emergencia, el intervalo se reduce para máxima trazabilidad.
 *   En estado normal, el intervalo depende del plan contratado.
 *
 * LÓGICA:
 *   PURSUIT → 10s (emergencia — ignora plan, necesitamos localización continua)
 *   ALERT   → 15s (alerta activa — ignora plan)
 *   MOVING/IDLE + PREMIUM  → 30s (monitoreo activo)
 *   MOVING/IDLE + FREEMIUM → 3600s (1h — solo heartbeat, conserva datos del plan)
 *
 * NOTA: Lee currentSystemState directamente sin mutex (ver system_flags.h).
 *   En la práctica, leer un enum de 4 bytes en ARM es atómico.
 *
 * @return Intervalo en microsegundos (uint64_t para compatibilidad con esp_timer_get_time()).
 */
static uint64_t getActiveTelemetryIntervalUs() {
    switch (currentSystemState) {
        case STATE_PURSUIT: return 10ULL * 1000000ULL;   // 10s — emergencia
        case STATE_ALERT:   return 15ULL * 1000000ULL;   // 15s — alerta activa
        case STATE_MOVING:
        case STATE_IDLE:
        default:
            if (planType == PLAN_PREMIUM) {
                return 30ULL * 1000000ULL;    // 30s — monitoreo normal PREMIUM
            } else {
                return 3600ULL * 1000000ULL;  // 1h  — FREEMIUM, conservar datos
            }
    }
}

// ─── Generación de Device ID único ───────────────────────────────────────────

/**
 * @brief Inicializa g_deviceId a partir de la MAC del ESP32.
 *
 * PROPÓSITO:
 *   Generar un identificador único y persistente para el dispositivo.
 *   El backend usa este ID para asociar el frame TCP al dispositivo en la BD.
 *
 * FLUJO:
 *   1. esp_efuse_mac_get_default(mac) → 6 bytes de MAC desde eFuse.
 *   2. snprintf con mac[2..5] → "ARGUS-AABBCCDD" (8 hex chars).
 *
 * NOTA: Se usan mac[2..5] (4 bytes) y no los 6 completos porque:
 *   - Los primeros 3 bytes son el OUI del fabricante (igual para todos los ESP32).
 *   - Los últimos 3 bytes son el serial dentro del lote.
 *   - 4 bytes (mac[2..5]) dan 32 bits de entropía real, suficiente para distinguir
 *     dispositivos de la misma instalación. El formato resultante de 8 hex chars
 *     es legible y corto para URLs y logs.
 */
static void initDeviceId() {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(g_deviceId, sizeof(g_deviceId), "ARGUS-%02X%02X%02X%02X",
             mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device ID: %s | Plan: %s",
             g_deviceId, (planType == PLAN_PREMIUM) ? "PREMIUM" : "FREEMIUM");
}

// ─── Estado de fix GPS ────────────────────────────────────────────────────────

/**
 * Estado previo de fix GPS. Permite detectar transiciones fix→no-fix y no-fix→fix
 * sin generar eventos duplicados en cada ciclo de telemetría.
 */
static bool previousFixValid = false;

/**
 * @brief Notifica al sistema cuando cambia el estado del fix GPS.
 *
 * PROPÓSITO:
 *   Detectar transiciones de estado del GNSS (fix adquirido / fix perdido) y
 *   notificarlas al sistema via xEventQueue y xSystemFlags. Solo envía un evento
 *   cuando HAY UN CAMBIO (no en cada ciclo si el estado es el mismo).
 *
 * FLUJO:
 *   Si gps.fix_valid && !previousFixValid → EVENT_GPS_FIX_OK + FLAG_GPS_VALID.
 *   Si !gps.fix_valid && previousFixValid → EVENT_GPS_FIX_LOST + limpiar FLAG_GPS_VALID.
 *   Si no hay cambio → no hace nada.
 *
 * NOTA: xQueueSend con timeout=0. Si la cola está llena, el evento GPS se pierde.
 *   Es aceptable: los eventos de GPS no son críticos para la seguridad del sistema.
 *
 * @param gps Datos GPS actuales (solo se usa gps.fix_valid y gps.satellites).
 */
static void notifyGpsStatusChange(const GpsData_t& gps) {
    if (gps.fix_valid && !previousFixValid) {
        EventMessage_t msg = {};
        msg.event    = EVENT_GPS_FIX_OK;
        msg.data.gps = gps;
        xQueueSend(xEventQueue, &msg, 0);
        xEventGroupSetBits(xSystemFlags, FLAG_GPS_VALID);
        ESP_LOGI(TAG, "GPS fix adquirido (%d satélites)", gps.satellites);

    } else if (!gps.fix_valid && previousFixValid) {
        EventMessage_t msg = {};
        msg.event = EVENT_GPS_FIX_LOST;
        xQueueSend(xEventQueue, &msg, 0);
        xEventGroupClearBits(xSystemFlags, FLAG_GPS_VALID);
        ESP_LOGW(TAG, "GPS fix perdido");
    }
    previousFixValid = gps.fix_valid;
}

// ─── Construcción del payload JSON ───────────────────────────────────────────

/**
 * @brief Calcula el número de días desde el 1 de enero del año civil 1 (era civil).
 *
 * PROPÓSITO:
 *   Algoritmo auxiliar para convertir una fecha civil (año, mes, día) al número
 *   de días desde la época de referencia usada en gpsTimestampToEpoch(). Se usa
 *   el algoritmo de Howard Hinnant (C++, dominio público) que es correcto para
 *   fechas desde el año 0 al infinito sin recursión ni tabla de meses.
 *
 * ALGORITMO:
 *   Trabaja en "eras" de 400 años (el ciclo gregoriano) para eficiencia.
 *   year -= 1 si month <= 2 (el año "empieza" en marzo, simplifica bisiestos).
 *   El resultado es el número de días que hay entre el 1 de marzo del año 0
 *   y la fecha dada, desplazado a la época Unix (-719468 días a la época Unix).
 *
 * NOTA: La resta de 719468 al final alinea el resultado con Unix epoch (1970-01-01).
 *   Sin esa resta, el resultado sería días desde la época civil (año 0, 1 de enero).
 *
 * @param year  Año (ej. 2024).
 * @param month Mes (1-12).
 * @param day   Día (1-31).
 * @return Número de días desde Unix epoch hasta esa fecha.
 */
static int64_t daysFromCivil(int year, unsigned month, unsigned day) {
    year -= (month <= 2U) ? 1 : 0;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = (unsigned)(year - era * 400);
    const unsigned doy = (153U * (month + (month > 2U ? -3 : 9)) + 2U) / 5U + day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
}

/**
 * @brief Convierte el timestamp GPS del A7670 a Unix epoch en segundos.
 *
 * PROPÓSITO:
 *   Parsear el timestamp del GNSS (formato "YYYYMMDD HHMMSS.mmm") y convertirlo
 *   a Unix epoch (segundos desde 1970-01-01 00:00:00 UTC) para incluir en el
 *   payload del protocolo Argus.
 *
 * FLUJO:
 *   1. sscanf con formato "%4d%2d%2d %2d%2d%2d" → year/month/day/hour/min/sec.
 *   2. Validar rangos (mes 1-12, día 1-31, hora 0-23, min/sec 0-59/60).
 *   3. daysFromCivil(year, month, day) → días desde epoch.
 *   4. days × 86400 + hour×3600 + min×60 + sec → seconds from epoch.
 *
 * NOTA: sec puede ser 60 (leap second). Se acepta el rango 0-60 por esto.
 *   La librería estándar maneja leap seconds de forma inconsistente;
 *   aquí simplemente aceptamos el valor del GNSS sin corrección.
 *
 * NOTA: Retorna epoch en segundos. El caller multiplica por 1000 para ms.
 *
 * @param gpsTimestamp String en formato "YYYYMMDD HHMMSS.mmm" (del A7670).
 * @param outEpoch     Puntero donde escribir el resultado en segundos.
 * @return true si el parsing y la validación fueron exitosos.
 */
static bool gpsTimestampToEpoch(const char* gpsTimestamp, uint64_t* outEpoch) {
    if (gpsTimestamp == nullptr || outEpoch == nullptr) return false;
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
    if (sscanf(gpsTimestamp, "%4d%2d%2d %2d%2d%2d",
               &year, &month, &day, &hour, &minute, &second) != 6) return false;
    if (month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23   || minute < 0 || minute > 59 ||
        second < 0 || second > 60) return false;
    const int64_t days = daysFromCivil(year, (unsigned)month, (unsigned)day);
    if (days < 0) return false;
    *outEpoch = (uint64_t)days * 86400ULL +
                (uint64_t)hour  * 3600ULL +
                (uint64_t)minute * 60ULL  +
                (uint64_t)second;
    return true;
}

/**
 * @brief Calcula el CRC32 sobre un buffer de bytes usando el polinomio Ethernet.
 *
 * PROPÓSITO:
 *   Generar la firma de integridad del frame Argus. El backend recalcula este
 *   CRC con el mismo secreto para verificar que el frame no fue alterado en tránsito.
 *
 * ALGORITMO:
 *   CRC32 estándar Ethernet (ISO 3309, IEEE 802.3):
 *   - Polinomio: 0xEDB88320 (representación bit-reverse de 0x04C11DB7).
 *   - Valor inicial: 0xFFFFFFFF.
 *   - XOR final: ~crc (complemento).
 *   - El "mask" trick: `-(crc & 1)` genera 0xFFFFFFFF si el bit es 1, 0 si es 0,
 *     evitando un branch en el loop interno. Más rápido sin FPU en ESP32.
 *
 * NOTA: Implementación tabless (bit a bit) — más lenta que tabla de 256 entradas
 *   pero no requiere 1KB de RAM para la tabla. Para frames de ~100 bytes es
 *   indetectable en velocidad de CPU a 240MHz.
 *
 * @param data Puntero al buffer de bytes a firmar.
 * @param len  Número de bytes en el buffer.
 * @return CRC32 de 32 bits en formato host-endian (se imprime como hex en el frame).
 */
static uint32_t crc32Argus(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            // mask es 0xFFFFFFFF si el LSB es 1, 0 si es 0.
            // XOR condicional sin branch: (crc >> 1) XOR (polinomio & mask).
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;  // XOR final estándar CRC32
}

/**
 * @brief Resuelve el epoch de reporte a partir de los datos GPS.
 *
 * PROPÓSITO:
 *   Extraer el timestamp Unix en milisegundos del GPS para incluir en el frame.
 *   Si el timestamp no es parseable, retorna 0 como señal al backend de que
 *   debe usar Date.now() como timestamp del reporte.
 *
 * @param gps Datos GPS del ciclo actual.
 * @return Unix epoch en milisegundos, o 0 si el timestamp no es válido.
 */
static uint64_t resolveReportEpoch(const GpsData_t& gps) {
    uint64_t epoch = 0;
    // Retornar Unix ms. Sin fix → 0 para que el backend use Date.now() como fallback.
    if (gpsTimestampToEpoch(gps.timestamp, &epoch)) return epoch * 1000ULL;
    return 0;
}

/**
 * @brief Construye el frame TCP de métricas de conducción (prefijo DRIVE).
 *
 * PROPÓSITO:
 *   Formatear el payload que lleva las métricas del MPU6050 acumuladas desde
 *   el último ciclo GPS. El backend lo recibe, lo guarda en MongoDB (colección
 *   'drivemetrics') y lo usa para construir los reportes de conducción en la app.
 *
 * FORMATO:
 *   "DRIVE|<id>|<epoch>|<lat>|<lon>|<peakAccel>|<peakGyro>|<hard>|<soft>|<crc32>\n"
 *   - peakAccel: pico de desviación de aceleración en g (proxy de agresividad)
 *   - peakGyro:  pico de magnitud del giroscopio en °/s (proxy de giro brusco)
 *   - hard:      eventos HARD + IMPACT en la ventana (uint)
 *   - soft:      eventos SOFT en la ventana (uint)
 *
 * FIRMA CRC32:
 *   Mismo payload base que buildArgusPacket() — el backend reutiliza verifySignature()
 *   sin cambios: "{deviceId}|{epoch}|{lat:.6f}|{lon:.6f}|{SECRET}"
 *
 * VENTANA:
 *   Las métricas se acumulan desde el último reset (commTask hace memset tras enviar).
 *   Con PREMIUM (GPS cada 30s) la ventana es ~30s.
 *   Con FREEMIUM en ALERT/PURSUIT (GPS cada 15-10s) la ventana puede ser más corta.
 *
 * @param buf      Buffer destino.
 * @param maxLen   Tamaño del buffer.
 * @param lat      Latitud del ciclo GPS actual.
 * @param lon      Longitud del ciclo GPS actual.
 * @param epoch    Unix epoch en ms del ciclo GPS actual.
 */
static void buildDriveFrame(char* buf, size_t maxLen,
                             double lat, double lon, uint64_t epoch) {
    // Snapshot atómico de las métricas antes de que sensorTask las modifique.
    // Se leen las volatiles una sola vez para no mezclar valores de distintos instantes.
    const float    peakAccel = g_driveMetrics.peakAccelDev;
    const float    peakGyro  = g_driveMetrics.peakGyroMag;
    const uint16_t hardCount = g_driveMetrics.hardCount;
    const uint16_t softCount = g_driveMetrics.softCount;

    // Misma firma CRC32 que buildArgusPacket() — el backend ya sabe validarla.
    char signatureBase[128];
    int len = snprintf(signatureBase, sizeof(signatureBase),
                       "%s|%" PRIu64 "|%.6f|%.6f|%s",
                       g_deviceId, epoch, lat, lon, ARGUS_SIGNATURE_SECRET);
    if (len < 0) { buf[0] = '\0'; return; }

    const uint32_t crc = crc32Argus((const uint8_t*)signatureBase, (size_t)len);

    snprintf(buf, maxLen,
             "DRIVE|%s|%" PRIu64 "|%.6f|%.6f|%.4f|%.2f|%u|%u|%08" PRIX32 "\n",
             g_deviceId, epoch, lat, lon,
             peakAccel, peakGyro, hardCount, softCount, crc);
}

/**
 * @brief Construye el frame TCP del protocolo Argus con firma CRC32.
 *
 * PROPÓSITO:
 *   Formatear el payload que se envía al backend TCP con todos los campos del
 *   protocolo Argus: ID de dispositivo, timestamp, coordenadas y firma CRC32.
 *   El frame incluye '\n' al final como delimitador de mensaje para el servidor TCP.
 *
 * FLUJO:
 *   1. Construir la cadena de firma: "<id>|<epoch>|<lat>|<lon>|<secret>".
 *   2. crc32Argus() → CRC32 de 32 bits.
 *   3. snprintf del frame final: "ARGUS|<id>|<epoch>|<lat>|<lon>|<crc32_hex>\n".
 *
 * FORMATO CAMPO A CAMPO:
 *   "ARGUS"    → prefijo fijo (el servidor TCP lo usa para identificar el protocolo)
 *   g_deviceId → "ARGUS-AABBCCDD" (ID único del dispositivo)
 *   epoch      → milisegundos Unix (uint64_t, %" PRIu64 ")
 *   lat/lon    → %.6f (6 decimales = ~11cm de precisión)
 *   crc32      → %08" PRIX32 " (8 hex mayúsculas, siempre 8 chars)
 *   "\n"       → delimitador de mensaje para el servidor TCP (lectura por línea)
 *
 * NOTA: Si snprintf del signatureBase falla (retorna <0), buf se vacía ('\0').
 *   En la práctica, el buffer de 128 bytes es suficiente para la cadena de firma.
 *
 * @param buf    Buffer destino para el frame TCP.
 * @param maxLen Tamaño del buffer destino.
 * @param lat    Latitud en grados decimales WGS-84.
 * @param lon    Longitud en grados decimales WGS-84.
 * @param epoch  Unix epoch en milisegundos (o 0 si no hay timestamp GPS válido).
 */
static void buildArgusPacket(char* buf, size_t maxLen,
                              double lat, double lon, uint64_t epoch,
                              float speedKmh) {
    // La cadena de firma cubre id|epoch|lat|lon — NO incluye speed.
    // Speed es telemetría de display, no afecta decisiones de seguridad.
    // Mantener speed fuera del CRC simplifica el verificador del backend.
    char signatureBase[128];
    int signatureBaseLen = snprintf(signatureBase, sizeof(signatureBase),
                                    "%s|%" PRIu64 "|%.6f|%.6f|%s",
                                    g_deviceId, epoch, lat, lon,
                                    ARGUS_SIGNATURE_SECRET);
    if (signatureBaseLen < 0) { buf[0] = '\0'; return; }

    const uint32_t signature = crc32Argus((const uint8_t*)signatureBase,
                                          (size_t)signatureBaseLen);

    // Frame Argus TCP v2: ARGUS|id|epoch|lat|lon|speed_kmh|crc32 (7 campos).
    // speed: 1 decimal es suficiente para mostrar en UI (ej: "47.3 km/h").
    snprintf(buf, maxLen,
             "ARGUS|%s|%" PRIu64 "|%.6f|%.6f|%.1f|%08" PRIX32 "\n",
             g_deviceId, epoch, lat, lon, speedKmh, signature);
}

// ─── Encolar según prioridad del estado actual ────────────────────────────────

/**
 * @brief Encola el payload en la EventQueue con prioridad según el estado del sistema.
 *
 * PROPÓSITO:
 *   Asignar la prioridad correcta al evento de telemetría según el estado actual.
 *   El SyncManager usará la prioridad para decidir el orden de envío y la política
 *   de reintentos (CRITICAL se persiste en NVS y no se descarta nunca).
 *
 * LÓGICA DE PRIORIDAD:
 *   STATE_PURSUIT → submitCriticalAlert() → PRIORITY_CRITICAL (persistido en NVS)
 *   STATE_ALERT   → submitAlertGps()      → PRIORITY_HIGH     (reintentado varias veces)
 *   STATE_MOVING  → submitGps()           → PRIORITY_NORMAL   (reintentado pocas veces)
 *   STATE_IDLE armado  → submitGps()      → PRIORITY_NORMAL   (vigilancia activa)
 *   STATE_IDLE desarmado → submitHeartbeat() → PRIORITY_LOW   (solo keepalive)
 *
 * NOTA: En STATE_IDLE desarmado, el sistema no está vigilando activamente pero
 *   sigue enviando heartbeats para que el backend sepa que el dispositivo está vivo.
 *   Si el backend no recibe heartbeat en X tiempo, puede alertar al usuario.
 *
 * @param payload  Frame TCP del protocolo Argus (terminado en '\n').
 * @param sysState Estado actual del sistema (para determinar prioridad).
 * @param nowUs    Timestamp actual en microsegundos (esp_timer_get_time()).
 */
static void submitReport(const char* payload, SystemState_t sysState, uint64_t nowUs) {
    switch (sysState) {
        case STATE_PURSUIT:
            syncManager.submitCriticalAlert(payload, nowUs);
            ESP_LOGI(TAG, "[CRITICAL/PURSUIT] encolado");
            break;
        case STATE_ALERT:
            syncManager.submitAlertGps(payload, nowUs);
            ESP_LOGI(TAG, "[HIGH/ALERT] encolado");
            break;
        case STATE_MOVING:
            syncManager.submitGps(payload, nowUs);
            ESP_LOGI(TAG, "[NORMAL/MOVING] encolado");
            break;
        case STATE_IDLE:
        default:
            if (systemArmed) {
                syncManager.submitGps(payload, nowUs);
                ESP_LOGI(TAG, "[NORMAL/IDLE-armado] encolado");
            } else {
                // Desarmado: solo heartbeat para mantener la sesión TCP del backend.
                syncManager.submitHeartbeat(payload, nowUs);
            }
            break;
    }
}

// ─── Tracking de estado para detección de transiciones ───────────────────────

/**
 * Último estado del sistema visto por commTask.
 * Se compara con currentSystemState en cada tick para detectar transiciones
 * y enviar el frame EVENT correspondiente al backend.
 * Se inicializa en commTask() (antes del while) para no generar eventos falsos en boot.
 */
static SystemState_t prevSysState    = STATE_IDLE;

/**
 * Último valor de systemArmed visto por commTask.
 * ARM y DISARM no cambian el estado (siguen en STATE_IDLE), solo cambian
 * este flag, por eso se rastrea por separado.
 */
static bool          prevSystemArmed = false;

// ─── Construcción de frame EVENT ──────────────────────────────────────────────

/**
 * @brief Construye el frame TCP del protocolo Argus para un evento de seguridad.
 *
 * PROPÓSITO:
 *   Formatear el payload EVENT que se envía al backend cuando el sistema cambia
 *   de estado de seguridad (ARM, DISARM, STATE_ALERT, STATE_PURSUIT, STATE_IDLE).
 *   El backend distingue este frame del ARGUS GPS por el prefijo "EVENT".
 *
 * FORMATO DEL FRAME:
 *   "EVENT|<device_id>|<type>|<epoch_ms>|<lat>|<lon>|<crc32_hex>\n"
 *   Ejemplo: "EVENT|ARGUS-1A2B|STATE_ALERT|1715027200000|19.432608|-99.133209|3F8A1B2C\n"
 *
 * FIRMA CRC32:
 *   El CRC se calcula sobre el MISMO payload que buildArgusPacket():
 *   "<device_id>|<epoch_ms>|<lat.6f>|<lon.6f>|<ARGUS_SIGNATURE_SECRET>"
 *   El campo 'type' NO está en el CRC intencionalmente:
 *   1. Mantiene el mismo algoritmo de firma que el backend ya implementa para ARGUS.
 *   2. El type viene de un device ya autenticado por whitelist + CRC — no hay
 *      amenaza real en que un device legítimo "falsifique" su propio tipo de evento.
 *   3. Simplifica el firmware: reutiliza crc32Argus() sin cambios.
 *
 * DEPENDENCIAS:
 *   - g_deviceId:             ID del dispositivo, inicializado en initDeviceId().
 *   - ARGUS_SIGNATURE_SECRET: secreto compartido con el backend.
 *   - crc32Argus():           Función CRC32 de este módulo.
 *
 * @param buf     Buffer destino para el frame TCP.
 * @param maxLen  Tamaño del buffer destino.
 * @param type    Tipo de evento: "ARM", "DISARM", "STATE_ALERT", "STATE_PURSUIT", "STATE_IDLE".
 * @param lat     Latitud en grados decimales (0.0 si sin fix GPS al momento del evento).
 * @param lon     Longitud en grados decimales (0.0 si sin fix GPS).
 * @param epoch   Unix epoch en milisegundos (0 si sin timestamp GPS válido).
 */
static void buildEventFrame(char* buf, size_t maxLen,
                             const char* type,
                             double lat, double lon, uint64_t epoch) {
    // Payload de firma: mismo formato que buildArgusPacket() — el backend
    // usa verifySignature(deviceId, timestamp, lat, lng, sig) para ambos frames.
    char signatureBase[128];
    int len = snprintf(signatureBase, sizeof(signatureBase),
                       "%s|%" PRIu64 "|%.6f|%.6f|%s",
                       g_deviceId, epoch, lat, lon, ARGUS_SIGNATURE_SECRET);
    if (len < 0) { buf[0] = '\0'; return; }

    const uint32_t crc = crc32Argus((const uint8_t*)signatureBase, (size_t)len);

    // Frame EVENT: mismo separador '|' y terminador '\n' que el frame ARGUS GPS.
    snprintf(buf, maxLen,
             "EVENT|%s|%s|%" PRIu64 "|%.6f|%.6f|%08" PRIX32 "\n",
             g_deviceId, type, epoch, lat, lon, crc);
}

/**
 * @brief Encola un frame EVENT en el SyncManager con la prioridad correcta.
 *
 * PROPÓSITO:
 *   Enviar al backend un cambio de estado de seguridad del dispositivo,
 *   reutilizando toda la infraestructura de backoff y confiabilidad del SyncManager.
 *   La prioridad se asigna según la urgencia del evento:
 *   - STATE_ALERT / STATE_PURSUIT → HIGH (el backend debe notificar al usuario YA)
 *   - ARM / DISARM / STATE_IDLE   → NORMAL (confirmación, menos urgente)
 *
 * COORDENADAS EN EVENTOS:
 *   Se usan las últimas coordenadas conocidas (lastKnownGps) aunque el evento
 *   no sea un reporte GPS. Esto permite al backend saber DÓNDE ocurrió la alerta.
 *   Si nunca hubo fix GPS, se envían (0.0, 0.0) y el backend guarda null.
 *
 * DEPENDENCIAS:
 *   - buildEventFrame(): construye el frame TCP.
 *   - syncManager: encola con prioridad para envío en próximo tick.
 *   - hasEverHadFix / lastKnownGps: coordenadas del último fix conocido.
 *   - resolveReportEpoch(): timestamp del último fix GPS.
 *
 * @param type   Tipo de evento ("STATE_ALERT", "ARM", "DISARM", etc.).
 * @param nowUs  Timestamp actual en µs (esp_timer_get_time()).
 */
static void submitEventFrame(const char* type, uint64_t nowUs) {
    char eventPacket[320];

    // Usar las últimas coordenadas conocidas. Si no hubo fix, (0,0) indica al
    // backend que guarde lat=null/lon=null (ver tcpServer.js, manejo de 0,0).
    const double   lat   = hasEverHadFix ? (double)lastKnownGps.latitude  : 0.0;
    const double   lon   = hasEverHadFix ? (double)lastKnownGps.longitude : 0.0;
    const uint64_t epoch = hasEverHadFix ? resolveReportEpoch(lastKnownGps) : 0ULL;

    buildEventFrame(eventPacket, sizeof(eventPacket), type, lat, lon, epoch);

    // buildEventFrame() deja buf[0]='\0' solo si snprintf retornó < 0 (error de formato).
    // En la práctica esto nunca ocurre con los type strings que usamos.
    if (eventPacket[0] == '\0') {
        ESP_LOGE(TAG, "buildEventFrame falló para type=%s", type);
        return;
    }

    // Prioridad HIGH para alertas activas: deben llegar al backend antes que
    // los frames GPS normales que pudieran estar en la cola.
    if (strcmp(type, "STATE_ALERT") == 0 || strcmp(type, "STATE_PURSUIT") == 0) {
        syncManager.submitAlertGps(eventPacket, nowUs);
    } else {
        syncManager.submitGps(eventPacket, nowUs);
    }

    ESP_LOGI(TAG, "EVENT encolado → type=%s lat=%.4f lon=%.4f epoch=%" PRIu64,
             type, lat, lon, epoch);
}

// ─── commTask ─────────────────────────────────────────────────────────────────

/**
 * @brief Función principal de la tarea FreeRTOS de comunicación.
 *
 * PROPÓSITO:
 *   Ejecutar el ciclo de telemetría adaptativa del sistema Argus. Cada segundo,
 *   drena la cola de eventos via SyncManager. Cada N segundos (según estado/plan),
 *   obtiene GPS, construye payload y encola el reporte.
 *
 * FLUJO DE ARRANQUE:
 *   1. initDeviceId()           → "ARGUS-AABBCCDD" desde MAC de eFuse.
 *   2. eventQueue.init()        → inicializar cola con mutex interno.
 *   3. eventQueue.loadFromNVS() → restaurar eventos CRITICAL del boot anterior.
 *   4. syncManager.init()       → preparar el manager de sync con el device ID.
 *   5. a7670.init()             → inicializar modem (AT commands, GNSS, TCP).
 *      - Si falla: loguear y continuar (el modem puede recuperarse en ciclos posteriores).
 *   6. Inicializar lastSendTime = now - interval para enviar en el primer ciclo.
 *
 * LOOP (cada 1s):
 *   A. CICLO DE TELEMETRÍA (si now - lastSendTime >= telemetryInterval):
 *      1. Calcular estado de inactividad (IDLE + >5min sin movimiento).
 *      2. Si ACTIVO:
 *         a. Si volvemos de pausa: verificar a7670.isAlive(), recover() si necesario.
 *         b. a7670.getGpsPosition(gpsData)     → obtener posición GPS actual.
 *         c. notifyGpsStatusChange(gpsData)    → notificar fix/no-fix al sistema.
 *         d. xQueueOverwrite(xGpsDataQueue)    → actualizar para bleTask.
 *         e. Seleccionar coordenadas:
 *            - fix válido    → posición actual + epoch GPS.
 *            - fix perdido   → lastKnownGps + epoch=0.
 *            - sin fix nunca → skip (no enviar coords (0,0)).
 *         f. buildArgusPacket() → CRC32 → frame TCP.
 *         g. submitReport() → encolar con prioridad según estado.
 *      3. Si PAUSADO (inactivo >5min en IDLE):
 *         a. Log de pausa (solo una vez).
 *         b. Si cola vacía y hay fix previo: enviar keepalive con lastKnownGps.
 *         c. a7670.isAlive() + recover() si modem caído.
 *      4. Actualizar lastSendTime = now.
 *   B. SYNC TICK (cada iteración):
 *      syncManager.tick(now) → drena hasta 5 eventos, TCP send, backoff.
 *      Detectar cambios en totalSent/totalFailed → EVENT_COMMS_SUCCESS/FAILURE.
 *   C. GUARDAR NVS (cada 5 minutos):
 *      eventQueue.saveToNVS() → persistir eventos CRITICAL pendientes.
 *   D. a7670.maintain() → mantener sesión TCP activa (watchdog AT).
 *   E. vTaskDelay(1000ms) → yield y esperar el próximo ciclo.
 *
 * NOTA SOBRE lastSendTime INICIAL:
 *   Se inicializa a (now - telemetryInterval) para que el primer ciclo de telemetría
 *   ocurra inmediatamente al arrancar, sin esperar N segundos.
 *
 * NOTA SOBRE METRICS Y EVENTS:
 *   prevTotalSent / prevTotalFailed son static dentro del bloque — persisten entre
 *   iteraciones del loop sin ocupar stack de la tarea.
 *
 * @param pvParameters No usado (nullptr). Requerido por la firma de tarea FreeRTOS.
 */
void commTask(void* pvParameters) {
    ESP_LOGI(TAG, "Comm task iniciada");

    initDeviceId();

    eventQueue.init();
    esp_err_t nvsRet = eventQueue.loadFromNVS();
    if (nvsRet == ESP_OK) {
        uint32_t restored = eventQueue.size();
        if (restored > 0) {
            // Eventos CRITICAL sobrevivieron un reboot — el sistema estaba en PURSUIT.
            ESP_LOGW(TAG, "%lu evento(s) crítico(s) restaurados desde NVS",
                     (unsigned long)restored);
        }
    }

    syncManager.init(g_deviceId);

    esp_err_t ret = a7670.init();
    if (ret != ESP_OK) {
        // No abortar la tarea: el modem puede recuperarse en ciclos posteriores.
        // a7670.maintain() y a7670.recover() manejan la recuperación automática.
        ESP_LOGE(TAG, "Error inicializando A7670 (%s) — continuando...",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "A7670 listo");
    }

    GpsData_t gpsData         = {};
    char      tcpPacket[320];  // Máximo frame Argus GPS: ~150 chars + margen
    char      tcpDrivePacket[320]; // Frame DRIVE: ~180 chars + margen

    // Inicializar timestamps de inactividad si sensorTask aún no los actualizó (arranque frío).
    // Sin esto, tiempoSinMovimiento = now - 0 = uptime completo → GPS sleep inmediato al boot.
    {
        const uint64_t bootNowUs = esp_timer_get_time();
        if (lastMovementTimestamp == 0)     lastMovementTimestamp     = bootNowUs;
        if (lastHardMovementTimestamp == 0) lastHardMovementTimestamp = bootNowUs;
    }

    // Restar el intervalo para enviar inmediatamente en el primer ciclo.
    uint64_t lastSendTime       = esp_timer_get_time() - getActiveTelemetryIntervalUs();
    uint64_t lastNvsSave        = esp_timer_get_time();
    uint64_t lastKeepaliveSent  = 0; // 0 → primer keepalive se envía al entrar en pausa
    bool     telemetriaPausada  = false;

    // Inicializar tracking de estado al valor ACTUAL antes de arrancar el loop.
    // Esto evita que el primer tick del loop interprete el estado de boot como
    // una "transición" y envíe un EVENT falso al backend.
    prevSysState    = currentSystemState;
    prevSystemArmed = systemArmed;

    // ── Sync de estado armado al conectar ─────────────────────────────────────
    // El firmware puede reiniciar (corte de batería, crash, actualización OTA) con
    // un estado distinto al que el backend tiene en MongoDB. Al encolar este EVENT
    // ahora, el backend recibe el estado real en cuanto TCP conecte y resuelve el
    // desfase optimista que queda cuando un DISARM llegó por BLE sin confirmación TCP.
    //
    // Es idempotente: si el backend ya tiene el mismo estado, el upsert no cambia nada.
    //
    // FREEMIUM pasivo: no usa TCP en modo normal — no encolar porque el event
    // llegaría al entrar en ALERT/PURSUIT con el estado posiblemente ya cambiado.
    if (planType != PLAN_FREEMIUM) {
        const uint64_t bootNow = esp_timer_get_time();
        submitEventFrame(systemArmed ? "ARM" : "DISARM", bootNow);
        ESP_LOGI(TAG, "Boot sync → %s (encolado para primera conexión TCP)",
                 systemArmed ? "ARMED" : "DISARMED");
    }

    // FREEMIUM: rastrea si el ciclo anterior fue pasivo para detectar transiciones.
    bool prevFreemiumPassive = false;

    while (true) {
        uint64_t now = esp_timer_get_time();

        // Estado del sistema para este ciclo (se lee una vez para consistencia).
        SystemState_t sysState = currentSystemState;

        // FREEMIUM en modo pasivo: IDLE o MOVING sin alerta activa.
        // En este modo NO se abre TCP — ARM/DISARM llega solo por BLE.
        // TCP solo se usa cuando hay robo confirmado (ALERT o PURSUIT).
        bool freemiumPassive = (planType == PLAN_FREEMIUM) &&
                                (sysState != STATE_ALERT) &&
                                (sysState != STATE_PURSUIT);

        if (freemiumPassive) {
            // ── FREEMIUM modo pasivo ──────────────────────────────────────────
            // Sin GPS, sin TCP. Solo mantener el modem registrado en red.

            if (!prevFreemiumPassive) {
                // Transición activo→pasivo: cerrar socket TCP si quedó abierto.
                a7670.disconnectTcp();
                ESP_LOGI(TAG, "[FREEMIUM] Modo pasivo — ARM/DISARM solo via BLE");
            }
            prevFreemiumPassive = true;

            // maintain() mantiene la red 4G activa sin abrir TCP.
            a7670.maintain();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // ── Modo activo (PREMIUM o FREEMIUM en ALERT/PURSUIT) ────────────────

        if (prevFreemiumPassive) {
            // FREEMIUM saliendo de pausa → verificar salud del modem antes de enviar.
            ESP_LOGI(TAG, "[FREEMIUM] Saliendo de modo pasivo — verificando modem...");
            if (!a7670.isAlive()) {
                ESP_LOGW(TAG, "[FREEMIUM] Modem sin respuesta → recuperando...");
                a7670.recover();
            }
        }
        prevFreemiumPassive = false;

        // ── Ciclo de telemetría ───────────────────────────────────────────────
        uint64_t telemetryInterval = getActiveTelemetryIntervalUs();

        if (now - lastSendTime >= telemetryInterval) {

            // Usar el clock de movimiento DURO para el gate de GPS sleep.
            // lastMovementTimestamp se resetea con cualquier SOFT event (incluyendo
            // vibración de tráfico vía ISR del MPU6050). En Bogotá, un camión cada
            // 2-3 min reseteaba el contador → 5 min nunca se alcanzaban.
            // lastHardMovementTimestamp solo se resetea con HARD/IMPACT (alguien
            // manipulando la moto de verdad). Tráfico no lo resetea.
            uint64_t tiempoSinMovimientoDuro = now - lastHardMovementTimestamp;

            // Pausa por inactividad: IDLE o MOVING-suave sin movimiento DURO >5min.
            // Se extiende a STATE_MOVING porque un SOFT event (vibración de tráfico)
            // causa IDLE→MOVING sin que nadie toque la moto. Si >5min sin HARD/IMPACT,
            // el MOVING es ruido ambiental y no merece GPS activo.
            // ALERT/PURSUIT siempre reportan (el HARD ya reseteó lastHardMovementTimestamp).
            bool inactivoPausable = (sysState == STATE_IDLE || sysState == STATE_MOVING) &&
                                    (tiempoSinMovimientoDuro > INACTIVITY_TIMEOUT_US);

            if (!inactivoPausable) {

                if (telemetriaPausada) {
                    // Transición de pausa→activo: log y verificar modem.
                    ESP_LOGI(TAG, "Actividad detectada → telemetría reanudada");
                    telemetriaPausada = false;
                    if (!a7670.isAlive()) {
                        // El modem puede haberse desconectado durante la pausa larga.
                        ESP_LOGW(TAG, "A7670 sin respuesta tras inactividad — recuperando...");
                        a7670.recover();
                    }
                }

                ESP_LOGI(TAG, "Consultando GPS (estado: %s)...",
                         StateMachine::stateName(sysState));
                memset(&gpsData, 0, sizeof(gpsData));
                a7670.getGpsPosition(gpsData);
                notifyGpsStatusChange(gpsData);
                // Overwrite: si bleTask no leyó el dato anterior, lo descartamos.
                // El dato más reciente es siempre el más útil.
                xQueueOverwrite(xGpsDataQueue, &gpsData);

                // Determinar coordenadas a reportar según disponibilidad de fix:
                double   reportLat, reportLon;
                uint64_t reportEpoch;
                float    reportSpeed = 0.0f;
                bool     hasCoords   = false;

                if (gpsData.fix_valid) {
                    // Fix actual válido: usar posición, timestamp y velocidad del GNSS.
                    lastKnownGps  = gpsData;
                    hasEverHadFix = true;
                    reportLat   = (double)gpsData.latitude;
                    reportLon   = (double)gpsData.longitude;
                    reportEpoch = resolveReportEpoch(gpsData);
                    reportSpeed = gpsData.speed_kmh;  // velocidad real del GNSS
                    hasCoords   = true;
                } else if (hasEverHadFix) {
                    // Fix perdido temporalmente: usar última posición conocida.
                    // epoch=0 indica al backend que use Date.now() como timestamp.
                    // speed=0 porque no hay fix activo — moto podría estar en cualquier vel.
                    reportLat   = (double)lastKnownGps.latitude;
                    reportLon   = (double)lastKnownGps.longitude;
                    reportEpoch = 0;
                    reportSpeed = 0.0f;
                    hasCoords   = true;
                    ESP_LOGW(TAG, "GPS sin fix; reportando última posición conocida");
                } else {
                    // Nunca hubo fix: no enviar (las coords (0,0) son ruido).
                    ESP_LOGW(TAG, "GPS sin fix y sin posición previa — omitiendo ciclo");
                }

                if (hasCoords) {
                    buildArgusPacket(tcpPacket, sizeof(tcpPacket),
                                     reportLat, reportLon, reportEpoch, reportSpeed);
                    xEventGroupSetBits(xSystemFlags, FLAG_COMMS_BUSY);
                    submitReport(tcpPacket, sysState, now);

                    // Frame DRIVE: métricas de conducción acumuladas desde el ciclo anterior.
                    // Solo se envía si hay al menos un evento registrado — evita frames
                    // vacíos cuando la moto estuvo quieta durante toda la ventana.
                    if (g_driveMetrics.hardCount > 0 || g_driveMetrics.softCount > 0 ||
                        g_driveMetrics.peakAccelDev > 0.01f) {
                        buildDriveFrame(tcpDrivePacket, sizeof(tcpDrivePacket),
                                        reportLat, reportLon, reportEpoch);
                        if (tcpDrivePacket[0] != '\0') {
                            // Prioridad NORMAL: las métricas de conducción no son críticas
                            // para la seguridad — se pueden retrasar o descartar en caso
                            // de congestión en la cola del SyncManager.
                            syncManager.submitGps(tcpDrivePacket, now);
                            ESP_LOGI(TAG, "[DRIVE] accel=%.3fg gyro=%.1f°/s hard=%u soft=%u",
                                     g_driveMetrics.peakAccelDev, g_driveMetrics.peakGyroMag,
                                     g_driveMetrics.hardCount, g_driveMetrics.softCount);
                        }
                    }

                    // Resetear métricas para la próxima ventana.
                    // Se hace DESPUÉS de buildDriveFrame() para no perder datos
                    // en caso de que buildDriveFrame() falle (buf[0]=='\0').
                    memset(&g_driveMetrics, 0, sizeof(g_driveMetrics));

                    xEventGroupClearBits(xSystemFlags, FLAG_COMMS_BUSY);
                }

            } else {
                // Sistema inactivo >5min en IDLE: pausar telemetría activa.
                if (!telemetriaPausada) {
                    ESP_LOGW(TAG, "Inactivo >5min en IDLE — telemetría pausada");
                    telemetriaPausada  = true;
                    // Forzar primer keepalive inmediato al entrar en pausa,
                    // para que el servidor sepa que el dispositivo está quieto.
                    lastKeepaliveSent  = now - KEEPALIVE_INTERVAL_US;
                }

                // Keepalive cada 5 minutos — mantiene el socket TCP abierto sin
                // gastar datos innecesarios. El servidor tiene timeout de 10 min,
                // así que un paquete cada 5 min es suficiente margen.
                // Los comandos (ARM/DISARM) llegan en ~1s vía URC del modem,
                // independientemente de este intervalo.
                if (eventQueue.isEmpty() && hasEverHadFix &&
                    (now - lastKeepaliveSent >= KEEPALIVE_INTERVAL_US)) {
                    buildArgusPacket(tcpPacket, sizeof(tcpPacket),
                                     (double)lastKnownGps.latitude,
                                     (double)lastKnownGps.longitude,
                                     0, 0.0f);
                    syncManager.submitHeartbeat(tcpPacket, now);
                    lastKeepaliveSent = now;
                    ESP_LOGI(TAG, "Keepalive enviado (próximo en 5 min)");
                }

                if (!a7670.isAlive()) {
                    // El modem puede haberse desconectado por timeout TCP del servidor.
                    a7670.recover();
                }
            }

            lastSendTime = now;
        }

        // ── SyncManager tick (cada iteración del loop = cada 1s) ─────────────
        // Drena hasta 5 eventos de la EventQueue, los envía via TCP, y gestiona
        // el backoff exponencial en caso de fallos de red.
        syncManager.tick(now);

        // Procesar comandos recibidos del backend via TCP (escritos en el socket tras cada envío).
        // pollForCommands() actúa solo si el A7670 emitió +CIPRXGET: 1,0 (datos disponibles),
        // de lo contrario es un no-op de costo ~0. Reduce latencia ARM/DISARM de ~30s a ~1s.
        a7670.pollForCommands();
        {
            char serverCmd[64];
            if (a7670.readLastServerCommand(serverCmd, sizeof(serverCmd))) {
                EventMessage_t msg = {};
                if (strncmp(serverCmd, "CMD|ARM", 7) == 0) {
                    msg.event = EVENT_ARM_CMD;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] ARM remoto recibido");
                } else if (strncmp(serverCmd, "CMD|DISARM", 10) == 0) {
                    msg.event = EVENT_DISARM_CMD;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] DISARM remoto recibido");
                } else if (strncmp(serverCmd, "CMD|SIREN_ON", 12) == 0) {
                    // Prende el buzzer directamente, SIN cambiar la máquina de estados.
                    // Uso: bocina de búsqueda para localizar la moto a distancia.
                    // No activa STATE_ALERT, no inicia timers, no afecta motorCut.
                    msg.event = EVENT_SIREN_ON;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] SIREN_ON → buzzer ON (sin cambio de estado)");
                } else if (strncmp(serverCmd, "CMD|SIREN_OFF", 13) == 0) {
                    // Apaga el buzzer directamente, SIN cambiar la máquina de estados.
                    msg.event = EVENT_SIREN_OFF;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] SIREN_OFF → buzzer OFF (sin cambio de estado)");
                } else if (strncmp(serverCmd, "CMD|PURSUIT_CONFIRM", 19) == 0) {
                    msg.event = EVENT_PURSUIT_CONFIRM;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] PURSUIT_CONFIRM remoto recibido");
                } else if (strncmp(serverCmd, "CMD|ENGINE_CUT", 14) == 0) {
                    // ENGINE_CUT es preventivo/silencioso: corta el relé sin cambiar estado
                    // ni activar sirena. s_motorManualCut persiste a través de transiciones.
                    // Para robo confirmado con sirena, usar CMD|PURSUIT_CONFIRM.
                    msg.event = EVENT_ENGINE_CUT_SILENT;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] ENGINE_CUT remoto → corte preventivo silencioso");
                } else if (strncmp(serverCmd, "CMD|ENGINE_RESTORE", 18) == 0) {
                    // ENGINE_RESTORE restaura el motor sin desarmar el sistema.
                    // STATE_PURSUIT + EVENT_ENGINE_RESTORE → STATE_IDLE (armed).
                    // Motor libre, sistema sigue protegiendo la moto.
                    msg.event = EVENT_ENGINE_RESTORE;
                    xQueueSend(xEventQueue, &msg, 0);
                    ESP_LOGI(TAG, "[CMD] ENGINE_RESTORE → motor liberado (sistema sigue armado)");
                } else if (strncmp(serverCmd, "CMD|SENSITIVITY_VERY_LOW", 24) == 0) {
                    setSensitivityLevel(SENSITIVITY_VERY_LOW);
                    ESP_LOGI(TAG, "[CMD] Sensibilidad → VERY_LOW");
                } else if (strncmp(serverCmd, "CMD|SENSITIVITY_LOW", 19) == 0) {
                    setSensitivityLevel(SENSITIVITY_LOW);
                    ESP_LOGI(TAG, "[CMD] Sensibilidad → LOW");
                } else if (strncmp(serverCmd, "CMD|SENSITIVITY_MEDIUM", 22) == 0) {
                    setSensitivityLevel(SENSITIVITY_MEDIUM);
                    ESP_LOGI(TAG, "[CMD] Sensibilidad → MEDIUM");
                } else if (strncmp(serverCmd, "CMD|SENSITIVITY_HIGH", 20) == 0) {
                    setSensitivityLevel(SENSITIVITY_HIGH);
                    ESP_LOGI(TAG, "[CMD] Sensibilidad → HIGH");
                } else if (strncmp(serverCmd, "CMD|SENSITIVITY_VERY_HIGH", 25) == 0) {
                    setSensitivityLevel(SENSITIVITY_VERY_HIGH);
                    ESP_LOGI(TAG, "[CMD] Sensibilidad → VERY_HIGH");
                } else if (strcmp(serverCmd, "ACK") == 0) {
                    // El backend confirma recepción del paquete GPS. No requiere acción.
                } else if (strcmp(serverCmd, "ERR") == 0) {
                    ESP_LOGW(TAG, "[CMD] Backend rechazó paquete (coordenadas inválidas o CRC)");
                } else {
                    ESP_LOGW(TAG, "[CMD] Comando desconocido: '%.32s'", serverCmd);
                }
            }
        }

        // Detectar transiciones de éxito/fallo para notificar a control_task.
        // Solo notificamos cuando hay un CAMBIO en los contadores, no en cada tick.
        {
            SyncMetrics_t sm;
            syncManager.getMetrics(sm);

            // static dentro del bloque: persisten entre iteraciones sin ser globales.
            static uint32_t prevTotalSent   = 0;
            static uint32_t prevTotalFailed = 0;

            if (sm.totalSent > prevTotalSent) {
                EventMessage_t msg = {};
                msg.event = EVENT_COMMS_SUCCESS;
                xQueueSend(xEventQueue, &msg, 0);
                prevTotalSent = sm.totalSent;
            }

            // Solo notificar fallo si hay 2+ fallos nuevos (evitar spam en backoff).
            if (sm.totalFailed > prevTotalFailed + 2) {
                EventMessage_t msg = {};
                msg.event = EVENT_COMMS_FAILURE;
                xQueueSend(xEventQueue, &msg, 0);
                prevTotalFailed = sm.totalFailed;
            }
        }

        // ── Detectar transiciones de estado/armed → enviar EVENT al backend ─────
        //
        // LÓGICA DE POLLING:
        //   controlTask escribe currentSystemState y systemArmed via xEventQueue.
        //   comm_task.cpp los lee aquí, una vez por segundo, y detecta cambios
        //   comparando con los valores del tick anterior.
        //   El retraso máximo es 1 segundo — aceptable: las alertas GPS llegan cada
        //   10-15s en emergencias, un EVENT con 1s de retraso no impacta la UX.
        //
        // POR QUÉ NO USAR xEventQueue PARA ESTO:
        //   xEventQueue ya está ocupada con los eventos del firmware (MOVEMENT,
        //   GPS_FIX, etc.) y la lee controlTask. Agregar un reader secundario
        //   requeriría convertirla en un broadcast o agregar otra cola.
        //   El polling de variables atómicas es más simple y suficiente aquí.
        {
            const SystemState_t curState = currentSystemState;
            const bool          curArmed = systemArmed;

            if (curState != prevSysState) {
                // Transición de estado detectada.
                const char* eventType = nullptr;
                switch (curState) {
                    case STATE_ALERT:
                        // Alerta activa: movimiento confirmado sospechoso.
                        // Este es el evento más importante para el backend.
                        eventType = "STATE_ALERT";
                        break;
                    case STATE_PURSUIT:
                        // Persecución confirmada: robo activo.
                        // Máxima urgencia — el operador debe saberlo inmediatamente.
                        eventType = "STATE_PURSUIT";
                        break;
                    case STATE_IDLE:
                        // Retorno a IDLE: solo reportar si veníamos de ALERT o PURSUIT.
                        // Las transiciones MOVING→IDLE son frecuentes y no son eventos
                        // de seguridad significativos para el backend.
                        if (prevSysState == STATE_ALERT || prevSysState == STATE_PURSUIT) {
                            eventType = "STATE_IDLE";
                        }
                        break;
                    case STATE_MOVING:
                        // STATE_MOVING es transitorio y potencialmente ruidoso.
                        // El backend no necesita saber cada vez que el sensor detecta
                        // movimiento — solo cuando se confirma como ALERT o PURSUIT.
                        break;
                    default:
                        break;
                }

                if (eventType != nullptr) {
                    submitEventFrame(eventType, now);
                }
                prevSysState = curState;
            }

            // ARM y DISARM no cambian el estado de la máquina (siguen en STATE_IDLE),
            // solo cambian systemArmed. Se detectan por separado aquí.
            // FREEMIUM: ARM/DISARM se gestiona solo via BLE — no reportar al backend.
            if (curArmed != prevSystemArmed) {
                if (planType != PLAN_FREEMIUM) {
                    submitEventFrame(curArmed ? "ARM" : "DISARM", now);
                }
                prevSystemArmed = curArmed;
            }
        }

        // ── Guardar NVS periódicamente ────────────────────────────────────────
        // Persiste eventos CRITICAL pendientes para que sobrevivan a un reboot.
        // Solo eventos CRITICAL se guardan (ver eventQueue.saveToNVS()).
        if (now - lastNvsSave >= NVS_SAVE_INTERVAL_US) {
            eventQueue.saveToNVS();
            lastNvsSave = now;
        }

        // maintain() envía un AT check mínimo para mantener la sesión TCP activa.
        // Si el servidor TCP cierra la conexión por timeout, maintain() la reabre.
        a7670.maintain();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/comm_task.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   comm_task.cpp hace "hablar" al ESP32 con el backend. Cada segundo,
   el módulo decide si es hora de reportar la posición GPS y, si lo es,
   obtiene las coordenadas del modem 4G, las firma con CRC32 y las encola
   para enviarlas al servidor TCP. El SyncManager se encarga de reintentar
   si la conexión falla, con backoff exponencial.

   La frecuencia de reporte es adaptativa: en emergencia (persecución) reporta
   cada 10 segundos. En alerta cada 15s. En modo normal, cada 30s para PREMIUM
   o cada hora para FREEMIUM. Si el dispositivo está quieto más de 5 minutos,
   pausamos la telemetría GPS activa para ahorrar batería y datos.

   PSEUDOCÓDIGO DEL LOOP PRINCIPAL:
   ```
   al arrancar (antes del loop):
     prevSystemArmed = systemArmed
     if PREMIUM: encolar EVENT|ARM o EVENT|DISARM  ← boot sync

   loop (cada 1s):
     if tiempo_transcurrido >= intervalo_activo:
       if inactivo_>5min_en_IDLE:
         → keepalive mínimo + watchdog modem
       else:
         → getGpsPosition()
         → buildArgusPacket() [con CRC32]
         → submitReport() [con prioridad según estado]
     syncManager.tick()        → drena cola, TCP send, backoff
     if cambio en métricas:   → EVENT_COMMS_SUCCESS/FAILURE
     if >5min desde NVS save: → saveToNVS()
     a7670.maintain()          → watchdog TCP
     delay 1s
   ```

   PROTOCOLO ARGUS:
   "ARGUS|ARGUS-1A2B3C4D|1715027200000|19.432608|-99.133209|3F8A1B2C\n"
    └── prefijo
                └── device_id (MAC derivada)
                                └── epoch_ms (Unix)
                                                 └── lat
                                                              └── lon
                                                                        └── crc32 (hex)

   DEUDA TÉCNICA:
   1. ARGUS_SIGNATURE_SECRET hardcodeado — vulnerabilidad de seguridad en producción.
   2. lastMovementTimestamp: lectura de uint64_t de sensorTask sin mutex.
      En ARM Cortex-M4 con LDRD, la lectura de 64 bits NO es atómica.
   3. currentSystemState: leído sin mutex desde controlTask. Aceptable en la práctica
      (ARM 32-bit read atómico para enum) pero no formalmente seguro.
   4. No hay lógica de rechazo de frames si buildArgusPacket() retorna '\0'.
      tcpPacket[0]=='\0' se encolaría como payload vacío.

   ═══════════════════════════════════════════════════════════ */
