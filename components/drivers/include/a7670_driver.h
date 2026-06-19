#pragma once

/**
 * @file a7670_driver.h
 * @brief Driver del modem 4G SIM7670/A7670 para GPS y comunicación TCP del sistema Argus.
 *
 * PROPÓSITO:
 *   Encapsular toda la comunicación con el modem A7670 via UART2. Provee:
 *   1. Inicialización y recuperación del modem (boot AT, GNSS, red 4G, APN).
 *   2. Obtención de posición GPS via comandos AT+CGNSINF.
 *   3. Envío de paquetes TCP al backend via AT+CIPOPEN/AT+CIPSEND/AT+CIPCLOSE.
 *   4. Infraestructura AT asíncrona: UART reader task + URC handler task.
 *   5. Watchdog de conectividad y maintain() para keepalive de la sesión TCP.
 *
 * HARDWARE:
 *   Modem:    SIM7670G / A7670E (compatible con comandos AT SIMCom)
 *   UART:     UART2 del ESP32 (puerto UART_NUM_2)
 *   GPIO TX:  GPIO17 → UART2 TX → Modem RX
 *   GPIO RX:  GPIO16 ← UART2 RX ← Modem TX
 *   GPIO PWR: GPIO4  → señal de power-on/reset del modem (pulso activo en HIGH)
 *   Baud:     115200 8N1
 *   Buffers:  RX=4096 bytes, TX=1024 bytes
 *
 * BACKEND TCP:
 *   Host:  34.69.219.193 (GCP, Google Cloud Platform)
 *   Port:  9000 (servidor TCP Argus Backend en Node.js)
 *   Protocolo: TCP directo (no HTTP/MQTT). Frame Argus con '\n' como delimitador.
 *
 * ARQUITECTURA INTERNA — DOS TAREAS ASÍNCRONAS:
 *
 *   uartReaderTask (IRAM, prio alta):
 *     Reads bytes from UART FIFO byte-by-byte via processIncomingByte()
 *     → acumula en parserBuf[] hasta '\n' → llama emitParserLine()
 *     → emitParserLine() llama handleParsedLine() para clasificar y despachar
 *
 *   urcHandlerTask (prio normal):
 *     Consume de urcQueue (URCs: UnsolicIted Result Codes del modem)
 *     → handleUrcLine() → actualiza urcState (GNSS ready, señal, registro)
 *
 *   sendAT() (caller externo, commTask context):
 *     1. beginDispatch() → configura DispatchSession para que uartReaderTask
 *        sepa qué respuesta esperar y dónde enrutarla.
 *     2. UART TX: escribe "AT+CMD\r\n".
 *     3. waitForExpectedLineLocked() → lee de atResponseQueue con timeout.
 *     4. endDispatch() → limpia DispatchSession.
 *
 *   El spinlock dispatchMux protege el acceso a DispatchSession_t entre el
 *   hilo del caller y uartReaderTask.
 *
 * FLUJO DE INICIALIZACIÓN (init()):
 *   initHardware() → UART + GPIO setup
 *   startAsyncInfrastructure() → crear colas FreeRTOS + lanzar tasks
 *   bootModem() → power pulse GPIO4 → esperar "AT" response
 *   configureAT() → AT+CMEE=2, AT+CEREG=1, AT+CGREG=1, etc.
 *   enableGNSS() → AT+CGNSPWR=1, AT+CGNSINF para verificar
 *   initDataContext() → AT+CGDCONT=1,"IP","internet"
 *   ensureNetOpen() → AT+NETOPEN o verificar si ya está abierta
 *
 * FLUJO DE tcpSend():
 *   ensureNetOpen() → AT+CIPOPEN=0,"TCP","34.69.219.193",9000 → AT+CIPSEND=0,len
 *   → [prompt ">" ] → escribir datos → AT+CIPCLOSE=0
 *
 * FLUJO DE getGpsPosition():
 *   AT+CGNSINF → respuesta "+CGNSINF: 1,1,timestamp,lat,lon,alt,..."
 *   parseGNSINF() → GpsData_t con lat/lon/timestamp/satellites/fix_valid
 *
 * APN:
 *   "internet" (APN genérico para la mayoría de operadores en México/LATAM).
 *   ARQUITECTURA: El APN esta hardcodeado. Para despliegue internacional
 *      o con SIMs de otras operadoras (ej. Telcel, AT&T MX, Claro) debe ser
 *      configurable desde NVS o provisioning.
 *
 * VARIABLES CRÍTICAS:
 *   - state (ModemState): estado actual del modem (UART→AT→NET→TCP→ERROR).
 *   - netStackOpen (bool): true si AT+NETOPEN completó exitosamente.
 *   - urcState (UrcState_t): estado GNSS y red actualizado por URCs del modem.
 *   - dispatchSession (DispatchSession_t): sesión AT activa (protegida por dispatchMux).
 *   - atResponseQueue (QueueHandle_t cap=32): líneas de respuesta AT para sendAT().
 *   - urcQueue (QueueHandle_t cap=16): URCs del modem para urcHandlerTask.
 *   - waitUrcSem: semáforo para waitForUrcPrefix() (espera sincronizada de URCs).
 *   - parserBuf[1024]: buffer del parser de líneas UART (acumulación byte a byte).
 *
 * CONCURRENCIA:
 *   - uartReaderTask escribe en atResponseQueue y urcQueue. Usa dispatchMux (spinlock)
 *     para acceder a DispatchSession_t sin ser interrumpido por sendATLocked().
 *   - urcHandlerTask consume de urcQueue. No accede a dispatchSession.
 *   - sendATLocked() corre en contexto de commTask. No puede bloquearse por más de
 *     A7670_TIMEOUT_LONG_MS (120s) si el modem no responde.
 *   - xModemMutex (global en main.cpp): mutex FreeRTOS recursivo. TODOS los callers
 *     externos (commTask, bleTask) deben tomarlo antes de llamar sendAT() o tcpSend().
 *     A7670Driver NO toma xModemMutex internamente — es responsabilidad del caller.
 *     ARQUITECTURA: Esto viola el principio de encapsulacion. El mutex deberia
 *        ser interno al driver.
 *
 * TIMEOUTS:
 *   A7670_TIMEOUT_SHORT_MS  =   3s  (comandos AT simples: AT, ATI)
 *   A7670_TIMEOUT_MEDIUM_MS =  15s  (comandos de red: AT+NETOPEN, AT+CIPOPEN)
 *   A7670_TIMEOUT_LONG_MS   = 120s  (boot del modem, datos GNSS tras fix frío)
 *
 * @module components/drivers/a7670_driver
 */

#include "system_events.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ─── UART ─────────────────────────────────────────────────────────────────────

/** Puerto UART del ESP32 conectado al modem A7670. UART2 deja libre UART0 (debug) y UART1 (libre). */
#define A7670_UART_PORT         UART_NUM_2

/** Velocidad de comunicación con el modem. A7670 default y máximo estable sin errores. */
#define A7670_UART_BAUD_RATE    115200

/**
 * Buffer RX del driver UART ESP-IDF. 4096 bytes para absorber respuestas largas de GNSS
 * (AT+CGNSINF retorna ~100 chars) y múltiples URCs llegando en ráfaga.
 */
#define A7670_UART_RX_BUF       4096

/**
 * Buffer TX del driver UART. 1024 bytes es suficiente para los comandos AT más largos
 * (AT+CIPSEND + frame TCP de ~300 chars).
 */
#define A7670_UART_TX_BUF       1024

// ─── Timeouts ─────────────────────────────────────────────────────────────────

/** Timeout para comandos AT simples que deben responder rápido (AT, ATI, AT+GMR). */
#define A7670_TIMEOUT_SHORT_MS   3000

/** Timeout para operaciones de red (AT+NETOPEN, AT+CIPOPEN) que requieren establecer conexión 4G. */
#define A7670_TIMEOUT_MEDIUM_MS  15000

/**
 * Timeout para operaciones lentas: boot del modem (~45s desde power-on),
 * primer fix GPS (cold start puede tardar >60s sin AGPS).
 */
#define A7670_TIMEOUT_LONG_MS    120000

// ─── Red / APN ────────────────────────────────────────────────────────────────

/**
 * APN (Access Point Name) para la conexión de datos 4G.
 * Claro Colombia: "internet.tigo.co.com"
 * ARQUITECTURA: Hardcodeado. Deberia ser configurable desde NVS.
 */
#define ARGUS_APN               "internet.tigo.co.com"

// ─── Servidor TCP backend ─────────────────────────────────────────────────────

/** IP pública del servidor backend Argus en Google Cloud Platform (GCP). */
#define TCP_SERVER_HOST    "34.69.219.193"

/** Puerto TCP del servidor backend. El servidor Node.js escucha en este puerto. */
#define TCP_SERVER_PORT    9000

// ─── Estado del módem ─────────────────────────────────────────────────────────

/**
 * @brief Estados de inicialización del modem A7670.
 *
 * El modem progresa linealmente a través de estos estados durante init().
 * Si hay un error, el estado queda en MODEM_STATE_ERROR hasta que se llama recover().
 *
 * UART_READY:      UART2 inicializado, pero el modem puede no estar respondiendo AT.
 * AT_READY:        El modem responde "OK" al comando "AT". Booteo exitoso.
 * NET_READY:       Red 4G registrada y contexto PDP activo. AT+NETOPEN exitoso.
 * MQTT_CONNECTED:  (nombre histórico) En la implementación actual: TCP stack abierto,
 *                  listo para AT+CIPOPEN. El nombre MQTT es un remanente de una
 *                  versión anterior que usaba MQTT.
 * ERROR:           Estado de fallo. recover() intenta volver a AT_READY.
 */
typedef enum {
    MODEM_STATE_UART_READY = 0,  // UART listo, modem sin verificar
    MODEM_STATE_AT_READY,        // AT "OK" recibido, modem funcional
    MODEM_STATE_NET_READY,       // Red 4G activa, AT+NETOPEN completado
    MODEM_STATE_MQTT_CONNECTED,  // TCP stack listo para AT+CIPOPEN (nombre histórico)
    MODEM_STATE_ERROR            // Error — recover() requerido
} ModemState;

/**
 * @brief Driver de alto nivel para el modem A7670 (GPS + 4G TCP).
 *
 * PROPÓSITO:
 *   Abstraer toda la comunicación AT con el modem A7670 en métodos de alto nivel.
 *   commTask solo llama init(), getGpsPosition(), tcpSend(), isAlive(), maintain().
 *   La complejidad de los comandos AT, la infraestructura asíncrona UART, el parsing
 *   de respuestas, y el manejo de URCs es completamente invisible para el caller.
 *
 * ESTADO INTERNO:
 *   El driver tiene estado persistente entre llamadas:
 *   - state: nivel de inicialización alcanzado.
 *   - netStackOpen: si AT+NETOPEN está activo.
 *   - urcState: estado GNSS y red reportado por URCs del modem.
 *   Este estado NO se persiste en NVS — se pierde en cada reboot. recover() lo reinicia.
 *
 * THREAD SAFETY:
 *   NO thread-safe. El caller (commTask) debe tomar xModemMutex antes de llamar
 *   cualquier método. Las tareas internas (uartReaderTask, urcHandlerTask) operan
 *   de forma asíncrona y solo comunican via colas FreeRTOS.
 */
class A7670Driver {
public:
    /**
     * @brief Constructor. Inicializa todas las variables de estado a valores seguros.
     *
     * NOTA: No realiza ninguna operación de hardware. init() hace la inicialización real.
     */
    A7670Driver();

    /**
     * @brief Retorna el estado actual del modem.
     * Sin mutex — solo el caller (commTask) lo lee y la tarea interna no lo escribe.
     */
    ModemState getState()        const { return state; }

    /**
     * @brief Retorna true si AT+NETOPEN completó exitosamente.
     * Se usa en commTask para verificar si el modem puede hacer AT+CIPOPEN.
     */
    bool       isNetStackOpen() const { return netStackOpen; }

    /**
     * @brief Inicializa el modem completamente desde cero.
     *
     * PROPÓSITO:
     *   Realizar la secuencia completa de inicialización del modem: hardware,
     *   infraestructura asíncrona, boot AT, configuración, GNSS, y red 4G.
     *   Se llama una sola vez al arrancar commTask.
     *
     * FLUJO (pasos en orden):
     *   1. initHardware()              → UART2 + GPIO4 setup.
     *   2. startAsyncInfrastructure()  → colas FreeRTOS + uartReaderTask + urcHandlerTask.
     *   3. bootModem()                 → power pulse GPIO4, esperar "AT" response (hasta 60s).
     *   4. configureAT()               → AT+CMEE=2, AT+CEREG=1, AT+CGREG=1, AT+CGSMS=1.
     *   5. enableGNSS()                → AT+CGNSPWR=1, AT+CGNSINF verificación.
     *   6. initDataContext()           → AT+CGDCONT=1,"IP","internet".
     *   7. ensureNetOpen()             → AT+NETOPEN o verificar estado.
     *
     * MANEJO DE ERRORES:
     *   Si cualquier paso falla, retorna el código de error y la inicialización
     *   queda parcial. commTask loguea el error y continúa — el modem intentará
     *   recuperarse en ciclos posteriores con maintain() y recover().
     *
     * @return ESP_OK si el modem quedó completamente inicializado y listo para tcpSend().
     *         Código de error ESP si algún paso falló.
     */
    esp_err_t init();

    /**
     * @brief Intenta recuperar el modem después de un error o estado zombie.
     *
     * PROPÓSITO:
     *   Reinicializar el modem cuando está en un estado inoperable: TCP timeout,
     *   pérdida de señal prolongada, watchdog de SyncManager, etc.
     *
     * FLUJO:
     *   1. Cerrar la conexión TCP activa si existe (AT+CIPCLOSE=0).
     *   2. Cerrar el stack de red (AT+NETCLOSE).
     *   3. Resetear netStackOpen=false, state=UART_READY.
     *   4. hardReset() → pulso de poder en GPIO4 para reiniciar el hardware del modem.
     *   5. Re-ejecutar bootModem() + configureAT() + enableGNSS() + ensureNetOpen().
     *
     * CUÁNDO SE LLAMA:
     *   - SyncManager::checkWatchdog() → 15 minutos sin envío exitoso.
     *   - commTask → a7670.isAlive() retorna false (sin respuesta AT).
     *   - commTask → tras período de inactividad larga.
     *
     * @return ESP_OK si el modem se recuperó, error si el recovery también falló.
     */
    esp_err_t recover();

    /**
     * @brief Activa el módulo GNSS del modem y espera el primer fix.
     *
     * PROPÓSITO:
     *   Encender el receptor GNSS interno del A7670 con AT+CGNSPWR=1 y configurarlo
     *   para modo autónomo (sin AGPS externo). Puede tardar hasta 60s en cold start.
     *
     * FLUJO:
     *   AT+CGNSPWR=1 → activar hardware GNSS.
     *   AT+CGNSINF → verificar que el módulo respondió (fix_valid puede ser 0 aún).
     *
     * NOTA: enableGNSS() no espera el fix GPS. getGpsPosition() se llama después
     *   y puede retornar fix_valid=false hasta que el fix se establece.
     *
     * @return ESP_OK si AT+CGNSPWR=1 fue aceptado. Error si el comando fue rechazado.
     */
    esp_err_t enableGNSS();

    /**
     * @brief Obtiene la posición GPS actual del modem.
     *
     * PROPÓSITO:
     *   Enviar AT+CGNSINF al modem, recibir la respuesta, y parsearla en GpsData_t.
     *   Retorna fix_valid=false si el GNSS no tiene fix todavía.
     *
     * FLUJO:
     *   sendAT("AT+CGNSINF", "+CGNSINF:") → capturar respuesta en rxBuf.
     *   parseGNSINF(rxBuf, gps) → extraer lat/lon/timestamp/satellites/fix_valid.
     *
     * FORMATO AT+CGNSINF (SIMCom):
     *   "+CGNSINF: <mode>,<fix>,<timestamp>,<lat>,<lon>,<alt>,<speed>,<course>,<fix_mode>,
     *    <reserved>,<hdop>,<pdop>,<vdop>,<reserved>,<sats_inuse>,<reserved>,<hpa>,<vpa>"
     *   fix=1 indica fix válido. lat/lon en grados decimales (positivo=N/E).
     *
     * @param gps Estructura de salida. Se llena con la posición actual o queda parcial
     *            con fix_valid=false si no hay fix.
     * @return ESP_OK siempre (incluso sin fix — fix_valid indica si hay datos válidos).
     *         Error si el comando AT falló completamente.
     */
    esp_err_t getGpsPosition(GpsData_t& gps);

    /**
     * @brief Envía un paquete TCP al servidor backend.
     *
     * PROPÓSITO:
     *   Abrir una conexión TCP al backend (34.69.219.193:9000), enviar el frame
     *   del protocolo Argus, y cerrar la conexión. Es la operación principal del
     *   pipeline de telemetría.
     *
     * FLUJO (por conexión):
     *   1. ensureNetOpen() → verificar/abrir AT+NETOPEN.
     *   2. AT+CIPOPEN=0,"TCP","34.69.219.193",9000 → establecer conexión TCP.
     *   3. AT+CIPSEND=0,len → notificar que se enviarán len bytes.
     *   4. Esperar prompt ">" del modem.
     *   5. UART TX: escribir el frame TCP completo.
     *   6. Esperar "SEND OK" de confirmación.
     *   7. AT+CIPCLOSE=0 → cerrar la conexión TCP.
     *
     * NOTA SOBRE CONEXIÓN POR FRAME:
     *   Cada tcpSend() abre y cierra una conexión TCP individual.
     *   ARQUITECTURA: Abrir/cerrar TCP por cada frame es ineficiente.
     *      Una conexion persistente (keep-alive) reduciria la latencia de ~500ms
     *      (handshake TCP + AT overhead) a ~50ms. Requiere cambios en el servidor
     *      y manejo de reconexion en el driver.
     *
     * @param packet Frame TCP del protocolo Argus (terminado en '\n').
     * @return ESP_OK si el frame fue enviado y confirmado. Error si falló.
     */
    esp_err_t tcpSend(const char* packet);

    /**
     * @brief Verifica si el modem está respondiendo AT (liveness check).
     *
     * PROPÓSITO:
     *   Diagnosticar si el modem está en estado zombie: aparentemente iniciado
     *   pero sin responder a comandos AT. commTask llama isAlive() antes de
     *   reanudar la telemetría tras un período de inactividad.
     *
     * FLUJO:
     *   sendAT("AT", "OK", timeout=3s) → true si recibió "OK".
     *
     * @return true si el modem respondió "OK" al comando "AT" en 3 segundos.
     */
    bool isAlive();

    /**
     * @brief Tareas de mantenimiento periódico del modem. Llamar cada 1s desde commTask.
     *
     * PROPÓSITO:
     *   Mantener el modem en estado operativo entre ciclos de telemetría.
     *   Verifica que la red esté registrada y que el stack TCP esté abierto.
     *   Si detecta pérdida de red, intenta reconectar sin recover() completo.
     *
     * CUÁNDO SE LLAMA:
     *   Al final del loop de commTask, después de syncManager.tick() y vTaskDelay(1s).
     *   Funciona como "watchdog liviano" antes de que SyncManager active el recover() completo.
     */
    void maintain();

    /**
     * @brief Envía un comando AT y espera la respuesta esperada.
     *
     * PROPÓSITO:
     *   Interfaz pública para envío de comandos AT desde commTask u otras tareas.
     *   Internally delegates to sendATLocked() con la infraestructura de dispatch.
     *
     * FLUJO:
     *   1. Escribir "command\r\n" en UART TX.
     *   2. waitForExpectedLineLocked(expectedResp, timeoutMs) → leer de atResponseQueue.
     *   3. Retornar true si se recibió la respuesta esperada antes del timeout.
     *
     * NOTA: El caller debe tener xModemMutex tomado antes de llamar sendAT().
     *   Sin el mutex, dos callers concurrentes podrían intercalar sus respuestas AT.
     *
     * @param command      Comando AT sin "\r\n" (ej. "AT", "AT+CGNSINF").
     * @param expectedResp Substring que debe estar en la respuesta (ej. "OK", "+CGNSINF:").
     * @param timeoutMs    Timeout en ms. Default = A7670_TIMEOUT_SHORT_MS = 3000ms.
     * @param silent       Si true, no loguear el comando ni la respuesta (para AT verbosos).
     * @return true si se recibió la respuesta esperada, false si hubo timeout o error.
     */
    bool sendAT(const char* command, const char* expectedResp,
                uint32_t timeoutMs = A7670_TIMEOUT_SHORT_MS, bool silent = false);

    /** Lee y consume el último comando recibido del servidor via TCP. */
    bool readLastServerCommand(char* buf, size_t size);

    /**
     * @brief Sondea el socket TCP por comandos pendientes del servidor.
     *
     * PROPÓSITO:
     *   Permite que commTask detecte comandos ARM/DISARM enviados por el backend
     *   sin tener que esperar al próximo ciclo GPS (hasta 30s en PREMIUM IDLE).
     *   Se llama cada segundo desde el loop principal de commTask.
     *
     * LÓGICA:
     *   1. Lee _hasPendingRx bajo spinlock. Si false → retorna false (costo ~0).
     *   2. Si true → toma xModemMutex (timeout 200ms) y llama tryReadServerCommand().
     *   3. Si el mutex no está disponible (tcpSend activo), restaura el flag y retorna false.
     *      tryReadServerCommand() será llamado igualmente al final de tcpSend().
     *
     * NOTA: Es no-op si no hay notificación URC pendiente; no genera tráfico celular.
     *
     * @return true si se ejecutó tryReadServerCommand(); false si no había datos o mutex ocupado.
     */
    bool pollForCommands();

    /**
     * @brief Cierra el socket TCP activo explícitamente (AT+CIPCLOSE=0).
     *
     * PROPÓSITO:
     *   Usado por el plan FREEMIUM para cerrar la conexión TCP al salir de
     *   STATE_ALERT/PURSUIT y volver a modo pasivo. Ahorra batería y datos SIM
     *   al no mantener el socket abierto cuando no se necesita.
     *   No hace nada si el socket ya estaba cerrado (_tcpConnected=false).
     */
    void disconnectTcp();

private:
    // ── Constantes del parser UART ────────────────────────────────────────────

    /** Longitud máxima de una línea AT del modem. 1024 bytes para respuestas GNSS largas. */
    static constexpr size_t   MODEM_LINE_MAX        = 1024;

    /** Longitud máxima de la cadena de respuesta esperada en sendAT(). */
    static constexpr size_t   EXPECTED_MAX           = 96;

    /** Longitud máxima del prefijo derivado para matching (ej. "+CGNSINF" de "AT+CGNSINF"). */
    static constexpr size_t   PREFIX_MAX             = 48;

    /** Capacidad de la cola de respuestas AT. 32 líneas para absorber respuestas en ráfaga. */
    static constexpr size_t   AT_QUEUE_DEPTH         = 32;

    /** Capacidad de la cola de URCs. 16 entradas para URCs no procesados. */
    static constexpr size_t   URC_QUEUE_DEPTH        = 16;

    /** Crashes consecutivos de GNSS activo antes de deshabilitar GNSS para la sesión. */
    static constexpr uint8_t  GNSS_MAX_CONSEC_CRASHES = 3;

    /**
     * Tiempo de espera adicional para capturar respuestas que llegan después del "OK".
     * Algunos comandos AT emiten datos adicionales después del OK (ej. AT+CGNSINF).
     */
    static constexpr uint32_t MODEM_LATE_CAPTURE_MS  = 5000;

    // ── Tipos internos ────────────────────────────────────────────────────────

    /**
     * @brief Línea de texto del modem para las colas internas.
     *
     * Se usa como elemento de atResponseQueue y urcQueue.
     * El campo text[] es un buffer fijo para evitar malloc en el parser.
     */
    typedef struct {
        char text[MODEM_LINE_MAX];  // Línea AT parseada (terminada en '\0')
    } ModemLine_t;

    /**
     * @brief Estado de la sesión de dispatch AT activa.
     *
     * Configura cómo uartReaderTask debe enrutar las líneas de respuesta del modem
     * durante una operación sendAT() activa.
     *
     * active:               Si true, hay un sendAT() en progreso. Las líneas que
     *                       coincidan con `expected` van a atResponseQueue.
     * captureOnlyExpected:  Si true, solo capturar líneas que contengan `expected`.
     *                       Si false, capturar todas las líneas (modo raw).
     * expectPrompt:         Si true, el comando espera el prompt ">" antes de enviar datos.
     *                       Usado en AT+CIPSEND que usa modo prompt para datos TCP.
     * waitForOk:            Si true, esperar "OK" o "ERROR" después de la línea esperada.
     * expected[96]:         Substring que debe contener la respuesta (ej. "+CGNSINF:").
     * derivedPrefix[48]:    Prefijo derivado del comando AT (ej. "+CGNSINF" de "AT+CGNSINF").
     *                       Construido por buildDerivedPrefix().
     */
    typedef struct {
        bool active;
        bool captureOnlyExpected;
        bool expectPrompt;
        bool waitForOk;
        char expected[EXPECTED_MAX];
        char derivedPrefix[PREFIX_MAX];
    } DispatchSession_t;

    /**
     * @brief Estado de red y GNSS reportado por URCs del modem.
     *
     * Los URCs (Unsolicited Result Codes) son mensajes que el modem envía
     * espontáneamente sin que se les haya pedido. Por ejemplo:
     *   "+CGREG: 1" → registrado en red
     *   "+GNSS: READY" → GNSS listo para fix
     *
     * urcHandlerTask procesa estos mensajes y actualiza urcState para que
     * el resto del driver conozca el estado actual de la red y el GNSS.
     */
    typedef struct {
        bool gnssReady;        // GNSS hardware listo (no necesariamente con fix)
        bool hasSignal;        // Señal 4G detectada (rssi > 0)
        bool hasRegistration;  // Registrado en la red del operador
        int  rssi;             // RSSI de señal (0-31, 99=desconocido)
        int  ber;              // Bit Error Rate (0-7, 99=desconocido)
        int  cgRegMode;        // Modo de registro CGREG (0=disable, 1=enable, 2=+loc)
        int  cgRegStat;        // Estado de registro (0=no reg, 1=home, 5=roaming)
    } UrcState_t;

    // ── Variables de estado ───────────────────────────────────────────────────

    /** Estado de inicialización del modem. Ver ModemState enum. */
    ModemState     state;

    /** true si UART2 fue instalado con uart_driver_install(). */
    bool           uartInstalled;

    /** true si GPIO4 fue configurado como output para el control de power del modem. */
    bool           gpioInitialized;

    /** true si uartReaderTask y urcHandlerTask están corriendo. */
    bool           asyncStarted;

    /** true si AT+CGNSPWR=1 fue ejecutado exitosamente. */
    bool           gnssEnabled;

    /**
     * Contador de crashes del módulo (*ATREADY) con GNSS activo al momento del crash.
     * Se incrementa en handleUrcLine(); se resetea a 0 en getGpsPosition() exitoso.
     * Protegido por dispatchMux.
     */
    uint8_t        gnssConsecCrashCount;

    /**
     * true cuando gnssConsecCrashCount alcanzó GNSS_MAX_CONSEC_CRASHES.
     * enableGNSS() retorna ESP_FAIL inmediatamente sin mandar AT+CGNSSPWR=1,
     * permitiendo que el TCP siga operando con lastKnownGps en commTask.
     * Se mantiene hasta que el ESP32 se reinicie. Protegido por dispatchMux.
     */
    bool           gnssDisabledForSession;

    /** true si AT+NETOPEN completó y el stack TCP del modem está activo. */
    bool           netStackOpen;

    /** Buffer compartido entre sendAT() y su caller para capturar respuestas AT. */
    char           rxBuf[A7670_UART_RX_BUF];

    // ── Colas FreeRTOS ────────────────────────────────────────────────────────

    /**
     * Cola de respuestas AT. uartReaderTask produce, sendATLocked() consume.
     * Capacidad: AT_QUEUE_DEPTH=32 líneas × sizeof(ModemLine_t)=1024 bytes = 32KB.
     * ARQUITECTURA: 32KB de cola es mucho para un sistema embedded. Considerar
     *    reducir MODEM_LINE_MAX o AT_QUEUE_DEPTH si hay problemas de memoria.
     */
    QueueHandle_t  atResponseQueue;

    /**
     * Cola de URCs. uartReaderTask produce, urcHandlerTask consume.
     * Capacidad: URC_QUEUE_DEPTH=16 líneas × 1024 bytes = 16KB.
     */
    QueueHandle_t  urcQueue;

    // ── Handles de tareas ─────────────────────────────────────────────────────

    /** Handle de la tarea UART reader. Creada en startAsyncInfrastructure(). */
    TaskHandle_t   uartReaderTaskHandle;

    /** Handle de la tarea URC handler. Creada en startAsyncInfrastructure(). */
    TaskHandle_t   urcHandlerTaskHandle;

    // ── Estado de dispatch ────────────────────────────────────────────────────

    /**
     * Sesión AT activa. Protegida por dispatchMux (spinlock).
     * Configura cómo uartReaderTask enruta las respuestas durante un sendAT().
     */
    DispatchSession_t dispatchSession;

    /** Estado de red/GNSS actualizado por urcHandlerTask. */
    UrcState_t        urcState;

    /**
     * Spinlock que protege dispatchSession entre el thread del caller (sendATLocked)
     * y uartReaderTask. Se usa portMUX_TYPE (spinlock de ESP-IDF) en lugar de
     * mutex FreeRTOS porque uartReaderTask puede correr en core diferente en dual-core.
     */
    portMUX_TYPE      dispatchMux;

    // ── Estado del parser UART ────────────────────────────────────────────────

    /**
     * Longitud actual del buffer del parser (bytes acumulados en parserBuf).
     * Se incrementa en processIncomingByte() y se resetea en emitParserLine().
     */
    size_t            parserLen;

    /**
     * true si la línea actual excedió MODEM_LINE_MAX y fue truncada.
     * Se resetea en emitParserLine() cuando la línea se emite o se descarta.
     */
    bool              parserOverflow;

    /**
     * Buffer acumulador del parser byte a byte. uartReaderTask llama
     * processIncomingByte() para cada byte del UART, que acumula aquí hasta
     * recibir '\n'. Entonces emitParserLine() extrae la línea completa.
     */
    char              parserBuf[MODEM_LINE_MAX];

    // ── Infraestructura de espera de URC específico ───────────────────────────

    /**
     * Semáforo para waitForUrcPrefix(). Se da cuando urcHandlerTask detecta
     * un URC que coincide con waitUrcPrefix[]. Permite que sendATLocked() espere
     * un URC específico de forma sincronizada con timeout.
     */
    SemaphoreHandle_t waitUrcSem;

    /**
     * Prefijo del URC que waitForUrcPrefix() está esperando.
     * Vacío si no hay espera activa de URC. Escrito por waitForUrcPrefix(),
     * leído por urcHandlerTask para decidir si dar el semáforo.
     */
    char              waitUrcPrefix[64];

    /**
     * true si waitForUrcPrefix() está activa (hay una espera en curso).
     * Protege la lógica de signaling del semáforo waitUrcSem.
     */
    bool              waitUrcActive;

    /**
     * Buffer donde urcHandlerTask copia el URC encontrado para que
     * waitForUrcPrefix() pueda leerlo después de que el semáforo se da.
     */
    char              waitUrcResult[MODEM_LINE_MAX];

    // ── Cola FIFO de comandos recibidos del servidor via TCP ─────────────────
    // Hasta 4 comandos simultáneos (suficiente para ARM+ENGINE_CUT llegando juntos).
    // tryReadServerCommand() extrae TODOS los CMD| del bloque de 128 bytes en una
    // sola pasada; readLastServerCommand() los entrega de a uno al comm_task.
    static constexpr uint8_t CMD_QUEUE_SIZE = 4;
    char              _cmdQueue[CMD_QUEUE_SIZE][64];
    uint8_t           _cmdQueueHead;   // índice del próximo a leer
    uint8_t           _cmdQueueCount;  // cantidad de comandos en la cola
    bool              _hasPendingCmd;  // true si _cmdQueueCount > 0 (alias de conveniencia)
    bool              _tcpConnected;   // true si el socket 0 está abierto y usable
    bool              _hasPendingRx;   // true cuando el A7670 emitió +CIPRXGET: 1,0 (datos entrantes)

    // ── Métodos privados de hardware y boot ───────────────────────────────────

    /**
     * @brief Inicializa UART2 y el GPIO de power del modem.
     * Configura uart_driver_install() con los buffers RX/TX y los pines GPIO16/17/4.
     */
    esp_err_t initHardware();

    /**
     * @brief Crea las colas FreeRTOS y lanza uartReaderTask y urcHandlerTask.
     * Debe llamarse después de initHardware() y antes de bootModem().
     */
    esp_err_t startAsyncInfrastructure();

    /**
     * @brief Secuencia de boot del modem A7670.
     * Pulso en GPIO4, esperar que el modem responda "AT/OK" (hasta 60s).
     */
    esp_err_t bootModem();

    /**
     * @brief Configura los parámetros AT básicos del modem.
     * AT+CMEE=2 (errores verbosos), AT+CEREG=1, AT+CGREG=1, etc.
     */
    esp_err_t configureAT();

    /**
     * @brief Genera un pulso de ms milisegundos en GPIO4 para power-on del modem.
     * El A7670 requiere un pulso mínimo de ~500ms en la línea PWRKEY para arrancar.
     */
    void powerPulse(uint32_t ms);

    /**
     * @brief Ejecuta un hard reset del modem via GPIO4.
     * Secuencia de pulso largo (>1s) que fuerza el reinicio del hardware del modem.
     */
    esp_err_t hardReset();

    // ── Métodos privados de red y PDP ─────────────────────────────────────────

    /**
     * @brief Configura el contexto PDP (datos móviles) con el APN.
     * AT+CGDCONT=1,"IP","internet" → contexto 1, IP, APN="internet".
     */
    bool initDataContext();

    /**
     * @brief Verifica y abre el stack de red del modem si es necesario.
     * AT+NETOPEN si !netStackOpen, o verifica el estado con AT+NETOPEN?.
     * @return true si el stack de red está abierto.
     */
    bool ensureNetOpen();

    /**
     * @brief Reinicia el stack de red del modem (AT+NETCLOSE + AT+NETOPEN).
     * Útil cuando ensureNetOpen() falla por estado corrupto del modem.
     */
    bool resetNetStack();

    /**
     * @brief Hace polling de AT+NETOPEN? hasta que el stack reporta listo.
     * @param maxSecs Tiempo máximo de polling en segundos.
     * @return true si el stack reportó listo dentro del tiempo.
     */
    bool pollNetOpenReady(int maxSecs);

    /**
     * @brief Reinicia el contexto PDP (AT+CGDCONT + AT+CGACT).
     * Para recuperar la conexión de datos si se perdió la sesión de red.
     */
    bool resetPdp();

    /**
     * @brief Verifica que la red 4G está registrada y con datos activos.
     * Consulta AT+CGREG? y AT+CGACT? para confirmar el estado.
     */
    bool checkNetworkReady();

    // ── Métodos privados de infraestructura AT ────────────────────────────────

    /** Versión interna de sendAT() que asume que el caller tiene el mutex. */
    bool sendATLocked(const char* command, const char* expectedResp,
                      uint32_t timeoutMs, bool silent = false);

    /** Espera una línea de atResponseQueue que contenga `expected`, con timeout. */
    bool waitForExpectedLineLocked(const char* expected, uint32_t timeoutMs,
                                   bool waitForOk, bool silent = false);

    /** Espera cualquier línea de atResponseQueue que contenga `expected`. */
    bool waitForLineLocked(const char* expected, uint32_t timeoutMs, bool silent = false);

    /** Configura DispatchSession para que uartReaderTask enrute respuestas al caller. */
    void beginDispatch(const char* command, const char* expectedResp,
                       bool waitForOk, bool expectPrompt, bool captureOnlyExpected);

    /** Limpia DispatchSession al finalizar un sendAT(). */
    void endDispatch();

    /** Espera un URC específico (por prefijo) con timeout. */
    bool waitForUrcPrefix(const char* prefix, uint32_t timeoutMs, bool copyToRxBuf = true);

    /** Construye el prefijo AT esperado de un comando (ej. "+CGNSINF" de "AT+CGNSINF"). */
    void buildDerivedPrefix(const char* command, char* out, size_t outSize) const;

    /** Purga todos los elementos de una cola FreeRTOS. */
    void purgeQueue(QueueHandle_t queue);

    /** Limpia el buffer rxBuf[]. */
    void clearRxBuf();

    /** Lee hasta 128 bytes del socket 0 con AT+CIPRXGET y encola todos los CMD| encontrados en _cmdQueue. */
    void tryReadServerCommand();

    /** Agrega una línea al final de rxBuf[] con '\n' separador. */
    void appendRxLine(const char* line);

    /** Retorna true si la línea es "ERROR" o "+CME ERROR:" (respuesta de error AT). */
    bool isErrorLine(const char* line) const;

    /** Retorna true si la línea es un URC conocido (ej. "+CGREG:", "+GNSS:"). */
    bool isKnownUrc(const char* line) const;

    /** Retorna true si la línea debe capturarse en la sesión de dispatch activa. */
    bool matchesDispatch(const char* line, const DispatchSession_t& session) const;

    /** Retorna true si la línea debe enrutarse a atResponseQueue (vs urcQueue). */
    bool shouldRouteToResponse(const char* line, const DispatchSession_t& session) const;

    /**
     * @brief Clasifica y enruta una línea AT parseada.
     * Corre en uartReaderTask. Decide si va a atResponseQueue, urcQueue, o se descarta.
     */
    void handleParsedLine(const char* line);

    /**
     * @brief Emite la línea acumulada en parserBuf[] hacia handleParsedLine().
     * Llama handleParsedLine() y resetea parserLen=0 y parserOverflow=false.
     */
    void emitParserLine();

    /**
     * @brief Procesa un byte del UART, acumulando en parserBuf hasta '\n'.
     * El corazón del parser: convierte el stream UART en líneas AT discretas.
     */
    void processIncomingByte(uint8_t byte);

    /**
     * @brief Procesa un URC completo en el contexto de urcHandlerTask.
     * Actualiza urcState según el contenido del URC (registro, GNSS, señal).
     */
    void handleUrcLine(const char* line);

    // ── Método privado GPS ────────────────────────────────────────────────────

    /**
     * @brief Parsea la respuesta AT+CGNSINF en una estructura GpsData_t.
     *
     * FORMATO DE RESPUESTA CGNSINF (SIMCom):
     *   "+CGNSINF: <mode>,<fix>,<utcdate>,<lat>,<lon>,<alt>,<speed>,<course>,
     *    <fixmode>,<reserved>,<hdop>,<pdop>,<vdop>,<reserved>,<sats_inuse>,..."
     *
     * @param response String completo de la respuesta AT+CGNSINF.
     * @param gps      Estructura de salida donde se escriben los datos parseados.
     * @return true si el parsing fue exitoso y hay datos válidos.
     */
    bool parseGNSINF(const char* response, GpsData_t& gps);

    // ── Entry points de tareas ────────────────────────────────────────────────

    /**
     * @brief Entry point estático para uartReaderTask (requerido por xTaskCreate).
     * Convierte el `arg` void* de nuevo a A7670Driver* y llama uartReaderTask().
     */
    static void uartReaderTaskEntry(void* arg);

    /**
     * @brief Entry point estático para urcHandlerTask.
     */
    static void urcHandlerTaskEntry(void* arg);

    /**
     * @brief Implementación de la tarea UART reader.
     * Loop infinito: uart_read_bytes() → processIncomingByte() por cada byte.
     */
    void uartReaderTask();

    /**
     * @brief Implementación de la tarea URC handler.
     * Loop infinito: xQueueReceive(urcQueue) → handleUrcLine().
     */
    void urcHandlerTask();
};


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/drivers/include/a7670_driver.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   A7670Driver es el "traductor" entre el ESP32 y el modem 4G. El modem 4G
   entiende comandos AT (comandos de texto como "AT+CGNSINF" para pedir el GPS
   o "AT+CIPOPEN" para abrir una conexión TCP). El driver convierte las llamadas
   de alto nivel de commTask (getGpsPosition(), tcpSend()) en secuencias de
   comandos AT, espera las respuestas, y retorna los datos en forma utilizables.

   Para no bloquear la CPU mientras espera respuestas del modem, el driver usa
   dos tareas FreeRTOS internas:
   - uartReaderTask: lee bytes del UART en tiempo real y los agrupa en líneas.
   - urcHandlerTask: procesa las notificaciones espontáneas del modem (URCs).

   MAPA DE MÉTODOS PÚBLICOS:
   Método           | Usa para
   -----------------|----------------------------------------------
   init()           | Inicialización completa al arrancar commTask
   recover()        | Recuperación tras fallo o zombie
   enableGNSS()     | Encender receptor GPS del modem
   getGpsPosition() | Obtener latitud/longitud del GPS
   tcpSend()        | Enviar frame TCP al backend
   isAlive()        | Verificar que el modem responde
   maintain()       | Keepalive periódico de la conexión
   sendAT()         | Enviar comando AT directamente (público)

   TABLA DE CONSTANTES:
   Constante             | Valor  | Descripción
   ----------------------|--------|-----------------------------
   A7670_UART_PORT       | 2      | UART2 del ESP32
   A7670_UART_BAUD_RATE  | 115200 | Velocidad de comunicación
   A7670_TIMEOUT_SHORT   | 3s     | Comandos AT simples
   A7670_TIMEOUT_MEDIUM  | 15s    | Operaciones de red
   A7670_TIMEOUT_LONG    | 120s   | Boot, GNSS cold start
   TCP_SERVER_HOST       | 34.69.219.193 | Backend GCP
   TCP_SERVER_PORT       | 9000   | Puerto TCP backend

   DEUDA TÉCNICA:
   1. APN hardcodeado "internet" — no configurable sin recompilar.
   2. xModemMutex debe ser tomado externamente — no encapsulado en el driver.
   3. Conexión TCP abre/cierra por frame — sin keep-alive persistente.
   4. AT_QUEUE_DEPTH y URC_QUEUE_DEPTH × MODEM_LINE_MAX = 48KB de colas en heap.

   ═══════════════════════════════════════════════════════════ */
