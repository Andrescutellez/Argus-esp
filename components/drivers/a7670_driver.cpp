/**
 * @file    a7670_driver.cpp
 * @brief   Driver de bajo nivel para el módem celular A7670/SIM7670G.
 *
 * @details
 * ### Rol en el sistema
 * Único punto de contacto entre el firmware ESP32 y el hardware GSM/GNSS.
 * Expone una API síncrona de alto nivel (init, tcpSend, getGpsPosition) mientras
 * gestiona internamente toda la asincronía de UART, URCs y estados de red.
 *
 * ### Hardware
 * | Recurso        | Pin / valor       | Notas                              |
 * |----------------|-------------------|------------------------------------|
 * | UART2 TX       | PIN_A7670_TX=17   | ESP32→módem                        |
 * | UART2 RX       | PIN_A7670_RX=16   | módem→ESP32                        |
 * | PWRKEY         | PIN_A7670_PWRKEY=4| Nivel ALTO enciende/apaga          |
 * | Baudrate       | 115200 8N1 APB    | APB evita rebase en freq. dinámica |
 *
 * ### Arquitectura interna asíncrona
 * ```
 * UART HW ──► uartReaderTask (prio 5) ──► processIncomingByte()
 *                                              │
 *                                    emitParserLine() → handleParsedLine()
 *                                              │
 *                           ┌─────────────────┴──────────────────┐
 *                      atResponseQueue (32)              urcQueue (16)
 *                           │                                     │
 *               waitForExpectedLineLocked()          urcHandlerTask (prio 4)
 *               (llamado desde sendATLocked)              │
 *                                                  handleUrcLine()
 *                                                  (actualiza urcState, netStackOpen, etc.)
 * ```
 *
 * ### Variables críticas / flags de estado
 * | Variable         | Tipo            | Protección         | Significado                   |
 * |------------------|-----------------|--------------------|-------------------------------|
 * | state            | ModemState_t    | xModemMutex        | Progreso de inicialización    |
 * | netStackOpen     | bool (volatile) | dispatchMux        | TCP/IP stack activo           |
 * | gnssEnabled      | bool            | xModemMutex        | Motor GNSS encendido          |
 * | dispatchSession  | DispatchSession_t | dispatchMux (spinlock) | Comando AT en vuelo    |
 * | waitUrcActive    | bool            | dispatchMux        | Waiter de URC armado          |
 * | waitUrcPrefix    | char[]          | dispatchMux        | Prefijo que activa waitUrcSem |
 * | waitUrcResult    | char[]          | dispatchMux        | Copia de la línea URC recibida|
 * | urcState         | UrcState_t      | dispatchMux        | CSQ, CGREG, GNSS flags        |
 * | parserBuf/Len    | char[]/size_t   | uartReaderTask     | Buffer de línea en formación  |
 *
 * ### Riesgos de concurrencia
 * - dispatchMux es un spinlock FreeRTOS (portMUX_TYPE). Las secciones críticas
 *   protegidas por él deben ser brevísimas (sin llamadas bloqueantes).
 * - xModemMutex es un semáforo recursivo externo (definido en main.cpp).
 *   Permite re-entrancia desde la misma tarea (init→enableGNSS).
 * - uartReaderTask corre a prioridad 5 (la más alta de las tareas de aplicación)
 *   para vaciar el buffer de HW antes de que el driver de UART lo desborde.
 * - handleParsedLine() es llamado desde uartReaderTask; nunca bloquea (xQueueSend
 *   con timeout=0, evicción inmediata si la cola está llena).
 *
 * ### Deuda técnica
 * - ARQUITECTURA ⚠️ APN hardcoded en ARGUS_APN (compile-time).
 * - ARQUITECTURA ⚠️ xModemMutex es global externo (acoplamiento implícito con main).
 * - ARQUITECTURA ⚠️ TCP abre/cierra socket por cada trama (latencia extra ~2-4s).
 * - ARQUITECTURA ⚠️ MQTT URCs en isKnownUrc() son residuos del diseño original.
 *
 * @dependencies
 * - FreeRTOS: tareas, colas, semáforos, spinlocks.
 * - ESP-IDF: uart_driver, gpio, esp_timer, esp_mac.
 * - a7670_driver.h: tipos, constantes, declaración de clase.
 * - pin_config.h: PIN_A7670_TX/RX/PWRKEY.
 */

#include "a7670_driver.h"
#include "pin_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "freertos/semphr.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "A7670";

// ─── Utilidades internas ──────────────────────────────────────────────────────
namespace {

/**
 * @brief RAII guard para xSemaphoreTakeRecursive / xSemaphoreGiveRecursive.
 *
 * @details
 * ### Propósito
 * Garantizar que el mutex del módem se libere aunque la función retorne
 * por múltiples paths (error temprano, return normal, etc.).  Sin este
 * patrón cada path de salida requiere xSemaphoreGiveRecursive manual,
 * lo que históricamente produce deadlocks cuando se agrega un return nuevo.
 *
 * ### Flujo
 * 1. Constructor: intenta tomar el mutex con el timeout dado.
 * 2. locked() informa al caller si tuvo éxito.
 * 3. Destructor: devuelve el mutex si fue tomado.
 *
 * @note El mutex es recursivo para permitir que init() llame a enableGNSS()
 *       desde la misma tarea sin deadlock.
 */
class ModemLockGuard {
public:
    /**
     * @param mutex   Handle del semáforo recursivo a tomar.
     * @param timeout Ticks FreeRTOS a esperar (por defecto: portMAX_DELAY).
     */
    explicit ModemLockGuard(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
        : mutex_(mutex), locked_(false) {
        if (mutex_ != nullptr) {
            // Recursive: la misma tarea puede tomar el mutex N veces
            // sin bloquearse; debe dar N veces en el destructor.
            locked_ = (xSemaphoreTakeRecursive(mutex_, timeout) == pdTRUE);
        }
    }
    ~ModemLockGuard() {
        // Solo liberar si realmente se tomó; evitar Give sin Take previo
        // (comportamiento indefinido en FreeRTOS).
        if (locked_ && mutex_ != nullptr) {
            xSemaphoreGiveRecursive(mutex_);
        }
    }
    bool locked() const { return locked_; }
private:
    SemaphoreHandle_t mutex_;
    bool              locked_;
};

/**
 * @brief Extrae el primer entero encontrado en 'text' tras la subcadena 'marker'.
 *
 * @details
 * ### Propósito
 * Parseo genérico de respuestas AT del tipo "+FOO: 3" o "+BAR:0".
 * Evita duplicar lógica strstr+atoi en cada handler de URC.
 *
 * ### Flujo
 * 1. Busca 'marker' en 'text'.
 * 2. Avanza pasando espacios y ':'.
 * 3. Convierte con atoi (detiene en primer carácter no numérico).
 *
 * @param text     Línea completa recibida del módem.
 * @param marker   Subcadena a buscar (ej. "+NETOPEN").
 * @param outValue Entero extraído.
 * @return true si marker fue encontrado y se pudo extraer un entero.
 *
 * @note atoi() retorna 0 si no hay dígitos; el caller debe distinguir
 *       "0 legítimo" de "no encontrado" usando el valor de retorno bool.
 */
bool extractFirstIntegerAfter(const char* text, const char* marker, int* outValue) {
    const char* start = strstr(text, marker);
    if (start == nullptr) return false;
    start += strlen(marker);
    // Saltar separadores opcionales (el módulo usa ":" y " " indistintamente)
    while (*start == ' ' || *start == ':') start++;
    *outValue = atoi(start);
    return true;
}

/**
 * @brief Extrae el código de resultado de una línea "+NETOPEN: N".
 *
 * @details
 * ### Propósito
 * Wrapper específico de extractFirstIntegerAfter() para +NETOPEN, con nombre
 * semánticamente claro en los callers.
 *
 * @param response Línea URC recibida.
 * @param outCode  Código (0=éxito, otro=error de red).
 * @return true si se pudo parsear.
 */
bool parseNetopenCode(const char* response, int* outCode) {
    return extractFirstIntegerAfter(response, "+NETOPEN", outCode);
}

}  // namespace

// ─── Constructor ──────────────────────────────────────────────────────────────

/**
 * @brief Inicializa todos los miembros a valores seguros antes del primer uso.
 *
 * @details
 * ### Propósito
 * Garantizar que ningún miembro quede sin inicializar.  El objeto puede ser
 * construido antes de que FreeRTOS esté activo (variable global en main.cpp),
 * por lo que no se crean semáforos ni tareas aquí — eso ocurre en init().
 *
 * ### Valores de sentinel
 * - rssi/ber = -1 → "dato no recibido aún" (99 en AT+CSQ significa sin señal).
 * - cgRegMode/Stat = -1 → "nunca se recibió +CGREG".
 * - dispatchMux = portMUX_INITIALIZER_UNLOCKED → spinlock listo para usar
 *   sin llamada de inicialización adicional.
 */
A7670Driver::A7670Driver()
    : state(MODEM_STATE_UART_READY),
      uartInstalled(false),
      gpioInitialized(false),
      asyncStarted(false),
      gnssEnabled(false),
      gnssConsecCrashCount(0),
      gnssDisabledForSession(false),
      netStackOpen(false),
      atResponseQueue(nullptr),
      urcQueue(nullptr),
      uartReaderTaskHandle(nullptr),
      urcHandlerTaskHandle(nullptr),
      dispatchMux(portMUX_INITIALIZER_UNLOCKED),
      parserLen(0),
      parserOverflow(false),
      waitUrcSem(nullptr),
      waitUrcActive(false),
      _cmdQueueHead(0),
      _cmdQueueCount(0),
      _hasPendingCmd(false),
      _tcpConnected(false),
      _hasPendingRx(false),
      _lastCmdPollUs(0) {
    memset(_cmdQueue, 0, sizeof(_cmdQueue));
    memset(rxBuf, 0, sizeof(rxBuf));
    memset(&dispatchSession, 0, sizeof(dispatchSession));
    memset(&urcState, 0, sizeof(urcState));
    memset(parserBuf, 0, sizeof(parserBuf));
    // -1 distingue "sin dato" de "señal nula" (0) o "sin señal reportada" (99)
    urcState.rssi = -1;
    urcState.ber  = -1;
    // -1 indica que nunca se procesó un +CGREG; útil para detectar primer registro
    urcState.cgRegMode = -1;
    urcState.cgRegStat = -1;
    // Cadenas vacías → ningún URC waiter activo en este estado
    waitUrcPrefix[0] = '\0';
    waitUrcResult[0] = '\0';
    memset(_cmdQueue, 0, sizeof(_cmdQueue));
}

// ─── Hardware ─────────────────────────────────────────────────────────────────

/**
 * @brief Instala el driver UART2 y configura GPIO de PWRKEY.
 *
 * @details
 * ### Propósito
 * Configurar el hardware físico una única vez de forma idempotente.
 * Separado de startAsyncInfrastructure() para permitir reintentos independientes.
 *
 * ### Flujo
 * 1. Si !uartInstalled: configura parámetros UART → asigna pines → instala driver.
 * 2. Si !gpioInitialized: configura GPIO4 como salida y lo deja en LOW.
 *
 * ### Por qué UART_SCLK_APB
 * El clock APB es estable bajo todas las configuraciones de freq. dinámica
 * del ESP32; UART_SCLK_REF puede derivar si se activa el power management.
 *
 * ### Por qué GPIO en LOW al inicio
 * El módulo A7670 interpreta un pulso en PWRKEY de ≥1s para encender/apagar.
 * LOW en reposo garantiza que no se dispare un encendido/apagado accidental
 * al inicializar el GPIO.
 *
 * @return ESP_OK si el hardware quedó listo; error de ESP-IDF de lo contrario.
 *
 * @note Idempotente: llamadas posteriores retornan ESP_OK sin efecto.
 */
esp_err_t A7670Driver::initHardware() {
    if (!uartInstalled) {
        uart_config_t cfg = {};
        cfg.baud_rate  = A7670_UART_BAUD_RATE;   // 115200
        cfg.data_bits  = UART_DATA_8_BITS;
        cfg.parity     = UART_PARITY_DISABLE;
        cfg.stop_bits  = UART_STOP_BITS_1;
        cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE; // A7670 no usa CTS/RTS
        cfg.source_clk = UART_SCLK_APB;            // estable bajo DVFS

        esp_err_t ret = uart_param_config(A7670_UART_PORT, &cfg);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(ret)); return ret; }
        ret = uart_set_pin(A7670_UART_PORT, PIN_A7670_TX, PIN_A7670_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(ret)); return ret; }
        // RX_BUF grande para absorber ráfagas de URCs sin perder bytes
        ret = uart_driver_install(A7670_UART_PORT, A7670_UART_RX_BUF, A7670_UART_TX_BUF, 0, nullptr, 0);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(ret)); return ret; }
        uartInstalled = true;
        ESP_LOGI(TAG, "UART2 (TX=%d RX=%d, %d baud)", PIN_A7670_TX, PIN_A7670_RX, A7670_UART_BAUD_RATE);
    }

    if (!gpioInitialized) {
        gpio_config_t io = {};
        io.pin_bit_mask  = (1ULL << PIN_A7670_PWRKEY);
        io.mode          = GPIO_MODE_OUTPUT;
        io.pull_up_en    = GPIO_PULLUP_DISABLE;    // nivel controlado por ESP32
        io.pull_down_en  = GPIO_PULLDOWN_DISABLE;
        io.intr_type     = GPIO_INTR_DISABLE;      // no se necesita interrupción
        gpio_config(&io);
        gpio_set_level(PIN_A7670_PWRKEY, 0);       // LOW = reposo; pulso controlado en powerPulse()
        gpioInitialized = true;
    }
    return ESP_OK;
}

/**
 * @brief Crea las colas FreeRTOS y lanza las tareas de procesamiento UART/URC.
 *
 * @details
 * ### Propósito
 * Arrancar el pipeline asíncrono que convierte bytes UART en líneas AT y
 * las enruta al canal correcto (atResponseQueue o urcQueue).
 *
 * ### Flujo
 * 1. Guard: si asyncStarted → retornar ESP_OK sin efecto.
 * 2. Crear atResponseQueue (32 slots) y urcQueue (16 slots).
 * 3. Crear waitUrcSem (binario, para espera puntual de URCs en tcpSend).
 * 4. Lanzar uartReaderTask (prio 5) y urcHandlerTask (prio 4).
 *
 * ### Tamaños de cola
 * - atResponseQueue=32: un comando AT puede generar muchas líneas de respuesta
 *   (ATI, +CSQ, +CPIN, etc.) antes de "OK"; 32 evita pérdidas.
 * - urcQueue=16: los URCs llegan en ráfagas cortas; 16 cubre arranque del módulo
 *   (RDY, SMS READY, PB DONE, Call Ready, *ATREADY).
 *
 * ### Por qué uartReaderTask tiene prioridad 5
 * Si uartReaderTask no consume el buffer de HW rápido, el driver de UART
 * desborda y se pierden bytes → tramas corruptas → timeouts AT.
 * Corre más alto que commTask (3) y controlTask (2).
 *
 * @return ESP_OK si la infraestructura quedó lista; ESP_ERR_NO_MEM o ESP_FAIL si no.
 *
 * @note Idempotente: llamadas posteriores retornan ESP_OK sin efecto.
 */
esp_err_t A7670Driver::startAsyncInfrastructure() {
    if (asyncStarted) return ESP_OK;

    atResponseQueue = xQueueCreate(AT_QUEUE_DEPTH, sizeof(ModemLine_t));
    urcQueue        = xQueueCreate(URC_QUEUE_DEPTH, sizeof(ModemLine_t));
    if (!atResponseQueue || !urcQueue) {
        ESP_LOGE(TAG, "No se pudieron crear colas AT/URC");
        return ESP_ERR_NO_MEM;
    }
    // Semáforo binario para sincronización punto-a-punto en tcpSend:
    // handleParsedLine() lo da cuando llega el URC; tcpSend() lo espera.
    waitUrcSem = xSemaphoreCreateBinary();
    if (!waitUrcSem) {
        ESP_LOGE(TAG, "No se pudo crear waitUrcSem");
        return ESP_ERR_NO_MEM;
    }

    // Stack 4096: suficiente para processIncomingByte + handleParsedLine.
    // Prioridad 5 (máxima de aplicación) para drenar HW FIFO antes que nadie.
    BaseType_t r1 = xTaskCreate(&A7670Driver::uartReaderTaskEntry, "A7670UartRx", 4096, this, 5, &uartReaderTaskHandle);
    // Prioridad 4: handleUrcLine puede actualizar netStackOpen etc. rápido,
    // pero cede ante uartReaderTask para no bloquearlo.
    BaseType_t r2 = xTaskCreate(&A7670Driver::urcHandlerTaskEntry, "A7670Urc",    4096, this, 4, &urcHandlerTaskHandle);
    if (r1 != pdPASS || r2 != pdPASS) {
        ESP_LOGE(TAG, "No se pudieron crear tareas UART/URC");
        return ESP_FAIL;
    }

    asyncStarted = true;
    ESP_LOGI(TAG, "Infraestructura AT asincrona iniciada");
    return ESP_OK;
}

/**
 * @brief Genera un pulso en PWRKEY de la duración indicada.
 *
 * @details
 * ### Propósito
 * El A7670 entra o sale de estado de energía con un pulso en PWRKEY:
 * - ≥1.0 s → enciende si está apagado.
 * - ≥3.0 s → apaga forzado si está encendido o colgado.
 *
 * ### Por qué bloquea (vTaskDelay)
 * El pulso es un requisito de hardware de temporización precisa; no se puede
 * entregar en background.  El caller (bootModem/recover) ya posee xModemMutex
 * y asume que el módulo estará listo para AT cuando powerPulse() retorne.
 *
 * @param ms Duración del pulso en milisegundos (1500 para boot, 3500 para recovery).
 */
void A7670Driver::powerPulse(uint32_t ms) {
    ESP_LOGI(TAG, "PWRKEY: pulso %lu ms", (unsigned long)ms);
    gpio_set_level(PIN_A7670_PWRKEY, 1);    // Flanco de subida inicia la secuencia interna del módulo
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(PIN_A7670_PWRKEY, 0);    // Flanco de bajada: módulo registra la duración del pulso
}

// ─── init / boot / recover ───────────────────────────────────────────────────

/**
 * @brief Secuencia de inicialización completa del módem.
 *
 * @details
 * ### Propósito
 * Punto de entrada único para dejar el driver en estado MODEM_STATE_NET_READY.
 * Llamado en el arranque desde main.cpp y desde recover() después de un reset.
 *
 * ### Flujo
 * 1. Tomar xModemMutex (exclusividad frente a commTask).
 * 2. initHardware() — UART + GPIO.
 * 3. startAsyncInfrastructure() — colas + tareas (idempotente).
 * 4. bootModem() — encender si no responde AT.
 * 5. configureAT() — deshabilitar echo, zona horaria, etc.
 * 6. GNSS NO se habilita aquí (ver nota).
 * 7. initDataContext() — APN + PDP + NETOPEN.
 *
 * ### Por qué GNSS es lazy
 * Habilitar GNSS (AT+CGNSSPWR=1) durante init() provoca un spike de corriente
 * ~8 s después que cuelga el módulo en hardware V1.11.2.  Se habilita en la
 * primera llamada a getGpsPosition().
 *
 * @return ESP_OK si el módem quedó listo para tcpSend y getGpsPosition.
 */
esp_err_t A7670Driver::init() {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) { ESP_LOGE(TAG, "No se pudo tomar xModemMutex en init()"); return ESP_FAIL; }

    esp_err_t ret = initHardware();
    if (ret != ESP_OK) return ret;

    ret = startAsyncInfrastructure();
    if (ret != ESP_OK) return ret;

    // Resetear flag: recover() puede llamar init() con el stack ya cerrado
    netStackOpen = false;

    ret = bootModem();
    if (ret != ESP_OK) return ret;

    ret = configureAT();
    if (ret != ESP_OK) return ret;

    // GNSS se habilita lazy desde getGpsPosition(). Habilitarlo en init()
    // causa crashes de power spike (~8s post-CGNSSPWR=1) en V1.11.2.

    if (!initDataContext()) {
        // No falla init(): commTask reintentará en cada tcpSend vía ensureNetOpen
        ESP_LOGW(TAG, "initDataContext falló — se reintentará en tcpSend");
    }

    return ESP_OK;
}

/**
 * @brief Enciende el módulo si no responde AT.
 *
 * @details
 * ### Propósito
 * Distinguir entre "módulo ya encendido" (solo reiniciado el ESP32) y
 * "módulo apagado" para evitar un power-cycle innecesario.
 *
 * ### Flujo
 * 1. isAlive(): envía "AT" y espera "OK" (500 ms).
 * 2. Si ya responde → retornar ESP_OK sin powerPulse.
 * 3. powerPulse(1500 ms) → arranca secuencia de encendido.
 * 4. Polling 30 × 500 ms (15 s) hasta que "AT" responda con "OK".
 *
 * ### Por qué 1500 ms de pulso
 * Datasheet A7670: PWRKEY debe estar en HIGH ≥1.0 s para disparar encendido.
 * 1500 ms da margen a variaciones de hardware.
 *
 * @return ESP_OK si el módulo responde; ESP_FAIL si no respondió en 15 s.
 */
esp_err_t A7670Driver::bootModem() {
    if (isAlive()) {
        ESP_LOGI(TAG, "Módulo ya activo; se omite power-on");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Módulo sin respuesta; ejecutando secuencia de encendido");
    powerPulse(1500);
    // Esperar hasta 15 s en chunks de 500 ms para no bloquear el RTOS más de lo necesario
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (sendATLocked("AT", "OK", 500, true)) {
            ESP_LOGI(TAG, "Módulo activo (~%d s)", (i + 1) / 2);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "El módulo no respondió tras 15 s");
    state = MODEM_STATE_ERROR;
    return ESP_FAIL;
}

/**
 * @brief Configura el módem con los parámetros base de operación.
 *
 * @details
 * ### Propósito
 * Llevar el módulo de "recién encendido" a "listo para comandos de red"
 * con el perfil mínimo necesario para Argus.
 *
 * ### Comandos y por qué cada uno
 * | Comando        | Propósito                                                   |
 * |----------------|-------------------------------------------------------------|
 * | ATI            | Identificación (solo log; no crítico)                       |
 * | ATE0           | Desactiva echo → evita que la respuesta incluya el comando  |
 * | AT+CMEE=0      | Desactiva CME ERROR extendido → respuestas más compactas    |
 * | AT+CTZR=0      | Desactiva URCs de zona horaria → reduce ruido en urcQueue   |
 * | AT+CTZU=1      | Auto-actualiza reloj desde la red (NITZ)                    |
 * | AT+CNMP=2      | Modo de red automático (LTE primero, fallback 2G/3G)        |
 * | AT+CSCLK=0     | Desactiva sleep del módulo → UART siempre activo            |
 * | AT+CSQ         | Señal inicial (solo log)                                    |
 * | AT+CPIN?       | Verifica SIM presente y desbloqueada                        |
 *
 * ### Por qué 3 s de delay al inicio
 * Después de powerPulse, el módulo tarda ~2-3 s en estabilizar el firmware
 * interno antes de poder procesar comandos AT.
 *
 * @return Siempre ESP_OK (fallos individuales son no-fatales; se loguean).
 */
esp_err_t A7670Driver::configureAT() {
    // Esperar que el firmware del módulo inicialice su stack AT interno
    vTaskDelay(pdMS_TO_TICKS(3000));
    sendATLocked("ATI",       "OK", A7670_TIMEOUT_SHORT_MS);
    ESP_LOGI(TAG, "ATI: %.120s", rxBuf);
    {
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        ESP_LOGI(TAG, "Base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    // Echo off: sin esto, cada respuesta empieza con el eco del comando enviado
    sendATLocked("ATE0",      "OK", A7670_TIMEOUT_SHORT_MS);
    // CMEE=0: los errores CME extendidos (+CME ERROR: N) se convierten en "ERROR" simple
    sendATLocked("AT+CMEE=0", "OK", A7670_TIMEOUT_SHORT_MS, true);
    // CTZR=0: silencia URCs "+CTZV: ..." que llenan la cola sin aportar valor
    sendATLocked("AT+CTZR=0", "OK", A7670_TIMEOUT_SHORT_MS, true);
    // CTZU=1: el módulo actualiza su RTC vía NITZ automáticamente → timestamps correctos en GNSS
    sendATLocked("AT+CTZU=1", "OK", A7670_TIMEOUT_SHORT_MS, true);
    // CNMP=2: permite LTE y fallback a 2G/3G; sin esto el módulo puede quedar bloqueado en un modo
    sendATLocked("AT+CNMP=2", "OK", A7670_TIMEOUT_SHORT_MS, true);
    // CSCLK=0: sleep haría que el módulo no responda al UART entre comandos
    sendATLocked("AT+CSCLK=0","OK", A7670_TIMEOUT_SHORT_MS);
    sendATLocked("AT+CSQ",    "OK", A7670_TIMEOUT_SHORT_MS);
    // CIPRXGET=1: buffer incoming TCP data so we can poll it with AT+CIPRXGET=2
    sendATLocked("AT+CIPRXGET=1", "OK", A7670_TIMEOUT_SHORT_MS, true);

    // CMEE=2 temporalmente para ver el codigo de error real de la SIM en el log.
    // Se restaura CMEE=0 al final del bloque.
    sendATLocked("AT+CMEE=2", "OK", A7670_TIMEOUT_SHORT_MS, true);
    {
        bool simReady = false;
        for (int i = 0; i < 5 && !simReady; i++) {
            if (sendATLocked("AT+CPIN?", "READY", A7670_TIMEOUT_SHORT_MS)) {
                ESP_LOGI(TAG, "SIM lista (intento %d/5)", i + 1);
                simReady = true;
            } else if (strstr(rxBuf, "NOT INSERTED") || strstr(rxBuf, "+CME ERROR: 10")) {
                ESP_LOGE(TAG, "SIM no insertada — verificar hardware");
                break;
            } else {
                ESP_LOGW(TAG, "SIM no lista (intento %d/5): %.80s", i + 1, rxBuf);
                if (i < 4) vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
        if (!simReady) {
            ESP_LOGE(TAG, "SIM no disponible — no se podra conectar a la red");
        }
    }
    sendATLocked("AT+CMEE=0", "OK", A7670_TIMEOUT_SHORT_MS, true);

    state = MODEM_STATE_AT_READY;
    return ESP_OK;
}

/**
 * @brief Recupera el módulo mediante un power-cycle forzado más largo.
 *
 * @details
 * ### Propósito
 * Restaurar el driver después de un fallo de red persistente detectado por
 * SyncManager (watchdog 15 min).  Más agresivo que un reinicio de stack TCP.
 *
 * ### Flujo
 * 1. Tomar xModemMutex.
 * 2. powerPulse(3500 ms) — 3.5 s fuerza apagado incluso si el módulo estaba colgado.
 * 3. vTaskDelay(5000 ms) — el módulo necesita ~5 s para apagar completamente.
 * 4. Resetear state a UART_READY.
 * 5. Llamar init() (que llamará bootModem + configureAT + initDataContext).
 *
 * ### Por qué 3500 ms (vs 1500 ms en boot)
 * 1500 ms es el mínimo para encender.  Para forzar apagado desde un estado
 * desconocido, el datasheet requiere ≥3.0 s; 3500 ms da margen.
 *
 * @return ESP_OK si el módem se reinició y re-inicializó correctamente.
 */
esp_err_t A7670Driver::recover() {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) { ESP_LOGE(TAG, "No se pudo tomar xModemMutex en recover()"); return ESP_FAIL; }

    ESP_LOGW(TAG, "recover(): power-cycle completo");
    netStackOpen  = false;   // Invalidar estado de red inmediatamente
    _tcpConnected = false;   // El modem se reinicia — cualquier socket abierto se pierde

    // Power-cycle limpia el estado NVM del módulo: el GNSS no se auto-restaura.
    // Resetear contador para dar otra oportunidad al GNSS tras la recuperación.
    taskENTER_CRITICAL(&dispatchMux);
    gnssConsecCrashCount   = 0;
    gnssDisabledForSession = false;
    taskEXIT_CRITICAL(&dispatchMux);

    powerPulse(3500);       // Apagado forzado + posterior encendido
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar descarga de caps internos del módulo
    state = MODEM_STATE_UART_READY;
    return init();  // Re-inicialización completa (xModemMutex ya tomado, recursive OK)
}

/**
 * @brief Reset de software vía AT+CFUN=1,1 (equivalente a reboot del módulo).
 *
 * @details
 * ### Propósito
 * Alternativa a powerPulse cuando el módulo responde AT pero el stack de red
 * está en un estado irrecuperable sin cycle eléctrico.
 *
 * ### Flujo
 * 1. AT+CFUN=1,1 → el módulo ejecuta reset completo de firmware.
 * 2. vTaskDelay(30000 ms) → tiempo de re-arranque del firmware (~25-30 s).
 * 3. init() para re-configurar desde cero.
 *
 * @note Este método no toma xModemMutex; debe ser llamado ya bajo el mutex
 *       o en un contexto donde la exclusividad está garantizada.
 *
 * @return ESP_OK si la re-inicialización fue exitosa.
 */
esp_err_t A7670Driver::hardReset() {
    ESP_LOGE(TAG, "HARD RESET via AT+CFUN=1,1");
    netStackOpen = false;

    // CFUN=1,1 provoca reboot interno; el módulo no responde por ~25-30 s
    sendATLocked("AT+CFUN=1,1", "OK", A7670_TIMEOUT_SHORT_MS, true);
    vTaskDelay(pdMS_TO_TICKS(30000));
    state = MODEM_STATE_UART_READY;
    return init();
}

/**
 * @brief Verifica si el módulo responde al comando AT básico.
 *
 * @details
 * ### Propósito
 * Test de vivacidad rápido usado en bootModem() y externamente para
 * decisiones de health-check.  No realiza ninguna acción sobre el módulo.
 *
 * @return true si el módulo responde "OK" en A7670_TIMEOUT_SHORT_MS.
 */
bool A7670Driver::isAlive() {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return false;
    // silent=true: no loguear errores (se espera que falle antes del boot)
    return sendATLocked("AT", "OK", A7670_TIMEOUT_SHORT_MS, true);
}

void A7670Driver::disconnectTcp() {
    taskENTER_CRITICAL(&dispatchMux);
    bool wasConnected = _tcpConnected;
    _tcpConnected = false;
    taskEXIT_CRITICAL(&dispatchMux);
    if (wasConnected) {
        ModemLockGuard lock(xModemMutex);
        if (lock.locked()) {
            sendATLocked("AT+CIPCLOSE=0", "OK", A7670_TIMEOUT_SHORT_MS, true);
        }
        ESP_LOGI(TAG, "[TCP] Socket cerrado (FREEMIUM modo pasivo)");
    }
}

// ─── GNSS ─────────────────────────────────────────────────────────────────────

/**
 * @brief Enciende el motor GNSS del módulo si no estaba activo.
 *
 * @details
 * ### Propósito
 * Activar el receptor GPS/GLONASS/BeiDou del A7670 para permitir
 * getGpsPosition().  Se llama lazily desde getGpsPosition() en lugar
 * de en init() para evitar el power spike V1.11.2 (ver init()).
 *
 * ### Flujo
 * 1. Tomar xModemMutex.
 * 2. Si gnssEnabled → retornar ESP_OK (idempotente).
 * 3. AT+CGNSSPWR? → si ya está ON (": 1") → marcar gnssEnabled y retornar.
 * 4. AT+CGNSSPWR=1 → encender motor.
 * 5. waitForLineLocked "+CGNSSPWR: READY!" 30 s → el módulo emite este URC
 *    cuando el chip GNSS finalizó su inicialización.
 * 6. Fallback: si el URC no llega en 30 s → 5 s de delay (el chip puede estar
 *    listo aunque no emita el URC en ciertas versiones de firmware).
 *
 * ### Por qué esperar "+CGNSSPWR: READY!"
 * Sin esperar este URC, el primer AT+CGNSSINFO puede retornar ERROR porque
 * el engine no terminó de arrancar.
 *
 * @return ESP_OK si GNSS quedó listo; ESP_FAIL si el comando falló.
 */
esp_err_t A7670Driver::enableGNSS() {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return ESP_FAIL;

    taskENTER_CRITICAL(&dispatchMux);
    bool sessionDisabled = gnssDisabledForSession;
    taskEXIT_CRITICAL(&dispatchMux);

    if (sessionDisabled) {
        ESP_LOGW(TAG, "[GNSS] Deshabilitado para esta sesion (%d crashes con GNSS activo); "
                      "reiniciar ESP32 para rehabilitar", (int)GNSS_MAX_CONSEC_CRASHES);
        return ESP_FAIL;
    }

    if (gnssEnabled) return ESP_OK;  // Guard: no re-habilitar si ya está activo

    // Consultar estado actual para no hacer AT+CGNSSPWR=1 si ya está ON
    if (sendATLocked("AT+CGNSSPWR?", "+CGNSSPWR:", A7670_TIMEOUT_SHORT_MS, true) &&
        strstr(rxBuf, "+CGNSSPWR: 1") != nullptr) {
        ESP_LOGI(TAG, "Motor GNSS ya activo");
        gnssEnabled = true;
        return ESP_OK;
    }

    if (!sendATLocked("AT+CGNSSPWR=1", "OK", A7670_TIMEOUT_MEDIUM_MS)) {
        ESP_LOGE(TAG, "No se pudo habilitar GNSS");
        return ESP_FAIL;
    }

    // El chip GNSS tarda varios segundos en inicializar su firmware interno
    if (!waitForLineLocked("+CGNSSPWR: READY!", 30000, true)) {
        // Algunas versiones de firmware no emiten el URC → fallback conservador
        ESP_LOGW(TAG, "+CGNSSPWR: READY! no recibido; fallback delay");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    gnssEnabled = true;
    urcState.gnssReady = true;
    ESP_LOGI(TAG, "GNSS habilitado (primer fix: 1-3 min)");
    return ESP_OK;
}

/**
 * @brief Lee la posición GPS actual del módulo.
 *
 * @details
 * ### Propósito
 * Obtener coordenadas, velocidad, altitud y timestamp en un solo comando.
 * Maneja la habilitación lazy de GNSS y reintenta si el engine se reseteó.
 *
 * ### Flujo
 * 1. Tomar xModemMutex.
 * 2. memset gps (fix_valid=false por defecto).
 * 3. Si !gnssEnabled → enableGNSS().
 * 4. AT+CGNSSINFO → parsear con parseGNSINF().
 * 5. Si falla y no es "ERROR" literal → el engine puede haberse reseteado:
 *    marcar gnssEnabled=false, re-enableGNSS(), reintentar comando.
 *
 * ### Por qué AT+CGNSSINFO y no +CGNSINF
 * +CGNSINF es el comando SIMCom estándar pero el A7670 de SIMCOM usa
 * AT+CGNSSINFO con un CSV diferente.  Confundirlos da parseo silenciosamente
 * incorrecto (campos fuera de posición).
 *
 * @param gps Referencia a GpsData_t que se rellena con el resultado.
 * @return ESP_OK si fix_valid==true; ESP_ERR_TIMEOUT o ESP_ERR_NOT_FOUND si no.
 */
esp_err_t A7670Driver::getGpsPosition(GpsData_t& gps) {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return ESP_FAIL;

    memset(&gps, 0, sizeof(gps));   // fix_valid=false por defecto
    if (!gnssEnabled) {
        esp_err_t ret = enableGNSS();
        if (ret != ESP_OK) { gps.fix_valid = false; return ret; }
    }

    bool ok = sendATLocked("AT+CGNSSINFO", "+CGNSSINFO:", A7670_TIMEOUT_MEDIUM_MS);
    if (!ok && gnssEnabled) {
        // Si no es un "ERROR" explícito, el engine GNSS puede haberse reseteado
        // (ej. power spike); re-habilitarlo y reintentar una vez.
        if (strstr(rxBuf, "\r\nERROR\r\n") == nullptr) {
            gnssEnabled = false;
            if (enableGNSS() == ESP_OK) {
                ok = sendATLocked("AT+CGNSSINFO", "+CGNSSINFO:", A7670_TIMEOUT_MEDIUM_MS);
            }
        }
    }

    if (!ok) { gps.fix_valid = false; return ESP_ERR_TIMEOUT; }
    if (!parseGNSINF(rxBuf, gps)) { gps.fix_valid = false; return ESP_ERR_NOT_FOUND; }

    // Fix exitoso: GNSS funciona — resetear contador de crashes para no deshabilitar
    // en una sesión futura donde el hardware/firmware esté corregido.
    taskENTER_CRITICAL(&dispatchMux);
    gnssConsecCrashCount = 0;
    taskEXIT_CRITICAL(&dispatchMux);

    ESP_LOGI(TAG, "GPS: lat=%.6f lon=%.6f sats=%d spd=%.1f km/h",
             gps.latitude, gps.longitude, gps.satellites, gps.speed_kmh);
    return ESP_OK;
}

// ─── Red / PDP ────────────────────────────────────────────────────────────────

/**
 * @brief Espera hasta maxSecs segundos confirmando que NETOPEN esté activo.
 *
 * @details
 * ### Propósito
 * AT+NETOPEN responde "OK" inmediatamente pero el stack tarda varios segundos
 * en quedar completamente listo.  Esta función espera hasta esa confirmación
 * usando dos mecanismos complementarios.
 *
 * ### Flujo (cada 2 s, hasta maxSecs/2 ciclos)
 * 1. Chequear netStackOpen (flag actualizado por handleUrcLine vía +CGEV).
 * 2. Si no → AT+NETOPEN? y verificar "+NETOPEN: 1" en respuesta.
 *
 * ### Por qué doble chequeo
 * El URC "+CGEV: EPS PDN ACT" llega asíncronamente y puede llegar antes de
 * que el polling ocurra.  Combinar ambos mecanismos evita esperas innecesarias.
 *
 * @param maxSecs Segundos máximos de espera (se revisa cada 2 s).
 * @return true si NETOPEN fue confirmado dentro del timeout.
 */
bool A7670Driver::pollNetOpenReady(int maxSecs) {
    int cycles = maxSecs / 2;
    for (int i = 0; i < cycles; i++) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Leer netStackOpen bajo spinlock: handleUrcLine puede modificarlo concurrentemente
        taskENTER_CRITICAL(&dispatchMux);
        bool byUrc = netStackOpen;
        taskEXIT_CRITICAL(&dispatchMux);
        if (byUrc) {
            ESP_LOGI(TAG, "NETOPEN: abierto (URC dispatcher, iter=%d)", i + 1);
            return true;
        }

        // Fallback polling: algunos firmwares no emiten +CGEV correctamente
        if (sendATLocked("AT+NETOPEN?", "+NETOPEN:", A7670_TIMEOUT_SHORT_MS, true) &&
            strstr(rxBuf, "+NETOPEN: 1") != nullptr) {
            ESP_LOGI(TAG, "NETOPEN: abierto (query, iter=%d)", i + 1);
            return true;
        }
    }
    return false;
}

/**
 * @brief Cierra y reabre el stack TCP/IP (NETCLOSE → NETOPEN).
 *
 * @details
 * ### Propósito
 * Restaurar el stack de red cuando NETOPEN falló, la conexión se cayó,
 * o PDP fue deactivado por la red.
 *
 * ### Flujo
 * 1. NETCLOSE (cerrar stack si estaba abierto).
 * 2. NETOPEN + pollNetOpenReady(30 s).
 * 3. Si éxito: AT+IPADDR para loguear la IP asignada.
 * 4. Fallback: reset PDP (CGACT=0 → CGACT=1) + segundo intento NETOPEN.
 *
 * ### Por qué el fallback con CGACT
 * La red puede desactivar el PDP context (bearer) sin notificar con URC,
 * especialmente en redes LTE con inactividad.  CGACT=0/1 fuerza renegociación.
 *
 * @return true si el stack quedó abierto.
 */
bool A7670Driver::resetNetStack() {
    netStackOpen = false;   // Invalidar antes del reset para evitar lecturas stale
    ESP_LOGW(TAG, "Reset stack de red: NETCLOSE -> NETOPEN");

    sendATLocked("AT+NETCLOSE", "+NETCLOSE:", A7670_TIMEOUT_MEDIUM_MS, true);
    vTaskDelay(pdMS_TO_TICKS(1000));    // El módulo necesita un ciclo para liberar recursos internos
    sendATLocked("AT+NETOPEN", "OK", A7670_TIMEOUT_MEDIUM_MS, true);

    if (pollNetOpenReady(30)) {
        netStackOpen = true;
        if (sendATLocked("AT+IPADDR", "+IPADDR:", A7670_TIMEOUT_SHORT_MS, true)) {
            ESP_LOGI(TAG, "IP asignada: %.48s", rxBuf);
        }
        return true;
    }

    // Fallback: reset PDP context (sin CGATT) y segundo intento
    ESP_LOGW(TAG, "NETOPEN timeout — reset PDP y reintento");
    sendATLocked("AT+CGACT=0,1", "OK", A7670_TIMEOUT_MEDIUM_MS, true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    sendATLocked("AT+CGACT=1,1", "OK", A7670_TIMEOUT_MEDIUM_MS, true);
    vTaskDelay(pdMS_TO_TICKS(3000));    // PDP tarda ~3 s en activarse con la red
    sendATLocked("AT+NETOPEN", "OK", A7670_TIMEOUT_MEDIUM_MS, true);

    if (pollNetOpenReady(30)) {
        netStackOpen = true;
        if (sendATLocked("AT+IPADDR", "+IPADDR:", A7670_TIMEOUT_SHORT_MS, true)) {
            ESP_LOGI(TAG, "IP asignada: %.48s", rxBuf);
        }
        return true;
    }

    ESP_LOGE(TAG, "No se pudo abrir el stack TCP/IP");
    return false;
}

/**
 * @brief Garantiza que el stack TCP/IP esté abierto antes de un tcpSend.
 *
 * @details
 * ### Propósito
 * Guard clause que evita duplicar la lógica de apertura de red en tcpSend.
 * Si el flag netStackOpen ya es true (estado normal), retorna inmediatamente.
 *
 * @return true si el stack está disponible.
 */
bool A7670Driver::ensureNetOpen() {
    if (netStackOpen) return true;  // Camino rápido: sin comandos AT si ya está abierto
    ESP_LOGW(TAG, "Stack de red cerrado; reabriendo");
    return resetNetStack();
}

/**
 * @brief Teardown y reset completo del PDP context y el stack TCP.
 *
 * @details
 * ### Propósito
 * Recuperación más agresiva que resetNetStack(): baja también el PDP bearer
 * (capa de datos celular) antes de reabrir NETOPEN.  Usada cuando CGACT
 * falló o el módulo reportó PDN DEACT.
 *
 * ### Flujo
 * NETCLOSE → CGACT=0,1 (desactivar PDP) → CGACT=1,1 (reactivar) → resetNetStack().
 *
 * @return true si el stack quedó operativo después del reset.
 */
bool A7670Driver::resetPdp() {
    ESP_LOGW(TAG, "Reset PDP: NETCLOSE -> CGACT=0 -> CGACT=1 -> NETOPEN");
    netStackOpen = false;

    sendATLocked("AT+NETCLOSE", "+NETCLOSE:", A7670_TIMEOUT_MEDIUM_MS, true);
    vTaskDelay(pdMS_TO_TICKS(1000));
    sendATLocked("AT+CGACT=0,1", "OK", A7670_TIMEOUT_MEDIUM_MS, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (!sendATLocked("AT+CGACT=1,1", "OK", A7670_TIMEOUT_MEDIUM_MS)) {
        ESP_LOGE(TAG, "No se pudo reactivar PDP");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    return resetNetStack();
}

/**
 * @brief Inicializa el contexto de datos APN + PDP + NETOPEN (hasta 3 intentos).
 *
 * @details
 * ### Propósito
 * Configurar la capa de datos celular completa desde cero: APN → adjunto →
 * activación PDP → apertura del stack TCP/IP.
 *
 * ### Flujo por intento
 * 1. AT+CGDCONT=1,"IP","<APN>" — declara el PDP context con el APN del operador.
 * 2. AT+CGATT? — verificar si ya está adjunto; si no: AT+CGATT=1.
 * 3. AT+CGACT? — verificar si el PDP está activo; si no: AT+CGACT=1,1.
 * 4. resetNetStack() — abrir NETOPEN.
 *
 * ### Por qué 3 intentos
 * La red puede rechazar CGATT o CGACT de forma transitoria (paging, handover).
 * 3 intentos con delays cubren la mayoría de los casos transitorios.
 *
 * ARQUITECTURA: El APN (ARGUS_APN) esta hardcoded en tiempo de compilacion.
 *    Dispositivos en redes con APN diferente requieren re-compilacion.
 *    Mejora: leer APN desde NVS configurado via BLE.
 *
 * @return true si el contexto quedó listo (state = NET_READY).
 */
bool A7670Driver::initDataContext() {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return false;

    netStackOpen = false;

    for (int i = 1; i <= 3; i++) {
        ESP_LOGI(TAG, "Inicializando contexto de datos (%d/3)", i);

        char cgdcont[64];
        // APN hardcoded: define qué red de datos usa el operador para este SIM
        snprintf(cgdcont, sizeof(cgdcont), "AT+CGDCONT=1,\"IP\",\"%s\"", ARGUS_APN);
        if (!sendATLocked(cgdcont, "OK", A7670_TIMEOUT_SHORT_MS)) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (!sendATLocked("AT+CGATT?", "+CGATT:", A7670_TIMEOUT_SHORT_MS) ||
            strstr(rxBuf, "+CGATT: 1") == nullptr) {
            ESP_LOGI(TAG, "No adjunto; enviando AT+CGATT=1");
            if (!sendATLocked("AT+CGATT=1", "OK", A7670_TIMEOUT_MEDIUM_MS)) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(5000)); // Adjunto puede tardar hasta 5 s en redes congestionadas
        }

        if (!sendATLocked("AT+CGACT?", "+CGACT:", A7670_TIMEOUT_SHORT_MS) ||
            strstr(rxBuf, "+CGACT: 1,1") == nullptr) {
            ESP_LOGI(TAG, "PDP no activo; enviando AT+CGACT=1,1");
            if (!sendATLocked("AT+CGACT=1,1", "OK", A7670_TIMEOUT_MEDIUM_MS)) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(3000));
        }

        if (resetNetStack()) {
            ESP_LOGI(TAG, "Contexto de datos listo (APN=%s)", ARGUS_APN);
            if (sendATLocked("AT+CNUM", "+CNUM:", A7670_TIMEOUT_SHORT_MS, true)) {
                ESP_LOGW(TAG, ">>> NUMERO SIM: %s <<<", rxBuf);
            }
            state = MODEM_STATE_NET_READY;
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGE(TAG, "No se pudo configurar el contexto de datos");
    return false;
}

/**
 * @brief Verifica que la red esté adjunta y el PDP context activo.
 *
 * @details
 * ### Propósito
 * Health-check rápido de la capa de datos antes de operar.  Más liviano que
 * initDataContext() porque no reconfigura CGDCONT si ya está listo.
 *
 * ### Flujo
 * 1. AT+CGATT? (3 intentos) → "+CGATT: 1".
 * 2. AT+CGACT? → "+CGACT: 1,1".
 *
 * @return true si la red está lista para NETOPEN.
 */
bool A7670Driver::checkNetworkReady() {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return false;

    bool attached = false;
    for (int i = 0; i < 3; i++) {
        if (sendATLocked("AT+CGATT?", "+CGATT:", A7670_TIMEOUT_SHORT_MS, true) &&
            strstr(rxBuf, "+CGATT: 1") != nullptr) {
            attached = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    if (!attached) { ESP_LOGE(TAG, "Red no lista: no adjunto"); return false; }

    if (!sendATLocked("AT+CGACT?", "+CGACT:", A7670_TIMEOUT_SHORT_MS, true) ||
        strstr(rxBuf, "+CGACT: 1,1") == nullptr) {
        ESP_LOGE(TAG, "Red no lista: PDP inactivo");
        return false;
    }

    ESP_LOGI(TAG, "NETWORK READY");
    state = MODEM_STATE_NET_READY;
    return true;
}

// ─── TCP send ────────────────────────────────────────────────────────────────

/**
 * @brief Envía un paquete de texto al servidor backend vía TCP.
 *
 * @details
 * ### Propósito
 * Transmitir una trama del Protocolo Argus al servidor.  Abre el socket,
 * envía el payload y cierra el socket en cada llamada.
 *
 * ### Flujo
 * 1. Tomar xModemMutex.
 * 2. maintain() — procesar URCs pendientes antes de enviar.
 * 3. ensureNetOpen() — abrir NETOPEN si estaba cerrado.
 * 4. AT+CIPCLOSE=0 — limpiar socket residual de envío anterior.
 * 5. Armar URC waiter para "+CIPOPEN:" ANTES del AT+CIPOPEN (anti-race).
 * 6. AT+CIPOPEN=0,"TCP",host,port.
 * 7. xSemaphoreTake(waitUrcSem, 15 s) — esperar confirmación de conexión.
 * 8. Parsear "+CIPOPEN: idx,code" → code==0 significa conectado.
 * 9. AT+CIPSEND=0,len — anunciar tamaño al módulo.
 * 10. beginDispatch con expectPrompt=true — esperar ">".
 * 11. Escribir bytes del paquete al UART.
 * 12. waitForExpectedLineLocked "OK" 10 s — confirmación de envío.
 * 13. AT+CIPCLOSE=0 — cerrar socket.
 *
 * ### Por qué armar el URC waiter ANTES de AT+CIPOPEN
 * Si el módulo responde muy rápido (≤1 ms), el URC "+CIPOPEN:" puede llegar
 * y ser procesado por urcHandlerTask antes de que xSemaphoreTake se ejecute.
 * Con el waiter armado primero, handleParsedLine() llama xSemaphoreGive()
 * y el Take posterior encuentra el semáforo disponible inmediatamente.
 *
 * ### Por qué socket por trama (no conexión persistente)
 * ARQUITECTURA: El servidor cierra la conexion tras recibir cada trama.
 *    Mantener una conexion persistente ahoraria ~2-4 s de latencia por envio.
 *    Requiere soporte de keep-alive en el servidor.
 *
 * ### Por qué "OK" y no "SEND OK"
 * El A7670 responde "OK" + "+CIPSEND: 0,N,N" al enviar.  El SIM800/SIM900
 * respondía "SEND OK".  Usar "SEND OK" como expected causaría timeout siempre.
 *
 * @param packet  Cadena null-terminated a enviar (formato Argus Protocol).
 * @return ESP_OK si el paquete fue enviado y confirmado.
 */
esp_err_t A7670Driver::tcpSend(const char* packet) {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return ESP_FAIL;
    if (packet == nullptr || packet[0] == '\0') return ESP_ERR_INVALID_ARG;

    maintain();  // Drenar URCs acumulados antes de interactuar con el módulo

    if (!ensureNetOpen()) {
        ESP_LOGE(TAG, "[TCP] Red no disponible");
        return ESP_FAIL;
    }

    // Abrir socket solo si no hay conexión persistente activa.
    taskENTER_CRITICAL(&dispatchMux);
    bool alreadyConnected = _tcpConnected;
    taskEXIT_CRITICAL(&dispatchMux);

    if (!alreadyConnected) {
        // Limpiar socket residual por si el envío anterior abortó antes del CIPCLOSE.
        sendATLocked("AT+CIPCLOSE=0", "OK", A7670_TIMEOUT_SHORT_MS, true);
        vTaskDelay(pdMS_TO_TICKS(300));

        // Armar captura de URC +CIPOPEN: ANTES de enviar CIPOPEN para evitar race condition
        // (el URC puede llegar antes de que xSemaphoreTake se ejecute)
        taskENTER_CRITICAL(&dispatchMux);
        waitUrcActive = true;
        strncpy(waitUrcPrefix, "+CIPOPEN:", sizeof(waitUrcPrefix) - 1);
        waitUrcPrefix[sizeof(waitUrcPrefix) - 1] = '\0';
        waitUrcResult[0] = '\0';
        xSemaphoreTake(waitUrcSem, 0);
        taskEXIT_CRITICAL(&dispatchMux);

        char cipCmd[96];
        snprintf(cipCmd, sizeof(cipCmd), "AT+CIPOPEN=0,\"TCP\",\"%s\",%d",
                 TCP_SERVER_HOST, TCP_SERVER_PORT);
        sendATLocked(cipCmd, "OK", A7670_TIMEOUT_SHORT_MS, true);

        bool connected = false;
        if (xSemaphoreTake(waitUrcSem, pdMS_TO_TICKS(15000)) == pdTRUE) {
            taskENTER_CRITICAL(&dispatchMux);
            int idx = -1, code = -1;
            sscanf(waitUrcResult, "+CIPOPEN: %d,%d", &idx, &code);
            waitUrcActive = false;
            taskEXIT_CRITICAL(&dispatchMux);
            connected = (code == 0);
        } else {
            taskENTER_CRITICAL(&dispatchMux);
            waitUrcActive = false;
            taskEXIT_CRITICAL(&dispatchMux);
            ESP_LOGE(TAG, "[TCP] Timeout abriendo conexion al servidor");
            return ESP_FAIL;
        }

        if (!connected) {
            ESP_LOGE(TAG, "[TCP] Servidor no aceptó la conexion");
            return ESP_FAIL;
        }

        taskENTER_CRITICAL(&dispatchMux);
        _tcpConnected = true;
        taskEXIT_CRITICAL(&dispatchMux);
        ESP_LOGI(TAG, "[TCP] Socket persistente abierto");
    }

    // CIPSEND anuncia cuántos bytes se van a enviar; el módulo responde con ">"
    size_t pktLen = strlen(packet);
    char sendCmd[32];
    snprintf(sendCmd, sizeof(sendCmd), "AT+CIPSEND=0,%zu", pktLen);

    clearRxBuf();
    beginDispatch(sendCmd, ">", false, true, false);
    char fullCmd[48];
    snprintf(fullCmd, sizeof(fullCmd), "%s\r\n", sendCmd);
    ESP_LOGI(TAG, "[AT CMD] %s", sendCmd);
    uart_write_bytes(A7670_UART_PORT, fullCmd, strlen(fullCmd));

    if (!waitForExpectedLineLocked(">", A7670_TIMEOUT_SHORT_MS, false, false)) {
        endDispatch();
        ESP_LOGE(TAG, "[TCP] No llegó prompt '>' — socket posiblemente cerrado");
        taskENTER_CRITICAL(&dispatchMux);
        _tcpConnected = false;
        taskEXIT_CRITICAL(&dispatchMux);
        sendATLocked("AT+CIPCLOSE=0", "OK", A7670_TIMEOUT_SHORT_MS, true);
        return ESP_FAIL;
    }

    // Escribir bytes del payload directamente (sin '\r\n'; el módulo ya sabe el tamaño)
    uart_write_bytes(A7670_UART_PORT, packet, pktLen);

    // A7670 responde "OK" + "+CIPSEND: 0,N,N" — no "SEND OK" como SIM800
    bool sent = waitForExpectedLineLocked("OK", 10000, true, false);
    endDispatch();

    if (!sent) {
        ESP_LOGE(TAG, "[TCP] SEND FAIL — cerrando socket para reconectar en proximo ciclo");
        taskENTER_CRITICAL(&dispatchMux);
        _tcpConnected = false;
        taskEXIT_CRITICAL(&dispatchMux);
        sendATLocked("AT+CIPCLOSE=0", "OK", A7670_TIMEOUT_SHORT_MS, true);
        return ESP_FAIL;
    }

    // Leer ACK del servidor + CMD opcional (ARM, DISARM, etc.)
    tryReadServerCommand();

    ESP_LOGI(TAG, "[TCP] Paquete enviado: %.*s", (int)(pktLen > 80 ? 80 : pktLen), packet);
    return ESP_OK;
}

// ─── tryReadServerCommand ─────────────────────────────────────────────────────

void A7670Driver::tryReadServerCommand() {
    vTaskDelay(pdMS_TO_TICKS(200));  // tiempo para que el servidor escriba ACK + CMD

    // 128 bytes: suficiente para ACK\r\n + hasta 3 comandos (ARM+ENGINE_CUT+SIREN, ~15B c/u)
    static const char* READ_CMD = "AT+CIPRXGET=2,0,128\r\n";
    clearRxBuf();
    beginDispatch("AT+CIPRXGET=2,0,128", "+CIPRXGET:", false, false, false);
    uart_write_bytes(A7670_UART_PORT, READ_CMD, strlen(READ_CMD));
    bool gotResp = waitForExpectedLineLocked("OK", 1500, false, true);
    endDispatch();

    if (!gotResp) return;

    // rxBuf = "+CIPRXGET: 2,0,<actual>,<rem>\r\nACK\r\n[CMD|...\r\n][CMD|...\r\n]OK\r\n"
    const char* hdr = strstr(rxBuf, "+CIPRXGET: 2,0,");
    if (!hdr) return;

    int actual = 0, rem = 0;
    if (sscanf(hdr, "+CIPRXGET: 2,0,%d,%d", &actual, &rem) != 2 || actual <= 0) return;
    // Si quedan bytes en el buffer del modem (rem>0), re-armar el flag para que
    // pollForCommands() los lea en el próximo tick. El A7670 no siempre genera un
    // URC adicional cuando hay datos pendientes tras una lectura parcial.
    if (rem > 0) {
        taskENTER_CRITICAL(&dispatchMux);
        _hasPendingRx = true;
        taskEXIT_CRITICAL(&dispatchMux);
    }

    const char* nl = strstr(hdr, "\r\n");
    if (!nl) return;
    const char* data = nl + 2;

    // Saltar línea ACK o ERR (primera respuesta del servidor, antes de los CMD|)
    if (strncmp(data, "ACK", 3) == 0 || strncmp(data, "ERR", 3) == 0) {
        const char* after = strstr(data, "\r\n");
        if (!after) return;
        data = after + 2;
    }

    // Parsear TODOS los CMD| del bloque en variables locales (sin sección crítica aún).
    // Esto soluciona el caso donde ARM y ENGINE_CUT llegan en el mismo bloque TCP:
    // el A7670 genera un solo URC +CIPRXGET:1,0 para ambos → antes solo se leía ARM.
    char   parsed[CMD_QUEUE_SIZE][64];
    uint8_t parsedCount = 0;

    while (*data != '\0' && strncmp(data, "OK", 2) != 0 && parsedCount < CMD_QUEUE_SIZE) {
        if (strncmp(data, "CMD|", 4) != 0) {
            const char* skip = strstr(data, "\r\n");
            if (!skip) skip = strchr(data, '\n');
            if (!skip) break;
            data = skip + (strncmp(skip, "\r\n", 2) == 0 ? 2 : 1);
            continue;
        }
        const char* end_nl = strstr(data, "\r\n");
        if (!end_nl) end_nl = strchr(data, '\n');
        size_t cmdlen = end_nl ? (size_t)(end_nl - data) : strnlen(data, 63);
        if (cmdlen == 0 || cmdlen >= 64) break;

        memcpy(parsed[parsedCount], data, cmdlen);
        parsed[parsedCount][cmdlen] = '\0';
        parsedCount++;

        if (!end_nl) break;
        data = end_nl + (strncmp(end_nl, "\r\n", 2) == 0 ? 2 : 1);
    }

    // Encolar bajo sección crítica — sin logging aquí (printf dentro de critical = crash)
    taskENTER_CRITICAL(&dispatchMux);
    for (uint8_t i = 0; i < parsedCount; i++) {
        if (_cmdQueueCount < CMD_QUEUE_SIZE) {
            uint8_t slot = (_cmdQueueHead + _cmdQueueCount) % CMD_QUEUE_SIZE;
            memcpy(_cmdQueue[slot], parsed[i], 64);
            _cmdQueueCount++;
            _hasPendingCmd = true;
        }
    }
    taskEXIT_CRITICAL(&dispatchMux);

    // Logging fuera de sección crítica
    for (uint8_t i = 0; i < parsedCount; i++) {
        ESP_LOGI(TAG, "[CMD] Backend → '%s'", parsed[i]);
    }
    if (parsedCount == 0) {
        ESP_LOGD(TAG, "[CMD] Bloque TCP sin comandos CMD|");
    }
}

// ─── readLastServerCommand ────────────────────────────────────────────────────

bool A7670Driver::readLastServerCommand(char* buf, size_t size) {
    taskENTER_CRITICAL(&dispatchMux);
    bool has = (_cmdQueueCount > 0);
    if (has) {
        strncpy(buf, _cmdQueue[_cmdQueueHead], size - 1);
        buf[size - 1] = '\0';
        _cmdQueueHead = (_cmdQueueHead + 1) % CMD_QUEUE_SIZE;
        _cmdQueueCount--;
        _hasPendingCmd = (_cmdQueueCount > 0);
    }
    taskEXIT_CRITICAL(&dispatchMux);
    return has;
}

/**
 * @brief Sondea el socket TCP por comandos del servidor disparado por URC +CIPRXGET: 1,0.
 *
 * PROPÓSITO:
 *   Permite que commTask detecte ARM/DISARM en ~1s en lugar de esperar hasta el
 *   próximo ciclo GPS (hasta 30s en PREMIUM IDLE). Solo actúa cuando el A7670
 *   emitió +CIPRXGET: 1,0 (flag _hasPendingRx=true), por lo que no genera
 *   ningún overhead cuando no hay comandos pendientes del servidor.
 *
 * FLUJO:
 *   1. Leer y limpiar _hasPendingRx bajo spinlock (costo ~0 si es false).
 *   2. Si false → retornar false de inmediato (sin mutex, sin UART, sin datos celulares).
 *   3. Si true → intentar xModemMutex con timeout de 200ms.
 *   4. Si no lo obtiene (tcpSend activo) → restaurar flag, retornar false.
 *      tryReadServerCommand() se llamará igualmente al final de tcpSend().
 *   5. Verificar que el socket sigue conectado.
 *   6. tryReadServerCommand() → lee AT+CIPRXGET=2,0,128 → encola todos los CMD| en _cmdQueue.
 *
 * @return true si se ejecutó tryReadServerCommand(); false si no había datos o mutex ocupado.
 */
bool A7670Driver::pollForCommands() {
    // Paso 1: leer _hasPendingRx y evaluar force-poll bajo un solo spinlock.
    // Force-poll cada 5s cuando el socket está conectado: el A7670 (documentado SIMCom)
    // solo genera +CIPRXGET:1,0 cuando llegan datos a un buffer VACÍO. Si el buffer ya tenía
    // datos de lectura parcial (rem>0 ignorado), o si el modem dejó de emitir URCs tras
    // las primeras lecturas por sesión, los comandos quedan atrapados en el buffer del modem.
    // El force-poll garantiza que el buffer se drena aunque el URC nunca llegue.
    taskENTER_CRITICAL(&dispatchMux);
    bool hasPending = _hasPendingRx;
    if (hasPending) _hasPendingRx = false;
    const uint64_t nowUs  = esp_timer_get_time();
    const bool forceCheck = !hasPending && _tcpConnected &&
                            (nowUs - _lastCmdPollUs) >= 5000000ULL;
    if (forceCheck) _lastCmdPollUs = nowUs;
    taskEXIT_CRITICAL(&dispatchMux);

    if (!hasPending && !forceCheck) return false;

    // Paso 2: intentar tomar el mutex sin bloquear demasiado.
    // Si tcpSend() lo tiene, tryReadServerCommand() ya correrá al final de ese envío.
    ModemLockGuard lock(xModemMutex, pdMS_TO_TICKS(200));
    if (!lock.locked()) {
        // Restaurar solo si era URC real (el force-poll se reintenta al próximo tick)
        if (hasPending) {
            taskENTER_CRITICAL(&dispatchMux);
            _hasPendingRx = true;
            taskEXIT_CRITICAL(&dispatchMux);
        }
        return false;
    }

    // Paso 3: verificar que el socket siga activo
    taskENTER_CRITICAL(&dispatchMux);
    bool connected = _tcpConnected;
    taskEXIT_CRITICAL(&dispatchMux);

    if (!connected) return false;

    // Paso 4: leer datos del buffer del A7670 y parsear el comando
    if (hasPending) {
        ESP_LOGI(TAG, "[pollForCommands] URC recibido — leyendo datos del servidor");
    } else {
        ESP_LOGD(TAG, "[pollForCommands] Force-poll 5s — verificando buffer TCP");
    }
    tryReadServerCommand();
    return true;
}

// ─── maintain ─────────────────────────────────────────────────────────────────

/**
 * @brief Drena y procesa todos los URCs pendientes en urcQueue.
 *
 * @details
 * ### Propósito
 * Evitar que urcQueue se llene mientras el driver está ocupado en un sendATLocked
 * largo.  Se llama antes de cada interacción con el módulo para procesar
 * actualizaciones de estado (netStackOpen, señal, registro) antes de actuar.
 *
 * ### Por qué timeout=0
 * maintain() debe ser no-bloqueante: se llama desde funciones que ya poseen
 * xModemMutex y no deben ceder el scheduler (riesgo de inversión de prioridad).
 */
void A7670Driver::maintain() {
    if (urcQueue == nullptr) return;
    ModemLine_t line = {};
    // Drenar hasta vaciar la cola sin esperar: timeout=0
    while (xQueueReceive(urcQueue, &line, 0) == pdTRUE) {
        handleUrcLine(line.text);
    }
}

// ─── AT infrastructure ────────────────────────────────────────────────────────

/**
 * @brief Wrapper público de sendATLocked() con adquisición de xModemMutex.
 *
 * @details
 * ### Propósito
 * Permitir que código externo (ej. diagnóstico) envíe comandos AT sin acceder
 * al mutex directamente.  La mayoría de métodos internos usa sendATLocked().
 *
 * @param command      Comando AT sin CR/LF (ej. "AT+CSQ").
 * @param expectedResp Subcadena esperada en la respuesta.
 * @param timeoutMs    Timeout en milisegundos.
 * @param silent       Si true, suprime logs de error.
 * @return true si la respuesta esperada fue recibida antes del timeout.
 */
bool A7670Driver::sendAT(const char* command, const char* expectedResp,
                         uint32_t timeoutMs, bool silent) {
    ModemLockGuard lock(xModemMutex);
    if (!lock.locked()) return false;
    return sendATLocked(command, expectedResp, timeoutMs, silent);
}

/**
 * @brief Envía un comando AT y espera una respuesta específica.
 *
 * @details
 * ### Propósito
 * Núcleo de toda comunicación AT: enviar → esperar respuesta → retornar éxito/fallo.
 * Debe llamarse solo con xModemMutex ya tomado.
 *
 * ### Flujo
 * 1. maintain() — procesar URCs pendientes antes de limpiar rxBuf.
 * 2. clearRxBuf() — borrar respuesta anterior.
 * 3. beginDispatch() — armar DispatchSession_t para routing.
 * 4. Escribir "command\r\n" al UART.
 * 5. waitForExpectedLineLocked() — esperar expected dentro de timeoutMs.
 * 6. Si falla y no es "OK"/">" → late capture retry (5 s extra):
 *    el módulo puede responder tarde en condiciones de carga.
 * 7. endDispatch() — limpiar DispatchSession_t.
 *
 * ### Por qué late capture
 * En condiciones de RF intensa, el módulo puede procesar comandos AT más
 * lento de lo normal.  5 s de captura tardía recupera esas respuestas sin
 * aumentar el timeout nominal de todos los comandos.
 *
 * @param command      Comando AT sin CR/LF.
 * @param expectedResp Subcadena esperada (ej. "+CSQ:", "OK", ">").
 * @param timeoutMs    Timeout principal en ms.
 * @param silent       Si true, suprime logs de error.
 * @return true si la respuesta esperada fue recibida.
 *
 * @note No toma xModemMutex: debe ser llamado con el mutex ya tomado.
 */
bool A7670Driver::sendATLocked(const char* command, const char* expectedResp,
                               uint32_t timeoutMs, bool silent) {
    if (command == nullptr || expectedResp == nullptr || atResponseQueue == nullptr) return false;

    const bool expectPrompt = (strcmp(expectedResp, ">") == 0);
    const bool waitForOk    = (strcmp(expectedResp, "OK") == 0);

    maintain();     // Procesar URCs antes de enviar para no leer respuestas viejas
    clearRxBuf();   // Limpiar acumulador de respuesta
    beginDispatch(command, expectedResp, waitForOk, expectPrompt, false);

    char fullCmd[256];
    snprintf(fullCmd, sizeof(fullCmd), "%s\r\n", command);
    ESP_LOGI(TAG, "[AT CMD] %s", command);
    uart_write_bytes(A7670_UART_PORT, fullCmd, strlen(fullCmd));

    bool ok = waitForExpectedLineLocked(expectedResp, timeoutMs, waitForOk, silent);

    // Late capture: si el expected no es "OK" ni ">" puede llegar con delay
    if (!ok && !waitForOk && !expectPrompt) {
        ok = waitForExpectedLineLocked(expectedResp, MODEM_LATE_CAPTURE_MS, waitForOk, true);
        if (ok) ESP_LOGW(TAG, "[AT] respuesta tardía capturada para [%s]", expectedResp);
    }

    endDispatch();
    if (!ok && !silent) ESP_LOGE(TAG, "[ERROR] Comando fallido: %s", command);
    return ok;
}

/**
 * @brief Loop de recepción con deadline: lee atResponseQueue hasta encontrar 'expected'.
 *
 * @details
 * ### Propósito
 * Abstraer la espera de una respuesta AT específica sobre la cola de líneas
 * parseadas.  Maneja "OK", ">", errores y líneas de datos de forma unificada.
 *
 * ### Flujo
 * - Calcula deadlineUs = now + timeoutMs × 1000.
 * - Loop mientras now < deadline:
 *   - Calcular remainingUs → waitMs (máximo 100 ms para no dormir más allá del deadline).
 *   - xQueueReceive(atResponseQueue, waitMs).
 *   - appendRxLine() → acumular en rxBuf.
 *   - "OK" → sawOk=true; si expected=="OK" → matched=true.
 *   - ">" → si expected==">" → matched=true; si !waitForOk → retornar true.
 *   - isErrorLine() → si no había "OK" aún → retornar false.
 *   - strstr(line, expected) → matched=true.
 *   - Si matched && (!waitForOk || sawOk) → retornar true.
 *
 * ### Por qué chunks de 100 ms
 * xQueueReceive con timeoutMs grande dormiría más allá del deadline.
 * Con chunks de 100 ms el error máximo de deadline es 100 ms.
 *
 * ### Por qué waitForOk
 * Algunos comandos devuelven datos ANTES de "OK" (ej. AT+CSQ → "+CSQ: 18,0" → "OK").
 * waitForOk=true requiere ver "OK" además de la línea de datos.
 *
 * @param expected    Subcadena a buscar en cada línea.
 * @param timeoutMs   Timeout total en ms.
 * @param waitForOk   Si true, requiere también ver "OK" después de 'expected'.
 * @param silent      Si true, suprime logs de timeout.
 * @return true si 'expected' fue encontrado (y "OK" si waitForOk).
 */
bool A7670Driver::waitForExpectedLineLocked(const char* expected, uint32_t timeoutMs,
                                            bool waitForOk, bool silent) {
    if (atResponseQueue == nullptr) return false;

    bool    matched  = false;
    bool    sawOk    = false;
    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    ModemLine_t line = {};

    while (esp_timer_get_time() < deadlineUs) {
        int64_t  remainingUs = deadlineUs - esp_timer_get_time();
        uint32_t waitMs      = (remainingUs <= 0) ? 1U : (uint32_t)(remainingUs / 1000U);
        // Cap de 100 ms para evitar dormir más allá del deadline
        if (waitMs > 100U) waitMs = 100U;
        if (waitMs == 0U) waitMs = 1U;

        if (xQueueReceive(atResponseQueue, &line, pdMS_TO_TICKS(waitMs)) != pdTRUE) continue;

        appendRxLine(line.text);    // Acumular en rxBuf para post-procesamiento del caller
        if (!silent) ESP_LOGI(TAG, "[AT RSP] %s", line.text);

        if (strcmp(line.text, "OK") == 0) {
            sawOk = true;
            if (strcmp(expected, "OK") == 0) matched = true;
        } else if (strcmp(line.text, ">") == 0) {
            if (strcmp(expected, ">") == 0) matched = true;
            // ">" no va seguido de "OK"; retornar inmediatamente si es el expected
            if (matched && !waitForOk) return true;
        } else if (isErrorLine(line.text)) {
            if (!silent) ESP_LOGE(TAG, "[ERROR] Módem respondió error: %s", line.text);
            // Si ya vimos "OK" y no necesitamos "OK" post-data → no es un error fatal
            if (!sawOk || waitForOk) return false;
        } else if (strstr(line.text, expected) != nullptr) {
            matched = true;
        }

        if (matched && (!waitForOk || sawOk)) return true;
    }

    if (!silent) ESP_LOGE(TAG, "[ERROR] Timeout esperando [%s]", expected);
    return false;
}

/**
 * @brief Espera un URC específico en atResponseQueue sin enviar ningún comando.
 *
 * @details
 * ### Propósito
 * Esperar líneas espontáneas del módulo (ej. "+CGNSSPWR: READY!") que llegan
 * sin un comando AT precedente.
 *
 * ### Flujo
 * 1. purgeQueue(atResponseQueue) → descartar respuestas anteriores.
 * 2. clearRxBuf().
 * 3. beginDispatch(nullptr, expected, captureOnlyExpected=true) → solo la línea esperada.
 * 4. waitForExpectedLineLocked().
 * 5. endDispatch().
 *
 * @param expected   Subcadena esperada en la línea URC.
 * @param timeoutMs  Timeout en ms.
 * @param silent     Si true, suprime logs de timeout.
 * @return true si la línea llegó antes del timeout.
 */
bool A7670Driver::waitForLineLocked(const char* expected, uint32_t timeoutMs, bool silent) {
    if (expected == nullptr || atResponseQueue == nullptr) return false;
    purgeQueue(atResponseQueue);    // Limpiar respuestas de comandos anteriores
    clearRxBuf();
    // captureOnlyExpected=true: ignorar todas las líneas excepto la esperada
    beginDispatch(nullptr, expected, false, false, true);
    bool ok = waitForExpectedLineLocked(expected, timeoutMs, false, silent);
    endDispatch();
    return ok;
}

/**
 * @brief Arma el mecanismo de espera puntual de URC por prefijo vía semáforo.
 *
 * @details
 * ### Propósito
 * Mecanismo alternativo para esperar URCs que llegan de forma completamente
 * asíncrona (fuera de una sesión AT activa).  Usado en tcpSend() para
 * "+CIPOPEN:".
 *
 * ### Flujo
 * 1. Bajo spinlock: marcar waitUrcActive=true, copiar prefijo, vaciar semáforo.
 * 2. xSemaphoreTake(waitUrcSem, timeoutMs).
 * 3. Si éxito: copiar waitUrcResult a rxBuf si copyToRxBuf.
 * 4. Siempre: limpiar waitUrcActive bajo spinlock.
 *
 * ### Diferencia con waitForLineLocked
 * waitForLineLocked usa atResponseQueue (para URCs que el dispatcher redirige
 * ahí).  waitForUrcPrefix usa handleParsedLine() → xSemaphoreGive() para URCs
 * que llegan mientras hay una sesión AT activa (y no serían enrutados a la cola).
 *
 * @param prefix       Prefijo del URC a esperar (ej. "+CIPOPEN:").
 * @param timeoutMs    Timeout en ms.
 * @param copyToRxBuf  Si true, copia la línea URC a rxBuf para parseo posterior.
 * @return true si el URC llegó antes del timeout.
 */
bool A7670Driver::waitForUrcPrefix(const char* prefix, uint32_t timeoutMs, bool copyToRxBuf) {
    if (prefix == nullptr || waitUrcSem == nullptr) return false;

    taskENTER_CRITICAL(&dispatchMux);
    waitUrcActive = true;
    strncpy(waitUrcPrefix, prefix, sizeof(waitUrcPrefix) - 1);
    waitUrcPrefix[sizeof(waitUrcPrefix) - 1] = '\0';
    waitUrcResult[0] = '\0';
    // Vaciar semáforo residual para que el Take siguiente sea limpio
    xSemaphoreTake(waitUrcSem, 0);
    taskEXIT_CRITICAL(&dispatchMux);

    if (xSemaphoreTake(waitUrcSem, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
        taskENTER_CRITICAL(&dispatchMux);
        if (copyToRxBuf) { clearRxBuf(); appendRxLine(waitUrcResult); }
        waitUrcActive = false;
        taskEXIT_CRITICAL(&dispatchMux);
        return true;
    }

    taskENTER_CRITICAL(&dispatchMux);
    waitUrcActive = false;
    taskEXIT_CRITICAL(&dispatchMux);
    return false;
}

/**
 * @brief Marca el inicio de una sesión de despacho AT bajo spinlock.
 *
 * @details
 * ### Propósito
 * Publicar los parámetros del comando AT en vuelo para que handleParsedLine()
 * (que corre en uartReaderTask) pueda enrutar las respuestas al canal correcto.
 *
 * ### Por qué spinlock y no mutex
 * handleParsedLine() corre en uartReaderTask (prio 5) que puede interrumpir
 * sendATLocked() (corriendo en commTask, prio 3).  Un mutex de FreeRTOS
 * requeriría ceder el scheduler; el spinlock es apropiado para una sección
 * crítica de microsegundos.
 *
 * @param command             Comando AT (puede ser nullptr para waitForLineLocked).
 * @param expectedResp        Respuesta esperada (copiada a dispatchSession.expected).
 * @param waitForOk           Si true, la sesión requiere "OK" tras el expected.
 * @param expectPrompt        Si true, la sesión espera ">".
 * @param captureOnlyExpected Si true, ignorar todas las líneas no-expected.
 */
void A7670Driver::beginDispatch(const char* command, const char* expectedResp,
                                bool waitForOk, bool expectPrompt, bool captureOnlyExpected) {
    taskENTER_CRITICAL(&dispatchMux);
    memset(&dispatchSession, 0, sizeof(dispatchSession));
    dispatchSession.active               = true;
    dispatchSession.captureOnlyExpected  = captureOnlyExpected;
    dispatchSession.expectPrompt         = expectPrompt;
    dispatchSession.waitForOk            = waitForOk;
    if (expectedResp) snprintf(dispatchSession.expected, sizeof(dispatchSession.expected), "%s", expectedResp);
    // derivedPrefix permite capturar "+CGNSSINFO:" para "AT+CGNSSINFO"
    buildDerivedPrefix(command, dispatchSession.derivedPrefix, sizeof(dispatchSession.derivedPrefix));
    taskEXIT_CRITICAL(&dispatchMux);
}

/**
 * @brief Cierra la sesión de despacho AT actual.
 *
 * @details
 * ### Propósito
 * Limpiar DispatchSession_t bajo spinlock para que handleParsedLine() deje de
 * enrutar líneas a atResponseQueue (las líneas posteriores irán a urcQueue
 * si son URCs conocidos, o serán descartadas como noise).
 *
 * @note Debe llamarse siempre en par con beginDispatch(), incluso en paths de error.
 */
void A7670Driver::endDispatch() {
    taskENTER_CRITICAL(&dispatchMux);
    memset(&dispatchSession, 0, sizeof(dispatchSession));   // active=false implícito por memset
    taskEXIT_CRITICAL(&dispatchMux);
}

/**
 * @brief Genera el prefijo de respuesta esperado para un comando AT+X.
 *
 * @details
 * ### Propósito
 * "AT+CGNSSINFO" → "+CGNSSINFO:" para que shouldRouteToResponse() capture
 * líneas de datos del comando aunque el expected sea distinto (ej. ">").
 *
 * ### Algoritmo
 * 1. Si command no empieza con "AT+" → out="".
 * 2. out[0] = '+'.
 * 3. Copiar chars de command+3 hasta '=', '?', espacio o fin de cadena.
 * 4. Agregar ':'.
 *
 * @param command  Comando AT completo (ej. "AT+CGNSSINFO").
 * @param out      Buffer destino.
 * @param outSize  Tamaño del buffer.
 */
void A7670Driver::buildDerivedPrefix(const char* command, char* out, size_t outSize) const {
    if (out == nullptr || outSize == 0U) return;
    out[0] = '\0';
    if (command == nullptr || strncmp(command, "AT+", 3) != 0) return;

    size_t w = 0;
    out[w++] = '+';
    for (const char* p = command + 3; *p != '\0' && w < (outSize - 2U); ++p) {
        // Detener en modificadores: '=' (parámetro), '?' (query), espacio
        if (*p == '=' || *p == '?' || isspace((unsigned char)*p)) break;
        out[w++] = *p;
    }
    out[w++] = ':';
    out[w]   = '\0';
}

/**
 * @brief Descarta todos los elementos de una cola FreeRTOS.
 *
 * @details
 * ### Propósito
 * Limpiar atResponseQueue antes de waitForLineLocked() para que respuestas de
 * comandos anteriores no interfieran con la espera del URC.
 *
 * @param queue  Cola a vaciar (puede ser nullptr; se maneja de forma segura).
 */
void A7670Driver::purgeQueue(QueueHandle_t queue) {
    if (queue == nullptr) return;
    ModemLine_t dummy = {};
    // timeout=0: no bloquear, solo sacar lo que hay
    while (xQueueReceive(queue, &dummy, 0) == pdTRUE) {}
}

/**
 * @brief Resetea el buffer acumulador de respuestas AT.
 *
 * @details
 * ### Propósito
 * Garantizar que sendATLocked() no mezcle respuestas de comandos anteriores
 * con la respuesta del comando actual.
 */
void A7670Driver::clearRxBuf() {
    memset(rxBuf, 0, sizeof(rxBuf));
}

/**
 * @brief Agrega una línea al buffer de respuesta acumulada.
 *
 * @details
 * ### Propósito
 * Construir en rxBuf la respuesta completa multi-línea del módulo para que
 * el caller pueda hacer strstr() sobre ella después de sendATLocked().
 *
 * ### Por qué \r\n como separador
 * Los callers usan strstr(rxBuf, "...") y esperan que las líneas estén
 * separadas por \r\n (formato nativo del protocolo AT).
 *
 * @param line Línea a agregar (sin \r\n).
 */
void A7670Driver::appendRxLine(const char* line) {
    if (line == nullptr || line[0] == '\0') return;
    size_t used = strlen(rxBuf);
    if (used >= sizeof(rxBuf) - 3U) return;    // Dejar espacio para "\r\n\0"
    size_t remaining = sizeof(rxBuf) - used;
    int written = snprintf(rxBuf + used, remaining, "%s\r\n", line);
    if (written < 0) rxBuf[used] = '\0';        // Truncar limpiamente si snprintf falla
}

/**
 * @brief Determina si una línea AT es un mensaje de error del módulo.
 *
 * @param line Línea recibida del módulo.
 * @return true si es "ERROR", "+CME ERROR:" o "+CMS ERROR:".
 */
bool A7670Driver::isErrorLine(const char* line) const {
    if (line == nullptr) return false;
    return (strcmp(line, "ERROR") == 0) ||
           (strncmp(line, "+CME ERROR:", 11) == 0) ||
           (strncmp(line, "+CMS ERROR:", 11) == 0);
}

/**
 * @brief Determina si una línea es un URC conocido del A7670.
 *
 * @details
 * ### Propósito
 * Permitir que handleParsedLine() enrute líneas que no son respuesta AT
 * pero tampoco son "ruido" desconocido → van a urcQueue para handleUrcLine().
 *
 * ### Nota sobre URCs MQTT
 * Los prefijos +CMQTT* son residuos del diseño original (antes de migrar a TCP).
 * Se mantienen para no cambiar la lista y causar regresiones de routing.
 *
 * @param line Línea parseada del UART.
 * @return true si la línea empieza con alguno de los 28+ prefijos conocidos.
 */
bool A7670Driver::isKnownUrc(const char* line) const {
    if (line == nullptr || line[0] == '\0') return false;

    static const char* URC_PREFIXES[] = {
        "+CGNSSPWR:",
        "+CGEV:",
        "+CSQ:",
        "+CGREG:",
        "+CREG:",
        "+CEREG:",
        "+NETOPEN:",
        "+NETCLOSE:",
        "+CPIN:",
        "+APP PDP:",
        // MQTT URCs — residuales del diseño original (antes de migrar a TCP directo)
        "+CMQTTSTART:",
        "+CMQTTSTOP:",
        "+CMQTTCONN:",
        "+CMQTTDISC:",
        "+CMQTTCONNLOST:",
        "+CMQTTPUB:",
        "+CMQTTRXSTART:",
        "+CMQTTRXTOPIC:",
        "+CMQTTRXPAYLOAD:",
        "+CMQTTRXEND:",
        "+CIPRXGET:",   // Notificación de datos entrantes en socket TCP ("+CIPRXGET: 1,0")
        // Secuencia de arranque del módulo
        "*ATREADY",
        "*ISIMAID:",
        "RDY",
        "SMS READY",
        "SMS DONE",
        "PB DONE",
        "Call Ready",
        "NORMAL POWER DOWN",
        "UNDER-VOLTAGE WARNING",
    };

    for (size_t i = 0; i < sizeof(URC_PREFIXES) / sizeof(URC_PREFIXES[0]); i++) {
        if (strncmp(line, URC_PREFIXES[i], strlen(URC_PREFIXES[i])) == 0) return true;
    }
    return false;
}

/**
 * @brief Determina si una línea coincide con los criterios de la sesión AT activa.
 *
 * @param line    Línea a verificar.
 * @param session Snapshot de DispatchSession_t.
 * @return true si la línea es la respuesta esperada por la sesión.
 */
bool A7670Driver::matchesDispatch(const char* line, const DispatchSession_t& session) const {
    if (!session.active || line == nullptr) return false;
    if (session.expectPrompt && strcmp(line, ">") == 0) return true;
    if (session.expected[0] != '\0' && strstr(line, session.expected) != nullptr) return true;
    // derivedPrefix captura la línea de datos del comando aunque expected sea "OK"
    if (session.derivedPrefix[0] != '\0' &&
        strncmp(line, session.derivedPrefix, strlen(session.derivedPrefix)) == 0) return true;
    return false;
}

/**
 * @brief Determina si una línea debe ir a atResponseQueue o a urcQueue.
 *
 * @details
 * ### Lógica de routing
 * - Si hay sesión activa y la línea coincide, o es "OK" o error → atResponseQueue.
 * - Si captureOnlyExpected y no coincide → descartar (no a urcQueue).
 * - Si es URC conocido → urcQueue.
 * - Caso restante → descartar (noise).
 *
 * @param line    Línea a clasificar.
 * @param session Snapshot de la sesión activa.
 * @return true si debe ir a atResponseQueue.
 */
bool A7670Driver::shouldRouteToResponse(const char* line, const DispatchSession_t& session) const {
    if (!session.active) return false;
    // "OK" y errores siempre van a atResponseQueue para que waitForExpectedLineLocked los vea
    if (matchesDispatch(line, session) || strcmp(line, "OK") == 0 || isErrorLine(line)) return true;
    // En modo captureOnly, ignorar todo lo que no matchea
    if (session.captureOnlyExpected) return false;
    // URCs conocidos van a su propia cola aunque haya sesión activa
    if (isKnownUrc(line)) return false;
    // Línea desconocida con sesión activa → puede ser datos del comando; enrutar a response
    return true;
}

/**
 * @brief Enruta una línea parseada al canal correcto (AT queue, URC queue, o descarte).
 *
 * @details
 * ### Propósito
 * Puente entre el parser de bytes (uartReaderTask) y los consumidores:
 * - waitForExpectedLineLocked() consume de atResponseQueue.
 * - handleUrcLine() (via urcHandlerTask) consume de urcQueue.
 *
 * ### Flujo
 * 1. Snapshot de dispatchSession bajo spinlock (sección crítica mínima).
 * 2. Verificar waitUrcActive: si el prefijo coincide → xSemaphoreGive (independiente del routing).
 * 3. Determinar target (atResponseQueue / urcQueue / nullptr).
 * 4. xQueueSend(target, timeout=0): no bloquear desde uartReaderTask.
 * 5. Cola llena: descartar el elemento más antiguo (xQueueReceive + xQueueSend).
 *
 * ### Por qué snapshot fuera del spinlock para el routing
 * shouldRouteToResponse() puede ser lento (múltiples strncmp); mantener el
 * spinlock durante esa operación bloquearía tcpSend() en secciones críticas propias.
 * El snapshot garantiza consistencia del estado leído.
 *
 * @param line Línea null-terminated ya parseada por emitParserLine().
 */
void A7670Driver::handleParsedLine(const char* line) {
    if (line == nullptr || line[0] == '\0') return;

    // Snapshot bajo spinlock: mínima sección crítica para leer el estado
    DispatchSession_t snapshot = {};
    taskENTER_CRITICAL(&dispatchMux);
    snapshot = dispatchSession;
    taskEXIT_CRITICAL(&dispatchMux);

    ModemLine_t msg = {};
    snprintf(msg.text, sizeof(msg.text), "%s", line);

    // Notificar al waiter de URC (independiente del routing de cola)
    taskENTER_CRITICAL(&dispatchMux);
    if (waitUrcActive && waitUrcPrefix[0] != '\0' &&
        strncmp(line, waitUrcPrefix, strlen(waitUrcPrefix)) == 0) {
        strncpy(waitUrcResult, line, sizeof(waitUrcResult) - 1);
        waitUrcResult[sizeof(waitUrcResult) - 1] = '\0';
        xSemaphoreGive(waitUrcSem);  // Desbloquear a tcpSend() que espera en xSemaphoreTake
    }
    taskEXIT_CRITICAL(&dispatchMux);

    QueueHandle_t target = nullptr;
    if (shouldRouteToResponse(line, snapshot)) {
        target = atResponseQueue;
    } else if (isKnownUrc(line)) {
        target = urcQueue;
    } else {
        ESP_LOGD(TAG, "[NOISE] %s", line);  // Línea desconocida sin sesión activa
        return;
    }

    if (target == nullptr) return;
    if (xQueueSend(target, &msg, 0) == pdTRUE) return;

    // Cola llena: política de evicción = descartar el elemento más antiguo
    // Mantener la línea más reciente es preferible a perder eventos nuevos
    ModemLine_t dropped = {};
    xQueueReceive(target, &dropped, 0);
    xQueueSend(target, &msg, 0);
}

/**
 * @brief Finaliza una línea acumulada en parserBuf y la emite a handleParsedLine.
 *
 * @details
 * ### Propósito
 * Convertir el buffer de bytes acumulados (parserBuf) en una línea lógica
 * procesable.  Realiza trimming de espacios para normalizar respuestas AT.
 *
 * ### Flujo
 * 1. Si parserLen==0 → reset overflow y retornar (línea vacía).
 * 2. null-terminate parserBuf.
 * 3. Trim leading/trailing spaces.
 * 4. Si parserOverflow → loguear advertencia.
 * 5. Si hay contenido → handleParsedLine().
 * 6. Reset parserLen, parserOverflow, parserBuf[0].
 */
void A7670Driver::emitParserLine() {
    if (parserLen == 0U) { parserOverflow = false; return; }
    parserBuf[parserLen] = '\0';

    // Trim leading whitespace
    size_t start = 0;
    while (parserBuf[start] == ' ' || parserBuf[start] == '\t') start++;
    // Trim trailing whitespace
    size_t end = strlen(parserBuf);
    while (end > start && (parserBuf[end - 1] == ' ' || parserBuf[end - 1] == '\t')) parserBuf[--end] = '\0';

    // Overflow indica que la línea fue más larga que MODEM_LINE_MAX; puede estar truncada
    if (parserOverflow) ESP_LOGW(TAG, "Línea UART truncada por longitud");
    if (end > start) handleParsedLine(parserBuf + start);

    parserLen = 0U;
    parserOverflow = false;
    parserBuf[0] = '\0';
}

/**
 * @brief Procesa un byte del stream UART y acumula líneas AT.
 *
 * @details
 * ### Propósito
 * Implementar el parser de líneas AT byte a byte.  Es el primer nivel del
 * pipeline de procesamiento: bytes crudos → líneas → colas.
 *
 * ### Reglas de parsing
 * | Byte         | Acción                                              |
 * |--------------|-----------------------------------------------------|
 * | '\r' o '\n'  | Emitir línea acumulada (fin de línea AT)            |
 * | '>' en pos 0 | Emitir como prompt inmediatamente (sin '\n' previo) |
 * | < 0x20       | Descartar (control chars corrompen líneas AT)       |
 * | resto        | Acumular en parserBuf                               |
 *
 * ### Por qué '>' se maneja especialmente
 * El prompt de envío de datos (">" en CIPSEND) no va seguido de '\r\n'.
 * Si se espera a un '\n', nunca se emitiría y CIPSEND haría timeout.
 *
 * @param byte Byte recibido del UART.
 */
void A7670Driver::processIncomingByte(uint8_t byte) {
    if (byte == '\r' || byte == '\n') { emitParserLine(); return; }
    // '>' solo al inicio de línea es el prompt CIPSEND/data-mode
    if (byte == '>' && parserLen == 0U) { handleParsedLine(">"); return; }
    // Descartar caracteres de control (evitar que bytes de init del módulo contaminen líneas)
    if (byte < 0x20 && byte != '\t') return;
    if (parserLen >= MODEM_LINE_MAX - 1U) { parserOverflow = true; return; }
    parserBuf[parserLen++] = (char)byte;
    parserBuf[parserLen]   = '\0';
}

/**
 * @brief Procesa un URC recibido y actualiza el estado interno del driver.
 *
 * @details
 * ### Propósito
 * Mantener sincronizados los flags de estado (netStackOpen, gnssEnabled, urcState)
 * con los eventos asíncronos del módulo celular.
 *
 * ### URCs manejados
 * | URC              | Efecto                                                    |
 * |------------------|-----------------------------------------------------------|
 * | *ATREADY         | Reset completo: netStackOpen=false, gnssEnabled=false     |
 * | +CGNSSPWR:       | gnssReady / gnssEnabled según valor ": 1" o ": 0"        |
 * | +CSQ:            | urcState.rssi, .ber, .hasSignal (99=sin señal)           |
 * | +CGREG:          | cgRegMode, cgRegStat, hasRegistration (1 o 5)            |
 * | +CGEV:           | EPS PDN ACT→netStackOpen=true; PDN DEACT→false           |
 * | +NETOPEN:        | netStackOpen = (code==0 || code==1)                      |
 * | +NETCLOSE:       | netStackOpen = false                                     |
 *
 * ### Por qué *ATREADY es crítico
 * Indica que el módulo se reinició inesperadamente (power glitch, watchdog interno).
 * Resetear todos los flags fuerza re-inicialización completa en el próximo tcpSend.
 *
 * ### Concurrencia
 * Todo acceso a netStackOpen, gnssEnabled y urcState se hace bajo dispatchMux
 * para consistencia con handleParsedLine() y tcpSend().
 *
 * @param line Texto del URC recibido (sin \r\n).
 */
void A7670Driver::handleUrcLine(const char* line) {
    ESP_LOGI(TAG, "[URC] %s", line);

    // *ATREADY indica que el módulo se reinició inesperadamente.
    // Resetear todos los flags para forzar re-inicialización completa.
    if (strncmp(line, "*ATREADY", 8) == 0) {
        ESP_LOGE(TAG, "==============================");
        ESP_LOGE(TAG, "[!!!] EL MODULO A7670 SE REINICIO (recibio *ATREADY)");
        ESP_LOGE(TAG, "[!!!] Resetando estado interno — re-inicializacion en proximo intento");
        ESP_LOGE(TAG, "==============================");

        taskENTER_CRITICAL(&dispatchMux);
        netStackOpen = false;
        gnssEnabled  = false;
        state        = MODEM_STATE_UART_READY;
        // Contar TODOS los crashes: el módulo restaura el estado GNSS desde su NVM
        // al reiniciarse, de modo que el engine GNSS puede crashear el módulo
        // aunque gnssEnabled fuera false desde la perspectiva del driver.
        gnssConsecCrashCount++;
        if (gnssConsecCrashCount >= GNSS_MAX_CONSEC_CRASHES) {
            gnssDisabledForSession = true;
        }
        bool   nowDisabled = gnssDisabledForSession;
        uint8_t crashCount  = gnssConsecCrashCount;
        taskEXIT_CRITICAL(&dispatchMux);

        // Loguear fuera del spinlock (ESP_LOG puede bloquear)
        if (nowDisabled) {
            ESP_LOGE(TAG, "[GNSS] DESHABILITADO — %d reinicios consecutivos del modulo; "
                          "TCP continua con lastKnownGps; reiniciar ESP32 para rehabilitar", crashCount);
        } else {
            ESP_LOGW(TAG, "[GNSS] Reinicio del modulo (%d/%d) — posible GNSS interno activo tras reset NVM",
                      crashCount, (int)GNSS_MAX_CONSEC_CRASHES);
        }
        return;
    }

    taskENTER_CRITICAL(&dispatchMux);

    if (strncmp(line, "+CGNSSPWR:", 10) == 0) {
        // "READY!" en el URC indica que el engine GNSS terminó de inicializar
        if (strstr(line, "READY!") || strstr(line, ": 1")) { urcState.gnssReady = true;  gnssEnabled = true; }
        else if (strstr(line, ": 0"))                       { urcState.gnssReady = false; gnssEnabled = false; }
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+CSQ:", 5) == 0) {
        int rssi = -1, ber = -1;
        if (sscanf(line, "+CSQ: %d,%d", &rssi, &ber) == 2) {
            urcState.rssi = rssi;
            urcState.ber  = ber;
            // 99 es el código AT para "no se pudo medir"; rssi>=0 && !=99 significa señal válida
            urcState.hasSignal = (rssi >= 0 && rssi != 99);
        }
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+CGREG:", 7) == 0) {
        int n = -1, stat = -1;
        if (sscanf(line, "+CGREG: %d,%d", &n, &stat) == 2) {
            // Formato extendido (CTZR=1): "+CGREG: mode,stat"
            urcState.cgRegMode = n;
            urcState.cgRegStat = stat;
            urcState.hasRegistration = (stat == 1 || stat == 5);  // 1=home, 5=roaming
        } else if (sscanf(line, "+CGREG: %d", &stat) == 1) {
            // Formato simple (CTZR=0): "+CGREG: stat"
            urcState.cgRegStat = stat;
            urcState.hasRegistration = (stat == 1 || stat == 5);
        }
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+CGEV:", 6) == 0) {
        // EPS PDN ACT: la red activó el bearer de datos → stack TCP disponible
        if (strstr(line, "EPS PDN ACT")) {
            netStackOpen = true;
        // PDN DEACT: la red cerró el bearer (timeout de inactividad o handover) → stack caído
        } else if (strstr(line, "PDN DEACT")) {
            netStackOpen = false;
        }
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+NETOPEN:", 9) == 0) {
        int code = -1;
        if (extractFirstIntegerAfter(line, "+NETOPEN", &code)) {
            // code=0: éxito; code=1: ya estaba abierto — ambos son estados válidos
            netStackOpen = (code == 0 || code == 1);
        }
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+NETCLOSE:", 10) == 0) {
        netStackOpen  = false;   // Stack TCP/IP cerrado: cualquier socket pendiente es inválido
        _tcpConnected = false;
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+CIPCLOSE:", 10) == 0) {
        _tcpConnected = false;   // Servidor o carrier cerró el socket
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    if (strncmp(line, "+CIPRXGET:", 10) == 0) {
        // Código 1 = notificación de datos disponibles en el buffer del socket.
        // Código 2 = respuesta a AT+CIPRXGET=2 (llega por atResponseQueue, no por aquí).
        // Solo marcar el flag para código 1; tryReadServerCommand() leerá el dato.
        int code = 0;
        if (sscanf(line, "+CIPRXGET: %d,", &code) == 1 && code == 1) {
            _hasPendingRx = true;
        }
        taskEXIT_CRITICAL(&dispatchMux);
        return;
    }

    taskEXIT_CRITICAL(&dispatchMux);
}

// ─── GPS parser ───────────────────────────────────────────────────────────────

/**
 * @brief Parsea la respuesta CSV de AT+CGNSSINFO a una estructura GpsData_t.
 *
 * @details
 * ### Propósito
 * Convertir el string CSV del A7670 en coordenadas geográficas, velocidad,
 * altitud, timestamp y conteo de satélites.
 *
 * ### Formato de respuesta AT+CGNSSINFO
 * ```
 * +CGNSSINFO: <fixMode>,<GPSsats>,<GLOsats>,<BDsats>,<lat>,<N/S>,<lon>,<E/W>,<date>,<UTC>,<alt>,<spd>,...
 * ```
 * | Índice | Campo      | Notas                                              |
 * |--------|------------|----------------------------------------------------|
 * | 0      | fixMode    | 0=sin fix; 1,2,3=fix 2D/3D                        |
 * | 1      | GPS sats   | Satélites GPS usados                               |
 * | 2      | GLO sats   | GLONASS (vacío en A7670 Colombia — campo presente) |
 * | 3      | BD sats    | Satélites BeiDou usados                            |
 * | 4      | total_used | Total de satélites usados                          |
 * | 5      | Latitud    | Grados decimales positivos                         |
 * | 6      | N/S        | 'S' → negativo; vacío si lat=0                     |
 * | 7      | Longitud   | Grados decimales positivos                         |
 * | 8      | E/W        | 'W' → negativo; vacío si lon=0                     |
 * | 9      | Fecha      | DDMMYY (6 dígitos) — convertido a YYYYMMDD         |
 * | 10     | UTC        | HHMMSS.ss                                          |
 * | 11     | Altitud    | Metros                                             |
 * | 12     | Velocidad  | km/h                                               |
 *
 * ### Por qué splitter manual y no strtok
 * strtok trata `,,` (dos comas seguidas) como UN solo separador, saltando
 * el campo vacío. Cuando el A7670 emite lat=0/lon=0 sin indicadores N/S y E/W,
 * strtok los salta y la fecha "DDMMYY" acaba en el índice de longitud →
 * lon=160626.0 en lugar de 0.0. El splitter manual asigna cada campo a su
 * índice correcto sin importar si está vacío.
 *
 * @param response Contenido de rxBuf tras AT+CGNSSINFO.
 * @param gps      Estructura a llenar.
 * @return true si fixMode != 0 y se pudo parsear correctamente.
 */
bool A7670Driver::parseGNSINF(const char* response, GpsData_t& gps) {
    const char* dataStart = strstr(response, "+CGNSSINFO: ");
    if (dataStart == nullptr) return false;
    dataStart += strlen("+CGNSSINFO: ");

    // Copiar a buffer mutable (el splitter pone '\0' en los separadores)
    char buf[256];
    strncpy(buf, dataStart, sizeof(buf) - 1U);
    buf[sizeof(buf) - 1U] = '\0';

    // Splitter manual que PRESERVA campos vacíos (strtok los salta, corriendo
    // todos los índices siguientes y causando que la fecha acabe en el campo
    // de longitud cuando lat/lon=0 y los indicadores N/S y E/W son vacíos).
    char* fields[20] = {nullptr};
    int fieldCount = 0;
    char* p = buf;
    fields[fieldCount++] = p;
    while (*p && fieldCount < 20) {
        if (*p == ',' || *p == '\r' || *p == '\n') {
            *p = '\0';
            fields[fieldCount++] = p + 1;
        }
        p++;
    }

    // Índices de campo con todos los slots presentes (incluido GLONASS vacío en [2]):
    // [0]=fixMode [1]=GPS_sats [2]=GLONASS_sats [3]=BEIDOU_sats [4]=total_used
    // [5]=lat [6]=N/S [7]=lon [8]=E/W [9]=date [10]=time [11]=alt [12]=speed
    if (fieldCount < 13) return false;

    int fixMode = fields[0] ? atoi(fields[0]) : 0;
    if (fixMode == 0) { gps.fix_valid = false; return false; }

    gps.latitude = fields[5] ? atof(fields[5]) : 0.0;
    if (fields[6] && fields[6][0] == 'S') gps.latitude = -gps.latitude;
    gps.longitude = fields[7] ? atof(fields[7]) : 0.0;
    if (fields[8] && fields[8][0] == 'W') gps.longitude = -gps.longitude;

    // El A7670 a veces reporta fixMode!=0 pero devuelve lat=0.0 lon=0.0
    // (estado transitorio de re-adquisición). Rechazar como fix inválido.
    if (gps.latitude == 0.0 && gps.longitude == 0.0) {
        gps.fix_valid = false;
        return false;
    }

    // Reordenar fecha de DDMMYY a YYYYMMDD para que gpsTimestampToEpoch pueda parsearla.
    // El A7670 entrega fields[9]="DDMMYY" (6 dígitos, año 2 cifras).
    // gpsTimestampToEpoch espera "YYYYMMDD HHMMSS.ss" (año 4 cifras, día-mes invertido).
    if (fields[9] && fields[10]) {
        int dd = 0, mm = 0, yy = 0;
        if (sscanf(fields[9], "%2d%2d%2d", &dd, &mm, &yy) == 3) {
            snprintf(gps.timestamp, sizeof(gps.timestamp),
                     "%04d%02d%02d %s", 2000 + yy, mm, dd, fields[10]);
        } else {
            snprintf(gps.timestamp, sizeof(gps.timestamp), "%s %s", fields[9], fields[10]);
        }
    }

    gps.altitude_m = fields[11] ? (float)atof(fields[11]) : 0.0f;
    gps.speed_kmh  = fields[12] ? (float)atof(fields[12]) : 0.0f;

    // Sumar satélites de todas las constelaciones para total de satélites usados
    uint8_t satsGps = fields[1] ? (uint8_t)atoi(fields[1]) : 0U;
    uint8_t satsGlo = fields[2] ? (uint8_t)atoi(fields[2]) : 0U;
    uint8_t satsBd  = fields[3] ? (uint8_t)atoi(fields[3]) : 0U;
    gps.satellites  = satsGps + satsGlo + satsBd;
    gps.fix_valid   = true;
    return true;
}

// ─── Tasks ────────────────────────────────────────────────────────────────────

/**
 * @brief Entry points estáticos para xTaskCreate (C linkage requerido).
 *
 * @details
 * xTaskCreate no puede recibir un puntero a método de instancia directamente.
 * Los entry points estáticos reciben 'this' como void* y lo reinterpretan.
 */
void A7670Driver::uartReaderTaskEntry(void* arg) { static_cast<A7670Driver*>(arg)->uartReaderTask(); }
void A7670Driver::urcHandlerTaskEntry(void* arg)  { static_cast<A7670Driver*>(arg)->urcHandlerTask(); }

/**
 * @brief Tarea de lectura de UART: convierte bytes crudos en líneas parseadas.
 *
 * @details
 * ### Propósito
 * Drenar continuamente el buffer de HW de UART2 y alimentar el parser
 * byte a byte.  Corre a prioridad 5 para vaciar el FIFO antes de que
 * el driver de UART lo desborde (buffer overflow silencioso).
 *
 * ### Flujo
 * 1. uart_read_bytes() con timeout 100 ms: no bloquea indefinidamente
 *    pero tampoco hace busy-wait (cede el scheduler si no hay datos).
 * 2. Para cada byte → processIncomingByte().
 * 3. Loop infinito (tarea permanente).
 *
 * ### Por qué chunks de 128 bytes
 * El FIFO de HW del UART del ESP32 es de 128 bytes.  Leer chunks del mismo
 * tamaño vacía el FIFO en una sola llamada al driver.
 *
 * @note Esta tarea nunca retorna.  No toma xModemMutex (solo escribe a parserBuf
 *       y llama handleParsedLine() que usa su propio spinlock).
 */
void A7670Driver::uartReaderTask() {
    uint8_t chunk[128];  // Tamaño = FIFO HW del ESP32 UART
    while (true) {
        int read = uart_read_bytes(A7670_UART_PORT, chunk, sizeof(chunk), pdMS_TO_TICKS(100));
        if (read <= 0) continue;   // Timeout sin datos: ceder scheduler y reintentar
        for (int i = 0; i < read; i++) processIncomingByte(chunk[i]);
    }
}

/**
 * @brief Tarea de manejo de URCs: procesa líneas enrutadas a urcQueue.
 *
 * @details
 * ### Propósito
 * Desacoplar el procesamiento de URCs del uartReaderTask.  Mientras
 * uartReaderTask solo acumula y emite líneas, esta tarea actualiza el estado
 * interno del driver (netStackOpen, urcState, etc.).
 *
 * ### Por qué portMAX_DELAY
 * La tarea no necesita correr si no hay URCs; portMAX_DELAY cede el scheduler
 * indefinidamente hasta que handleParsedLine() envía algo a urcQueue.
 *
 * ### Por qué prioridad 4 (menor que uartReaderTask)
 * handleUrcLine() puede tomar tiempo (múltiples strncmp + taskENTER_CRITICAL).
 * Ceder ante uartReaderTask garantiza que el buffer de HW nunca se desborde
 * por procesamiento lento de URCs.
 *
 * @note Esta tarea nunca retorna.
 */
void A7670Driver::urcHandlerTask() {
    ModemLine_t line = {};
    while (true) {
        // Bloquear hasta que llegue un URC; no consume CPU en reposo
        if (xQueueReceive(urcQueue, &line, portMAX_DELAY) == pdTRUE) {
            handleUrcLine(line.text);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════════
 * RESUMEN PARA HUMANO
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * Este archivo implementa el driver completo del módem celular A7670/SIM7670G
 * para el ESP32.  El objetivo es que el resto del firmware trate al módem como
 * una caja negra con tres operaciones: init(), tcpSend(), getGpsPosition().
 *
 * ARQUITECTURA INTERNA:
 * ┌──────────────────────────────────────────────────────────┐
 * │  uartReaderTask (prio 5)                                 │
 * │    └─ processIncomingByte() → emitParserLine()           │
 * │         └─ handleParsedLine()                            │
 * │              ├─ atResponseQueue → waitForExpectedLineLocked│
 * │              ├─ urcQueue → urcHandlerTask → handleUrcLine │
 * │              └─ waitUrcSem → tcpSend (para +CIPOPEN:)    │
 * └──────────────────────────────────────────────────────────┘
 *
 * FLUJO TÍPICO tcpSend():
 *   maintain() → ensureNetOpen() → CIPCLOSE → arm URC waiter
 *   → CIPOPEN → wait +CIPOPEN: 0,0 → CIPSEND → wait ">" → datos
 *   → wait "OK" → CIPCLOSE
 *
 * FLUJO TÍPICO getGpsPosition():
 *   enableGNSS() (lazy) → AT+CGNSSINFO → parseGNSINF()
 *
 * ESTADOS DEL MÓDEM:
 *   UART_READY → AT_READY → NET_READY → (MQTT_CONNECTED alias NET_READY)
 *
 * DEUDA TÉCNICA:
 * - APN hardcoded: requiere recompilación para cambiar operador.
 * - xModemMutex global: acoplamiento implícito con main.cpp.
 * - Socket TCP por trama: latencia extra ~2-4 s en cada envío.
 * - URCs MQTT en isKnownUrc(): residuos del diseño original, no funcionales.
 *
 * PSEUDOCÓDIGO tcpSend:
 *   lock(xModemMutex)
 *   maintain()
 *   if not netStackOpen: resetNetStack()
 *   CIPCLOSE=0 (limpiar)
 *   arm waitUrcPrefix("+CIPOPEN:")
 *   AT+CIPOPEN=0,"TCP",host,port
 *   wait semáforo 15s → parse code==0
 *   AT+CIPSEND=0,len → wait ">"
 *   uart_write_bytes(packet)
 *   wait "OK" 10s
 *   CIPCLOSE=0
 *   unlock
 *
 * DIAGRAMA MENTAL:
 *   ESP32 ←UART2→ A7670
 *     │                │
 *     │  AT commands   │  → respuestas/URCs
 *     │                │
 *   uartReaderTask parses bytes → líneas
 *   líneas → atResponseQueue (para ACK de comandos)
 *          → urcQueue (para eventos asíncronos de red/GNSS)
 *          → waitUrcSem (para +CIPOPEN: en tcpSend)
 * ═══════════════════════════════════════════════════════════════════════════════
 */
