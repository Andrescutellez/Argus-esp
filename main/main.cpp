/**
 * @file    main.cpp
 * @brief   Punto de entrada del firmware Argus Secure — ESP32 + A7670.
 *
 * @details
 * ### Rol en el sistema
 * Este archivo es el único punto de arranque del firmware.  Su responsabilidad
 * termina cuando retorna app_main(): las tareas FreeRTOS toman el control y el
 * scheduler gestiona la ejecución concurrente desde ese momento.
 *
 * ### Arquitectura general del firmware
 * ```
 *                          ┌─────────────────────┐
 *                          │       app_main()     │  ← Este archivo
 *                          │  NVS → primitivas   │
 *                          │  → crea 4 tareas    │
 *                          └─────────┬───────────┘
 *                                    │ (retorna; FreeRTOS toma el control)
 *           ┌────────────────────────┼────────────────────────┐
 *           ▼                        ▼                         ▼                 ▼
 *     sensorTask (5)          controlTask (4)           commTask (3)       bleTask (2)
 *     MPU6050 I2C             Máquina de estados        A7670 + GPS        NimBLE GATT
 *     GPIO27 interrupt        IDLE/MOVING/ALERT/        TCP send           BLE commands
 *     5-level sensitivity     PURSUIT                   SyncManager        WRITE_NO_RSP
 *           │                        ▲                         │
 *           └──── xEventQueue ───────┘                         │
 *                                    ▲                         │
 *           └──── xEventQueue ───────┘ (también desde BLE)     │
 *                                                              ▼
 *                                                    Backend TCP 34.69.219.193:9000
 * ```
 *
 * ### Mapa de módulos (todos sus headers en include/)
 * | Módulo            | Archivo               | Prioridad | Stack  | Rol                         |
 * |-------------------|-----------------------|-----------|--------|-----------------------------|
 * | sensorTask        | sensor_task.h/.cpp    | 5         | 4096   | MPU6050, detección movimiento |
 * | controlTask       | control_task.h/.cpp   | 4         | 4096   | FSM, MOSFETs, buzzer, LED    |
 * | commTask          | comm_task.h/.cpp      | 3         | 8192   | GPS, TCP, SyncManager        |
 * | bleTask           | ble_task.h/.cpp       | 2         | 6144   | BLE GATT NimBLE              |
 * | A7670Driver       | a7670_driver.h/.cpp   | interno   | 4096×2 | UART2, AT, GNSS, TCP         |
 * | EventQueue        | event_queue.h/.cpp    | -         | -      | Cola de eventos con NVS      |
 * | SyncManager       | sync_manager.h/.cpp   | -         | -      | Backoff, watchdog, métricas  |
 * | StateMachine      | state_machine.h/.cpp  | -         | -      | Lógica de transición FSM     |
 * | MosfetControl     | mosfet_control.h/.cpp | -         | -      | GPIO: buzzer/motor/LED       |
 *
 * ### Variables globales críticas (definidas aquí, declaradas extern en los headers)
 * | Variable              | Tipo                  | Protección   | Consumidores                   |
 * |-----------------------|-----------------------|--------------|--------------------------------|
 * | xEventQueue           | QueueHandle_t         | FreeRTOS     | sensorTask→, bleTask→, controlTask← |
 * | xGpsDataQueue         | QueueHandle_t         | FreeRTOS     | commTask→, otros←              |
 * | xSystemFlags          | EventGroupHandle_t    | FreeRTOS     | todas las tareas               |
 * | xStateMutex           | SemaphoreHandle_t     | FreeRTOS     | controlTask, commTask          |
 * | xModemMutex           | SemaphoreHandle_t     | FreeRTOS     | A7670Driver (recursivo)        |
 * | systemArmed           | volatile bool         | ninguna*     | sensorTask, controlTask, BLE   |
 * | remoteAlert           | volatile bool         | ninguna*     | controlTask                    |
 * | currentSystemState    | volatile SystemState_t| xStateMutex  | controlTask, commTask          |
 * | planType              | volatile PlanType_t   | ninguna*     | commTask (intervalo telemetría) |
 * | lastMovementTimestamp     | volatile uint64_t     | ninguna* | commTask/control (auto-ARM)   |
 * | lastHardMovementTimestamp | volatile uint64_t     | ninguna* | commTask (GPS sleep gate)     |
 * | motionSensitivity         | volatile SensitivityLevel_t | ninguna* | sensorTask (MPU6050)   |
 *
 * *Los booleans/uint64 marcados "ninguna" se acceden atómicamente en ESP32
 *  (Xtensa LX6 garantiza atomicidad en writes de 32 bits; bool es 8 bits pero
 *  el compilador los trata atómicamente en arquitectura Harvard con caché).
 *  ARQUITECTURA: Acceso a systemArmed/remoteAlert/planType/lastMovementTimestamp
 *     sin mutex. En ESP32 dual-core esto puede causar lecturas stale.
 *     Mejora: usar atomic<bool> o proteger con xStateMutex o FLAG en xSystemFlags.
 *
 * ### Flujo completo de arranque
 * 1. nvs_flash_init() — requerido por NimBLE y para NVS de EventQueue/sensibilidad.
 * 2. Crear 5 primitivas FreeRTOS (2 colas + 1 event group + 2 mutexes).
 * 3. Crear 4 tareas. Si alguna falla se loguea pero no se detiene el sistema
 *    (excepción: assertHandle() detiene el sistema si las primitivas fallan).
 * 4. app_main() retorna → FreeRTOS scheduler ejecuta las tareas indefinidamente.
 *
 * ### Mejoras recomendadas
 * 1. Fijar sensorTask y uartReaderTask al Core 1 para separar tiempo-real de IO.
 * 2. Reemplazar planType volatile con un flag en xSystemFlags.
 * 3. Agregar watchdog de hardware (esp_task_wdt) para detectar tareas colgadas.
 * 4. Agregar versión de firmware en NVS para validar compatibilidad de NVS.
 *
 * ### Deuda técnica
 * - planType está hardcoded a PLAN_PREMIUM en compilación; no se carga desde NVS.
 * - La verificación de fallo de xTaskCreate solo loguea; no hay estrategia de retry.
 *
 * @dependencies
 * - FreeRTOS: task, queue, event_groups, semphr.
 * - ESP-IDF: nvs_flash, esp_log.
 * - system_events.h: EventMessage_t, GpsData_t, SystemState_t, PlanType_t.
 * - system_flags.h: definición de bits de xSystemFlags.
 * - sensor_task.h, control_task.h, comm_task.h, ble_task.h: prototipos de tareas.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_log.h"

// Módulos del proyecto Argus
#include "system_events.h"
#include "system_flags.h"
#include "sensor_task.h"
#include "control_task.h"
#include "comm_task.h"
#include "ble_task.h"

static const char* TAG = "ArgusMain";

// ─── Definición de variables globales ────────────────────────────────────────
// Declaradas como extern en system_events.h y system_flags.h.
// Se DEFINEN aquí (una única definición en todo el proyecto, regla ODR de C++).

/**
 * @brief Cola de eventos del sistema.
 *
 * Producers: sensorTask (movimiento/impacto), bleTask (comandos del usuario).
 * Consumer: controlTask (única tarea que desencola y actúa).
 * Capacidad 10: buffer para ráfagas de eventos durante transiciones de estado.
 * Tipo: EventMessage_t.
 */
QueueHandle_t      xEventQueue    = nullptr;

/**
 * @brief Cola GPS de un solo elemento (dato más reciente).
 *
 * Producer: commTask (xQueueOverwrite tras cada lectura GPS exitosa).
 * Consumer: cualquier tarea que haga xQueuePeek (sin desencolar).
 * Capacidad 1 + xQueueOverwrite: garantiza que siempre hay el dato más fresco.
 * Tipo: GpsData_t.
 */
QueueHandle_t      xGpsDataQueue  = nullptr;

/**
 * @brief Event Group de flags de estado del sistema.
 *
 * Bits definidos en system_flags.h (FLAG_COMMS_BUSY, FLAG_HIGH_FREQ_MODE,
 * FLAG_REMOTE_ALERT, FLAG_GNSS_VALID, etc.).
 * Permite lectura sin bloqueo (xEventGroupGetBits) desde cualquier tarea.
 * Escritura atómica bit a bit (xEventGroupSetBits / xEventGroupClearBits).
 */
EventGroupHandle_t xSystemFlags   = nullptr;

/**
 * @brief Mutex para currentSystemState.
 *
 * Protege lecturas y escrituras de currentSystemState contra condiciones
 * de carrera entre controlTask y commTask.
 * Tipo: mutex estándar (no recursivo; ninguna tarea lo toma dos veces).
 */
SemaphoreHandle_t  xStateMutex    = nullptr;

/**
 * @brief Mutex recursivo global del módem A7670.
 *
 * Serializa todas las sesiones AT: init(), tcpSend(), getGpsPosition().
 * Recursivo: permite que init() llame internamente a enableGNSS() sin deadlock.
 * Definido aquí y referenciado en a7670_driver.cpp como extern.
 */
SemaphoreHandle_t  xModemMutex    = nullptr;

/**
 * @brief Flag: el sistema está armado (habilitado para detectar y responder).
 *
 * false=desarmado (sensorTask ignora movimientos).
 * true=armado (sensorTask encola eventos de movimiento).
 * Acceso sin mutex (ver ARQUITECTURA ⚠️ en el encabezado del archivo).
 */
volatile bool          systemArmed           = false;

/**
 * @brief Flag: alerta remota activa enviada desde la app o el operador.
 *
 * Leído en controlTask para distinguir SOFT_ALERT (sensor) de REMOTE_ALERT.
 * Se limpia al pasar a IDLE.
 */
volatile bool          remoteAlert           = false;

/**
 * @brief Flag: modo geocerca de estacionamiento activo.
 *
 * Cuando true, STATE_ALERT suprime el buzzer. Solo el backend puede desactivarlo
 * (CMD|PARK_MODE_OFF) al detectar que la moto salió de la geocerca, momento en
 * que la alarma completa se activa. También se limpia con CMD|DISARM.
 */
volatile bool          flagParkMode          = false;

/**
 * @brief Flag: corte de motor diferido pendiente de ejecución.
 *
 * Se activa cuando comm_task recibe CMD|ENGINE_CUT mientras la moto está en
 * movimiento (lastHardMovementTimestamp reciente). control_task ejecuta el corte
 * real en el próximo tick (250ms) donde detecte ≥3s sin HARD/IMPACT.
 * Razón del diferimiento: cortar el motor en movimiento puede causar un accidente
 * mortal — la moto pierde freno motor y el ladrón puede perder el control.
 */
volatile bool          flagEngineCutPending  = false;

/**
 * @brief Estado actual de la máquina de estados del sistema.
 *
 * Valores: STATE_IDLE, STATE_MOVING, STATE_ALERT, STATE_PURSUIT.
 * Protegido por xStateMutex para escritura; commTask lo lee para elegir
 * el intervalo de telemetría.
 */
volatile SystemState_t currentSystemState    = STATE_IDLE;

/**
 * @brief Tipo de plan del dispositivo: PLAN_PREMIUM o PLAN_FREEMIUM.
 *
 * Determina la frecuencia de telemetría en commTask:
 * - PREMIUM: 30 s en MOVING/IDLE, 15 s en ALERT, 10 s en PURSUIT.
 * - FREEMIUM: 3600 s en MOVING/IDLE.
 *
 * ARQUITECTURA: Hardcoded en tiempo de compilacion.
 *    Mejora: leer desde NVS configurado via BLE durante el onboarding.
 */
volatile PlanType_t    planType              = PLAN_PREMIUM;

/**
 * @brief Timestamp (µs) del último evento de movimiento (SOFT, HARD o IMPACT).
 *
 * Escrito por sensorTask en CUALQUIER evento de movimiento.
 * Leído por: commTask (auto-ARM delay), control_task (auto-ARM gate).
 * Inicializado a 0; commTask lo ajusta al boot para evitar sleep inmediato.
 */
volatile uint64_t      lastMovementTimestamp = 0;

/**
 * @brief Timestamp (µs) del último evento DURO (MOVEMENT_HARD o IMPACT_DETECTED).
 *
 * NO se resetea con vibraciones de tráfico (MOVEMENT_SOFT vía ISR).
 * Leído por: commTask como gate exclusivo del GPS sleep por inactividad.
 * Razón de separación: en Bogotá, un camión cada 2-3 min reseteaba
 * lastMovementTimestamp con SOFTs → GPS sleep nunca se activaba.
 * Inicializado a 0; commTask lo ajusta al boot igual que lastMovementTimestamp.
 */
volatile uint64_t      lastHardMovementTimestamp = 0;

/**
 * @brief Nivel de sensibilidad del MPU6050.
 *
 * Valores: SENSITIVITY_VERY_LOW(0) … SENSITIVITY_VERY_HIGH(4).
 * Cargado desde NVS en sensorTask al arrancar.
 * Modificable vía BLE (comando 0x10-0x14).
 * Define el umbral de aceleración que dispara eventos de movimiento.
 */
volatile SensitivityLevel_t motionSensitivity = SENSITIVITY_MEDIUM;

// Auto-ARM por inactividad: deshabilitado por defecto, delay=5min.
volatile bool     autoArmEnabled = false;
volatile uint32_t autoArmDelayMs = 5UL * 60UL * 1000UL;  // 5 minutos en ms

// ─── Función auxiliar de verificación ────────────────────────────────────────

/**
 * @brief Detiene el sistema si una primitiva FreeRTOS no pudo crearse.
 *
 * @details
 * ### Propósito
 * Las primitivas (colas, mutexes, event groups) son necesarias para la
 * operación de todas las tareas.  Si una falla (generalmente por RAM insuficiente),
 * continuar causaría null-pointer dereferences silenciosos en las tareas.
 *
 * ### Por qué un loop infinito y no esp_restart()
 * Un restart inmediato podría entrar en un boot loop si la RAM sigue fragmentada.
 * El loop con vTaskDelay(1s) mantiene el sistema vivo pero visible en logs,
 * permitiendo diagnóstico vía monitor serie sin reiniciar continuamente.
 *
 * ### En producción
 * Se debería agregar un contador: si N iteraciones del loop → esp_restart()
 * para recuperar de fragmentación de heap tras un reinicio limpio.
 *
 * @param handle Puntero al handle creado (nullptr indica fallo).
 * @param name   Nombre de la primitiva para el log de error.
 */
static void assertHandle(void* handle, const char* name) {
    if (handle == nullptr) {
        ESP_LOGE(TAG, "ERROR CRÍTICO: No se pudo crear %s. Deteniendo sistema.", name);
        // Loop de fallo visible: LOGE cada segundo para diagnóstico en monitor serie
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// ─── Punto de entrada principal ───────────────────────────────────────────────

/**
 * @brief Punto de entrada del firmware ESP-IDF (equivalente a main() de C).
 *
 * @details
 * ### Propósito
 * Inicializar el sistema completo y ceder el control al scheduler FreeRTOS.
 * Toda la lógica de aplicación vive en las tareas; app_main() solo
 * bootstrap y retorna.
 *
 * ### Flujo
 * 1. Banner de arranque (identificación en logs).
 * 2. nvs_flash_init() — requerido antes de cualquier NimBLE o NVS propio.
 * 3. Crear primitivas inter-tarea (fallo fatal → assertHandle detiene todo).
 * 4. Crear las 4 tareas (fallo solo loguea; otras tareas pueden seguir).
 * 5. Retornar (FreeRTOS continúa; app_main() se convierte en una tarea idle).
 *
 * ### Por qué no hay bucle en app_main
 * En ESP-IDF, app_main() corre en la tarea "main" de FreeRTOS con prioridad 1.
 * Retornar es equivalente a vTaskDelete(nullptr): la tarea se elimina y el
 * scheduler puede reasignar su stack.  No es necesario (ni deseable) bloquearse.
 *
 * ### Por qué xModemMutex es recursivo
 * A7670Driver::init() → A7670Driver::enableGNSS() ambos toman xModemMutex.
 * Un mutex estándar causaría deadlock; el recursivo permite re-entrancia desde
 * la misma tarea.
 *
 * @note extern "C": requerido para que ESP-IDF (que es C) encuentre el símbolo
 *       sin mangling de C++.
 */
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     ARGUS SECURE - Iniciando...      ║");
    ESP_LOGI(TAG, "║     Sistema de Seguridad GPS 4G      ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    // ── Paso 1: Inicializar NVS ───────────────────────────────────────────────
    // NimBLE requiere NVS para almacenar bonds y claves de emparejamiento.
    // EventQueue lo usa para persistir eventos CRITICAL ante power-off.
    // sensorTask lo usa para leer/escribir la sensibilidad del MPU6050.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS corrupto (ej. flash incompleta) o versión incompatible (cambio de schema):
        // borrar y reinicializar; se pierden los datos almacenados pero el sistema arranca.
        ESP_LOGW(TAG, "NVS corrupto o incompatible, borrando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // Si nvs_flash_init falla aquí, el sistema no puede operar → pánico inmediato
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializado correctamente");

    // ── Paso 2: Crear primitivas de comunicación inter-tarea ─────────────────
    // Todas son críticas: si alguna falla (sin RAM suficiente), assertHandle()
    // detiene el sistema en lugar de continuar con nullptr.

    // Capacidad 10: absorbe ráfagas durante transiciones de estado
    // (ej. múltiples impactos del MPU antes de que controlTask procese el primero)
    xEventQueue = xQueueCreate(10, sizeof(EventMessage_t));
    assertHandle(xEventQueue, "xEventQueue");

    // Capacidad 1: xQueueOverwrite() garantiza que siempre hay el dato más reciente.
    // Si se usara xQueueSend() con cap>1, commTask podría publicar GPS viejo
    // que nadie lee → data stale acumulada.
    xGpsDataQueue = xQueueCreate(1, sizeof(GpsData_t));
    assertHandle(xGpsDataQueue, "xGpsDataQueue");

    // EventGroup: 24 bits útiles en ESP32 (los superiores los usa FreeRTOS internamente).
    // Permite leer múltiples flags de estado en una sola operación sin bloqueo.
    xSystemFlags = xEventGroupCreate();
    assertHandle(xSystemFlags, "xSystemFlags");

    // Mutex estándar (no recursivo): controlTask y commTask lo toman por períodos cortos
    // al leer/escribir currentSystemState.
    xStateMutex = xSemaphoreCreateMutex();
    assertHandle(xStateMutex, "xStateMutex");

    // Mutex recursivo: A7670Driver::init() puede llamar a enableGNSS() que también
    // toma el mutex → recursivo evita deadlock desde la misma tarea.
    xModemMutex = xSemaphoreCreateRecursiveMutex();
    assertHandle(xModemMutex, "xModemMutex");

    ESP_LOGI(TAG, "Primitivas FreeRTOS creadas correctamente");

    // ── Paso 3: Crear tareas FreeRTOS ────────────────────────────────────────
    // Orden de creación: de mayor a menor prioridad.
    // Todas corren en Core 0 por defecto (xTaskCreate sin affinity).
    // /* ARQUITECTURA ⚠️ Todas las tareas en Core 0. sensorTask y uartReaderTask
    //    (A7670Driver, prio 5) compiten en el mismo core.
    //    Mejora: sensorTask en Core 1 con xTaskCreatePinnedToCore para aislar
    //    el tiempo-real del IO del módem. */

    BaseType_t taskRet;

    // sensorTask: interfaz con MPU6050 vía I2C + GPIO27 interrupt.
    // Prioridad 5 (la más alta de aplicación): respuesta a interrupciones de movimiento
    // debe ser inmediata para no perder eventos breves.
    // Stack 4096: suficiente para I2C driver + NVS read + cálculo de sensibilidad.
    taskRet = xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        nullptr,
        5,
        &xSensorTaskHandle
    );
    if (taskRet != pdPASS) {
        ESP_LOGE(TAG, "Error creando SensorTask");
    } else {
        ESP_LOGI(TAG, "SensorTask creada (prio=5, stack=4096)");
    }

    // controlTask: FSM IDLE/MOVING/ALERT/PURSUIT + actuadores (buzzer/motor/LED).
    // Prioridad 4: debe procesar eventos de sensorTask rápido, pero puede ceder
    // ante el sensor si ambos están listos.
    // Stack 4096: FSM + FreeRTOS timers + GPIO.
    taskRet = xTaskCreate(
        controlTask,
        "ControlTask",
        4096,
        nullptr,
        4,
        &xControlTaskHandle
    );
    if (taskRet != pdPASS) {
        ESP_LOGE(TAG, "Error creando ControlTask");
    } else {
        ESP_LOGI(TAG, "ControlTask creada (prio=4, stack=4096)");
    }

    // commTask: GPS via AT+CGNSSINFO + TCP via A7670 + SyncManager + EventQueue.
    // Prioridad 3: comunicación puede tolerar latencias de decenas de segundos;
    // no debe preemptar la FSM ni el sensor.
    // Stack 8192: buffers AT (256 bytes), JSON, Argus protocol string, SyncManager.
    taskRet = xTaskCreate(
        commTask,
        "CommTask",
        8192,
        nullptr,
        3,
        &xCommTaskHandle
    );
    if (taskRet != pdPASS) {
        ESP_LOGE(TAG, "Error creando CommTask");
    } else {
        ESP_LOGI(TAG, "CommTask creada (prio=3, stack=8192)");
    }

    // bleTask: servidor GATT NimBLE para configuración local (sensibilidad, armar/desarmar).
    // Prioridad 2: la más baja; BLE es interacción humana puntual, no tiempo-real.
    // Stack 6144: NimBLE crea internamente nimbleHostTask; 6144 para bleTask propia.
    taskRet = xTaskCreate(
        bleTask,
        "BleTask",
        6144,
        nullptr,
        2,
        &xBleTaskHandle
    );
    if (taskRet != pdPASS) {
        ESP_LOGE(TAG, "Error creando BleTask");
    } else {
        ESP_LOGI(TAG, "BleTask creada (prio=2, stack=6144)");
    }

    // ── Paso 4: Retornar ─────────────────────────────────────────────────────
    // app_main() puede retornar en ESP-IDF: la tarea "main" se elimina y el
    // scheduler cede CPU a las 4 tareas creadas.  No se necesita ningún loop.
    ESP_LOGI(TAG, "Sistema Argus Secure inicializado. Tareas en ejecución.");
    ESP_LOGI(TAG, "Estado inicial: IDLE | Armado: NO | BLE: Advertising...");
}

/* ═══════════════════════════════════════════════════════════════════════════════
 * RESUMEN PARA HUMANO
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * main.cpp es el bootstrap del firmware Argus.  Su única responsabilidad es
 * crear las variables globales compartidas y arrancar las 4 tareas FreeRTOS.
 * Una vez que retorna, nunca más se ejecuta código desde este archivo.
 *
 * VARIABLES GLOBALES:
 *   Colas:      xEventQueue (10 EventMessage_t) · xGpsDataQueue (1 GpsData_t)
 *   Flags:      xSystemFlags (EventGroup de bits)
 *   Mutexes:    xStateMutex (currentSystemState) · xModemMutex (A7670, recursivo)
 *   Flags raw:  systemArmed · remoteAlert · currentSystemState · planType
 *               lastMovementTimestamp · motionSensitivity
 *
 * TAREAS Y PRIORIDADES:
 *   5: sensorTask   — MPU6050, GPIO27, movimiento → xEventQueue
 *   4: controlTask  — FSM, buzzer/motor/LED, FreeRTOS timers
 *   3: commTask     — GPS, TCP, SyncManager, backoff
 *   2: bleTask      — NimBLE GATT, comandos de usuario
 *   (internos del driver):
 *   5: uartReaderTask  — drenar buffer HW de UART2
 *   4: urcHandlerTask  — procesar URCs del módem
 *
 * FLUJO DE DATOS TÍPICO:
 *   MPU6050 → sensorTask → xEventQueue → controlTask → MOSFETs/LED
 *   controlTask → currentSystemState → commTask → intervalo de telemetría
 *   commTask → A7670 → AT+CGNSSINFO → GPS → xGpsDataQueue
 *   commTask → A7670 → tcpSend → Backend 34.69.219.193:9000
 *   BLE → bleTask → xEventQueue → controlTask (armar/alertar/sensibilidad)
 *
 * PSEUDOCÓDIGO app_main:
 *   nvs_flash_init() (con erase si corrupto)
 *   xEventQueue    = xQueueCreate(10, sizeof EventMessage_t)
 *   xGpsDataQueue  = xQueueCreate(1, sizeof GpsData_t)
 *   xSystemFlags   = xEventGroupCreate()
 *   xStateMutex    = xSemaphoreCreateMutex()
 *   xModemMutex    = xSemaphoreCreateRecursiveMutex()
 *   assertHandle(cada primitiva)
 *   xTaskCreate(sensorTask, prio=5, stack=4096)
 *   xTaskCreate(controlTask, prio=4, stack=4096)
 *   xTaskCreate(commTask, prio=3, stack=8192)
 *   xTaskCreate(bleTask, prio=2, stack=6144)
 *   return  ← FreeRTOS scheduler toma el control
 *
 * DIAGRAMA MENTAL:
 *   app_main() = fábrica + lanzadera
 *   Crea los "canales de comunicación" (colas, flags, mutexes) y
 *   lanza los "procesos" (tareas).  Luego desaparece.
 *   El sistema vive en las tareas, no en main.
 *
 * DEUDA TÉCNICA:
 * - planType hardcoded: requiere recompilación para cambiar de plan.
 * - Todas las tareas en Core 0: sensorTask y uartReaderTask compiten.
 * - Fallo de xTaskCreate solo loguea, no tiene retry ni failsafe.
 * - Acceso a booleans volátiles sin mutex (potencial lectura stale en dual-core).
 *
 * MEJORAS RECOMENDADAS:
 * 1. xTaskCreatePinnedToCore(sensorTask, ..., 1) para separar tiempo-real.
 * 2. esp_task_wdt_add() en cada tarea para watchdog por tarea.
 * 3. Mover planType a xSystemFlags o NVS para configuración en campo.
 * 4. Agregar versión de firmware en NVS para detectar incompatibilidades.
 * ═══════════════════════════════════════════════════════════════════════════════
 */
