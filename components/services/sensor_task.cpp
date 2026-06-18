/**
 * @file sensor_task.cpp
 * @brief Implementación de la tarea FreeRTOS del sensor MPU6050 con clasificación de movimiento.
 *
 * PROPÓSITO:
 *   Implementar el pipeline completo de detección de movimiento: desde la ISR de
 *   hardware (GPIO27) hasta el evento en xEventQueue que consume control_task.
 *   Esta tarea opera a la mayor prioridad del sistema (prio=5) para garantizar
 *   que ningún evento de seguridad se pierda por starvation de CPU.
 *
 * ARQUITECTURA DE DETECCIÓN (pipeline de dos fases):
 *
 *   FASE 1 — HARDWARE (ISR, ~1µs, ejecuta en contexto de interrupción):
 *     MPU6050 genera señal en GPIO27 cuando detecta movimiento > umbral hardware (30mg).
 *     motionISRHandler() IRAM_ATTR se ejecuta: da el semáforo xMotionSemaphore.
 *     portYIELD_FROM_ISR() solicita cambio de contexto si sensorTask tiene mayor prioridad.
 *
 *   FASE 2 — SOFTWARE (sensorTask, ~500µs, ejecuta en contexto de tarea):
 *     xSemaphoreTake(xMotionSemaphore, 1000ms) desbloquea (ISR o timeout 1Hz).
 *     readSensorData() lee 14 bytes del MPU6050 via I2C (~200µs).
 *     classifyMovement() aplica umbrales de SENSITIVITY_TABLE (~10µs con FPU).
 *     xQueueSend(xEventQueue, EventMessage_t, 50ms) entrega el evento a control_task.
 *
 * TABLA DE SENSIBILIDAD (SENSITIVITY_TABLE[5]):
 *   Indexada por SensitivityLevel_t (0=VERY_LOW a 4=VERY_HIGH).
 *   Cada entrada tiene 4 umbrales: accelSoft, accelHard, gyroSoft, gyroHard.
 *   La clasificación es: IMPACT (hardware) > HARD (software) > SOFT (software) > NONE.
 *
 *   VERY_LOW  [0]: accel 0.25/0.70g, gyro 15/60°/s   → para zonas urbanas bulliciosas
 *   LOW       [1]: accel 0.15/0.50g, gyro 10/40°/s   → zona semi-urbana
 *   MEDIUM    [2]: accel 0.05/0.30g, gyro  3/20°/s   → DEFAULT, balance óptimo
 *   HIGH      [3]: accel 0.02/0.15g, gyro  1/10°/s   → zona tranquila
 *   VERY_HIGH [4]: accel 0.01/0.08g, gyro  0.5/5°/s  → máxima vigilancia
 *
 * POLLING DE RESPALDO (1s timeout en xSemaphoreTake):
 *   Propósito: detectar movimiento lento que no disparó la ISR de hardware.
 *   El umbral de hardware es fijo (30mg, MPU6050_MOTION_THRESHOLD × 2mg/LSB).
 *   Con SENSITIVITY_VERY_HIGH, el umbral software de accelSoft es 10mg — menor que
 *   el hardware. El polling captura estos movimientos que el MPU6050 no interrumpió.
 *   isrTriggered=false en classifyMovement() cuando viene del timeout.
 *
 * ANTI-REBOTE (200ms):
 *   vTaskDelay(200ms) al final del loop evita inundar xEventQueue si el MPU6050
 *   genera rafagas de interrupciones (ej: moto con motor vibrando).
 *   Durante estos 200ms, señales adicionales del semáforo se colapsan en una.
 *
 * PERSISTENCIA EN NVS:
 *   Namespace "argus_cfg", key "sensitivity" (uint8_t).
 *   loadSensitivityFromNVS() al arranque. setSensitivityLevel() al cambiar.
 *   Nota: si NVS está corrupto o vacío, se usa SENSITIVITY_MEDIUM por defecto.
 *
 * VARIABLES CRÍTICAS:
 *   - xMotionSemaphore: semáforo binario entre ISR y sensorTask. Si es nullptr,
 *     la tarea se autoelimina (vTaskDelete) en init.
 *   - SENSITIVITY_TABLE: si se agrega un nivel de sensibilidad, agregar aquí.
 *     La constante SENSITIVITY_VERY_HIGH en system_flags.h debe coincidir con
 *     el último índice válido de esta tabla.
 *   - shouldProcess: la condición crítica que decide si enviar el evento.
 *     Sin ella, sensor_task inundaría xEventQueue con eventos cuando la moto
 *     está desarmada (ej: vibrando en tráfico).
 *
 * RIESGOS DE CONCURRENCIA:
 *   - xMotionSemaphore: binary semaphore, ISR-safe. Múltiples disparos de ISR
 *     antes de que sensorTask consuma el semáforo generan un solo "wake".
 *   - lastMovementTimestamp: escritura uint64_t no atómica en ARM. Ver system_flags.h.
 *   - motionSensitivity: escritura atómica (enum 4 bytes en ARM). Lectura en
 *     classifyMovement() es segura incluso sin mutex.
 *
 * @module components/services/sensor_task
 */

#include "sensor_task.h"
#include "system_events.h"
#include "system_flags.h"
#include "mpu6050_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c_master.h"
#include <string.h>
#include <cmath>

static const char* TAG = "SensorTask";

// Handle de la tarea (definición de la declaración extern en sensor_task.h)
TaskHandle_t xSensorTaskHandle = nullptr;

// Semáforo binario dado por la ISR GPIO27, tomado por sensorTask en el loop.
// Static porque solo sensor_task.cpp accede a él (no es parte de la API pública).
static SemaphoreHandle_t xMotionSemaphore = nullptr;

// Instancia del driver del MPU6050. Static: una sola instancia en todo el sistema.
static MPU6050Driver mpu6050;

// Métricas de conducción compartidas con commTask (declaradas en system_flags.h).
// Se acumulan aquí en cada lectura del MPU6050 y commTask las resetea tras cada frame DRIVE.
DriveMetrics_t g_driveMetrics = {};

// ─── Tabla de umbrales de clasificación por nivel de sensibilidad ─────────────

/**
 * Estructura de umbrales para clasificación de movimiento.
 * accelSoft/accelHard: desviación de la magnitud de aceleración respecto a 1g (gravedad).
 *   En reposo, |accel_vector| ≈ 1g. La desviación = ||accel_vector| - 1.0|.
 * gyroSoft/gyroHard: magnitud del vector de velocidad angular.
 */
struct SensThresholds {
    float accelSoft;  // mínima desviación de 1g para clasificar EVENT_MOVEMENT_SOFT
    float accelHard;  // mínima desviación de 1g para clasificar EVENT_MOVEMENT_HARD
    float gyroSoft;   // mínima magnitud de gyro (°/s) para SOFT
    float gyroHard;   // mínima magnitud de gyro (°/s) para HARD
};

// Indexada por SensitivityLevel_t (VERY_LOW=0 a VERY_HIGH=4).
// Debe tener exactamente 5 entradas (una por nivel) o el acceso al índice 4
// (SENSITIVITY_VERY_HIGH) produciría un buffer overflow.
static const SensThresholds SENSITIVITY_TABLE[5] = {
    { 0.25f, 0.70f, 15.0f, 60.0f },  // [0] VERY_LOW: moto en tráfico, muchos falsos positivos si más sensible
    { 0.15f, 0.50f, 10.0f, 40.0f },  // [1] LOW: zona semi-urbana, movimiento claro requerido
    { 0.08f, 0.30f,  3.0f, 20.0f },  // [2] MEDIUM: DEFAULT — 0.08g evita falsos positivos por bias del MPU6050 (az≈0.95g en reposo = 0.05g de desviación constante)
    { 0.02f, 0.15f,  1.0f, 10.0f },  // [3] HIGH: zona tranquila, detecta vibración leve
    { 0.01f, 0.08f,  0.5f,  5.0f },  // [4] VERY_HIGH: cualquier micro-movimiento
};

// Namespace y key de NVS para persistencia de la sensibilidad.
// Si se cambia el namespace, todos los dispositivos en campo perderán su configuración
// guardada en el NVS anterior al próximo reboot.
#define NVS_NAMESPACE_CFG   "argus_cfg"
#define NVS_KEY_SENSITIVITY "sensitivity"

// ─── ISR de movimiento ────────────────────────────────────────────────────────

/**
 * @brief Handler de interrupción del GPIO27 (pin INT del MPU6050).
 *
 * PROPÓSITO:
 *   Responder al flanco ascendente en GPIO27 con la mínima latencia posible.
 *   No lee el sensor (I2C no es ISR-safe). Solo da el semáforo y solicita
 *   cambio de contexto si sensorTask es la tarea de mayor prioridad lista.
 *
 * IRAM_ATTR:
 *   El atributo IRAM_ATTR coloca esta función en SRAM en lugar de flash.
 *   ESP32 usa un caché de instrucciones para ejecutar código desde flash.
 *   Si el sistema está haciendo una operación de escritura en flash, el caché
 *   se deshabilita. Sin IRAM_ATTR, la ISR podría faultar durante una escritura
 *   NVS. Con IRAM_ATTR, la ISR siempre está disponible en SRAM.
 *
 * portYIELD_FROM_ISR:
 *   Si xHigherPriorityTaskWoken es pdTRUE (sensorTask tiene mayor prioridad que
 *   la tarea interrumpida), solicita un cambio de contexto inmediato. Esto garantiza
 *   que sensorTask se ejecute en el siguiente tick del scheduler, minimizando la
 *   latencia entre la detección de movimiento y la lectura del sensor.
 *
 * @param arg No usado (nullptr, porque xMotionSemaphore es variable estática).
 */
static void IRAM_ATTR motionISRHandler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Da el semáforo binario. Si ya estaba dado (sensorTask aún no lo tomó),
    // esta llamada no tiene efecto adicional — las señales se colapsan.
    xSemaphoreGiveFromISR(xMotionSemaphore, &xHigherPriorityTaskWoken);
    // Solicitar cambio de contexto si sensorTask tiene mayor prioridad que la tarea actual.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Instala la ISR de movimiento en GPIO27.
 *
 * PROPÓSITO:
 *   Conectar la interrupción de hardware del MPU6050 (GPIO27) con motionISRHandler.
 *   Llamada internamente desde sensorTask después de crear xMotionSemaphore,
 *   garantizando que el semáforo exista antes del primer disparo de la ISR.
 *
 * NOTA: El tercer argumento (arg del ISR) es nullptr porque motionISRHandler
 *   accede a xMotionSemaphore como variable estática del módulo.
 */
void sensorTask_initISR() {
    mpu6050.installMotionISR(motionISRHandler, nullptr);
}

// ─── Persistencia NVS de sensibilidad ────────────────────────────────────────

/**
 * @brief Actualiza motionSensitivity y lo persiste en NVS.
 *
 * PROPÓSITO:
 *   Punto de entrada para cambios de sensibilidad desde bleTask o commTask.
 *   Sanitiza el valor, actualiza la variable global, escribe en NVS, y loguea
 *   los umbrales efectivos del nuevo nivel.
 *
 * SANITIZACIÓN:
 *   Si level > SENSITIVITY_VERY_HIGH (4), se usa SENSITIVITY_MEDIUM (2).
 *   Previene acceso fuera de SENSITIVITY_TABLE si llegara un valor inválido
 *   por un comando BLE malformado o corrupción de NVS.
 *
 * FLUJO NVS:
 *   nvs_open → nvs_set_u8 → nvs_commit → nvs_close.
 *   Si nvs_open falla (NVS no inicializado), solo se actualiza la variable en RAM.
 *   El valor se perderá en el próximo reboot, pero el sistema seguirá funcionando.
 */
void setSensitivityLevel(SensitivityLevel_t level) {
    // Sanitizar: valor fuera de rango → MEDIUM (seguro y balanceado)
    if ((uint8_t)level > (uint8_t)SENSITIVITY_VERY_HIGH) level = SENSITIVITY_MEDIUM;
    motionSensitivity = level;

    // Persistir en NVS para sobrevivir reboots
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE_CFG, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_SENSITIVITY, (uint8_t)level);
        nvs_commit(h);
        nvs_close(h);
    }

    // Log con los umbrales efectivos del nuevo nivel para facilitar debug
    static const char* LEVEL_NAMES[5] = {
        "MUY BAJA", "BAJA", "MEDIA", "ALTA", "MUY ALTA"
    };
    const SensThresholds& t = SENSITIVITY_TABLE[level];
    ESP_LOGI("SensorTask",
             "Sensibilidad: %s (nivel %d) | accel soft=%.2fg hard=%.2fg | gyro soft=%.1f hard=%.1f °/s",
             LEVEL_NAMES[level], (int)level,
             t.accelSoft, t.accelHard, t.gyroSoft, t.gyroHard);
}

/**
 * @brief Carga el nivel de sensibilidad guardado desde NVS.
 *
 * PROPÓSITO:
 *   Restaurar la configuración de sensibilidad tras un reboot, para que el
 *   usuario no tenga que reconigurarla cada vez que la moto se reinicia.
 *
 * FLUJO:
 *   nvs_open(READONLY) → nvs_get_u8("sensitivity") → sanitizar → nvs_close.
 *   Si no hay valor guardado (primera vez), mantener SENSITIVITY_MEDIUM (default).
 *   Nota: se usa NVS_READONLY porque solo se lee — evita crear el key si no existe.
 *
 * SANITIZACIÓN AL CARGAR:
 *   Solo acepta valores 0-4 (VERY_LOW a VERY_HIGH). El código original comparaba
 *   con <= SENSITIVITY_HIGH (3), perdiendo SENSITIVITY_VERY_HIGH (4). Mantenido
 *   como está para no cambiar comportamiento existente.
 *   ARQUITECTURA ⚠️: la condición debería ser <= SENSITIVITY_VERY_HIGH (4).
 */
static void loadSensitivityFromNVS() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE_CFG, NVS_READONLY, &h) == ESP_OK) {
        uint8_t val = (uint8_t)SENSITIVITY_MEDIUM;  // Default si no hay valor guardado
        nvs_get_u8(h, NVS_KEY_SENSITIVITY, &val);
        nvs_close(h);
        // ARQUITECTURA ⚠️: esta condición excluye SENSITIVITY_VERY_HIGH (4).
        // Debería ser: val <= (uint8_t)SENSITIVITY_VERY_HIGH
        if (val <= (uint8_t)SENSITIVITY_HIGH) {
            motionSensitivity = (SensitivityLevel_t)val;
        }
    }
}

// ─── Clasificación de evento ──────────────────────────────────────────────────

/**
 * @brief Clasifica los datos del sensor en un evento del sistema.
 *
 * PROPÓSITO:
 *   Aplicar los umbrales de SENSITIVITY_TABLE al nivel actual de sensibilidad
 *   para determinar qué tipo de evento generar (NONE, SOFT, HARD, IMPACT).
 *
 * JERARQUÍA DE CLASIFICACIÓN (de mayor a menor prioridad):
 *   1. IMPACT_DETECTED: el flag de hardware del MPU6050 indica impacto
 *      (magnitud del vector de aceleración > MPU6050_IMPACT_G_THRESHOLD = 2.5g).
 *   2. MOVEMENT_HARD: |accelDev| > accelHard OR gyroMag > gyroHard.
 *      Movimiento brusco, probable manipulación intencional de la moto.
 *   3. MOVEMENT_SOFT: isrTriggered OR data.motion_detected OR
 *      |accelDev| > accelSoft OR gyroMag > gyroSoft.
 *      Vibración leve, viento, o persona apoyándose en la moto.
 *   4. EVENT_NONE: ninguno de los criterios anteriores se cumple.
 *
 * MÉTRICA DE ACELERACIÓN (accelDev):
 *   Se calcula como |magnitud_vector - 1.0g|, es decir, la desviación de la
 *   magnitud de aceleración respecto a la gravedad en reposo.
 *   En reposo: |accel_vector| = 1g → accelDev = 0.
 *   En movimiento: |accel_vector| > 1g → accelDev > 0.
 *   Esto es invariante a la orientación del sensor (no depende de qué eje
 *   apunta hacia arriba, a diferencia de comparar un eje individual).
 *
 * @param data Datos del sensor ya convertidos a g y °/s (de readSensorData()).
 * @param isrTriggered true si la ISR de hardware disparó (vs. polling de respaldo).
 *        Si es true, incluso con accelDev y gyroMag bajos, se reporta MOVEMENT_SOFT.
 * @return El evento clasificado (EVENT_NONE, EVENT_MOVEMENT_SOFT/HARD, EVENT_IMPACT_DETECTED).
 */
static SystemEvent_t classifyMovement(const SensorData_t& data, bool isrTriggered) {
    // Prioridad 1: impacto detectado por hardware del MPU6050
    if (data.impact_detected) {
        return EVENT_IMPACT_DETECTED;
    }

    // Calcular magnitudes para comparar con umbrales (invariantes a orientación)
    float accelMag = sqrtf(data.accel_x * data.accel_x +
                           data.accel_y * data.accel_y +
                           data.accel_z * data.accel_z);
    float gyroMag  = sqrtf(data.gyro_x  * data.gyro_x  +
                           data.gyro_y  * data.gyro_y  +
                           data.gyro_z  * data.gyro_z);

    // Desviación de la magnitud respecto a 1g (en reposo siempre es ≈0)
    float accelDev = fabsf(accelMag - 1.0f);

    // Usar los umbrales del nivel de sensibilidad activo
    const SensThresholds& t = SENSITIVITY_TABLE[motionSensitivity];

    // Prioridad 2: movimiento brusco (supera umbrales HARD)
    if (accelDev > t.accelHard || gyroMag > t.gyroHard) {
        return EVENT_MOVEMENT_HARD;
    }

    // Prioridad 3: movimiento suave
    // isrTriggered=true significa que el hardware del MPU6050 ya confirmó movimiento
    // (aunque los valores de accel/gyro calculados aquí sean bajos — posible si el
    // movimiento fue breve y ya terminó cuando leímos el sensor).
    if (isrTriggered || data.motion_detected ||
        accelDev > t.accelSoft ||
        gyroMag  > t.gyroSoft) {
        return EVENT_MOVEMENT_SOFT;
    }

    // Ningún umbral superado: sin evento relevante
    return EVENT_NONE;
}

// ─── Tarea principal ──────────────────────────────────────────────────────────

/**
 * @brief Tarea FreeRTOS del sensor MPU6050.
 *
 * PROPÓSITO:
 *   Ejecutar el pipeline de detección de movimiento en un loop infinito.
 *   Combina detección por ISR (latencia baja) con polling de respaldo (1s).
 *
 * FLUJO DE ARRANQUE:
 *   loadSensitivityFromNVS() → crear semáforo → mpu6050.init() → installISR()
 *   Si cualquier paso crítico falla → vTaskDelete(nullptr) (la tarea se elimina).
 *
 * LOOP PRINCIPAL:
 *   xSemaphoreTake(1000ms) — espera ISR o polling de 1Hz.
 *   readSensorData() — leer datos del MPU6050 via I2C.
 *   classifyMovement() — determinar tipo de evento.
 *   actualizar lastMovementTimestamp si hay evento.
 *   shouldProcess = armado OR estado activo (MOVING/ALERT/PURSUIT).
 *   Si evento && shouldProcess → construir EventMessage_t → xQueueSend.
 *   vTaskDelay(200ms) — anti-rebote para no inundar la cola.
 *
 * CONDICIÓN shouldProcess:
 *   Evita que sensor_task inunde xEventQueue cuando el sistema está desarmado.
 *   Si !armado && STATE_IDLE: solo actualiza lastMovementTimestamp (para telemetría).
 *   Si armado O estado activo: envía el evento a control_task.
 *
 * ANTI-REBOTE:
 *   200ms de delay al final del loop. Durante este tiempo, el semáforo puede
 *   recibir señales adicionales de la ISR (colapsadas en una). Esto limita
 *   la tasa de eventos a 5 por segundo máximo, que es más que suficiente.
 */
void sensorTask(void* pvParameters) {
    ESP_LOGI(TAG, "Sensor task iniciada. Inicializando MPU6050...");

    // Cargar configuración de sensibilidad persistida en NVS
    loadSensitivityFromNVS();

    // Crear semáforo binario para comunicación ISR→tarea.
    // Debe crearse ANTES de installMotionISR para evitar que la ISR dispare
    // antes de que el semáforo exista (crash por dar a un handle nullptr).
    xMotionSemaphore = xSemaphoreCreateBinary();
    if (xMotionSemaphore == nullptr) {
        ESP_LOGE(TAG, "Error creando semáforo. Tarea abortada.");
        vTaskDelete(nullptr);
        return;
    }

    // Inicializar el sensor MPU6050: bus I2C + WHO_AM_I + escalas + detector de movimiento.
    esp_err_t ret = mpu6050.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando MPU6050 (%s). Tarea abortada.",
                 esp_err_to_name(ret));
        vTaskDelete(nullptr);
        return;
    }

    // Instalar ISR en GPIO27 (después de crear el semáforo)
    sensorTask_initISR();

    // Log de confirmación con los umbrales activos
    {
        const SensThresholds& t = SENSITIVITY_TABLE[motionSensitivity];
        ESP_LOGI(TAG,
                 "MPU6050 listo. Sensibilidad: %d | accel soft=%.2fg hard=%.2fg | gyro soft=%.0f hard=%.0f °/s",
                 (int)motionSensitivity, t.accelSoft, t.accelHard, t.gyroSoft, t.gyroHard);
    }

    EventMessage_t eventMsg;
    SensorData_t   sensorData;

    // Timeout de 1s para el polling de respaldo (detectar movimiento suave no interrumpido)
    const TickType_t POLLING_TIMEOUT = pdMS_TO_TICKS(1000);

    while (true) {
        // Esperar señal de la ISR (pdTRUE) o timeout de 1s (pdFALSE).
        // isrTriggered=true → el hardware del MPU6050 confirmó movimiento.
        // isrTriggered=false → polling de respaldo, solo los valores de accel/gyro deciden.
        bool isrTriggered = (xSemaphoreTake(xMotionSemaphore, POLLING_TIMEOUT) == pdTRUE);

        // Leer los 14 bytes del sensor: accel XYZ + temp + gyro XYZ
        ret = mpu6050.readSensorData(sensorData);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error leyendo MPU6050: %s", esp_err_to_name(ret));
            // Espera breve antes de reintentar (el bus I2C puede estar temporalmente ocupado)
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Clasificar el movimiento según los umbrales del nivel de sensibilidad activo
        SystemEvent_t detectedEvent = classifyMovement(sensorData, isrTriggered);

        // Actualizar timestamp y métricas de conducción aunque el sistema esté desarmado.
        // lastMovementTimestamp: cualquier evento (SOFT/HARD/IMPACT) — usado por auto-ARM.
        // lastHardMovementTimestamp: solo HARD/IMPACT — usado por commTask para GPS sleep.
        //   Vibraciones de tráfico (SOFT vía ISR) NO deben resetear el clock de sleep;
        //   de lo contrario la moto nunca entraría en sleep en zona urbana con tráfico.
        // g_driveMetrics captura comportamiento real y se envía en el frame DRIVE.
        if (detectedEvent != EVENT_NONE) {
            const uint64_t nowUs = esp_timer_get_time();
            lastMovementTimestamp = nowUs;
            if (detectedEvent == EVENT_MOVEMENT_HARD || detectedEvent == EVENT_IMPACT_DETECTED) {
                lastHardMovementTimestamp = nowUs;
            }

            // Calcular magnitudes para las métricas de conducción.
            // Misma fórmula que classifyMovement() — el costo de recalcular es
            // despreciable (~5 ciclos FPU) comparado con la lectura I2C previa (200µs).
            float accelMag = sqrtf(sensorData.accel_x * sensorData.accel_x +
                                   sensorData.accel_y * sensorData.accel_y +
                                   sensorData.accel_z * sensorData.accel_z);
            float gyroMag  = sqrtf(sensorData.gyro_x  * sensorData.gyro_x  +
                                   sensorData.gyro_y  * sensorData.gyro_y  +
                                   sensorData.gyro_z  * sensorData.gyro_z);
            float accelDev = fabsf(accelMag - 1.0f);

            // Picos: solo se actualizan si el nuevo valor supera el máximo acumulado.
            if (accelDev > g_driveMetrics.peakAccelDev) g_driveMetrics.peakAccelDev = accelDev;
            if (gyroMag  > g_driveMetrics.peakGyroMag)  g_driveMetrics.peakGyroMag  = gyroMag;

            // Contadores de eventos: HARD e IMPACT = maniobra brusca, SOFT = suave.
            if (detectedEvent == EVENT_MOVEMENT_HARD || detectedEvent == EVENT_IMPACT_DETECTED) {
                g_driveMetrics.hardCount = g_driveMetrics.hardCount + 1;
            } else if (detectedEvent == EVENT_MOVEMENT_SOFT) {
                g_driveMetrics.softCount = g_driveMetrics.softCount + 1;
            }
        }

        // Decidir si enviar el evento a la cola de control.
        // Solo procesar si el sistema está activamente vigilando:
        // armado en IDLE, o en estados MOVING/ALERT/PURSUIT (para reiniciar timers).
        SystemState_t state = currentSystemState;
        bool shouldProcess = systemArmed ||
                             state == STATE_MOVING ||
                             state == STATE_ALERT  ||
                             state == STATE_PURSUIT;

        if (detectedEvent == EVENT_NONE || !shouldProcess) {
            continue;  // Sin evento relevante o sin necesidad de procesarlo
        }

        // Construir el mensaje de evento con los datos del sensor adjuntos
        memset(&eventMsg, 0, sizeof(eventMsg));
        eventMsg.event       = detectedEvent;
        eventMsg.data.sensor = sensorData;

        // Enviar a xEventQueue con timeout de 50ms.
        // Si la cola está llena (control_task no está consumiendo), se descarta.
        // 50ms es suficiente para que control_task procese eventos pendientes.
        if (xQueueSend(xEventQueue, &eventMsg, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGW(TAG, "Cola de eventos llena, evento descartado");
        } else {
            // Señalizar que hay movimiento activo en el EventGroup (pulso breve)
            xEventGroupSetBits(xSystemFlags, FLAG_SENSOR_MOTION);

            // Log con nivel apropiado según la severidad del evento
            switch (detectedEvent) {
                case EVENT_IMPACT_DETECTED:
                    ESP_LOGE(TAG, "IMPACTO detectado");
                    break;
                case EVENT_MOVEMENT_HARD:
                    ESP_LOGW(TAG, "Movimiento DURO (ax=%.2fg ay=%.2fg az=%.2fg | gx=%.1f°/s)",
                             sensorData.accel_x, sensorData.accel_y, sensorData.accel_z,
                             sensorData.gyro_x);
                    break;
                case EVENT_MOVEMENT_SOFT:
                    ESP_LOGI(TAG, "Movimiento SUAVE (ax=%.2fg ay=%.2fg az=%.2fg)",
                             sensorData.accel_x, sensorData.accel_y, sensorData.accel_z);
                    break;
                default:
                    break;
            }
        }

        // Anti-rebote: 200ms de pausa para evitar inundar xEventQueue con el mismo evento.
        // Durante este tiempo, señales adicionales de la ISR se colapsan en el semáforo.
        vTaskDelay(pdMS_TO_TICKS(200));

        // Limpiar el flag de movimiento activo (pulso de ~200ms visible para otras tareas)
        xEventGroupClearBits(xSystemFlags, FLAG_SENSOR_MOTION);
    }
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/sensor_task.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo es la "vigilancia electrónica" de la moto. Una tarea de alta
   prioridad espera constantemente que el sensor de movimiento (MPU6050) detecte
   algo. Cuando ocurre, lee los datos del sensor y decide si fue una vibración
   leve (alguien se apoyó), un movimiento brusco (alguien intenta moverla), o
   un golpe fuerte (choque o caída). Luego le avisa a la parte del sistema que
   decide qué hacer con esa información (control_task).

   La sensibilidad se puede ajustar remotamente via BLE. En un parqueadero
   tranquilo puede usarse la sensibilidad más alta (detecta cualquier toque).
   En una calle con tráfico pesado, se usa la más baja para evitar falsas alarmas.

   PSEUDOCÓDIGO DEL LOOP:
   loop:
     isrTriggered = esperar semáforo (max 1s)
     datos = leer MPU6050 (14 bytes I2C)
     evento = clasificar(datos, isrTriggered):
       si impact_detected → IMPACT
       si accelDev > hard O gyro > hard → HARD
       si isrTriggered O accelDev > soft O gyro > soft → SOFT
       sino → NONE
     si evento != NONE: actualizar lastMovementTimestamp
     si sistema armado O estado activo:
       enviar evento a xEventQueue
     esperar 200ms (anti-rebote)

   TABLA DE UMBRALES (SENSITIVITY_TABLE):
   Nivel     | accel soft/hard    | gyro soft/hard
   ----------|--------------------|----------------
   VERY_LOW  | 0.25g / 0.70g     | 15°/s / 60°/s
   LOW       | 0.15g / 0.50g     | 10°/s / 40°/s
   MEDIUM    | 0.05g / 0.30g     |  3°/s / 20°/s  ← DEFAULT
   HIGH      | 0.02g / 0.15g     |  1°/s / 10°/s
   VERY_HIGH | 0.01g / 0.08g     | 0.5°/s / 5°/s

   DEUDA TÉCNICA:
   1. loadSensitivityFromNVS() tiene un bug: excluye SENSITIVITY_VERY_HIGH (4)
      de la carga — la condición debería ser <= SENSITIVITY_VERY_HIGH.
   2. lastMovementTimestamp escrita como uint64_t sin atomicidad — ver system_flags.h.
   3. El polling de respaldo (1s) ocurre aunque el sistema esté desarmado,
      gastando CPU y I2C innecesariamente cuando no se necesita vigilancia.
   4. Si mpu6050.init() falla en campo (clon con problema), la tarea desaparece
      silenciosamente sin notificar al backend.

   ═══════════════════════════════════════════════════════════ */
