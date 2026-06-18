/**
 * @file ble_task.cpp
 * @brief Servidor BLE del sistema Argus usando el stack NimBLE de ESP-IDF.
 *
 * PROPÓSITO:
 *   Implementar el servidor GATT que permite a la app móvil Argus controlar
 *   el sistema via BLE. Expone una única característica escribible que recibe
 *   comandos de un byte y los traduce a eventos del sistema o cambios directos
 *   de configuración (sensibilidad).
 *
 * ARQUITECTURA BLE DEL SISTEMA:
 *
 *   App móvil Argus (central BLE)
 *     → scan (descubre "Argus-Secure" en advertising)
 *     → conecta (connectable undirected mode)
 *     → descubre servicio 4FAFC201-...
 *     → escribe 1 byte en característica BEB5483E-...
 *     → cmdCharAccessCallback() en nimbleHostTask
 *     → mapeo byte → EventMessage_t ó setSensitivityLevel()
 *     → xQueueSend(xEventQueue) ó motionSensitivity=nivel
 *     → controlTask ó sensorTask reacciona
 *
 *   Tareas involucradas:
 *     bleTask       → inicializa NimBLE + se suspende
 *     nimbleHostTask → corre el loop de eventos BLE (interna de NimBLE)
 *     cmdCharAccessCallback → ejecutado en nimbleHostTask al recibir escritura
 *
 * SERVICIO GATT:
 *   UUID Servicio:        4FAFC201-1FB5-459E-8FCC-C5C9C331914B
 *   UUID Característica:  BEB5483E-36E1-4688-B7F5-EA07361B26A8
 *   Propiedades:          WRITE | WRITE_NO_RSP (escritura con y sin confirmación)
 *
 *   WRITE_NO_RSP se usa para baja latencia: la app no espera confirmación ATT.
 *   El handle cmdCharHandle lo asigna NimBLE al registrar los servicios.
 *   No se usa para notificaciones (NOTIFY/INDICATE no están implementadas).
 *
 * MAPA DE COMANDOS (byte único):
 *   0x01 → EVENT_ARM_CMD              → controlTask arma el sistema
 *   0x02 → EVENT_DISARM_CMD           → controlTask desarma el sistema
 *   0x03 → EVENT_TRIGGER_ALERT_CMD    → controlTask activa alerta manual
 *   0x10 → setSensitivityLevel(VERY_LOW)   → sensorTask actualiza umbrales
 *   0x11 → setSensitivityLevel(LOW)
 *   0x12 → setSensitivityLevel(MEDIUM)     (default de fábrica)
 *   0x13 → setSensitivityLevel(HIGH)
 *   0x14 → setSensitivityLevel(VERY_HIGH)
 *
 *   Los comandos 0x01-0x03 van a xEventQueue (procesados por controlTask).
 *   Los comandos 0x10-0x14 actúan directamente sobre motionSensitivity y NVS
 *   (sin pasar por xEventQueue — el efecto es inmediato en sensorTask).
 *
 * ADVERTISING:
 *   Nombre: "Argus-Secure"
 *   Intervalo: 100ms (itvl=160 × 0.625ms). Visible a corta distancia (~10m).
 *   La app móvil filtra por nombre para encontrar el dispositivo.
 *   Se reinicia automáticamente tras desconexión en onNimBLESync().
 *
 * DEPENDENCIAS:
 *   - system_events.h:  EventMessage_t, SystemEvent_t, xEventQueue
 *   - sensor_task.h:    setSensitivityLevel() (para comandos 0x10-0x14)
 *   - NimBLE headers:   nimble_port.h, ble_hs.h, ble_svc_gap.h, ble_svc_gatt.h
 *
 * VARIABLES CRÍTICAS:
 *   - cmdCharHandle:   Handle de la característica asignado por NimBLE. Requerido
 *                      si se quisiera hacer NOTIFY/INDICATE en el futuro.
 *   - ownAddrType:     Tipo de dirección BLE (pública o aleatoria). Se determina
 *                      en onNimBLESync() con ble_hs_id_infer_auto().
 *   - gattServices[]: Tabla de servicios GATT en ROM (const). No se modifica
 *                      en runtime. El terminador { } al final es obligatorio.
 *
 * CONCURRENCIA:
 *   - cmdCharAccessCallback() corre en nimbleHostTask (no en bleTask ni controlTask).
 *     xQueueSend con timeout=100ms es seguro desde cualquier contexto de tarea.
 *     Los 100ms de timeout son conservadores — si la cola está llena por 100ms,
 *     el comando BLE se pierde, pero el usuario puede repetirlo.
 *   - setSensitivityLevel() escribe motionSensitivity (volatile enum) — atómico en ARM.
 *     El NVS tiene mutex interno. Thread-safe.
 *   - ownAddrType: escrito en onNimBLESync() (nimbleHostTask) y leído en
 *     startAdvertising() (también nimbleHostTask). Sin races.
 *
 * ARQUITECTURA:
 *    Sin autenticacion BLE. Cualquier dispositivo puede conectarse y enviar
 *    comandos ARM/DISARM. Para produccion implementar bonding + MITM:
 *      ble_hs_cfg.sm_bonding = 1
 *      ble_hs_cfg.sm_mitm    = 1
 *      ble_hs_cfg.sm_sc      = 1 (Secure Connections, BLE 4.2+)
 *    Sin esto, un atacante puede desarmar remotamente el sistema.
 *
 * @module components/services/ble_task
 */

#include "ble_task.h"
#include "system_events.h"
#include "sensor_task.h"
#include "system_flags.h"
#include "esp_log.h"

// Headers del stack NimBLE de ESP-IDF
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_sm.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "nvs.h"

// TAG de logging para todos los mensajes de esta tarea.
static const char* TAG = "BleTask";

// Handle de la tarea. Asignado por xTaskCreate() en main.cpp.
TaskHandle_t xBleTaskHandle = nullptr;

// ─── Comandos BLE aceptados ───────────────────────────────────────────────────
// Bytes que la app móvil puede enviar a la característica BLE de comandos.
// Rango 0x01-0x03: eventos de sistema → xEventQueue → controlTask.
// Rango 0x10-0x14: comandos de sensibilidad → setSensitivityLevel() directo.

#define BLE_CMD_ARM                       0x01  // Armar el sistema (activar vigilancia)
#define BLE_CMD_DISARM                    0x02  // Desarmar el sistema (desactivar vigilancia)
#define BLE_CMD_TRIGGER_ALERT             0x03  // Disparar alerta manual de emergencia
#define BLE_CMD_SENSITIVITY_VERY_LOW      0x10  // 5 niveles de sensibilidad MPU6050
#define BLE_CMD_SENSITIVITY_LOW           0x11  // (ver sensor_task.h para umbrales)
#define BLE_CMD_SENSITIVITY_MEDIUM        0x12  // default de fábrica
#define BLE_CMD_SENSITIVITY_HIGH          0x13
#define BLE_CMD_SENSITIVITY_VERY_HIGH     0x14
// Auto-ARM por inactividad — solo PLAN_PREMIUM.
// Escritura de 1 byte [0x20] → habilitar con delay actual (default 5min).
// Escritura de 2 bytes [0x20, N] → habilitar con N minutos de delay (1-60).
// Escritura de 1 byte [0x21] → deshabilitar auto-ARM.
#define BLE_CMD_AUTO_ARM_ENABLE           0x20
#define BLE_CMD_AUTO_ARM_DISABLE          0x21

// ─── Identidad del dispositivo BLE ───────────────────────────────────────────
// Nombre dinámico: "ARGUS-XXYYZZ00" (últimos 4 bytes de MAC WiFi).
// Único por unidad — permite distinguir múltiples dispositivos Argus.
static char _deviceName[20] = {};  // Inicializado en setupDeviceIdentity()

// Token de autenticación de aplicación — 8 bytes aleatorios generados en fábrica.
// Se almacena en NVS ("argus_cfg" / "ble_token") para persistir entre reinicios.
// La app lo obtiene vía QR en el onboarding y lo envía al AUTH char para autorizar.
static uint8_t _bleToken[8] = {};

// true una vez que la app envía el token correcto al AUTH char.
// Se resetea a false en cada desconexión — cada sesión BLE debe autenticarse.
static bool _connAuthenticated = false;

// ─── UUIDs del servicio y característica ─────────────────────────────────────
//
// Los UUIDs de 128 bits se almacenan en little-endian como requiere el stack BLE.
// El UUID "4FAFC201-1FB5-459E-8FCC-C5C9C331914B" en little-endian se lee
// de derecha a izquierda: 0x4b, 0x91, 0x31, 0xc3, ...
//
// BLE_UUID128_INIT() acepta los bytes en el orden correcto para little-endian.
// Se usan UUIDs de 128 bits (en lugar de 16 bits) porque son UUIDs propietarios
// no asignados por el Bluetooth SIG.

// UUID del servicio principal Argus Secure.
static const ble_uuid128_t ARGUS_SVC_UUID = BLE_UUID128_INIT(
    0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
    0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f
);

// UUID de la característica de comandos.
static const ble_uuid128_t ARGUS_CMD_CHAR_UUID = BLE_UUID128_INIT(
    0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
);

/**
 * Handle de la característica de comandos, asignado por NimBLE al registrar la tabla GATT.
 * Inicialmente 0 (inválido) — NimBLE lo asigna en ble_gatts_add_svcs().
 */
static uint16_t cmdCharHandle = 0;

// UUID de la característica de estado — ESP32 notifica a la app el estado ARM/DISARM.
// BEB5483E-36E1-4688-B7F5-EA07361B26A9 (último byte: A9, diferencia del char de comandos).
static const ble_uuid128_t ARGUS_STATE_CHAR_UUID = BLE_UUID128_INIT(
    0xa9, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
);
// Handle del VALUE de la char de estado. Requerido por ble_gatts_notify_custom().
static uint16_t stateCharHandle = 0;

// UUID de la característica de autenticación — App envía el token de 8 bytes aquí.
// BEB5483E-36E1-4688-B7F5-EA07361B26AA (último byte: AA)
static const ble_uuid128_t ARGUS_AUTH_CHAR_UUID = BLE_UUID128_INIT(
    0xaa, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
);
static uint16_t authCharHandle = 0;

// Handle de la conexión BLE activa. BLE_HS_CONN_HANDLE_NONE cuando no hay cliente.
// Escrito por bleGapEventHandler() en nimbleHostTask. Leído por bleNotifyState().
static volatile uint16_t _connHandle = BLE_HS_CONN_HANDLE_NONE;

// Tipo de dirección BLE (pública o aleatoria). Determinado en onNimBLESync().
static uint8_t ownAddrType;

// ─── Identidad del dispositivo: nombre BLE + token de autenticación ──────────

/**
 * @brief Genera el nombre BLE único y carga (o crea) el token de autenticación.
 *
 * Nombre: "ARGUS-XXYYZZYY" usando los últimos 4 bytes de la MAC WiFi.
 *   → Único por unidad, sin colisiones en tiendas con múltiples Argus encendidos.
 *
 * Token: 8 bytes aleatorios (hardware RNG via esp_fill_random).
 *   → Se persiste en NVS para sobrevivir reinicios.
 *   → Si no existe en NVS (primera vez), se genera y almacena.
 *   → La app lo obtiene vía QR durante el onboarding.
 *
 * Se llama una sola vez desde bleTask() antes de nimble_port_init().
 */
static void setupDeviceIdentity() {
    // Nombre: ARGUS-{últimos 4 bytes de MAC WiFi en hex}
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(_deviceName, sizeof(_deviceName), "ARGUS-%02X%02X%02X%02X",
             mac[2], mac[3], mac[4], mac[5]);

    // Token: cargar de NVS o generar uno nuevo
    nvs_handle_t h;
    if (nvs_open("argus_cfg", NVS_READWRITE, &h) == ESP_OK) {
        size_t len = sizeof(_bleToken);
        esp_err_t err = nvs_get_blob(h, "ble_token", _bleToken, &len);
        if (err != ESP_OK || len != sizeof(_bleToken)) {
            // Primera vez: generar token aleatorio seguro y guardarlo
            esp_fill_random(_bleToken, sizeof(_bleToken));
            nvs_set_blob(h, "ble_token", _bleToken, sizeof(_bleToken));
            nvs_commit(h);
            ESP_LOGI(TAG, "[BLE-IDENTITY] Token generado y almacenado en NVS");
        }
        nvs_close(h);
    } else {
        // Si NVS falla, generar token en RAM (no persiste — solo para la sesión)
        esp_fill_random(_bleToken, sizeof(_bleToken));
        ESP_LOGW(TAG, "[BLE-IDENTITY] NVS no disponible — token en RAM (no persiste)");
    }

    // ══ Imprimir datos QR para vincular en la app ══════════════════════════════
    // Formato QR: ARGUS-XXXXXXXX:HHHHHHHHHHHHHHHHH (deviceId:16hexbytes)
    ESP_LOGI(TAG, "══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  BLE ID:    %s", _deviceName);
    ESP_LOGI(TAG, "  QR DATA:   %s:%02X%02X%02X%02X%02X%02X%02X%02X", _deviceName,
             _bleToken[0], _bleToken[1], _bleToken[2], _bleToken[3],
             _bleToken[4], _bleToken[5], _bleToken[6], _bleToken[7]);
    ESP_LOGI(TAG, "══════════════════════════════════════════════════");
}

// ─── Handler de escritura de la característica de comandos ───────────────────

/**
 * @brief Callback de acceso GATT — se invoca al recibir escritura en la característica.
 *
 * PROPÓSITO:
 *   Traducir el byte de comando BLE al evento del sistema correspondiente y
 *   encolarlo en xEventQueue para que controlTask lo procese. Para los comandos
 *   de sensibilidad, actúa directamente sobre motionSensitivity sin pasar por la cola.
 *
 * CONTEXTO DE EJECUCIÓN:
 *   Corre en nimbleHostTask (la tarea interna de NimBLE), no en bleTask.
 *   Por eso puede usar xQueueSend con timeout corto sin bloquear el stack BLE
 *   más tiempo del necesario.
 *
 * FLUJO:
 *   1. Verificar que la operación es BLE_GATT_ACCESS_OP_WRITE_CHR (no READ).
 *   2. Verificar que el mbuf tiene al menos 1 byte.
 *   3. ble_hs_mbuf_to_flat() → extraer el byte del mbuf a cmd.
 *   4. Switch sobre cmd:
 *      - 0x01/0x02/0x03: construir EventMessage_t + validCmd=true.
 *      - 0x10-0x14: setSensitivityLevel() + validCmd=false (no encolar).
 *      - default: loguear advertencia + validCmd=false.
 *   5. Si validCmd: xQueueSend(xEventQueue, timeout=100ms).
 *   6. Retornar 0 (éxito BLE ATT).
 *
 * NOTA SOBRE validCmd:
 *   Los comandos de sensibilidad no generan eventos en xEventQueue porque
 *   setSensitivityLevel() actúa directamente sobre motionSensitivity (volatile).
 *   El efecto es inmediato: la próxima vez que sensorTask ejecute classifyMovement(),
 *   usará los nuevos umbrales. No hay necesidad de pasar por la cola de eventos.
 *
 * NOTA SOBRE mbuf:
 *   NimBLE usa mblocks (memory blocks) encadenados para los datos BLE.
 *   OS_MBUF_PKTLEN() retorna el total de bytes en todos los mblocks del paquete.
 *   ble_hs_mbuf_to_flat() copia los bytes a un buffer plano (stack local).
 *
 * @param connHandle  Handle de la conexión BLE activa.
 * @param attrHandle  Handle del atributo GATT al que se accede.
 * @param ctxt        Contexto de la operación GATT (mbuf con datos, tipo de operación).
 * @param arg         Argumento de usuario (nullptr en esta implementación).
 * @return 0 si éxito, BLE_ATT_ERR_* si error de protocolo BLE.
 */
static int cmdCharAccessCallback(uint16_t connHandle, uint16_t attrHandle,
                                  struct ble_gatt_access_ctxt* ctxt, void* arg) {
    // NimBLE puede llamar este callback también para lecturas (OP_READ).
    // Solo procesamos escrituras — para lecturas retornamos éxito sin hacer nada.
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return 0;
    }

    // Verificar que la app envió el token correcto antes de aceptar comandos.
    // _connAuthenticated se pone en true en authCharAccessCallback() y se resetea
    // en BLE_GAP_EVENT_DISCONNECT — cada reconexión debe autenticarse de nuevo.
    if (!_connAuthenticated) {
        ESP_LOGW(TAG, "[BLE-AUTH] Comando rechazado — conexión no autenticada");
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    // Un comando Argus ocupa entre 1 y 4 bytes. Paquetes vacíos son inválidos.
    uint16_t pktLen = (uint16_t)OS_MBUF_PKTLEN(ctxt->om);
    if (pktLen < 1) {
        ESP_LOGW(TAG, "Escritura BLE recibida con longitud inválida");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    // Leer hasta 4 bytes del mbuf en un buffer plano.
    // El primer byte es siempre el opcode. Los bytes adicionales son parámetros.
    uint8_t payload[4] = {0};
    uint16_t readLen = pktLen < sizeof(payload) ? pktLen : (uint16_t)sizeof(payload);
    ble_hs_mbuf_to_flat(ctxt->om, payload, readLen, &readLen);
    uint8_t cmd = payload[0];

    ESP_LOGI(TAG, "Comando BLE recibido: 0x%02X (len=%u)", cmd, (unsigned)readLen);

    EventMessage_t eventMsg;
    memset(&eventMsg, 0, sizeof(eventMsg));
    bool validCmd = true;  // true → encolar en xEventQueue; false → no encolar

    switch (cmd) {
        case BLE_CMD_ARM:
            eventMsg.event = EVENT_ARM_CMD;
            ESP_LOGI(TAG, "Comando BLE: ARM");
            break;

        case BLE_CMD_DISARM:
            eventMsg.event = EVENT_DISARM_CMD;
            ESP_LOGI(TAG, "Comando BLE: DISARM");
            break;

        case BLE_CMD_TRIGGER_ALERT:
            eventMsg.event = EVENT_TRIGGER_ALERT_CMD;
            ESP_LOGW(TAG, "Comando BLE: TRIGGER_ALERT");
            break;

        // ── Comandos de sensibilidad (0x10-0x14) ─────────────────────────────
        // Actúan directamente sobre motionSensitivity y NVS sin pasar por la cola.
        // El efecto es inmediato en el próximo ciclo de classifyMovement() en sensorTask.
        case BLE_CMD_SENSITIVITY_VERY_LOW:
            setSensitivityLevel(SENSITIVITY_VERY_LOW);
            ESP_LOGI(TAG, "Comando BLE: SENSITIVITY_VERY_LOW (0x10)");
            validCmd = false;
            break;

        case BLE_CMD_SENSITIVITY_LOW:
            setSensitivityLevel(SENSITIVITY_LOW);
            ESP_LOGI(TAG, "Comando BLE: SENSITIVITY_LOW (0x11)");
            validCmd = false;
            break;

        case BLE_CMD_SENSITIVITY_MEDIUM:
            setSensitivityLevel(SENSITIVITY_MEDIUM);
            ESP_LOGI(TAG, "Comando BLE: SENSITIVITY_MEDIUM (0x12)");
            validCmd = false;
            break;

        case BLE_CMD_SENSITIVITY_HIGH:
            setSensitivityLevel(SENSITIVITY_HIGH);
            ESP_LOGI(TAG, "Comando BLE: SENSITIVITY_HIGH (0x13)");
            validCmd = false;
            break;

        case BLE_CMD_SENSITIVITY_VERY_HIGH:
            setSensitivityLevel(SENSITIVITY_VERY_HIGH);
            ESP_LOGI(TAG, "Comando BLE: SENSITIVITY_VERY_HIGH (0x14)");
            validCmd = false;
            break;

        // ── Auto-ARM por inactividad (0x20 / 0x21) — solo PLAN_PREMIUM ────────
        // 0x20 solo    → habilitar con delay actual (default 5min).
        // 0x20 + byte  → habilitar con N minutos de delay (byte[1] = 1-60).
        // 0x21         → deshabilitar auto-ARM.
        case BLE_CMD_AUTO_ARM_ENABLE:
            if (planType == PLAN_PREMIUM) {
                uint32_t delayMin = autoArmDelayMs / 60000UL;  // valor actual en minutos
                if (readLen >= 2 && payload[1] >= 1 && payload[1] <= 60) {
                    delayMin = payload[1];
                }
                autoArmDelayMs = delayMin * 60000UL;
                autoArmEnabled = true;
                ESP_LOGI(TAG, "[AUTO-ARM] Habilitado: %lum de inactividad",
                         (unsigned long)delayMin);
            } else {
                ESP_LOGW(TAG, "[AUTO-ARM] Solo disponible en plan PREMIUM");
            }
            validCmd = false;
            break;

        case BLE_CMD_AUTO_ARM_DISABLE:
            autoArmEnabled = false;
            ESP_LOGI(TAG, "[AUTO-ARM] Deshabilitado");
            validCmd = false;
            break;

        default:
            // Byte desconocido: la app enviará algo inválido solo si hay un bug en ella.
            ESP_LOGW(TAG, "Comando BLE desconocido: 0x%02X", cmd);
            validCmd = false;
            break;
    }

    if (validCmd) {
        // Encolar el evento con timeout de 100ms.
        // Si la cola está llena durante 100ms, el comando BLE se descarta.
        // El usuario puede repetir el gesto en la app (ARM/DISARM tarda <1ms normalmente).
        if (xQueueSend(xEventQueue, &eventMsg, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Cola de eventos llena al procesar comando BLE");
        }
    }

    return 0;  // 0 indica éxito al stack NimBLE (respuesta ATT OK si aplica)
}

/**
 * @brief Callback de acceso GATT para la característica de estado (READ).
 *
 * La app puede leer el estado actual (armado/desarmado) en cualquier momento.
 * Las notificaciones (NOTIFY) se envían proactivamente desde bleNotifyState().
 *
 * Formato: 1 byte — 0x01 = ARMADO, 0x00 = DESARMADO.
 */
static int stateCharAccessCallback(uint16_t connHandle, uint16_t attrHandle,
                                    struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return 0;
    uint8_t state = systemArmed ? 0x01 : 0x00;
    int rc = os_mbuf_append(ctxt->om, &state, sizeof(state));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/**
 * @brief Callback GATT para la característica de autenticación.
 *
 * La app escribe los 8 bytes del token (obtenido vía QR en onboarding).
 * Si el token coincide con el almacenado en NVS, se autoriza la conexión y
 * se pueden ejecutar comandos en la característica CMD.
 *
 * Seguridad en capas:
 *  - Capa 1: BLE Secure Connections (bonding + cifrado AES-128 automático)
 *  - Capa 2: Token de aplicación — conocimiento compartido app↔firmware
 *
 * Retorna BLE_ATT_ERR_INSUFFICIENT_AUTHEN si el token es incorrecto,
 * lo que cierra la conexión desde el punto de vista del protocolo ATT.
 */
static int authCharAccessCallback(uint16_t connHandle, uint16_t attrHandle,
                                   struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint8_t received[8] = {};
    uint16_t len = 0;
    ble_hs_mbuf_to_flat(ctxt->om, received, sizeof(received), &len);

    if (len == sizeof(_bleToken) &&
        memcmp(received, _bleToken, sizeof(_bleToken)) == 0) {
        _connAuthenticated = true;
        ESP_LOGI(TAG, "[BLE-AUTH] Token válido — conexión autorizada");
        bleNotifyState(systemArmed);  // Enviar estado actual al app
    } else {
        _connAuthenticated = false;
        ESP_LOGW(TAG, "[BLE-AUTH] Token INVÁLIDO (len=%u) — acceso denegado", (unsigned)len);
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    return 0;
}

// ─── Tabla de servicios GATT ──────────────────────────────────────────────────
//
// Los structs de NimBLE son structs C con muchos campos. Los campos no inicializados
// se ponen en cero automáticamente (inicialización agregada en C++). La advertencia
// -Wmissing-field-initializers es ruidosa pero correcta: los campos no listados
// son 0/nullptr que es el valor correcto para punteros nulos y flags vacíos.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

/**
 * Array de características del servicio Argus.
 * El terminador { } (todos los campos en cero) es obligatorio para NimBLE —
 * es la forma en que NimBLE detecta el fin del array (sentinel pattern).
 *
 * WRITE_NO_RSP: permite escrituras sin confirmación ATT (Write Command en BLE).
 *               La app puede usar Write Request (WRITE) o Write Command (WRITE_NO_RSP).
 * WRITE:        permite escrituras con confirmación ATT (Write Request en BLE).
 *               Ambas propiedades activan el mismo callback cmdCharAccessCallback.
 */
static const struct ble_gatt_chr_def argusCharacteristics[] = {
    {
        // CMD — requiere cifrado BLE (_ENC) que activa bonding automático en primera
        // escritura. Además requiere autenticación de aplicación (_connAuthenticated).
        .uuid         = &ARGUS_CMD_CHAR_UUID.u,
        .access_cb    = cmdCharAccessCallback,
        .descriptors  = nullptr,
        .flags        = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                        BLE_GATT_CHR_F_WRITE_ENC,
        .min_key_size = 0,
        .val_handle   = &cmdCharHandle,
        .cpfd         = nullptr,
    },
    {
        // STATE — pública (sin cifrado): la app puede leer y suscribirse a NOTIFY
        // incluso antes de autenticarse. NimBLE crea el CCCD automáticamente.
        .uuid         = &ARGUS_STATE_CHAR_UUID.u,
        .access_cb    = stateCharAccessCallback,
        .descriptors  = nullptr,
        .flags        = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle   = &stateCharHandle,
        .cpfd         = nullptr,
    },
    {
        // AUTH — App escribe el token de 8 bytes (QR) para autenticar la sesión.
        // Requiere cifrado (_ENC) para proteger el token en tránsito.
        .uuid         = &ARGUS_AUTH_CHAR_UUID.u,
        .access_cb    = authCharAccessCallback,
        .descriptors  = nullptr,
        .flags        = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
        .min_key_size = 0,
        .val_handle   = &authCharHandle,
        .cpfd         = nullptr,
    },
    { }  // Terminador obligatorio de NimBLE (sentinel)
};

/**
 * Tabla de servicios GATT del sistema Argus.
 * Un solo servicio primario con una sola característica.
 * El terminador { } al final es obligatorio.
 *
 * BLE_GATT_SVC_TYPE_PRIMARY: servicio principal (visible en descubrimiento de servicios).
 * includes=nullptr: no incluye otros servicios.
 */
static const struct ble_gatt_svc_def gattServices[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &ARGUS_SVC_UUID.u,
        .includes        = nullptr,              // Sin servicios incluidos
        .characteristics = argusCharacteristics,
    },
    { }  // Terminador obligatorio de NimBLE (sentinel)
};

#pragma GCC diagnostic pop

// ─── Callbacks del stack NimBLE ───────────────────────────────────────────────

// Forward declaration — bleGapEventHandler se define tras startAdvertising para
// poder llamar startAdvertising() en el handler de desconexión.
static int bleGapEventHandler(struct ble_gap_event* event, void* arg);

/**
 * @brief Inicia el advertising BLE para que los clientes puedan descubrir el dispositivo.
 *
 * PROPÓSITO:
 *   Configurar y activar el advertising BLE. Se llama al arrancar (desde onNimBLESync)
 *   y debería llamarse tras cada desconexión para volver a ser descubrible.
 *
 * FLUJO:
 *   1. ble_gap_adv_set_fields(): configurar el nombre del dispositivo en el paquete
 *      de advertising (los clientes ven este nombre al escanear).
 *   2. ble_gap_adv_start(): iniciar advertising connectable undirected.
 *      - conn_mode = UND: cualquier central puede conectarse (no directed a un peer).
 *      - disc_mode = GEN: modo general discoverable (visible siempre).
 *      - itvl_min = itvl_max = 160: intervalo fijo de 100ms (160 × 0.625ms).
 *        Un intervalo fijo evita variabilidad en la latencia de descubrimiento.
 *      - BLE_HS_FOREVER: no hay timeout de advertising (corre indefinidamente).
 *      - nullptr para los callbacks de gap: no se gestionan eventos de conexión aquí.
 *
 * NOTA: BLE_HS_EALREADY indica que ya había advertising activo — se ignora.
 *   Puede ocurrir si onNimBLESync se llama múltiples veces (raro pero posible).
 */
static void startAdvertising() {
    struct ble_gap_adv_params advParams = {};
    struct ble_hs_adv_fields fields = {};

    fields.name             = (const uint8_t*)_deviceName;
    fields.name_len         = strlen(_deviceName);
    fields.name_is_complete = 1;  // El nombre completo cabe en el paquete de advertising

    // F_DISC_GEN: general discoverable (visible en scans generales de la app).
    // F_BREDR_UNSUP: indicar que no soportamos Bluetooth Classic (BR/EDR).
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error configurando advertising fields: %d", rc);
        return;
    }

    advParams.conn_mode  = BLE_GAP_CONN_MODE_UND;   // Cualquier central puede conectarse
    advParams.disc_mode  = BLE_GAP_DISC_MODE_GEN;   // Visible en scans generales
    advParams.itvl_min   = 160;  // 160 × 0.625ms = 100ms
    advParams.itvl_max   = 160;  // Igual que min para intervalo fijo

    rc = ble_gap_adv_start(ownAddrType, nullptr, BLE_HS_FOREVER,
                            &advParams, bleGapEventHandler, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Error iniciando advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE advertising iniciado como \"%s\"", _deviceName);
    }
}

/**
 * @brief Handler de eventos GAP — gestiona conexión, desconexión y advertising.
 *
 * Se registra en ble_gap_adv_start(). Corre en el contexto de nimbleHostTask.
 *
 * CONNECT:    guarda el handle de conexión → _connHandle.
 * DISCONNECT: limpia _connHandle, reinicia advertising para aceptar próxima conexión.
 */
static int bleGapEventHandler(struct ble_gap_event* event, void* arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                _connHandle = event->connect.conn_handle;
                ESP_LOGI(TAG, "BLE cliente conectado (handle=%d)", _connHandle);
            } else {
                ESP_LOGW(TAG, "BLE conexión fallida (status=%d)", event->connect.status);
                startAdvertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE cliente desconectado (razón=%d) — reiniciando advertising",
                     event->disconnect.reason);
            _connHandle = BLE_HS_CONN_HANDLE_NONE;
            _connAuthenticated = false;  // Próxima sesión debe autenticarse de nuevo
            startAdvertising();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            // La app habilitó (o deshabilitó) NOTIFY en la característica de estado.
            // Si está habilitando: enviar el estado actual inmediatamente para que
            // la app sepa si la moto está armada al conectar, sin esperar un cambio.
            if (event->subscribe.cur_notify && !event->subscribe.prev_notify) {
                bleNotifyState(systemArmed);
                ESP_LOGI(TAG, "BLE NOTIFY habilitado → estado actual enviado (%s)",
                         systemArmed ? "ARMADO" : "DESARMADO");
            }
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            // El advertising expiró (no ocurre con BLE_HS_FOREVER).
            ESP_LOGI(TAG, "BLE advertising completado");
            break;

        default:
            break;
    }
    return 0;
}

/**
 * @brief Callback invocado cuando el stack NimBLE está sincronizado y listo.
 *
 * PROPÓSITO:
 *   Determinar la dirección BLE a usar (pública de la MAC del ESP32 o aleatoria)
 *   e iniciar el advertising. Es el punto de partida de la visibilidad BLE del dispositivo.
 *
 * FLUJO:
 *   1. ble_hs_util_ensure_addr(0): asegurar que hay una dirección BLE disponible.
 *      El '0' indica preferir dirección pública (si no hay, usa la aleatoria).
 *   2. ble_hs_id_infer_auto(0, &ownAddrType): inferir qué tipo de dirección usar
 *      y almacenar en ownAddrType (0=pública, 1=aleatoria).
 *   3. startAdvertising().
 *
 * CUÁNDO SE LLAMA:
 *   Cuando el host NimBLE completa la inicialización y está listo para operar.
 *   Se registra en bleTask() con: ble_hs_cfg.sync_cb = onNimBLESync.
 */
static void onNimBLESync() {
    ESP_LOGI(TAG, "Stack NimBLE sincronizado");

    // Asegurar dirección BLE disponible (0 = preferir pública).
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error asegurando dirección BLE: %d", rc);
        return;
    }

    // Inferir el tipo de dirección (pública o aleatoria según lo disponible).
    rc = ble_hs_id_infer_auto(0, &ownAddrType);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error infiriendo tipo de dirección: %d", rc);
        return;
    }

    startAdvertising();
}

/**
 * @brief Callback invocado cuando el stack NimBLE se resetea inesperadamente.
 *
 * PROPÓSITO:
 *   Registrar el reseteo para diagnóstico. NimBLE puede resetearse si el
 *   controlador BT del ESP32 reporta un error HCI (Host-Controller Interface)
 *   o si hay un error de inicialización en el host.
 *
 * NOTA: Tras un reset, NimBLE llamará onNimBLESync() nuevamente cuando se
 *   recupere, lo que reiniciará el advertising automáticamente.
 *
 * @param reason Código de razón del reset (BLE_HS_ETIMEOUT, BLE_HS_ENOENT, etc.).
 */
static void onNimBLEReset(int reason) {
    ESP_LOGW(TAG, "Stack NimBLE reseteado (razón: %d). Reiniciando...", reason);
}

// ─── Tarea host de NimBLE ─────────────────────────────────────────────────────

/**
 * @brief Tarea interna del host NimBLE — corre el loop de eventos BLE.
 *
 * PROPÓSITO:
 *   Ejecutar el loop de procesamiento de eventos del stack NimBLE.
 *   nimble_port_run() bloquea indefinidamente procesando eventos BLE (conexiones,
 *   escrituras GATT, advertising events, HCI commands).
 *   nimble_port_freertos_init() crea esta tarea automáticamente.
 *
 * CUÁNDO TERMINA:
 *   Solo cuando se llama nimble_port_stop() desde otro contexto.
 *   En el sistema Argus, no se llama nunca — BLE corre permanentemente.
 *
 * NOTA: Esta función NO es bleTask(). bleTask() inicializa NimBLE y se suspende.
 *   Esta función es la tarea interna de NimBLE que corre el loop de eventos BLE.
 *
 * @param param No usado por NimBLE.
 */
static void nimbleHostTask(void* param) {
    ESP_LOGI(TAG, "Tarea host NimBLE iniciada");
    nimble_port_run();  // Bloquea aquí mientras BLE está activo
    // Se llega aquí solo si nimble_port_stop() fue llamado (no ocurre en uso normal).
    nimble_port_freertos_deinit();
}

// ─── Tarea principal BLE ──────────────────────────────────────────────────────

/**
 * @brief Función principal de la tarea FreeRTOS BLE — inicializa NimBLE y se suspende.
 *
 * PROPÓSITO:
 *   Inicializar el servidor GATT del sistema Argus y lanzar la tarea host de NimBLE.
 *   Tras la inicialización, bleTask() se suspende. Todo el procesamiento BLE ocurre
 *   en nimbleHostTask de forma autónoma.
 *
 * FLUJO DE ARRANQUE:
 *   1. nimble_port_init()          → inicializar el controlador BT del ESP32.
 *      Si falla (ESP_OK esperado): vTaskDelete(nullptr).
 *   2. ble_hs_cfg.sync_cb/reset_cb → registrar callbacks del host.
 *   3. ble_svc_gap_device_name_set() → nombre GAP "Argus-Secure".
 *   4. ble_svc_gap_init() + ble_svc_gatt_init() → servicios GAP/GATT estándar.
 *   5. ble_gatts_count_cfg(gattServices) → contar atributos (asignación de memoria).
 *      Si falla: vTaskDelete(nullptr).
 *   6. ble_gatts_add_svcs(gattServices) → registrar servicios GATT propietarios.
 *      Si falla: vTaskDelete(nullptr).
 *   7. nimble_port_freertos_init(nimbleHostTask) → lanzar tarea host NimBLE.
 *      NimBLE creará su propia tarea FreeRTOS con prioridad y stack configurados
 *      en menuconfig (BLE_NimBLE_HOST_TASK_STACK_SIZE).
 *   8. vTaskSuspend(nullptr) → bleTask se suspende indefinidamente.
 *
 * MANEJO DE ERRORES:
 *   Si nimble_port_init() falla, el sistema sigue funcionando sin BLE.
 *   sensorTask/controlTask/commTask son independientes de BLE.
 *   ARM/DISARM solo puede hacerse via BLE actualmente — sin BLE el sistema
 *   queda en IDLE desarmado permanentemente hasta un reboot.
 *
 * @param pvParameters No usado (nullptr). Requerido por la firma de tarea FreeRTOS.
 */
void bleTask(void* pvParameters) {
    ESP_LOGI(TAG, "BLE task iniciada. Configurando stack NimBLE...");

    // Configurar nombre BLE único y cargar/generar token de autenticación.
    // Debe hacerse antes de nimble_port_init() para tener _deviceName listo.
    setupDeviceIdentity();

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando NimBLE: %s", esp_err_to_name(ret));
        vTaskDelete(nullptr);
        return;
    }

    // ── Security Manager: Secure Connections (BLE 4.2+) + bonding ────────────
    // Just Works: sin PIN visual. La capa de token de aplicación provee la
    // autenticación real. El cifrado BLE protege el token en tránsito.
    ble_hs_cfg.sm_io_cap           = BLE_HS_IO_NO_INPUT_OUTPUT;  // Just Works
    ble_hs_cfg.sm_bonding          = 1;   // Guardar claves LTK para reconexión rápida
    ble_hs_cfg.sm_sc               = 1;   // Secure Connections (ECDH, AES-CMAC)
    ble_hs_cfg.sm_our_key_dist     = BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC;

    // Registrar callbacks del host BLE. Deben configurarse antes de que
    // nimble_port_freertos_init() arranque la tarea host.
    ble_hs_cfg.sync_cb  = onNimBLESync;    // Llamado cuando el host está listo para operar
    ble_hs_cfg.reset_cb = onNimBLEReset;   // Llamado tras un reset inesperado del host

    // Nombre del dispositivo para el servicio GAP (visible en "device info" BLE).
    ble_svc_gap_device_name_set(_deviceName);

    // Inicializar los servicios estándar GAP y GATT de NimBLE.
    // Deben inicializarse ANTES de registrar servicios propietarios.
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // ble_gatts_count_cfg(): cuenta los atributos de nuestra tabla de servicios
    // y reserva la memoria necesaria en el GATT server de NimBLE.
    int rc = ble_gatts_count_cfg(gattServices);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error contando configuración GATT: %d", rc);
        vTaskDelete(nullptr);
        return;
    }

    // ble_gatts_add_svcs(): registra los servicios propietarios en el GATT server.
    // Asigna handles ATT a cada servicio y característica (incluido cmdCharHandle).
    rc = ble_gatts_add_svcs(gattServices);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error registrando servicios GATT: %d", rc);
        vTaskDelete(nullptr);
        return;
    }

    // Lanzar la tarea host de NimBLE. Esta tarea corre nimble_port_run() que
    // procesa todos los eventos BLE: conexiones, escrituras GATT, advertising, HCI.
    // bleTask() no necesita hacer nada más — el stack BLE corre de forma autónoma.
    nimble_port_freertos_init(nimbleHostTask);

    ESP_LOGI(TAG, "Servidor BLE inicializado. UUID servicio: 4FAFC201-...");

    // bleTask se suspende después de inicializar NimBLE.
    // Puede reanudarse en el futuro con vTaskResume(xBleTaskHandle) si se necesita
    // lógica adicional (ej. enviar notificaciones INDICATE a la app).
    vTaskSuspend(nullptr);
}


// ─── API pública BLE ──────────────────────────────────────────────────────────

/**
 * @brief Retorna true si hay un cliente BLE conectado actualmente.
 *
 * Thread-safe: _connHandle es volatile uint16_t, su lectura es atómica en Xtensa LX6.
 * Puede llamarse desde cualquier tarea FreeRTOS.
 */
bool bleIsConnected() {
    return _connHandle != BLE_HS_CONN_HANDLE_NONE;
}

/**
 * @brief Envía una notificación BLE a la app con el estado ARM/DISARM.
 *
 * PROPÓSITO:
 *   Notificar proactivamente a la app cuando el estado del sistema cambia,
 *   sin que la app tenga que hacer polling. Usado desde control_task / state_machine
 *   al procesar EVENT_ARM_CMD / EVENT_DISARM_CMD.
 *
 * FORMATO: 1 byte — 0x01 = ARMADO, 0x00 = DESARMADO.
 *
 * No hace nada si no hay cliente conectado o si el cliente no ha habilitado NOTIFY
 * (suscribiéndose al CCCD de esta característica). NimBLE maneja el error silenciosamente.
 *
 * Thread-safe: ble_gatts_notify_custom() es seguro desde cualquier tarea FreeRTOS.
 *
 * @param armed true = sistema armado, false = desarmado.
 */
void bleNotifyState(bool armed) {
    uint16_t connHandle = _connHandle;
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;

    uint8_t state = armed ? 0x01 : 0x00;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(&state, sizeof(state));
    if (!om) {
        ESP_LOGW(TAG, "bleNotifyState: sin memoria para mbuf");
        return;
    }

    int rc = ble_gatts_notify_custom(connHandle, stateCharHandle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE NOTIFY falló (rc=%d) — cliente puede no estar suscrito", rc);
    } else {
        ESP_LOGI(TAG, "BLE NOTIFY estado → %s", armed ? "ARMADO" : "DESARMADO");
    }
}

/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/services/ble_task.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   ble_task.cpp es el "intérprete" de comandos BLE. La app móvil Argus envía
   un byte al ESP32 via Bluetooth, y este módulo decide qué hacer con él:
   - ARM/DISARM/ALERT → pone el evento en la cola para que controlTask actúe.
   - Sensibilidad 0x10-0x14 → cambia el nivel directamente sin pasar por la cola.

   El servidor BLE usa el stack NimBLE (más ligero que Bluedroid). La tarea
   bleTask solo inicializa el servidor y se suspende. El stack NimBLE corre en
   su propia tarea interna (nimbleHostTask) que procesa todos los eventos BLE.

   PSEUDOCÓDIGO:
   ```
   bleTask():
     nimble_port_init()
     registrar callbacks (sync, reset)
     configurar nombre GAP "Argus-Secure"
     registrar tabla GATT (servicio + característica)
     iniciar tarea host nimbleHostTask
     vTaskSuspend()  ← bleTask queda suspendida

   nimbleHostTask():
     nimble_port_run()  ← loop infinito de eventos BLE

   onNimBLESync():
     determinar tipo de dirección BLE
     startAdvertising()  ← intervalo 100ms, connectable

   cmdCharAccessCallback():
     leer 1 byte del paquete BLE
     0x01/0x02/0x03 → xQueueSend(EVENT_ARM/DISARM/ALERT)
     0x10-0x14      → setSensitivityLevel()
   ```

   DEUDA TÉCNICA:
   1. Sin autenticación BLE (bonding/MITM). Cualquier dispositivo puede conectarse.
   2. No hay callback de desconexión: si el cliente se desconecta, el advertising
      se reinicia solo porque onNimBLESync se llama de nuevo tras cada resync.
      Pero si NimBLE no se resetea, el advertising puede no reiniciarse.
   3. cmdCharHandle nunca se usa después de la inicialización (no hay NOTIFY/INDICATE).
   4. No hay timeout de conexión: un cliente puede mantenerse conectado indefinidamente
      sin enviar comandos, bloqueando potencialmente a otros clientes.

   ═══════════════════════════════════════════════════════════ */
