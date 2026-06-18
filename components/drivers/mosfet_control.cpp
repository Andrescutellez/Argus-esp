/**
 * @file mosfet_control.cpp
 * @brief Implementación del control de MOSFETs y LEDs de estado del sistema Argus.
 *
 * PROPÓSITO:
 *   Implementar los métodos de MosfetControl: inicialización GPIO, control de buzzer,
 *   corte de motor y patrones de LEDs de estado. Este driver es la capa más baja
 *   del sistema de actuación — todo lo que el firmware quiera hacer al mundo físico
 *   (sonar una alarma, cortar un motor, encender un LED) pasa por aquí.
 *
 * LÓGICA DE CONTROL:
 *   Todos los actuadores usan lógica positiva:
 *   GPIO = HIGH (nivel 1) → MOSFET conduce → carga activada
 *   GPIO = LOW  (nivel 0) → MOSFET no conduce → carga desactivada
 *
 *   La función gpio_set_level() actualiza el registro de salida del GPIO inmediatamente.
 *   En el ESP32, las escrituras GPIO son atómicas para bits individuales usando el
 *   registro W1TS (Write 1 To Set) y W1TC (Write 1 To Clear).
 *
 * PATRÓN DE applyLedPattern():
 *   Se llama cada ~250ms desde control_task con un tick incremental.
 *   Los patrones se basan en el módulo del tick:
 *   - tick % 8 == 0: evento cada 8 × 250ms = cada 2 segundos (latido lento)
 *   - tick % 2 == 0: evento cada 2 × 250ms = cada 500ms (parpadeo rápido)
 *   La frecuencia de parpadeo es 1Hz para el latido y 2Hz para las alertas.
 *
 * VARIABLES CRÍTICAS:
 *   - buzzerState, engineCutState, led1State, led2State: variables de estado estáticas.
 *     Deben inicializarse a false (lo hace el compilador de C++ para estáticas) y
 *     mantenerse sincronizadas con el nivel real del GPIO en cada operación.
 *   - currentSystemState (system_flags.h): leído por applyLedPattern() sin mutex.
 *     Atómico para SystemState_t (enum de 4 bytes en ARM Cortex-M).
 *   - systemArmed (system_flags.h): leído por applyLedPattern() para distinguir
 *     IDLE armado (LED fijo) de IDLE desarmado (latido).
 *
 * CONCURRENCIA:
 *   - gpio_set_level() es thread-safe en ESP-IDF (usa registros atómicos W1TS/W1TC).
 *   - Los variables de estado bool (buzzerState, etc.) no son atómicos en teoría,
 *     pero en práctica solo control_task escribe estas variables, así que no hay
 *     race conditions en la arquitectura actual de Argus.
 *   - applyLedPattern() lee currentSystemState sin mutex. Aceptable porque:
 *     a) La lectura de un enum es atómica en ARM.
 *     b) Un valor transitorio de estado solo causa un frame de LED "incorrecto".
 *     c) El overhead de tomar xStateMutex cada 250ms no justifica el beneficio.
 *
 * RIESGOS DE HARDWARE:
 *   - GPIO13 (ENGINE_CUT): ver comentario en setEngineCut() sobre seguridad de activación.
 *   - GPIO33 (BUZZER): el buzzer en modo continuo puede drenar la batería de la moto.
 *     Si el sistema se queda en ALERT por días sin intervención, la batería se agota.
 *
 * @module components/drivers/mosfet_control
 */

#include "mosfet_control.h"
#include "pin_config.h"
#include "system_flags.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "MosfetControl";

// ─── Estado interno (inicializado a false por el compilador C++ para estáticas) ──
// Estos variables reflejan el estado actual de cada salida GPIO.
// Se actualizan en cada función set*() o toggle*() para mantener consistencia.
bool MosfetControl::buzzerState    = false;
bool MosfetControl::engineCutState = false;
bool MosfetControl::led1State      = false;
bool MosfetControl::led2State      = false;

// ─── Inicialización ───────────────────────────────────────────────────────────

/**
 * @brief Configura los cuatro GPIOs como salidas e inicializa todos en bajo.
 *
 * PROPÓSITO:
 *   Preparar las salidas de potencia antes de que el firmware comience a operar.
 *   Garantizar que el buzzer y el ENGINE_CUT no se activen accidentalmente en el
 *   boot del ESP32.
 *
 * FLUJO LÓGICO:
 *   gpio_config() para los 4 pines → gpio_set_level(0) para cada pin.
 *
 * NOTAS DE IMPLEMENTACIÓN:
 *   - pin_bit_mask usa una máscara de bits OR de los pines configurados.
 *     La API gpio_config() aplica la misma configuración a todos los pines en la máscara.
 *   - GPIO_INTR_DISABLE: sin interrupciones en las salidas — no tiene sentido detectar
 *     cambios en pines que el propio firmware controla.
 *   - GPIO_PULLDOWN_DISABLE y GPIO_PULLUP_DISABLE: en salidas, los pull-ups/down
 *     no tienen efecto eléctrico real (el GPIO conduce directamente) pero se deshabilitan
 *     para claridad y para evitar corriente parásita innecesaria.
 *
 * ORDEN DE OPERACIÓN:
 *   gpio_config() primero (sin este paso, gpio_set_level() tiene efecto indeterminado).
 *   gpio_set_level() después, para forzar bajo inmediatamente después de configurar.
 *   Esto minimiza la ventana de tiempo con estado indeterminado.
 */
void MosfetControl::init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    // Máscara con todos los pines a configurar en una sola operación.
    // Más eficiente que 4 llamadas a gpio_config() separadas.
    io_conf.pin_bit_mask = (1ULL << PIN_MOSFET_BUZZER) |
                           (1ULL << PIN_MOSFET_ENGINE) |
                           (1ULL << PIN_LED_STATUS_1)  |
                           (1ULL << PIN_LED_STATUS_2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Forzar nivel bajo en todos los pines inmediatamente después de configurar.
    // Si el PCB no tiene resistencias pull-down externas en los gates de los MOSFETs,
    // estos milisegundos entre power-on y esta línea son una ventana de riesgo.
    gpio_set_level(PIN_MOSFET_BUZZER, 0);
    gpio_set_level(PIN_MOSFET_ENGINE, 0);
    gpio_set_level(PIN_LED_STATUS_1,  0);
    gpio_set_level(PIN_LED_STATUS_2,  0);

    ESP_LOGI(TAG, "Salidas GPIO inicializadas (GPIO33 buzzer, GPIO13 motor, GPIO32/25 LEDs)");
}

// ─── Buzzer ───────────────────────────────────────────────────────────────────

/**
 * @brief Activa o desactiva el buzzer de alarma.
 *
 * PROPÓSITO:
 *   Control directo del buzzer piezoeléctrico o bocina conectada al MOSFET en GPIO33.
 *   La activación queda registrada en el log de sistema (WARN por ser evento anómalo).
 *
 * NOTA DE LOGGING:
 *   ESP_LOGW (warning) cuando se activa porque el buzzer sonando es siempre un
 *   evento que requiere atención (no debería activarse en operación normal).
 *   ESP_LOGI (info) cuando se desactiva porque es un retorno al estado normal.
 */
void MosfetControl::setBuzzer(bool active) {
    buzzerState = active;
    gpio_set_level(PIN_MOSFET_BUZZER, active ? 1 : 0);
    if (active) {
        ESP_LOGW(TAG, "Buzzer ACTIVADO");
    } else {
        ESP_LOGI(TAG, "Buzzer desactivado");
    }
}

/**
 * @brief Invierte el estado actual del buzzer.
 *
 * PROPÓSITO:
 *   Generar parpadeo de alarma sin que control_task tenga que rastrear el estado.
 *   Al llamar toggleBuzzer() periódicamente, el buzzer alterna ON/OFF creando
 *   el efecto de alarma intermitente característico del modo ALERT.
 *
 * NOTA DE IMPLEMENTACIÓN:
 *   La inversión de buzzerState y la escritura GPIO son dos operaciones separadas.
 *   Si el sistema tuviera múltiples tareas llamando toggleBuzzer() simultáneamente,
 *   podría ocurrir una race condition (dos toggles consecutivos que se anulan).
 *   En Argus, solo control_task llama esta función, así que no hay riesgo.
 */
void MosfetControl::toggleBuzzer() {
    buzzerState = !buzzerState;
    gpio_set_level(PIN_MOSFET_BUZZER, buzzerState ? 1 : 0);
}

bool MosfetControl::isBuzzerActive() {
    // Retorna el estado en memoria en lugar de leer el GPIO (evita un acceso al registro).
    // buzzerState siempre es coherente con GPIO33 porque cada función que modifica
    // el pin también actualiza esta variable.
    return buzzerState;
}

// ─── Corte de motor ───────────────────────────────────────────────────────────

/**
 * @brief Activa o desactiva el MOSFET de corte de motor/alimentación.
 *
 * PROPÓSITO:
 *   Control del actuador de mayor impacto del sistema: el MOSFET que corta el
 *   circuito de arranque o alimentación de la moto. Nivel alto en GPIO13 = corte activo.
 *
 * LOGGING ESPECIAL:
 *   ESP_LOGE (error level) cuando se activa el corte — no porque sea un error del software,
 *   sino porque es el evento más severo que el firmware puede ejecutar. Usar el nivel
 *   de error garantiza que aparezca en los logs incluso si el nivel de log está
 *   configurado en WARN o ERROR. Así siempre queda registro del momento de activación.
 *
 * ADVERTENCIA DE SEGURIDAD:
 *   Esta función ejecuta la acción sin verificación adicional. El caller (control_task.cpp)
 *   es responsable de verificar:
 *   1. Que la moto esté en modo PURSUIT (no activar en MOVING por error).
 *   2. Que el operador haya confirmado explícitamente la acción.
 *   3. Preferiblemente verificar que la velocidad GPS sea baja antes de cortar.
 *   ARQUITECTURA ⚠️: no hay verificación de velocidad antes de activar ENGINE_CUT.
 *
 * @param active true = GPIO13 HIGH (corte activo), false = GPIO13 LOW (motor libre).
 */
void MosfetControl::setEngineCut(bool active) {
    engineCutState = active;
    gpio_set_level(PIN_MOSFET_ENGINE, active ? 1 : 0);
    if (active) {
        // Nivel ERROR garantiza visibilidad en el log independientemente del nivel configurado.
        ESP_LOGE(TAG, "CORTE DE MOTOR ACTIVADO - GPIO13 en HIGH");
    } else {
        ESP_LOGI(TAG, "Corte de motor desactivado - GPIO13 en LOW");
    }
}

bool MosfetControl::isEngineCutActive() {
    return engineCutState;
}

// ─── LEDs ─────────────────────────────────────────────────────────────────────

// Los cuatro métodos de control de LED son simétricos: actualizan el estado
// interno y luego escriben el GPIO. Sin logging por ser operaciones de alta
// frecuencia (llamadas cada 250ms desde applyLedPattern).

void MosfetControl::setLed1(bool active) {
    led1State = active;
    gpio_set_level(PIN_LED_STATUS_1, active ? 1 : 0);
}

void MosfetControl::setLed2(bool active) {
    led2State = active;
    gpio_set_level(PIN_LED_STATUS_2, active ? 1 : 0);
}

void MosfetControl::toggleLed1() {
    led1State = !led1State;
    gpio_set_level(PIN_LED_STATUS_1, led1State ? 1 : 0);
}

void MosfetControl::toggleLed2() {
    led2State = !led2State;
    gpio_set_level(PIN_LED_STATUS_2, led2State ? 1 : 0);
}

/**
 * @brief Aplica el patrón de LEDs correspondiente al estado actual del sistema.
 *
 * PROPÓSITO:
 *   Implementar la máquina de estados visual del sistema Argus. Cada estado
 *   tiene un patrón de parpadeo distinto para que el propietario de la moto
 *   o un técnico pueda determinar el estado del sistema solo mirando los LEDs.
 *
 * TIMING:
 *   tick se incrementa cada ~250ms. Por lo tanto:
 *   - tick % 2 == 0: alterna cada 250ms → frecuencia de parpadeo = 2Hz
 *   - tick % 8 == 0: evento cada 2000ms → latido a 0.5Hz (visible como pulso breve)
 *
 * ESTADOS Y PATRONES:
 *   IDLE + desarmado:
 *     LED1 = ON solo cuando (tick % 8 == 0) → pulso de 250ms cada 2 segundos.
 *     LED2 = OFF siempre.
 *     El pulso único largo indica que el sistema está vivo pero pasivo.
 *     8 ticks × 250ms/tick = 2000ms de período.
 *
 *   IDLE + armado:
 *     LED1 = ON continuo.
 *     LED2 = OFF.
 *     El LED fijo indica vigilancia activa, moto en reposo.
 *
 *   MOVING:
 *     LED1 y LED2 = parpadeo sincronizado a 2Hz.
 *     Indica que el sistema detectó movimiento y está evaluando si es robo.
 *     El parpadeo de ambos LEDs al mismo tiempo crea un efecto de destello fuerte.
 *
 *   ALERT:
 *     Mismo patrón que MOVING (ambos a 2Hz sincronizados).
 *     ARQUITECTURA ⚠️: ALERT y MOVING son indistinguibles por LEDs. La diferencia
 *     solo es audible (buzzer activo en ALERT) o visible en la app del operador.
 *
 *   PURSUIT:
 *     LED1 y LED2 alternados: cuando LED1 está ON, LED2 está OFF y viceversa.
 *     Frecuencia 2Hz por eje → efecto de "sirena de policía" visual.
 *     El patrón alternado (no sincronizado) claramente distingue PURSUIT de ALERT.
 *
 * DEPENDENCIAS:
 *   - currentSystemState: extern de system_flags.h, escrito por stateMachine.
 *   - systemArmed: extern de system_flags.h, escrito por control_task.
 *   Ambas lecturas son atómicas para tipos simples en ARM Cortex-M.
 *
 * @param tick Contador de ciclos incrementado en cada llamada (~250ms entre llamadas).
 */
void MosfetControl::applyLedPattern(uint32_t tick) {
    // Leer el estado actual del sistema. Sin mutex por ser lectura atómica de enum.
    SystemState_t state = currentSystemState;

    switch (state) {
        case STATE_IDLE:
            if (systemArmed) {
                // Armado en reposo: LED1 encendido fijo como indicador de vigilancia activa.
                // Si el propietario ve LED1 fijo = la moto está siendo monitoreada.
                setLed1(true);
                setLed2(false);
            } else {
                // Desarmado: latido lento de LED1 (1 de cada 8 ticks = cada 2 segundos).
                // tick % 8 == 0 es true solo 1 tick de cada 8 → pulso de 250ms cada 2s.
                // Si los LEDs estuvieran completamente apagados, parecería que el sistema está muerto.
                setLed1((tick % 8) == 0);
                setLed2(false);
            }
            break;

        case STATE_MOVING:
            // Movimiento detectado: parpadeo rápido sincronizado de ambos LEDs.
            // tick % 2 alterna entre 0 y 1 en cada llamada → 2Hz de parpadeo.
            // Ambos LEDs parpadean al mismo tiempo para máxima visibilidad.
            {
                bool blink = (tick % 2) == 0;
                setLed1(blink);
                setLed2(blink);
            }
            break;

        case STATE_ALERT:
            // Alerta activa: mismo patrón que MOVING (parpadeo sincronizado rápido).
            // La distinción entre ALERT y MOVING es solo por el buzzer y el backend.
            // Si se necesita distinción visual, cambiar uno de los patrones.
            {
                bool blink = (tick % 2) == 0;
                setLed1(blink);
                setLed2(blink);
            }
            break;

        case STATE_PURSUIT:
            // Persecución activa: LEDs alternados como efecto de sirena.
            // Cuando LED1 está ON (tick par), LED2 está OFF, y viceversa.
            // El efecto "sirena" visualmente distingue PURSUIT de los demás estados.
            setLed1((tick % 2) == 0);   // LED1 ON en ticks pares
            setLed2((tick % 2) == 1);   // LED2 ON en ticks impares (siempre opuesto a LED1)
            break;
    }
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/drivers/mosfet_control.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo implementa el control físico de las "salidas de poder" del ESP32.
   Cuando el firmware decide que hay un robo (ALERT/PURSUIT), este módulo es quien
   realmente hace sonar la alarma y apaga el motor. También controla los LEDs de
   estado para que el propietario pueda ver de un vistazo qué está haciendo el sistema.

   El buzzer suena intermitente (toggle cada 250ms) para ahorrar energía y ser más
   audible que un tono continuo. El corte de motor (GPIO13) es el actuador más crítico
   del sistema — activarlo detiene la moto.

   PSEUDOCÓDIGO:
   init():
     → gpio_config({GPIO33, GPIO13, GPIO32, GPIO25}: OUTPUT, sin pull-ups)
     → gpio_set_level(todos, 0)  ← apagados al inicio

   setBuzzer(active):
     → buzzerState = active
     → gpio_set_level(GPIO33, active ? 1 : 0)
     → ESP_LOGW si active (evento anómalo)

   setEngineCut(active):
     → engineCutState = active
     → gpio_set_level(GPIO13, active ? 1 : 0)
     → ESP_LOGE si active (evento crítico)

   applyLedPattern(tick) [llamada cada 250ms]:
     STATE_IDLE + armado    → LED1=ON,  LED2=OFF
     STATE_IDLE + desarmado → LED1=ON si tick%8==0, LED2=OFF
     STATE_MOVING           → LED1=LED2=(tick%2==0) [parpadeo 2Hz]
     STATE_ALERT            → LED1=LED2=(tick%2==0) [mismo que MOVING]
     STATE_PURSUIT          → LED1=(tick%2==0), LED2=(tick%2==1) [sirena]

   DIAGRAMA MENTAL:
   control_task (cada 250ms)
     → applyLedPattern(tick++)  → gpio_set_level(GPIO32/25)
     → si ALERT: toggleBuzzer() → gpio_set_level(GPIO33)

   control_task (al recibir comando ENGINE_CUT)
     → setEngineCut(true) → gpio_set_level(GPIO13, HIGH)

   VARIABLES CRÍTICAS:
   - engineCutState: true = motor cortado. Solo false lo restaura.
   - buzzerState: true = buzzer sonando. toggle() lo alterna.

   DEUDA TÉCNICA:
   1. setEngineCut() no verifica velocidad GPS ni estado del sistema — el caller
      (control_task) es responsable de la seguridad.
   2. applyLedPattern() no controla el buzzer — solo los LEDs. El buzzer se
      gestiona por separado en control_task. Inconsistencia en la abstracción.
   3. ALERT y MOVING tienen el mismo patrón LED — no distinguibles visualmente.

   ═══════════════════════════════════════════════════════════ */
