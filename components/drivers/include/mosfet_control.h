#pragma once

/**
 * @file mosfet_control.h
 * @brief Interfaz de control de salidas digitales MOSFET y LEDs del sistema Argus.
 *
 * PROPÓSITO:
 *   Abstraer el control de las cuatro salidas GPIO de potencia del ESP32:
 *   buzzer de alarma (GPIO33), corte de motor (GPIO13), y dos LEDs de estado
 *   (GPIO32, GPIO25). Todas las interacciones con gpio_set_level() pasan por
 *   esta clase, garantizando que el estado interno (buzzerState, engineCutState,
 *   led1State, led2State) siempre sea coherente con el estado físico del pin.
 *
 * HARDWARE CONTROLADO:
 *   - GPIO33 → MOSFET de buzzer/alarma sonora: activa un buzzer piezoeléctrico
 *     o bocina. Nivel alto = ON. En ALERT y PURSUIT, control_task lo hace
 *     parpadear mediante llamadas periódicas a toggleBuzzer().
 *   - GPIO13 → MOSFET de corte de motor/alimentación: interrumpe el circuito
 *     de arranque de la moto. Nivel alto = corte activo. ACTUADOR DE ALTO RIESGO:
 *     si se activa accidentalmente, corta el motor mientras la moto está en marcha.
 *   - GPIO32 → LED de estado 1 (indicador principal visible para el operador).
 *   - GPIO25 → LED de estado 2 (indicador secundario, usado en modo sirena).
 *
 * LÓGICA DE ACTIVACIÓN:
 *   Todos los MOSFETs usan lógica positiva: nivel alto (1) = MOSFET conduciendo = carga activada.
 *   Si el MOSFET es de tipo N-channel con gate en el GPIO, nivel alto activa la carga.
 *   Si el hardware usara lógica invertida (MOSFET P-channel o driver inversor),
 *   las funciones set*() deberían invertir el nivel — no es el caso actual.
 *
 * DISEÑO COMO CLASE ESTÁTICA:
 *   Todos los métodos son estáticos porque hay un único set de GPIOs en el sistema.
 *   No tiene sentido instanciar múltiples MosfetControl para los mismos pines físicos.
 *   El estado interno (buzzerState, etc.) son variables estáticas compartidas.
 *   No hay necesidad de inyección de dependencias ni de polimorfismo para este driver.
 *
 * VARIABLES CRÍTICAS:
 *   - engineCutState: si es true, el motor de la moto está cortado. Cualquier código
 *     que llame setEngineCut(false) permite que el motor arranque nuevamente. Solo
 *     control_task y el handler de comandos TCP deben controlar este flag.
 *   - buzzerState: si es true, el buzzer está sonando. applyLedPattern() no controla
 *     el buzzer — solo los LEDs. El buzzer se controla desde control_task directamente.
 *   - led1State / led2State: estado de los LEDs de indicación visual. applyLedPattern()
 *     los actualiza en cada tick (cada ~250ms desde control_task).
 *
 * CONCURRENCIA:
 *   - gpio_set_level() es thread-safe según la documentación de ESP-IDF.
 *   - Los estados internos (buzzerState, etc.) son bool simples sin protección mutex.
 *     Si dos tareas llamaran simultáneamente a setBuzzer() y toggleBuzzer(), podría
 *     haber una race condition en la variable bool. En Argus, solo control_task
 *     controla estas salidas, así que no hay riesgo real en la arquitectura actual.
 *   - applyLedPattern() lee currentSystemState (variable global extern de system_flags.h).
 *     Esta lectura es atómica para tipos simples en ARM Cortex-M, pero técnicamente
 *     debería protegerse con xStateMutex si la consistencia es crítica.
 *
 * RIESGOS DE HARDWARE:
 *   - GPIO13 (ENGINE_CUT): si se activa con la moto en marcha a alta velocidad,
 *     puede causar una pérdida abrupta de tracción. Solo activar en situaciones
 *     de robo confirmado y a baja velocidad si el firmware tiene esa lógica.
 *   - GPIO33 (BUZZER): en ALERT/PURSUIT, el buzzer suena continuamente. Si el
 *     sistema se queda en ALERT sin intervención del operador, el buzzer puede
 *     drenar la batería de la moto en un tiempo prolongado.
 *
 * @module components/drivers/mosfet_control
 */

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Driver de salidas digitales MOSFET y LEDs. Todos los métodos son estáticos.
 *
 * DISEÑO:
 *   Los variables de estado estáticas (buzzerState, etc.) sincronizan con el
 *   estado físico del GPIO. Esto evita lecturas GPIO innecesarias (gpio_get_level
 *   en outputs tiene latencia) y garantiza consistencia si dos funciones consultan
 *   el estado actual.
 *
 * RESTRICCIÓN:
 *   init() DEBE llamarse antes que cualquier otro método. Sin init(), los GPIOs
 *   no están configurados como salidas y gpio_set_level() opera en un pin en modo
 *   input, sin efecto sobre el hardware conectado.
 */
class MosfetControl {
public:
    /**
     * @brief Configura los cuatro GPIOs como salidas digitales e inicializa en bajo.
     *
     * PROPÓSITO:
     *   Configurar GPIO33, GPIO13, GPIO32 y GPIO25 como salidas digitales sin
     *   interrupción y forzarlos a nivel bajo (apagados) al inicio del sistema.
     *   Crítico para evitar que el corte de motor (GPIO13) o el buzzer (GPIO33)
     *   se activen accidentalmente en el boot del ESP32 antes de que el firmware
     *   tome control.
     *
     * FLUJO LÓGICO:
     *   gpio_config() con todos los pines en modo salida, sin pull-ups ni pull-downs,
     *   sin interrupciones → gpio_set_level() a 0 para cada pin.
     *
     * NOTA DE ARRANQUE:
     *   Durante el boot del ESP32, los GPIOs están en modo input por defecto.
     *   El MOSFET del ENGINE_CUT podría estar en estado indeterminado si el GPIO
     *   tiene un nivel flotante. En el PCB de Argus, el pin de gate del MOSFET
     *   debe tener una resistencia de pull-down externa para garantizar nivel bajo
     *   antes de que el firmware configure el GPIO. Sin esa resistencia, hay un
     *   ventana de tiempo durante el boot donde el motor podría cortarse.
     *
     * DEPENDENCIAS:
     *   - pin_config.h: PIN_MOSFET_BUZZER (33), PIN_MOSFET_ENGINE (13),
     *                   PIN_LED_STATUS_1 (32), PIN_LED_STATUS_2 (25)
     */
    static void init();

    // ─── Control del buzzer ───────────────────────────────────────────────────

    /**
     * @brief Activa o desactiva el buzzer/alarma sonora (GPIO33).
     *
     * PROPÓSITO:
     *   Control directo del MOSFET del buzzer. Actualiza buzzerState y llama
     *   gpio_set_level(). Registra un log WARN cuando se activa (estado anómalo)
     *   y INFO cuando se desactiva.
     *
     * @param active true para activar el buzzer (GPIO33 = HIGH), false para apagarlo.
     */
    static void setBuzzer(bool active);

    /**
     * @brief Alterna el estado actual del buzzer.
     *
     * PROPÓSITO:
     *   Invierte buzzerState y llama gpio_set_level() con el nuevo estado.
     *   Útil para generar parpadeo de alarma desde control_task sin tener que
     *   rastrear el estado actual en la tarea.
     *   Se llama periódicamente (cada ~250ms) cuando el sistema está en ALERT
     *   para producir el efecto sonoro intermitente.
     */
    static void toggleBuzzer();

    /**
     * @brief Retorna true si el buzzer está activo actualmente.
     *
     * PROPÓSITO:
     *   Consultar el estado del buzzer sin leer el GPIO (más eficiente).
     *   buzzerState siempre es coherente con el nivel real del GPIO33 porque
     *   cada función que modifica el pin también actualiza buzzerState.
     *
     * @return true si GPIO33 está en HIGH (buzzer encendido).
     */
    static bool isBuzzerActive();

    // ─── Control de corte de motor ────────────────────────────────────────────

    /**
     * @brief Activa o desactiva el corte de motor/alimentación (GPIO13).
     *
     * PROPÓSITO:
     *   Control del MOSFET que interrumpe el circuito de arranque o alimentación
     *   de la moto. Activo = corte de motor activo (GPIO13 = HIGH).
     *
     * ADVERTENCIA DE SEGURIDAD:
     *   Activar el corte de motor mientras la moto está en marcha puede causar
     *   una situación peligrosa. El firmware de Argus debe implementar lógica
     *   de seguridad para este actuador (verificar velocidad GPS, confirmar
     *   identidad del operador, registrar el evento en el log de auditoría).
     *   Esta función no tiene esas protecciones — son responsabilidad del caller.
     *
     * DEPENDENCIAS:
     *   - Solo debe llamarse desde control_task.cpp (applyStateEffects) y desde
     *     el handler de comandos TCP (comando ENGINE_CUT del operador).
     *
     * @param active true para activar el corte (GPIO13 = HIGH), false para desactivar.
     */
    static void setEngineCut(bool active);

    /**
     * @brief Retorna true si el corte de motor está activo actualmente.
     * @return true si GPIO13 está en HIGH (corte activo).
     */
    static bool isEngineCutActive();

    // ─── Control de LEDs de estado ────────────────────────────────────────────

    /** @brief Activa o desactiva el LED de estado 1 (GPIO32). */
    static void setLed1(bool active);

    /** @brief Activa o desactiva el LED de estado 2 (GPIO25). */
    static void setLed2(bool active);

    /** @brief Alterna el estado del LED 1 (GPIO32). */
    static void toggleLed1();

    /** @brief Alterna el estado del LED 2 (GPIO25). */
    static void toggleLed2();

    /**
     * @brief Aplica el patrón de LEDs correspondiente al estado actual del sistema.
     *
     * PROPÓSITO:
     *   Implementar la visualización del estado del sistema mediante patrones
     *   de parpadeo de los LEDs. Se llama periódicamente desde control_task
     *   (cada ~250ms) con un contador de tick incremental para generar timing.
     *
     * PATRONES POR ESTADO:
     *   STATE_IDLE + desarmado: LED1 pulso único cada 2s (tick % 8 == 0)
     *     → el sistema está vivo pero no vigila activamente.
     *   STATE_IDLE + armado: LED1 fijo ON, LED2 OFF
     *     → vigilancia activa, moto en reposo.
     *   STATE_MOVING: ambos LEDs parpadeo rápido sincronizado (tick % 2)
     *     → movimiento detectado, evaluando si es autorizado.
     *   STATE_ALERT: ambos LEDs parpadeo rápido sincronizado (mismo patrón que MOVING)
     *     → alerta activa, movimiento no autorizado confirmado.
     *   STATE_PURSUIT: LED1 y LED2 alternados (sirena/policía)
     *     → operador activó persecución activa.
     *
     * DEPENDENCIAS:
     *   - system_flags.h: currentSystemState (SystemState_t), systemArmed (bool)
     *     Estos globales se leen sin mutex — lectura atómica en ARM para tipos simples.
     *
     * NOTA SOBRE ALERT vs MOVING:
     *   Ambos estados tienen el mismo patrón de LEDs (parpadeo rápido sincronizado).
     *   La distinción visual para el operador no es por LEDs sino por la alarma
     *   sonora (buzzer) y las notificaciones del backend. Si se necesita distinguir
     *   ALERT de MOVING visualmente, cambiar uno de los patrones.
     *
     * @param tick Contador de ciclos. Incrementar en 1 en cada llamada (~250ms/tick).
     *             Solo importa la paridad y el módulo 8, no el valor absoluto.
     */
    static void applyLedPattern(uint32_t tick);

private:
    // Estado interno de cada salida. Siempre coherente con el nivel físico del GPIO.
    // Se usan para evitar lecturas GPIO innecesarias en isBuzzerActive() y
    // para que toggleBuzzer()/toggleLed1()/toggleLed2() sepan el estado actual.
    static bool buzzerState;
    static bool engineCutState;
    static bool led1State;
    static bool led2State;
};


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/drivers/include/mosfet_control.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo define la interfaz para controlar las "salidas de poder" del sistema
   Argus. Hay cuatro salidas físicas conectadas al ESP32:
   - Buzzer (alarma sonora): suena cuando la moto es robada
   - Corte de motor: corta el arranque de la moto remotamente
   - LED 1 y LED 2: indican el estado del sistema visualmente

   Todos los métodos son estáticos porque hay exactamente una moto y un conjunto
   de GPIOs — no tiene sentido tener múltiples instancias de este controlador.

   TABLA DE MÉTODOS:
   Método              | GPIO | Propósito
   --------------------|------|------------------------------------------
   init()              | All  | Configurar como salidas, poner en bajo
   setBuzzer(active)   | 33   | Encender/apagar alarma sonora
   toggleBuzzer()      | 33   | Alternar buzzer (para parpadeo de alarma)
   isBuzzerActive()    | 33   | Consultar estado del buzzer
   setEngineCut(active)| 13   | Activar/desactivar corte de motor
   isEngineCutActive() | 13   | Consultar si el motor está cortado
   setLed1(active)     | 32   | Control LED de estado 1
   setLed2(active)     | 25   | Control LED de estado 2
   toggleLed1()        | 32   | Alternar LED 1
   toggleLed2()        | 25   | Alternar LED 2
   applyLedPattern(tk) | 32,25| Aplicar patrón según estado del sistema

   MAPA DE ESTADOS → PATRÓN DE LEDs:
   IDLE + desarmado → LED1 pulso/2s, LED2 OFF
   IDLE + armado    → LED1 fijo ON, LED2 OFF
   MOVING           → ambos parpadeo rápido sincronizado
   ALERT            → ambos parpadeo rápido sincronizado (= MOVING visualmente)
   PURSUIT          → LEDs alternados (efecto sirena)

   DEUDA TÉCNICA:
   1. ALERT y MOVING tienen el mismo patrón de LEDs — no se distinguen visualmente.
   2. Los estados internos (buzzerState, etc.) no están protegidos con mutex.
      Safe solo porque una sola tarea (control_task) usa estas funciones.
   3. engineCutState no tiene protección contra activación accidental.

   ═══════════════════════════════════════════════════════════ */
