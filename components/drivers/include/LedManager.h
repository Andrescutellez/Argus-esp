#pragma once

/**
 * @file LedManager.h
 * @brief Controlador no bloqueante de dos LEDs de estado con 18 patrones predefinidos.
 *
 * ARQUITECTURA:
 *   Cada LED corre una secuencia independiente de fases (on/off + duración_ms).
 *   update() avanza las fases usando esp_timer (equivalente a millis() en IDF).
 *   Sin delay(), sin bloqueo de tareas — apto para llamarse cada iteración del loop.
 *
 *   El único patrón que usa PWM hardware es CHARGING: LED1 bajo LEDC (efecto respiración).
 *   El resto controla GPIO directamente con gpio_set_level().
 *
 * INTEGRACIÓN:
 *   LedManager leds(PIN_LED_STATUS_1, PIN_LED_STATUS_2);
 *   leds.init();
 *   leds.setPattern(LedPattern::BOOT);
 *   // en cada iteración del loop / task:
 *   leds.update();
 *
 * @module components/drivers/LedManager
 */

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

// ─── Enum de patrones ─────────────────────────────────────────────────────────

/**
 * Patrón visual asignado a los dos LEDs del sistema.
 * El orden DEBE coincidir con la tabla PATTERNS[] en LedManager.cpp.
 * _PATTERN_COUNT es un centinela para el static_assert de tamaño de tabla.
 */
enum class LedPattern : uint8_t {
    BOOT,             // LED1 y LED2 parpadean rápido simultáneamente
    INITIALIZING,     // LED1 parpadeo lento · LED2 apagado
    READY,            // LED1 fijo encendido · LED2 apagado
    GPS_SEARCHING,    // LED1 y LED2 alternados lentamente
    GPS_FIXED,        // LED1 fijo · LED2 destello breve cada 5s
    LTE_CONNECTING,   // LED1 fijo · LED2 parpadeo rápido
    LORA_ACTIVITY,    // LED1 destello al llamar triggerActivity() · LED2 apagado
    BLE_CONNECTED,    // LED1 pulso lento · LED2 fijo
    ALERT,            // LED1 pulso lento · LED2 doble destello periódico
    PURSUIT,          // LED1 y LED2 alternados rápidamente (efecto sirena)
    JAMMER_DETECTED,  // Ambos LEDs parpadean sincronizados
    LOW_BATTERY,      // Ambos LEDs pulso lento sincronizado
    CHARGING,         // LED1 efecto respiración PWM (LEDC) · LED2 apagado
    FULLY_CHARGED,    // Ambos LEDs encendidos fijo
    ERROR_MPU,        // LED2 repite 1 destello + pausa · LED1 apagado
    ERROR_LORA,       // LED2 repite 2 destellos + pausa · LED1 apagado
    ERROR_A7670,      // LED2 repite 3 destellos + pausa · LED1 apagado
    ERROR_CRITICAL,   // LED2 repite 5 destellos + pausa · LED1 apagado
    _PATTERN_COUNT
};

// ─── Clase LedManager ─────────────────────────────────────────────────────────

class LedManager {
public:
    /**
     * Fase elemental de una secuencia: estado del LED y cuánto dura.
     * Expuesto como public para que PATTERNS[] en el .cpp no duplique la definición.
     */
    struct LedPhase {
        bool     on;
        uint32_t duration_ms;
    };

    /**
     * @param led1_pin GPIO del LED primario (PIN_LED_STATUS_1 = GPIO32).
     * @param led2_pin GPIO del LED secundario (PIN_LED_STATUS_2 = GPIO25).
     */
    LedManager(gpio_num_t led1_pin, gpio_num_t led2_pin);

    /** Configura ambos GPIOs como salidas y los fuerza a LOW. Llamar antes de update(). */
    void init();

    /** Cambia el patrón activo. Si el patrón es el mismo que el actual, no hace nada. */
    void setPattern(LedPattern pattern);

    LedPattern getPattern() const { return _pattern; }

    /**
     * Dispara un destello breve en LED1 (ACTIVITY_FLASH_MS = 80ms).
     * Diseñado para LORA_ACTIVITY, pero funciona en cualquier patrón.
     * No bloqueante: update() aplica el destello en la siguiente iteración.
     */
    void triggerActivity();

    /**
     * Avanza el estado de ambos LEDs según el patrón activo.
     * Debe llamarse en cada iteración del loop principal o desde una tarea FreeRTOS.
     * Tiempo de ejecución: O(1), sin bloqueo.
     */
    void update();

    /**
     * Control directo de GPIO, bypaseando el patrón activo.
     * Solo para uso en runHardwareTest() — no usar en lógica de producción.
     */
    void setLedsDirect(bool led1, bool led2);

private:
    // Estado de ejecución de una secuencia de fases para un LED.
    struct Channel {
        const LedPhase* seq;
        uint8_t         seq_len;
        uint8_t         phase;
        uint64_t        phase_start_ms;
    };

    // Descriptor de un patrón: qué secuencia usa cada LED y si LED1 está bajo LEDC.
    struct PatternDef {
        const LedPhase* led1_seq;
        uint8_t         led1_len;
        const LedPhase* led2_seq;
        uint8_t         led2_len;
        bool            led1_breathing;
    };

    gpio_num_t _led1;
    gpio_num_t _led2;
    LedPattern _pattern;
    Channel    _ch1;
    Channel    _ch2;

    // Estado del destello de actividad LoRa (one-shot, no periódico)
    bool       _activityPending;
    uint64_t   _activityStart;

    bool       _breathingActive;

    void _loadPattern(LedPattern p);
    void _updateChannel(Channel& ch, gpio_num_t pin);
    void _updateBreathing();
    void _initLedc();
    void _stopBreathing();
    void _setGpio(gpio_num_t pin, bool on);
    void _setLedcDuty(uint32_t duty);

    static uint64_t _ms();

    static const PatternDef PATTERNS[];

    static constexpr uint32_t       ACTIVITY_FLASH_MS = 80;
    static constexpr uint32_t       BREATH_PERIOD_MS  = 2000;
    static constexpr uint32_t       LEDC_MAX_DUTY     = 255;
    static constexpr ledc_timer_t   LEDC_TIMER_ID     = LEDC_TIMER_0;
    static constexpr ledc_channel_t LEDC_CH           = LEDC_CHANNEL_0;
    static constexpr ledc_mode_t    LEDC_MODE         = LEDC_LOW_SPEED_MODE;
};
