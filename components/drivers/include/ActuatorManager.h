#pragma once

/**
 * @file ActuatorManager.h
 * @brief Control de sirena (GPIO33) y corte de motor (GPIO13).
 *
 * DISEÑO:
 *   Todas las operaciones son inmediatas (gpio_set_level). Sin temporización
 *   interna — para patrones de activación/desactivación con tiempo, usar control_task.
 *
 * SEGURIDAD:
 *   cutEngine() no verifica velocidad GPS ni estado de la máquina de estados.
 *   Esa responsabilidad es del caller (control_task / handler de comando remoto).
 *
 * INTEGRACIÓN:
 *   ActuatorManager actuators(PIN_MOSFET_BUZZER, PIN_MOSFET_ENGINE);
 *   actuators.init();
 *   actuators.sirenOn();    // en ALERT
 *   actuators.cutEngine();  // en PURSUIT, solo con confirmación del operador
 *
 * @module components/drivers/ActuatorManager
 */

#include <stdint.h>
#include "driver/gpio.h"

// Forward declaration — evita incluir LedManager.h desde este header.
class LedManager;

// ─── Enum de estados ──────────────────────────────────────────────────────────

/**
 * Estado combinado de los dos actuadores.
 * Los dos ejes son independientes: la sirena y el corte de motor pueden
 * estar activos simultáneamente sin conflicto. El enum existe para logging
 * y para ser extendido si se agregan más actuadores.
 */
enum class ActuatorState : uint8_t {
    SIREN_OFF     = 0,
    SIREN_ON      = 1,
    ENGINE_NORMAL = 2,
    ENGINE_CUT    = 3
};

// ─── Clase ActuatorManager ────────────────────────────────────────────────────

class ActuatorManager {
public:
    /**
     * @param siren_pin  GPIO de la sirena/buzzer (PIN_MOSFET_BUZZER = GPIO33).
     * @param engine_pin GPIO del corte de motor (PIN_MOSFET_ENGINE = GPIO13).
     */
    ActuatorManager(gpio_num_t siren_pin, gpio_num_t engine_pin);

    /** Configura ambos GPIOs como salidas y los fuerza a LOW. */
    void init();

    // ── Sirena ────────────────────────────────────────────────────────────────
    void sirenOn();
    void sirenOff();
    void toggleSiren();
    bool isSirenActive() const { return _sirenActive; }

    // ── Corte de motor ────────────────────────────────────────────────────────
    void cutEngine();
    void restoreEngine();
    void toggleEngineCut();
    bool isEngineCutActive() const { return _engineCut; }

private:
    gpio_num_t _sirenPin;
    gpio_num_t _enginePin;
    bool       _sirenActive;
    bool       _engineCut;

    void _write(gpio_num_t pin, bool active);
};

// ─── Test de hardware ─────────────────────────────────────────────────────────

/**
 * Test secuencial de todos los actuadores y LEDs. Sin delay() — debe llamarse
 * en cada iteración del loop hasta que retorne true.
 *
 * Secuencia (1 segundo por etapa):
 *   Etapa 0 → LED1 ON
 *   Etapa 1 → LED2 ON  (LED1 OFF)
 *   Etapa 2 → Sirena ON  (ambos LEDs OFF)
 *   Etapa 3 → Corte de motor ON  (Sirena OFF)
 *   Etapa 4 → Todo OFF  → retorna true
 *
 * El estado interno es estático. Llamar resetHardwareTest() para repetir el test
 * en la misma sesión sin reiniciar el firmware.
 *
 * @return true cuando el test finalizó, false mientras está en curso.
 */
bool runHardwareTest(LedManager& leds, ActuatorManager& actuators);

/** Reinicia el estado interno de runHardwareTest() para una nueva ejecución. */
void resetHardwareTest();
