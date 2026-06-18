/**
 * @file ActuatorManager.cpp
 * @brief Implementación del control de sirena y corte de motor + test de hardware.
 *
 * LOGGING:
 *   sirenOn()   → ESP_LOGW  (warning: evento de alerta)
 *   cutEngine() → ESP_LOGE  (error: el nivel más alto garantiza visibilidad
 *                             incluso con log level = ERROR en producción)
 *   Los demás eventos (off / restore) → ESP_LOGI.
 *
 * TEST DE HARDWARE (runHardwareTest):
 *   Máquina de estados de 5 etapas con timing via esp_timer (sin delay).
 *   El estado se guarda en variables estáticas de archivo para sobrevivir
 *   múltiples llamadas al loop. resetHardwareTest() las reinicializa.
 *
 * @module components/drivers/ActuatorManager
 */

#include "ActuatorManager.h"
#include "LedManager.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG       = "ActuatorManager";
static const char* TAG_TEST  = "HwTest";

// ─── Constructor / init ───────────────────────────────────────────────────────

ActuatorManager::ActuatorManager(gpio_num_t siren_pin, gpio_num_t engine_pin)
    : _sirenPin(siren_pin), _enginePin(engine_pin),
      _sirenActive(false), _engineCut(false) {}

void ActuatorManager::init() {
    gpio_config_t io = {};
    io.pin_bit_mask  = (1ULL << _sirenPin) | (1ULL << _enginePin);
    io.mode          = GPIO_MODE_OUTPUT;
    io.pull_up_en    = GPIO_PULLUP_DISABLE;
    io.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(_sirenPin,  0);
    gpio_set_level(_enginePin, 0);
    ESP_LOGI(TAG, "init OK (sirena=GPIO%d motor=GPIO%d)", (int)_sirenPin, (int)_enginePin);
}

// ─── Sirena ───────────────────────────────────────────────────────────────────

void ActuatorManager::sirenOn() {
    _write(_sirenPin, true);
    _sirenActive = true;
    ESP_LOGW(TAG, "Sirena ON");
}

void ActuatorManager::sirenOff() {
    _write(_sirenPin, false);
    _sirenActive = false;
    ESP_LOGI(TAG, "Sirena OFF");
}

void ActuatorManager::toggleSiren() {
    _sirenActive ? sirenOff() : sirenOn();
}

// ─── Corte de motor ───────────────────────────────────────────────────────────

void ActuatorManager::cutEngine() {
    _write(_enginePin, true);
    _engineCut = true;
    // ESP_LOGE garantiza que este evento quede registrado aunque el nivel
    // de log esté configurado en WARN o ERROR en producción.
    ESP_LOGE(TAG, "CORTE DE MOTOR ACTIVADO — GPIO%d en HIGH", (int)_enginePin);
}

void ActuatorManager::restoreEngine() {
    _write(_enginePin, false);
    _engineCut = false;
    ESP_LOGI(TAG, "Motor restaurado — GPIO%d en LOW", (int)_enginePin);
}

void ActuatorManager::toggleEngineCut() {
    _engineCut ? restoreEngine() : cutEngine();
}

// ─── Privados ─────────────────────────────────────────────────────────────────

void ActuatorManager::_write(gpio_num_t pin, bool active) {
    gpio_set_level(pin, active ? 1 : 0);
}

// ─── Test de hardware ─────────────────────────────────────────────────────────

static constexpr uint32_t TEST_STAGE_MS = 1000;

// Estado persistente entre llamadas al loop.
static uint8_t  s_stage      = 0;
static uint64_t s_stageStart = 0;
static bool     s_running    = false;

static uint64_t hwtest_ms() {
    return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

void resetHardwareTest() {
    s_stage      = 0;
    s_stageStart = 0;
    s_running    = false;
}

bool runHardwareTest(LedManager& leds, ActuatorManager& actuators) {
    uint64_t now = hwtest_ms();

    // Inicialización de la primera llamada.
    if (!s_running) {
        s_running    = true;
        s_stage      = 0;
        s_stageStart = now;
        leds.setLedsDirect(true, false);
        ESP_LOGI(TAG_TEST, "[0/4] LED1 ON");
    }

    // Aún dentro de la etapa actual — esperar sin bloquear.
    if (now - s_stageStart < TEST_STAGE_MS) return false;

    // Transición a la siguiente etapa.
    s_stage++;
    s_stageStart = now;

    switch (s_stage) {
        case 1:
            leds.setLedsDirect(false, true);
            ESP_LOGI(TAG_TEST, "[1/4] LED2 ON");
            break;

        case 2:
            leds.setLedsDirect(false, false);
            actuators.sirenOn();
            ESP_LOGI(TAG_TEST, "[2/4] Sirena ON");
            break;

        case 3:
            actuators.sirenOff();
            actuators.cutEngine();
            ESP_LOGI(TAG_TEST, "[3/4] Corte de motor ON");
            break;

        case 4:
            actuators.restoreEngine();
            leds.setLedsDirect(false, false);
            ESP_LOGI(TAG_TEST, "[4/4] Test completado — todo apagado");
            s_running = false;
            return true;

        default:
            break;
    }

    return false;
}
