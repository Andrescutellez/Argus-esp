/**
 * @file LedManager.cpp
 * @brief Implementación del controlador no bloqueante de LEDs de estado.
 *
 * TIMING:
 *   _ms() retorna milisegundos desde el boot via esp_timer_get_time().
 *   _updateChannel() compara (now - phase_start_ms) contra la duración de la fase
 *   actual. Si se cumplió, avanza al siguiente elemento de la secuencia y escribe GPIO.
 *   El GPIO solo se escribe en transiciones de fase — no en cada llamada a update().
 *
 * BREATHING (CHARGING):
 *   LED1 pasa a control LEDC (hardware PWM). _initLedc() configura LEDC_TIMER_0 +
 *   LEDC_CHANNEL_0 en modo 8-bit a 5kHz. _updateBreathing() calcula la onda
 *   triangular en cada update() y escribe el duty via ledc_set_duty + ledc_update_duty.
 *   Al salir del patrón CHARGING, _stopBreathing() llama ledc_stop() y reconfigura
 *   el GPIO como salida digital normal.
 *
 * TABLA DE PATRONES:
 *   PATTERNS[] está indexada por LedPattern (uint8_t). El static_assert al final
 *   del archivo garantiza que la tabla no se desincronice del enum si se agregan
 *   nuevos patrones.
 *
 * @module components/drivers/LedManager
 */

#include "LedManager.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "LedManager";

// ─── Alias corto para construir la tabla ─────────────────────────────────────

using Ph = LedManager::LedPhase;

// ─── Secuencias de fases ──────────────────────────────────────────────────────
// Convención: cada arreglo repite en loop desde índice 0.
// 3600000ms (~1h) actúa como "infinito": el loop se produce pero el LED no cambia.

static const Ph PH_ON[]         = {{ true,  3600000 }};
static const Ph PH_OFF[]        = {{ false, 3600000 }};

static const Ph PH_FAST[]       = {{ true,  100 }, { false, 100 }};
static const Ph PH_SLOW[]       = {{ true,  500 }, { false, 500 }};
static const Ph PH_PULSE_SLOW[] = {{ true,  800 }, { false, 800 }};

// GPS_SEARCHING: alternado desfasado — LED1 arranca en ON, LED2 arranca en OFF.
static const Ph PH_ALT_LEAD[]   = {{ true,  700 }, { false, 700 }};
static const Ph PH_ALT_LAG[]    = {{ false, 700 }, { true,  700 }};

// PURSUIT: alternado rápido — mismo desfase que ALT pero a 150ms.
static const Ph PH_FAST_LEAD[]  = {{ true,  150 }, { false, 150 }};
static const Ph PH_FAST_LAG[]   = {{ false, 150 }, { true,  150 }};

// GPS_FIXED: LED2 apagado 4850ms + destello breve 150ms = período de 5s exactos.
static const Ph PH_GPS_FIX2[]   = {{ false, 4850 }, { true, 150 }};

// ALERT: LED2 con doble destello (100ms ON / 100ms OFF / 100ms ON) + pausa 1700ms.
// Período total = 2000ms. El patrón "dos golpes y silencio" es reconocible de noche.
static const Ph PH_DBL_FLASH[]  = {{ true, 100 }, { false, 100 },
                                    { true, 100 }, { false, 1700 }};

// Patrones de error: N destellos de 200ms con pausa hasta completar ~2s por ciclo.
// El número de destellos identifica el subsistema fallido sin necesidad de pantalla.
static const Ph PH_ERR1[] = {{ true,200 },{ false,1800 }};

static const Ph PH_ERR2[] = {{ true,200 },{ false,200 },
                              { true,200 },{ false,1400 }};

static const Ph PH_ERR3[] = {{ true,200 },{ false,200 },
                              { true,200 },{ false,200 },
                              { true,200 },{ false,1200 }};

static const Ph PH_ERR5[] = {{ true,200 },{ false,200 },
                              { true,200 },{ false,200 },
                              { true,200 },{ false,200 },
                              { true,200 },{ false,200 },
                              { true,200 },{ false,800  }};

// Macro: expande un arreglo a puntero + tamaño para la tabla PatternDef.
#define S(seq)  seq, (uint8_t)(sizeof(seq) / sizeof(seq[0]))

// ─── Tabla de patrones ────────────────────────────────────────────────────────
// El orden DEBE coincidir con el enum LedPattern (verificado por static_assert al final).
// led1_breathing=true → LED1 bajo LEDC; el motor de fases del ch1 no se usa.

const LedManager::PatternDef LedManager::PATTERNS[] = {
//  { led1_seq          , led2_seq          , led1_breathing }
    { S(PH_FAST),         S(PH_FAST),         false },  // BOOT
    { S(PH_SLOW),         S(PH_OFF),          false },  // INITIALIZING
    { S(PH_ON),           S(PH_OFF),          false },  // READY
    { S(PH_ALT_LEAD),     S(PH_ALT_LAG),      false },  // GPS_SEARCHING
    { S(PH_ON),           S(PH_GPS_FIX2),     false },  // GPS_FIXED
    { S(PH_ON),           S(PH_FAST),         false },  // LTE_CONNECTING
    { S(PH_OFF),          S(PH_OFF),          false },  // LORA_ACTIVITY (base off, flash via triggerActivity)
    { S(PH_PULSE_SLOW),   S(PH_ON),           false },  // BLE_CONNECTED
    { S(PH_PULSE_SLOW),   S(PH_DBL_FLASH),    false },  // ALERT
    { S(PH_FAST_LEAD),    S(PH_FAST_LAG),     false },  // PURSUIT
    { S(PH_FAST),         S(PH_FAST),         false },  // JAMMER_DETECTED
    { S(PH_SLOW),         S(PH_SLOW),         false },  // LOW_BATTERY
    { S(PH_OFF),          S(PH_OFF),          true  },  // CHARGING (LED1 = LEDC breathing)
    { S(PH_ON),           S(PH_ON),           false },  // FULLY_CHARGED
    { S(PH_OFF),          S(PH_ERR1),         false },  // ERROR_MPU
    { S(PH_OFF),          S(PH_ERR2),         false },  // ERROR_LORA
    { S(PH_OFF),          S(PH_ERR3),         false },  // ERROR_A7670
    { S(PH_OFF),          S(PH_ERR5),         false },  // ERROR_CRITICAL
};

// Nota: el static_assert de tamaño está en _loadPattern() — PATTERNS es private.

// ─── Constructor / init ───────────────────────────────────────────────────────

LedManager::LedManager(gpio_num_t led1_pin, gpio_num_t led2_pin)
    : _led1(led1_pin), _led2(led2_pin),
      _pattern(LedPattern::BOOT),
      _ch1{}, _ch2{},
      _activityPending(false), _activityStart(0),
      _breathingActive(false) {}

void LedManager::init() {
    gpio_config_t io = {};
    io.pin_bit_mask  = (1ULL << _led1) | (1ULL << _led2);
    io.mode          = GPIO_MODE_OUTPUT;
    io.pull_up_en    = GPIO_PULLUP_DISABLE;
    io.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(_led1, 0);
    gpio_set_level(_led2, 0);
    ESP_LOGI(TAG, "init OK (LED1=GPIO%d LED2=GPIO%d)", (int)_led1, (int)_led2);
}

// ─── API pública ──────────────────────────────────────────────────────────────

void LedManager::setPattern(LedPattern pattern) {
    if (pattern == _pattern) return;
    _loadPattern(pattern);
    ESP_LOGI(TAG, "patrón → %d", static_cast<int>(pattern));
}

void LedManager::triggerActivity() {
    _activityPending = true;
    _activityStart   = _ms();
}

void LedManager::setLedsDirect(bool led1, bool led2) {
    _setGpio(_led1, led1);
    _setGpio(_led2, led2);
}

void LedManager::update() {
    // El destello de actividad LoRa tiene prioridad sobre el patrón normal de LED1.
    if (_activityPending) {
        if (_ms() - _activityStart >= ACTIVITY_FLASH_MS) {
            _activityPending = false;
            _setGpio(_led1, _ch1.seq[_ch1.phase].on);  // restaurar fase actual
        } else {
            _setGpio(_led1, true);
        }
        // LED2 sigue su secuencia normal durante el destello.
        _updateChannel(_ch2, _led2);
        return;
    }

    if (_breathingActive) {
        _updateBreathing();
        _updateChannel(_ch2, _led2);
        return;
    }

    _updateChannel(_ch1, _led1);
    _updateChannel(_ch2, _led2);
}

// ─── Privados ─────────────────────────────────────────────────────────────────

void LedManager::_loadPattern(LedPattern p) {
    // Verificar que PATTERNS[] tiene exactamente _PATTERN_COUNT entradas.
    // Dentro del método tenemos acceso a miembros private.
    static_assert(
        sizeof(PATTERNS) / sizeof(PATTERNS[0]) ==
            static_cast<uint8_t>(LedPattern::_PATTERN_COUNT),
        "PATTERNS[] desincronizada del enum LedPattern — agregar entrada faltante"
    );
    const PatternDef& def = PATTERNS[static_cast<uint8_t>(p)];

    // Detener LEDC antes de transicionar a cualquier patrón que use GPIO puro.
    if (_breathingActive && !def.led1_breathing) {
        _stopBreathing();
    }

    uint64_t now = _ms();
    _ch1 = { def.led1_seq, def.led1_len, 0, now };
    _ch2 = { def.led2_seq, def.led2_len, 0, now };
    _pattern         = p;
    _activityPending = false;

    if (def.led1_breathing) {
        _breathingActive = true;
        _initLedc();
    } else {
        // Aplicar inmediatamente la fase 0 de cada canal.
        _setGpio(_led1, def.led1_seq[0].on);
        _setGpio(_led2, def.led2_seq[0].on);
    }
}

void LedManager::_updateChannel(Channel& ch, gpio_num_t pin) {
    if (!ch.seq || ch.seq_len == 0) return;

    uint64_t now = _ms();
    if (now - ch.phase_start_ms >= ch.seq[ch.phase].duration_ms) {
        ch.phase = (ch.phase + 1) % ch.seq_len;
        ch.phase_start_ms = now;
        _setGpio(pin, ch.seq[ch.phase].on);
    }
    // Si la duración no se cumplió, el GPIO ya tiene el valor correcto desde la
    // última transición de fase — no es necesario escribirlo en cada llamada.
}

void LedManager::_updateBreathing() {
    // Onda triangular 0→255→0 en BREATH_PERIOD_MS. Se llama cada iteración del loop,
    // por lo que la resolución temporal depende de la cadencia de update().
    uint32_t t    = static_cast<uint32_t>(_ms() % BREATH_PERIOD_MS);
    uint32_t half = BREATH_PERIOD_MS / 2;
    uint32_t duty = (t < half)
                  ? (t * LEDC_MAX_DUTY / half)
                  : ((BREATH_PERIOD_MS - t) * LEDC_MAX_DUTY / half);
    _setLedcDuty(duty);
}

void LedManager::_initLedc() {
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode      = LEDC_MODE;
    tcfg.duty_resolution = LEDC_TIMER_8_BIT;
    tcfg.timer_num       = LEDC_TIMER_ID;
    tcfg.freq_hz         = 5000;
    tcfg.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {};
    ccfg.gpio_num   = static_cast<int>(_led1);
    ccfg.speed_mode = LEDC_MODE;
    ccfg.channel    = LEDC_CH;
    ccfg.timer_sel  = LEDC_TIMER_ID;
    ccfg.intr_type  = LEDC_INTR_DISABLE;
    ccfg.duty       = 0;
    ccfg.hpoint     = 0;
    ledc_channel_config(&ccfg);
}

void LedManager::_stopBreathing() {
    // Dejar la salida en bajo antes de reconectar como GPIO digital.
    ledc_stop(LEDC_MODE, LEDC_CH, 0);
    _breathingActive = false;

    // ledc_channel_config() toma control del GPIO; hay que reclamarlo explícitamente.
    gpio_config_t io = {};
    io.pin_bit_mask  = (1ULL << _led1);
    io.mode          = GPIO_MODE_OUTPUT;
    io.pull_up_en    = GPIO_PULLUP_DISABLE;
    io.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(_led1, 0);
}

void LedManager::_setGpio(gpio_num_t pin, bool on) {
    gpio_set_level(pin, on ? 1 : 0);
}

void LedManager::_setLedcDuty(uint32_t duty) {
    ledc_set_duty(LEDC_MODE, LEDC_CH, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH);
}

uint64_t LedManager::_ms() {
    // esp_timer_get_time() retorna microsegundos desde el boot (int64_t).
    return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}
