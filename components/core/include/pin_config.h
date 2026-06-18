#pragma once

/**
 * @file pin_config.h
 * @brief Definición centralizada de todos los pines GPIO del hardware Argus Secure.
 *
 * PROPÓSITO:
 *   Ser la única fuente de verdad para las asignaciones de pines físicos del PCB.
 *   Cualquier código que necesite un número de GPIO debe incluir este archivo
 *   y usar las constantes aquí definidas, nunca hardcodear números mágicos.
 *   Si el PCB cambia una asignación de pin, solo este archivo necesita actualizarse.
 *
 * REGLA DE ORO:
 *   NUNCA usar números de GPIO directamente en código (gpio_set_level(13, 1) es MAL).
 *   SIEMPRE usar las constantes (gpio_set_level(PIN_MOSFET_ENGINE, 1) es BIEN).
 *   Esta regla permite detectar conflictos entre pines al leer este archivo y
 *   facilita el porting a otros revisions de hardware.
 *
 * MAPA DE PINES DEL ESP32 (ESP-WROOM-32):
 *   Las constantes aquí deben coincidir exactamente con el esquemático de hardware.
 *   Cualquier discrepancia causa comportamiento inesperado en el hardware real.
 *
 * PINES RESERVADOS (NO REASIGNAR):
 *   GPIO0:  BOOT button — activo bajo, uso del bootloader ESP32
 *   GPIO1:  UART0 TX — consola de debug (reservado por ESP-IDF)
 *   GPIO3:  UART0 RX — consola de debug (reservado por ESP-IDF)
 *   GPIO6-11: Flash SPI interno — absolutamente no tocar
 *   GPIO34-39: Solo entradas, sin pull-up/down interno disponible
 *
 * INTEGRACIÓN FUTURA (LORA):
 *   Los pines del SX1276 están reservados pero NO inicializados en el firmware
 *   actual. El driver LoRa no existe todavía. Están documentados aquí para
 *   reservar los pines y evitar que se asignen a funciones conflictivas en
 *   futuras revisiones del hardware.
 *
 * RIESGOS DE HARDWARE:
 *   - PIN_MOSFET_ENGINE (GPIO13): si se pone en HIGH accidentalmente, corta el
 *     motor de la moto. Asegurarse de que GPIO13 arranca en LOW.
 *   - PIN_A7670_PWRKEY (GPIO4): un pulso de encendido del A7670 dura ~3.5 segundos.
 *     Si se activa accidentalmente durante operación, el módulo puede reiniciarse.
 *   - PIN_MPU6050_INT (GPIO27): entrada con pull-down. Sin el pull-down, el pin
 *     flotante puede generar falsas interrupciones de movimiento al arrancar.
 *
 * @module components/core/pin_config
 */

#include "driver/gpio.h"

// ─── LEDs de estado del sistema ───────────────────────────────────────────────
// Controlados por MosfetControl. Lógica positiva: HIGH = LED encendido.
// El patrón de parpadeo indica el estado del sistema (ver mosfet_control.cpp).
#define PIN_LED_STATUS_1    GPIO_NUM_32   // LED principal de estado (lado izquierdo del PCB)
#define PIN_LED_STATUS_2    GPIO_NUM_25   // LED secundario (lado derecho, usado en modo sirena)

// ─── Salidas MOSFET ───────────────────────────────────────────────────────────
// Lógica positiva: HIGH = MOSFET conduce = carga activada.
// Inicializar en LOW (ver MosfetControl::init()) para evitar activación accidental en boot.

// GPIO33 → MOSFET → buzzer/bocina. HIGH = alarma sonando.
// En modos ALERT y PURSUIT, se hace parpadear o se activa continuamente.
#define PIN_MOSFET_BUZZER   GPIO_NUM_33

// GPIO13 → MOSFET → circuito de arranque/alimentación de la moto. HIGH = motor cortado.
// ACTUADOR DE ALTO RIESGO. Solo activar en STATE_PURSUIT por confirmación del operador.
// El PCB debe tener resistencia pull-down externa en el gate del MOSFET para garantizar
// nivel LOW durante el boot del ESP32 antes de que el firmware inicialice este pin.
#define PIN_MOSFET_ENGINE   GPIO_NUM_13

// ─── SX1276 LoRa (RESERVADO — sin implementación en firmware actual) ──────────
// Estos pines están conectados al módulo LoRa en el PCB pero el firmware no los usa.
// No reasignar estos pines a otras funciones hasta que LoRa sea eliminado del diseño
// de hardware o su driver sea implementado.
#define PIN_LORA_DIO0       GPIO_NUM_26   // Interrupción DIO0 del SX1276
#define PIN_LORA_RESET      GPIO_NUM_14   // Reset activo bajo del SX1276
#define PIN_SPI_MOSI        GPIO_NUM_23   // SPI MOSI compartido (SX1276 + futuro)
#define PIN_SPI_MISO        GPIO_NUM_19   // SPI MISO compartido
#define PIN_SPI_SCK         GPIO_NUM_18   // SPI Clock compartido
#define PIN_SPI_NSS         GPIO_NUM_5    // SPI NSS/CS del SX1276

// ─── MPU6050 — sensor de movimiento / aceleración ────────────────────────────

// Interrupción de movimiento del MPU6050. Activo alto (flanco ascendente).
// Configurado como entrada con pull-down (ver MPU6050Driver::installMotionISR()).
// La ISR motionISRHandler() en sensor_task.cpp responde a este pin.
#define PIN_MPU6050_INT     GPIO_NUM_27

// Bus I2C compartido (actualmente solo el MPU6050 lo usa).
// 100kHz con pull-ups internos del ESP32 (~45kΩ).
// Para agregar otros dispositivos I2C al mismo bus, usar i2c_master_bus_add_device()
// con el mismo busHandle creado en MPU6050Driver::init().
#define PIN_I2C_SCL         GPIO_NUM_22   // Reloj (clock) del bus I2C
#define PIN_I2C_SDA         GPIO_NUM_21   // Datos del bus I2C

// ─── A7670 — módulo 4G / GNSS ────────────────────────────────────────────────
// Comunicación UART2 a 115200 baudios. El A7670 envía respuestas AT y URCs asíncronos.
// Ver A7670Driver para la implementación completa del protocolo AT.

// UART2 RX del ESP32 ← conectado al TX del A7670.
// El ESP32 recibe aquí las respuestas AT y los URCs (eventos no solicitados) del módulo.
#define PIN_A7670_RX        GPIO_NUM_16

// UART2 TX del ESP32 → conectado al RX del A7670.
// El ESP32 envía aquí los comandos AT al módulo.
#define PIN_A7670_TX        GPIO_NUM_17

// GPIO de control del PWRKEY del A7670 (encendido/apagado del módulo).
// Pulso de 3-4 segundos en este pin enciende o apaga el módulo.
// Ver A7670Driver::powerPulse() para la implementación de boot y recovery.
#define PIN_A7670_PWRKEY    GPIO_NUM_4

// ─── Controles del sistema ────────────────────────────────────────────────────
// GPIO0 es el botón BOOT del ESP32 DevKit. Activo bajo.
// Puede usarse para armar/desarmar manualmente si se implementa un handler de GPIO.
// No está siendo utilizado en el firmware actual — futuro: ARM/DISARM físico.
#define PIN_BOOT_BUTTON     GPIO_NUM_0


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/core/include/pin_config.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo es el "mapa de cableado" del hardware Argus. Cada número de
   GPIO del ESP32 tiene un nombre significativo que describe qué está conectado
   físicamente en ese pin del PCB.

   Cuando el código dice "gpio_set_level(PIN_MOSFET_ENGINE, 1)", es más claro
   que "gpio_set_level(13, 1)" — el primero dice QUÉ hace, el segundo es un
   número mágico que requiere leer el esquemático para entender.

   MAPA VISUAL DEL PCB:
   ESP32 GPIO  | Nombre                | Conectado a
   ──────────────────────────────────────────────────
   GPIO32      | PIN_LED_STATUS_1      | LED izquierdo
   GPIO25      | PIN_LED_STATUS_2      | LED derecho
   GPIO33      | PIN_MOSFET_BUZZER     | MOSFET → Buzzer
   GPIO13      | PIN_MOSFET_ENGINE     | MOSFET → Relé motor [RIESGO]
   GPIO27      | PIN_MPU6050_INT       | INT del MPU6050
   GPIO22      | PIN_I2C_SCL           | SCL del MPU6050
   GPIO21      | PIN_I2C_SDA           | SDA del MPU6050
   GPIO16      | PIN_A7670_RX          | TX del A7670
   GPIO17      | PIN_A7670_TX          | RX del A7670
   GPIO4       | PIN_A7670_PWRKEY      | PWRKEY del A7670
   GPIO0       | PIN_BOOT_BUTTON       | Botón BOOT (no usado)
   GPIO26,14,  | PIN_LORA_*            | SX1276 LoRa (reservado,
   23,19,18,5  |                       | sin driver)

   DEUDA TÉCNICA:
   1. PIN_BOOT_BUTTON (GPIO0) no está siendo usado para funcionalidad de usuario.
      Se podría implementar: pulso corto = toggle ARM/DISARM.
   2. Los pines LoRa están definidos pero sin driver — confirmar si el módulo
      SX1276 está realmente montado en el PCB o si esos pines están libres.
   3. No hay definición de los niveles activos (HIGH o LOW) junto a cada pin,
      lo que requiere ir a mosfet_control.cpp para entender la lógica.

   ═══════════════════════════════════════════════════════════ */
