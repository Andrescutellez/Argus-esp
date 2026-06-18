#pragma once

/**
 * @file mpu6050_driver.h
 * @brief Interfaz del driver MPU6050 para detección de movimiento e impacto en Argus.
 *
 * PROPÓSITO:
 *   Abstraer toda la comunicación I2C con el MPU6050 (acelerómetro + giroscopio)
 *   detrás de una clase con métodos claros. El sistema Argus usa el MPU6050 para
 *   dos propósitos distintos:
 *   1. Detección de movimiento por hardware (interrupción en GPIO27): el MPU6050
 *      genera una señal de interrupción cuando detecta aceleración sostenida por
 *      encima de un umbral. La ISR en sensor_task.h responde en ~10µs sin
 *      necesidad de polling activo, lo que permite al sistema estar en modo de
 *      bajo consumo mientras espera.
 *   2. Lectura de datos de sensor (accel XYZ + gyro XYZ) para clasificar el tipo
 *      de movimiento: quieto, movimiento normal, impacto fuerte.
 *
 * HARDWARE:
 *   - Chip: MPU6050 o clon compatible (GY-521 con chip clon devuelve WHO_AM_I=0x70
 *     o 0x72 en lugar de 0x68 — el driver acepta los tres valores)
 *   - Bus: I2C0 (GPIO21=SDA, GPIO22=SCL) a 100kHz con pull-ups internos
 *   - Interrupción: GPIO27 en flanco ascendente (activo alto, latch hasta leer INT_STATUS)
 *   - Dirección I2C: 0x68 (pin AD0 conectado a GND)
 *   - API I2C: nueva API ESP-IDF 5.x/6.x (i2c_new_master_bus / i2c_master_transmit_receive)
 *     La API legacy fue eliminada en ESP-IDF 6.0.
 *
 * VARIABLES CRÍTICAS:
 *   - MPU6050_MOTION_THRESHOLD: umbral de detección de movimiento en LSB (1 LSB = 2mg).
 *     Valor 15 = 30mg. Si se reduce, el sistema detecta movimientos más sutiles pero
 *     aumentan las falsas alarmas por vibración del motor. Si se aumenta, se pierden
 *     movimientos legítimos de baja intensidad.
 *   - MPU6050_MOTION_DURATION: duración mínima en ms para confirmar movimiento.
 *     Valor 10ms filtra pulsos eléctricos y vibraciones mecánicas de alta frecuencia
 *     pero corta duración (típico en arranque de motor).
 *   - MPU6050_IMPACT_G_THRESHOLD: 2.5g como umbral de impacto. En reposo el sensor
 *     siempre lee ~1g (gravedad). Un choque o caída brusca produce picos de 3-8g.
 *     2.5g discrimina impactos reales de aceleraciones normales de conducción (~1.5g
 *     en frenadas bruscas).
 *   - ACCEL_SCALE = 16384.0 LSB/g: factor de conversión para escala ±2g.
 *     Si se cambia la escala del acelerómetro (registro 0x1C bits [4:3]), este valor
 *     DEBE actualizarse o las lecturas en 'g' serán incorrectas.
 *   - GYRO_SCALE = 131.0 LSB/(°/s): factor para escala ±250°/s.
 *     Si se cambia el rango del giroscopio (registro 0x1B bits [4:3]), actualizar.
 *
 * CONCURRENCIA:
 *   - init() debe llamarse exactamente UNA VEZ desde sensorTask() antes de cualquier
 *     otra operación. No es thread-safe para múltiples llamadas simultáneas.
 *   - readSensorData() puede llamarse desde cualquier tarea pero NO desde la ISR
 *     (la ISR solo debe dar un semáforo; la lectura I2C ocurre en la tarea).
 *   - configureMotionInterrupt() e installMotionISR() son solo para inicialización,
 *     no deben llamarse en runtime.
 *   - checkMotionFlag() limpia el flag al leer INT_STATUS. Si dos tareas lo llamaran
 *     simultáneamente, solo la primera vería el flag. No llamar desde múltiples tareas.
 *
 * RIESGOS DE HARDWARE:
 *   - El MPU6050 GY-521 con chip clon devuelve WHO_AM_I != 0x68. El driver lo acepta
 *     como "clon compatible" pero si el clon no implementa algún registro correctamente,
 *     el comportamiento es indefinido.
 *   - Los pull-ups internos del ESP32 (~45kΩ) son más débiles que los recomendados
 *     para I2C (4.7kΩ). A 100kHz funciona, pero si se sube la velocidad a 400kHz
 *     (MPU6050_I2C_FREQ_HZ), pueden aparecer errores de ACK. Ver comentario en la
 *     constante MPU6050_I2C_FREQ_HZ.
 *
 * @module components/drivers/mpu6050_driver
 */

#include "system_events.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ─── Dirección I2C ────────────────────────────────────────────────────────────
// 0x68 = dirección cuando AD0 (pin 9) está en GND.
// Si AD0 estuviera en VCC, sería 0x69. En el hardware Argus, AD0 está en GND.
// Los clones devuelven WHO_AM_I=0x70 o 0x72 pero responden en la misma dirección 0x68.
#define MPU6050_I2C_ADDR        0x68

// ─── Registros del MPU6050 ────────────────────────────────────────────────────
// Referencia: MPU-6000/MPU-6050 Product Specification, sección 4 (Register Map).
// Los clones compatibles implementan el mismo mapa de registros.
#define MPU6050_REG_SMPLRT_DIV          0x19  // Divisor de tasa de muestreo (1kHz / (1+div))
#define MPU6050_REG_CONFIG               0x1A  // Configuración DLPF y FSYNC
#define MPU6050_REG_GYRO_CONFIG          0x1B  // Rango del giroscopio (bits [4:3])
#define MPU6050_REG_ACCEL_CONFIG         0x1C  // Rango del acelerómetro (bits [4:3])
#define MPU6050_REG_MOT_THR              0x1F  // Umbral de detección de movimiento (LSB)
#define MPU6050_REG_MOT_DUR              0x20  // Duración mínima de movimiento (ms)
#define MPU6050_REG_INT_PIN_CFG          0x37  // Configuración del pin INT (polaridad, latch)
#define MPU6050_REG_INT_ENABLE           0x38  // Habilitación de interrupciones (bit 6 = motion)
#define MPU6050_REG_INT_STATUS           0x3A  // Estado de interrupciones (lectura limpia el flag)
#define MPU6050_REG_ACCEL_XOUT_H         0x3B  // Primer byte de 14 bytes: AX_H, AX_L, AY_H...
#define MPU6050_REG_MOT_DETECT_CTRL      0x69  // Control del acumulador de detección de movimiento
#define MPU6050_REG_PWR_MGMT_1           0x6B  // Gestión de energía (bit 6 = sleep mode)
#define MPU6050_REG_WHO_AM_I             0x75  // ID del chip (0x68 oficial, 0x70/0x72 en clones)

// ─── Parámetros de detección ──────────────────────────────────────────────────

// 15 LSB × 2mg/LSB = 30mg de umbral. Valor calibrado empíricamente para motos:
// suficientemente alto para ignorar vibración de motor en reposo (~10mg),
// suficientemente bajo para detectar un golpe de mano al manillar (~50mg).
// Si se cambia, actualizar también el log en configureMotionInterrupt().
#define MPU6050_MOTION_THRESHOLD    15

// 10ms de duración mínima. Filtra pulsos eléctricos < 10ms que pueden aparecer
// cuando el motor arranca o cuando la batería de la moto tiene fluctuaciones.
// Si la duración es 0, cualquier pico de ruido genera una interrupción.
#define MPU6050_MOTION_DURATION     10

// 2.5g de umbral de impacto (magnitud vectorial de aceleración).
// En reposo: |accel| ≈ 1.0g (solo gravedad). Frenada fuerte: ~1.8g.
// Choque o caída de la moto: >2.5g típicamente. Ajustar si hay falsos positivos
// en terrenos irregulares.
#define MPU6050_IMPACT_G_THRESHOLD  2.5f

// ─── Configuración I2C ────────────────────────────────────────────────────────

// 100kHz (standard mode) en lugar de 400kHz (fast mode) porque los pull-ups
// internos del ESP32 (~45kΩ) no son suficientemente fuertes para 400kHz con
// las capacitancias parásitas del PCB. Para subir a 400kHz habría que agregar
// resistencias externas de 4.7kΩ al bus I2C.
#define MPU6050_I2C_FREQ_HZ     100000

// 100ms de timeout por operación I2C. Si el bus está trabado (SDA en bajo
// permanente), la operación devuelve ESP_ERR_TIMEOUT en lugar de bloquear.
// El bus bloqueado ocurre si se interrumpe el power al sensor durante una transacción.
#define MPU6050_I2C_TIMEOUT_MS  100

/**
 * @brief Driver para el IMU MPU6050 (acelerómetro + giroscopio vía I2C).
 *
 * PROPÓSITO:
 *   Encapsular toda la comunicación I2C con el MPU6050. Los consumidores de esta
 *   clase (principalmente sensorTask) solo necesitan llamar init() al inicio y
 *   luego readSensorData() periódicamente o en respuesta a la ISR.
 *
 * ESTADO INTERNO:
 *   - busHandle: handle del bus I2C master (creado por init(), nullptr antes).
 *   - devHandle: handle del dispositivo MPU6050 en el bus (creado por init()).
 *   - initialized: flag para detectar llamadas a métodos antes de init().
 *     Si initialized es false y se llama readSensorData(), las operaciones I2C
 *     fallarán con ESP_ERR_INVALID_STATE porque devHandle es nullptr.
 *
 * USO TÍPICO (desde sensorTask):
 *   MPU6050Driver mpu;
 *   mpu.init();                           // configura I2C + motion interrupt
 *   mpu.installMotionISR(motionISR, sem); // conecta GPIO27 a la ISR
 *   while (true) {
 *     xSemaphoreTake(sem, portMAX_DELAY); // espera señal de la ISR
 *     mpu.readSensorData(data);           // lee accel + gyro
 *     classifyMovement(data);             // lógica de negocio
 *   }
 */
class MPU6050Driver {
public:
    MPU6050Driver();

    /**
     * @brief Inicializa el bus I2C0 y registra el MPU6050 como dispositivo esclavo.
     *
     * PROPÓSITO:
     *   Completar la cadena de inicialización: bus I2C → dispositivo → verificación
     *   WHO_AM_I → despertar el sensor → configurar escalas y DLPF → activar
     *   detección de movimiento por hardware. Al retornar ESP_OK, el sensor está
     *   listo para recibir la ISR y para readSensorData().
     *
     * FLUJO LÓGICO:
     *   1. i2c_new_master_bus() — crea el bus I2C0 con GPIO21/22 y pull-ups internos.
     *   2. i2c_master_bus_add_device() — registra el MPU6050 a 0x68 / 100kHz.
     *   3. readReg(WHO_AM_I) — verifica que el chip responde (0x68, 0x70, o 0x72).
     *   4. writeReg(PWR_MGMT_1, 0x00) — saca el sensor del sleep mode por defecto.
     *   5. writeReg(SMPLRT_DIV, 0x09) — tasa de muestreo = 1kHz/(1+9) = 100Hz.
     *   6. writeReg(CONFIG, 0x03) — DLPF 44Hz (filtra vibración de motor).
     *   7. writeReg(ACCEL_CONFIG, 0x00) — rango ±2g (máxima resolución).
     *   8. writeReg(GYRO_CONFIG, 0x00) — rango ±250°/s.
     *   9. configureMotionInterrupt() — umbral, duración, pin INT, habilitar interrupción.
     *
     * DEPENDENCIAS:
     *   - pin_config.h: PIN_I2C_SDA (GPIO21), PIN_I2C_SCL (GPIO22)
     *   - Llamar antes de installMotionISR() y readSensorData()
     *
     * POSIBLES MEJORAS:
     *   1. Permitir pasar la frecuencia I2C como parámetro para facilitar pruebas.
     *   2. Agregar reintentos en la verificación WHO_AM_I (el sensor puede tardar
     *      hasta 30ms en arrancar desde power-on según la datasheet).
     *   3. Guardar la escala configurada para verificar que ACCEL_SCALE/GYRO_SCALE
     *      son coherentes con los registros escritos.
     *
     * @return ESP_OK si todo el flujo fue exitoso.
     * @return ESP_ERR_NOT_FOUND si WHO_AM_I devuelve un valor desconocido.
     * @return Cualquier esp_err_t de i2c_new_master_bus, i2c_master_bus_add_device,
     *         o de writeReg si algún registro no pudo escribirse.
     */
    esp_err_t init();

    /**
     * @brief Lee los datos raw del sensor y los convierte a unidades físicas.
     *
     * PROPÓSITO:
     *   Leer los 14 bytes contiguos del MPU6050 (ACCEL_XOUT_H a GYRO_ZOUT_L),
     *   reconstruir los valores int16_t big-endian, convertirlos a g y °/s,
     *   y evaluar los flags motion_detected e impact_detected.
     *
     * FLUJO LÓGICO:
     *   1. readRegs(ACCEL_XOUT_H, buf, 14) — lectura burst de 14 bytes (eficiente).
     *   2. Reconstrucción big-endian: raw_ax = (buf[0] << 8) | buf[1], etc.
     *      Temperatura (buf[6], buf[7]) se descarta — no es relevante para Argus.
     *   3. Conversión: data.accel_x = raw_ax / 16384.0 (g), data.gyro_x = raw_gx / 131.0 (°/s).
     *   4. checkMotionFlag() — lee INT_STATUS para ver si la interrupción de movimiento
     *      está activa. La lectura limpia el flag automáticamente (latch mode).
     *   5. isImpact(data) — evalúa si |accel_vector| > 2.5g.
     *
     * DEPENDENCIAS:
     *   - init() debe haberse llamado antes (devHandle debe ser válido).
     *   - SensorData_t (system_events.h): struct de salida con campos accel_x/y/z,
     *     gyro_x/y/z, motion_detected, impact_detected.
     *
     * NOTA DE HARDWARE:
     *   Los bytes 6 y 7 del burst son temperatura (TEMP_OUT_H/L). Se leen pero no
     *   se almacenan en SensorData_t. Si en el futuro se agrega monitoreo de temperatura,
     *   int16_t raw_temp = (buf[6] << 8) | buf[7]; temp_C = raw_temp / 340.0 + 36.53.
     *
     * @param[out] data Estructura donde se escriben los datos del sensor.
     * @return ESP_OK si la lectura fue exitosa.
     * @return esp_err_t de i2c_master_transmit_receive si la comunicación falló.
     */
    esp_err_t readSensorData(SensorData_t& data);

    /**
     * @brief Configura el hardware de detección de movimiento del MPU6050.
     *
     * PROPÓSITO:
     *   Programar los registros MOT_THR, MOT_DUR, MOT_DETECT_CTRL, INT_PIN_CFG
     *   e INT_ENABLE para que el MPU6050 genere una señal en su pin INT cuando
     *   detecte movimiento por encima del umbral durante el tiempo mínimo.
     *   Esta configuración permite detección de movimiento por hardware sin
     *   polling activo desde la CPU.
     *
     * FLUJO LÓGICO:
     *   1. MOT_THR = MPU6050_MOTION_THRESHOLD (15) — umbral de 30mg.
     *   2. MOT_DUR = MPU6050_MOTION_DURATION (10) — confirmar en 10ms.
     *   3. MOT_DETECT_CTRL = 0x15 — decremento del acumulador habilitado.
     *      Valor 0x15 = decr_rate[1:0]=01 (decr 2 cada muestra), ff_count[1:0]=01,
     *      mot_count[1:0]=01. Controla la velocidad con que el acumulador de
     *      movimiento sube y baja. Sin este registro, la detección puede ser errática.
     *   4. INT_PIN_CFG = 0x30 — pin INT activo alto, latch hasta que se lea INT_STATUS,
     *      y limpiar el flag al leer cualquier registro (no solo INT_STATUS).
     *   5. INT_ENABLE = 0x40 — habilitar solo el bit 6 (MOT_INT). Los otros bits
     *      (DATA_RDY, FIFO_OFLOW, etc.) permanecen deshabilitados.
     *
     * POSIBLES MEJORAS:
     *   1. Exponer MOTION_THRESHOLD y MOTION_DURATION como parámetros de función
     *      para que sensor_task pueda cambiarlos al aplicar niveles de sensibilidad,
     *      en lugar de cambiar solo las variables de clasificación en software.
     *      Esto daría un verdadero ajuste de sensibilidad a nivel de hardware.
     *
     * @return ESP_OK si todos los registros se escribieron correctamente.
     * @return esp_err_t de writeReg si alguna escritura falló.
     */
    esp_err_t configureMotionInterrupt();

    /**
     * @brief Instala el handler ISR en GPIO27 (flanco ascendente).
     *
     * PROPÓSITO:
     *   Configurar GPIO27 como entrada con detección de flanco ascendente y registrar
     *   la función isrHandler para que sea invocada por el driver GPIO de FreeRTOS
     *   cuando el MPU6050 active su pin INT.
     *
     * FLUJO LÓGICO:
     *   1. gpio_config() — GPIO27: entrada, flanco ascendente, pull-down habilitado.
     *      El pull-down asegura nivel bajo estable cuando INT está inactivo.
     *      Sin pull-down, el pin flotante puede generar interrupciones espurias.
     *   2. gpio_install_isr_service(ESP_INTR_FLAG_IRAM) — instala el servicio de ISR
     *      global si no estaba instalado. ESP_INTR_FLAG_IRAM asegura que el código de
     *      dispatch de la ISR esté en IRAM y pueda ejecutarse incluso durante un
     *      flash cache miss. Si ya estaba instalado, esta llamada retorna ESP_ERR_INVALID_STATE
     *      que se ignora silenciosamente (correcto: no es un error si ya está instalado).
     *   3. gpio_isr_handler_add(PIN_MPU6050_INT, isrHandler, arg) — registra el handler.
     *
     * DEPENDENCIAS:
     *   - pin_config.h: PIN_MPU6050_INT (GPIO27)
     *   - isrHandler debe ser una función marcada IRAM_ATTR si accede a código/datos
     *     en flash (ver sensor_task.cpp: motionISRHandler IRAM_ATTR).
     *   - arg generalmente es un SemaphoreHandle_t para notificar a la tarea de sensor.
     *
     * RIESGO DE CONCURRENCIA:
     *   La ISR se invoca en el contexto de interrupción de hardware. El handler
     *   DEBE usar solo funciones ISR-safe de FreeRTOS (xSemaphoreGiveFromISR,
     *   xQueueSendFromISR, etc.). Nunca bloquear ni llamar funciones que requieran
     *   el scheduler en la ISR.
     *
     * @param isrHandler Función a invocar al detectar flanco en GPIO27. Debe ser IRAM_ATTR.
     * @param arg Argumento pasado a isrHandler (típicamente un SemaphoreHandle_t).
     * @return ESP_OK si el handler fue registrado exitosamente.
     * @return esp_err_t de gpio_config o gpio_isr_handler_add si hubo error.
     */
    esp_err_t installMotionISR(void (*isrHandler)(void* arg), void* arg);

    /**
     * @brief Lee INT_STATUS y retorna true si hay una interrupción de movimiento pendiente.
     *
     * PROPÓSITO:
     *   Consultar el bit 6 (MOT_INT) del registro INT_STATUS para confirmar que la
     *   interrupción fue generada por el detector de movimiento. La lectura del registro
     *   limpia automáticamente el flag (latch mode configurado en INT_PIN_CFG = 0x30).
     *
     * NOTA IMPORTANTE:
     *   Si este método retorna false cuando se esperaba true, puede deberse a que:
     *   1. Otro código ya leyó INT_STATUS antes (el flag se limpió).
     *   2. La interrupción fue generada por otro evento (DATA_RDY, etc.) — pero en
     *      la configuración actual solo MOT_INT está habilitado, así que esto no ocurre.
     *   3. La lectura I2C falló (ignorada silenciosamente aquí — mejorable).
     *
     * POSIBLES MEJORAS:
     *   - Retornar esp_err_t en lugar de bool para detectar fallos de comunicación.
     *   - Leer también otros bits de INT_STATUS para debug (DATA_RDY, FIFO_OFLOW).
     *
     * @return true si el bit MOT_INT está activo en INT_STATUS.
     * @return false si no hay interrupción de movimiento activa o si la lectura falló.
     */
    bool checkMotionFlag();

    /**
     * @brief Evalúa si la magnitud del vector de aceleración supera el umbral de impacto.
     *
     * PROPÓSITO:
     *   Detectar eventos de alto impacto (choques, caídas) calculando la norma euclidiana
     *   del vector de aceleración (accel_x, accel_y, accel_z) y comparándola con
     *   MPU6050_IMPACT_G_THRESHOLD (2.5g).
     *
     * POR QUÉ MAGNITUD Y NO UN EJE INDIVIDUAL:
     *   La orientación del sensor en la moto puede variar según la instalación.
     *   Un impacto puede distribuirse entre los tres ejes dependiendo de la dirección.
     *   La magnitud del vector es invariante a la orientación: siempre refleja la
     *   intensidad total de la aceleración independientemente de cómo esté montado el sensor.
     *
     * NOTA DE FÍSICA:
     *   En reposo, la magnitud siempre es ~1.0g (la gravedad). Para detectar impactos,
     *   el umbral de 2.5g implica que la aceleración total (gravedad + dinámica) supere 2.5g.
     *   Esto significa que la aceleración dinámica pura sería ~1.5g en el peor caso.
     *
     * @param data Datos del sensor ya convertidos a g. Debe haber sido llenado por readSensorData().
     * @return true si |accel_vector| > MPU6050_IMPACT_G_THRESHOLD.
     */
    static bool isImpact(const SensorData_t& data);

private:
    // Handle del bus I2C master creado por i2c_new_master_bus(). nullptr antes de init().
    // Si es nullptr cuando se llama writeReg/readReg, el resultado es ESP_ERR_INVALID_STATE.
    i2c_master_bus_handle_t busHandle;

    // Handle del dispositivo MPU6050 en el bus I2C. Creado por i2c_master_bus_add_device().
    // Este handle identifica al MPU6050 específicamente en el bus; si hubiera otros
    // dispositivos I2C, cada uno tendría su propio devHandle.
    i2c_master_dev_handle_t devHandle;

    // Flag que indica si init() completó exitosamente. Usado para detectar llamadas
    // prematuras a readSensorData() — no valida activamente en cada llamada (sin guard).
    bool initialized;

    /**
     * @brief Escribe un byte en un registro del MPU6050.
     * Usa i2c_master_transmit() con el formato [reg_addr, value] en una transacción.
     */
    esp_err_t writeReg(uint8_t reg, uint8_t value);

    /**
     * @brief Lee un byte de un registro del MPU6050.
     * Usa i2c_master_transmit_receive(): primero envía reg_addr, luego lee el valor.
     */
    esp_err_t readReg(uint8_t reg, uint8_t& value);

    /**
     * @brief Lee N bytes consecutivos desde un registro del MPU6050 (lectura burst).
     * El MPU6050 auto-incrementa la dirección de registro en cada byte leído.
     * Más eficiente que N llamadas individuales a readReg() para datos contiguos.
     */
    esp_err_t readRegs(uint8_t reg, uint8_t* buf, size_t len);
};


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/drivers/include/mpu6050_driver.h
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo define la interfaz del "sensor de movimiento" del sistema Argus.
   El MPU6050 es un chip que combina un acelerómetro (mide fuerzas de aceleración)
   y un giroscopio (mide velocidad de rotación). En Argus se usa principalmente
   para detectar dos cosas:
   1. ¿La moto está moviéndose? → interrupción por hardware en GPIO27
   2. ¿Fue un golpe fuerte o caída? → magnitud del vector de aceleración > 2.5g

   La comunicación con el chip es por I2C: un protocolo de dos cables (SDA y SCL)
   que permite leer y escribir los registros internos del sensor.

   TABLA DE MÉTODOS:
   Método                 | Fase       | Propósito
   -----------------------|------------|-------------------------------------------
   init()                 | Arranque   | Crea bus I2C, registra chip, configura escalas
   configureMotionInterrupt()| Arranque| Configura umbral/duración de movimiento
   installMotionISR()     | Arranque   | Conecta GPIO27 al handler de interrupción
   readSensorData()       | Runtime    | Lee 14 bytes y convierte a g y °/s
   checkMotionFlag()      | Runtime    | Verifica si hay interrupción de movimiento activa
   isImpact()             | Runtime    | Evalúa si la aceleración supera 2.5g

   PSEUDOCÓDIGO DE USO (desde sensorTask):
   mpu.init()
     → i2c_new_master_bus (GPIO21/22, 100kHz)
     → i2c_master_bus_add_device (0x68)
     → verificar WHO_AM_I (0x68/0x70/0x72)
     → despertar sensor (PWR_MGMT_1 = 0)
     → configurar 100Hz, DLPF 44Hz, ±2g, ±250°/s
     → configureMotionInterrupt (umbral=15, dur=10)
   mpu.installMotionISR(motionISRHandler, semaphore)
     → GPIO27: entrada, flanco ascendente, pull-down
     → gpio_isr_handler_add(GPIO27, handler, sem)

   En el loop de sensorTask:
   xSemaphoreTake(motionSem)  ← señal de la ISR
   mpu.readSensorData(data)   ← 14 bytes burst I2C
   classifyMovement(data)     ← lógica de negocio

   DIAGRAMA MENTAL:
   MPU6050 chip
     ↕ I2C (SDA=GPIO21, SCL=GPIO22, 100kHz)
   MPU6050Driver
     → writeReg / readReg / readRegs   (operaciones atómicas I2C)
     → init()                          (configuración completa al arranque)
     → readSensorData()                (14 bytes → g y °/s → motion/impact flags)
     → GPIO27 (INT pin del MPU6050)
          ↓ flanco ascendente
       motionISRHandler (IRAM_ATTR) → xSemaphoreGiveFromISR
          ↓
       sensorTask despierta → readSensorData → classifyMovement

   VARIABLES CRÍTICAS:
   - MPU6050_MOTION_THRESHOLD (15 LSB = 30mg): ajusta sensibilidad de detección
   - MPU6050_IMPACT_G_THRESHOLD (2.5g): ajusta detección de impactos
   - ACCEL_SCALE (16384): DEBE coincidir con la escala configurada en ACCEL_CONFIG
   - initialized: si es false, las operaciones I2C fallarán silenciosamente

   DEUDA TÉCNICA:
   1. configureMotionInterrupt() no expone sus parámetros como argumentos: si los
      5 niveles de sensibilidad (VERY_LOW→VERY_HIGH) deben ajustar el umbral en
      el hardware (no solo en software), hay que refactorizar esta función.
   2. checkMotionFlag() ignora errores I2C: si la lectura de INT_STATUS falla,
      retorna false sin indicar el problema.
   3. No hay guard en readSensorData() que verifique initialized antes de operar.

   ═══════════════════════════════════════════════════════════ */
