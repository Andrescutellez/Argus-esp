/**
 * @file mpu6050_driver.cpp
 * @brief Implementación del driver MPU6050 con la nueva API I2C Master de ESP-IDF 5.x/6.x.
 *
 * PROPÓSITO:
 *   Implementar todos los métodos de MPU6050Driver: inicialización del bus I2C,
 *   configuración del sensor, lectura de datos, y manejo de la interrupción de
 *   detección de movimiento. Este driver es la capa más baja del stack de sensores
 *   de Argus — todo lo que sensorTask sabe sobre el hardware pasa por aquí.
 *
 * MIGRACIÓN DE API (ESP-IDF 5.x/6.x):
 *   La API legacy fue eliminada en ESP-IDF 6.0. Equivalencias:
 *   LEGACY                              → NUEVA API
 *   i2c_param_config() + i2c_driver_install() → i2c_new_master_bus()
 *   i2c_cmd_link_create()              → (eliminado)
 *   i2c_master_start/write/read/stop() → i2c_master_transmit() / i2c_master_transmit_receive()
 *   i2c_master_cmd_begin()             → (integrado en transmit/receive)
 *   i2c_driver_delete()                → i2c_master_bus_rm_device() + i2c_del_master_bus()
 *
 * PROTOCOLO I2C DEL MPU6050:
 *   Escritura de registro:
 *     START → [addr|W] → [REG_ADDR] → [VALUE] → STOP
 *     Implementado en: writeReg() → i2c_master_transmit(buf={reg,val}, 2)
 *
 *   Lectura de registro:
 *     START → [addr|W] → [REG_ADDR] → START → [addr|R] → [VALUE] → STOP
 *     Implementado en: readReg() → i2c_master_transmit_receive(&reg, 1, &val, 1)
 *
 *   Lectura burst de N registros (auto-incremento):
 *     START → [addr|W] → [REG_ADDR] → START → [addr|R] → [V0][V1]...[VN-1] → STOP
 *     Implementado en: readRegs() → i2c_master_transmit_receive(&reg, 1, buf, N)
 *     El MPU6050 auto-incrementa la dirección del registro después de cada byte leído.
 *     Los 14 bytes desde ACCEL_XOUT_H cubren AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L,
 *     TEMP_H, TEMP_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L.
 *
 * FACTORES DE CONVERSIÓN:
 *   ACCEL_SCALE = 16384.0 LSB/g → para escala ±2g (bits [4:3] de ACCEL_CONFIG = 00b).
 *   GYRO_SCALE  = 131.0 LSB/(°/s) → para escala ±250°/s (bits [4:3] de GYRO_CONFIG = 00b).
 *   Si alguna escala se cambia, los factores deben actualizarse:
 *     ±4g   → ACCEL_SCALE = 8192
 *     ±8g   → ACCEL_SCALE = 4096
 *     ±16g  → ACCEL_SCALE = 2048
 *     ±500°/s  → GYRO_SCALE = 65.5
 *     ±1000°/s → GYRO_SCALE = 32.8
 *     ±2000°/s → GYRO_SCALE = 16.4
 *
 * VARIABLES CRÍTICAS:
 *   - ACCEL_SCALE: si no coincide con la escala en ACCEL_CONFIG, los valores en g
 *     serán incorrectos — isImpact() dará falsos positivos o negativos.
 *   - MPU6050_MOTION_THRESHOLD (mpu6050_driver.h): umbral de hardware del detector.
 *     Afecta directamente a la tasa de interrupciones GPIO27.
 *   - busHandle / devHandle: ambos deben ser válidos antes de cualquier I2C.
 *     Si init() falla a medias y devHandle queda nullptr, las operaciones posteriores
 *     producen ESP_ERR_INVALID_STATE o un crash por null pointer.
 *
 * RIESGOS DE CONCURRENCIA:
 *   - readSensorData() y checkMotionFlag() comparten el bus I2C con devHandle.
 *     En Argus, el MPU6050 es el único dispositivo en I2C0, y la nueva API I2C
 *     de ESP-IDF tiene locking interno por bus. No hay riesgo de concurrencia I2C
 *     entre tareas si solo sensorTask usa estas funciones.
 *   - Si en el futuro se agregan otros dispositivos I2C al mismo bus, la nueva API
 *     gestiona el arbitraje automáticamente a través del bus handle compartido.
 *
 * @module components/drivers/mpu6050_driver
 */

#include "mpu6050_driver.h"
#include "pin_config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <cmath>
#include <string.h>

static const char* TAG = "MPU6050";

// Factor de conversión para acelerómetro en escala ±2g.
// 16384 LSB por g → rango de -32768 a +32767 representa -2g a +2g.
// Si se cambia ACCEL_CONFIG a ±4g/±8g/±16g, este valor debe cambiar también.
static constexpr float ACCEL_SCALE = 16384.0f;

// Factor de conversión para giroscopio en escala ±250°/s.
// 131 LSB por °/s → rango de -32768 a +32767 representa -250°/s a +250°/s.
// Si se cambia GYRO_CONFIG a ±500/±1000/±2000°/s, este valor debe cambiar también.
static constexpr float GYRO_SCALE  = 131.0f;

// ─── Constructor ──────────────────────────────────────────────────────────────

/**
 * @brief Inicializa los handles a nullptr e initialized a false.
 *
 * Los handles son nullptr hasta que init() completa exitosamente.
 * Cualquier operación I2C antes de init() pasará un nullptr a la API y
 * recibirá ESP_ERR_INVALID_STATE o un crash — no hay guard explícito.
 */
MPU6050Driver::MPU6050Driver()
    : busHandle(nullptr), devHandle(nullptr), initialized(false) {}

// ─── Inicialización ───────────────────────────────────────────────────────────

/**
 * @brief Inicializa el bus I2C0, registra el MPU6050 y configura todos los parámetros.
 *
 * PROPÓSITO:
 *   Completar la secuencia de inicialización en un solo método para que sensorTask
 *   solo tenga que llamar mpu.init() al arranque. Si cualquier paso falla, se retorna
 *   el error y el sensor queda en estado no inicializado (initialized = false).
 *
 * FLUJO LÓGICO:
 *   Bus I2C → Registro del dispositivo → WHO_AM_I → Despertar → Configurar →
 *   configureMotionInterrupt()
 *
 * NOTAS DE HARDWARE:
 *   - glitch_ignore_cnt = 7: filtro de glitches de 7 ciclos de clock. Valor estándar
 *     recomendado por Espressif para I2C a 100kHz con pull-ups internos.
 *   - DLPF (Digital Low Pass Filter) = 0x03 = 44Hz bandwidth. Filtra frecuencias
 *     de vibración del motor (>100Hz) que podrían causar lecturas erróneas en el
 *     acelerómetro. 44Hz pasa las frecuencias de movimiento real de la moto (<10Hz).
 *   - SMPLRT_DIV = 9: tasa de salida = 1kHz / (1+9) = 100Hz. La tarea sensorTask
 *     puede leer a 100Hz si es necesario, aunque normalmente lee solo cuando
 *     la ISR de movimiento se dispara.
 *   - configureMotionInterrupt() al final: los parámetros de DLPF y SMPLRT_DIV
 *     deben estar configurados antes del detector de movimiento o el comportamiento
 *     del acumulador es indefinido según la datasheet del MPU6050.
 */
esp_err_t MPU6050Driver::init() {
    // Paso 1: Configurar y crear el bus I2C Master.
    // La nueva API crea el bus como un recurso reutilizable compartido entre dispositivos.
    // flags.enable_internal_pullup = true activa los pull-ups internos del ESP32 (~45kΩ).
    // Son más débiles que los pull-ups externos recomendados (4.7kΩ) pero suficientes
    // para 100kHz en PCBs con traza corta.
    i2c_master_bus_config_t busConfig = {};
    busConfig.i2c_port            = I2C_NUM_0;
    busConfig.sda_io_num          = PIN_I2C_SDA;      // GPIO21
    busConfig.scl_io_num          = PIN_I2C_SCL;      // GPIO22
    busConfig.clk_source          = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt   = 7;
    busConfig.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&busConfig, &busHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error creando bus I2C master: %s", esp_err_to_name(ret));
        return ret;
    }

    // Paso 2: Registrar el MPU6050 como dispositivo en el bus.
    // devConfig.device_address es la dirección I2C de 7 bits (sin el bit R/W).
    // La nueva API añade el bit R/W automáticamente en cada transacción.
    i2c_device_config_t devConfig = {};
    devConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devConfig.device_address  = MPU6050_I2C_ADDR;     // 0x68
    devConfig.scl_speed_hz    = MPU6050_I2C_FREQ_HZ;  // 100kHz

    ret = i2c_master_bus_add_device(busHandle, &devConfig, &devHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error registrando MPU6050 en bus I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Paso 3: Verificar comunicación con WHO_AM_I.
    // Chip original: WHO_AM_I = 0x68. Clones frecuentes devuelven 0x70 o 0x72.
    // Si el registro no responde o devuelve un valor desconocido, el hardware
    // no está conectado correctamente o AD0 no está en GND.
    uint8_t whoAmI = 0;
    ret = readReg(MPU6050_REG_WHO_AM_I, whoAmI);
    if (ret != ESP_OK || (whoAmI != 0x68 && whoAmI != 0x70 && whoAmI != 0x72)) {
        ESP_LOGE(TAG, "MPU6050 no responde (WHO_AM_I=0x%02X)", whoAmI);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MPU6050 detectado (WHO_AM_I=0x%02X)%s", whoAmI,
             whoAmI != 0x68 ? " [clon compatible]" : "");

    // Paso 4: Despertar el MPU6050.
    // Por defecto, el MPU6050 sale de reset en SLEEP mode (PWR_MGMT_1 bit6 = 1).
    // En sleep mode el sensor no toma muestras ni responde a interrupciones.
    // Escribir 0x00 limpia el bit de sleep y selecciona oscilador interno (bits[2:0]=000).
    ret = writeReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;

    // Paso 5: Tasa de muestreo = 1kHz / (1 + SMPLRT_DIV) = 1kHz / 10 = 100Hz.
    // La tasa base del giroscopio es 1kHz cuando DLPF != 0 (habilitado).
    // 100Hz es suficiente para clasificar movimiento de moto; más alto consumiría
    // CPU innecesariamente si la tarea lee datos en cada interrupción.
    ret = writeReg(MPU6050_REG_SMPLRT_DIV, 0x09);
    if (ret != ESP_OK) return ret;

    // Paso 6: DLPF = 44Hz de banda pasante (CONFIG[2:0] = 011).
    // El filtro elimina componentes de frecuencia > 44Hz. El motor de una moto
    // vibra entre 50-200Hz → filtrado. El movimiento real de la moto (aceleración,
    // frenada, giro) está por debajo de 10Hz → pasa intacto.
    // Sin este filtro, la vibración del motor saturará el acelerómetro.
    ret = writeReg(MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;

    // Paso 7: Escala del acelerómetro ±2g (ACCEL_CONFIG[4:3] = 00b).
    // ±2g es el rango más sensible: 16384 LSB/g. Adecuado para detección de
    // movimiento suave. Un impacto de >2g satura el sensor en este rango.
    // Para medir impactos más fuertes se necesitaría ±4g o ±8g, pero perderíamos
    // resolución en detección de movimiento suave.
    // NOTA: isImpact() usa 2.5g como umbral → satura el sensor en ±2g.
    // ARQUITECTURA ⚠️: el umbral de isImpact (2.5g) supera el rango del acelerómetro
    // (±2g). Los valores > 2g saturan a 32767 (max). isImpact podría detectar
    // cualquier saturación como impacto aunque no sea la intención.
    ret = writeReg(MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // Paso 8: Escala del giroscopio ±250°/s (GYRO_CONFIG[4:3] = 00b).
    // En Argus el giroscopio no se usa para clasificación (solo el acelerómetro).
    // Se configura de todas formas porque readSensorData() lee los 14 bytes incluyendo
    // los datos del giroscopio, y la escala debe coincidir con GYRO_SCALE = 131.
    ret = writeReg(MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    initialized = true;
    ESP_LOGI(TAG, "MPU6050 listo: 100Hz muestreo, DLPF 44Hz, Accel ±2g, Gyro ±250°/s");

    // La configuración del detector de movimiento va al final porque los registros
    // MOT_THR y MOT_DUR dependen de la configuración DLPF para calcular los tiempos
    // de acumulación internos del hardware.
    return configureMotionInterrupt();
}

// ─── Configuración de detección de movimiento ─────────────────────────────────

/**
 * @brief Configura el hardware de detección de movimiento del MPU6050.
 *
 * PROPÓSITO:
 *   Programar los registros del detector de movimiento por hardware para generar
 *   una señal en el pin INT cuando la aceleración supere el umbral durante la
 *   duración mínima. Esta detección ocurre en el propio chip, sin CPU.
 *
 * FLUJO LÓGICO:
 *   MOT_THR → MOT_DUR → MOT_DETECT_CTRL → INT_PIN_CFG → INT_ENABLE
 *
 * DETALLES DE CADA REGISTRO:
 *   MOT_DETECT_CTRL (0x69) = 0x15:
 *     - Bits [5:4] (ff_count): 01 → decremento de free-fall counter cada 2 muestras
 *     - Bits [3:2] (mot_count): 01 → decremento de motion counter cada 2 muestras
 *     - Bits [1:0] (accel_on_delay): 01 → 1 ms de delay de power del acelerómetro
 *     Este registro controla la dinámica del acumulador de movimiento. Sin él,
 *     el detector puede generar interrupciones en ráfagas o perderse eventos.
 *
 *   INT_PIN_CFG (0x37) = 0x30:
 *     - Bit 5 (LATCH_INT_EN): 1 → pin INT mantiene nivel alto hasta leer INT_STATUS
 *     - Bit 4 (INT_RD_CLEAR): 1 → limpiar flag al leer CUALQUIER registro (no solo INT_STATUS)
 *     El latch es necesario para que la ISR en GPIO27 tenga tiempo de ejecutarse.
 *     Sin latch, el pulso del pin INT puede ser tan corto que se pierda.
 *
 *   INT_ENABLE (0x38) = 0x40:
 *     - Bit 6 (MOT_EN): 1 → habilitar solo interrupción de movimiento
 *     Todos los otros bits (DATA_RDY_EN, FIFO_OFLOW_EN, etc.) son 0.
 *     Si se habilitaran otros bits, la ISR recibiría interrupciones por múltiples
 *     causas y tendría que leer INT_STATUS para distinguirlas.
 */
esp_err_t MPU6050Driver::configureMotionInterrupt() {
    esp_err_t ret;

    // Umbral de detección: 1 LSB = 2mg, umbral = 15 × 2mg = 30mg.
    // Calibrado para ignorar vibración de motor (~10mg) y detectar movimiento
    // intencional (mano al manillar, desplazamiento de la moto ~50mg).
    ret = writeReg(MPU6050_REG_MOT_THR, MPU6050_MOTION_THRESHOLD);
    if (ret != ESP_OK) return ret;

    // Duración mínima de movimiento continuo para disparar la interrupción.
    // 10ms filtra pulsos electromagnéticos y vibraciones mecánicas de corta
    // duración típicas en el arranque del motor de la moto.
    ret = writeReg(MPU6050_REG_MOT_DUR, MPU6050_MOTION_DURATION);
    if (ret != ESP_OK) return ret;

    // Control del acumulador de detección. 0x15 es el valor recomendado por
    // Invensense (fabricante del MPU6050) en la nota de aplicación AN-MPU-6000A-03.
    // Sin este registro configurado, el detector puede comportarse erráticamente
    // y generar interrupciones falsas o no generarlas cuando debería.
    ret = writeReg(MPU6050_REG_MOT_DETECT_CTRL, 0x15);
    if (ret != ESP_OK) return ret;

    // Pin INT activo alto, latch hasta leer INT_STATUS, clear al leer cualquier reg.
    // 0x30 = 0b00110000: bit5=LATCH_INT_EN=1, bit4=INT_RD_CLEAR=1.
    // Activo alto porque GPIO27 del ESP32 está en pull-down → detecta flanco ascendente.
    ret = writeReg(MPU6050_REG_INT_PIN_CFG, 0x30);
    if (ret != ESP_OK) return ret;

    // Habilitar solo la interrupción de movimiento (bit 6 = MOT_EN).
    // Los otros bits (7=FF_EN, 4=FIFO_OFLOW_EN, 3=I2C_MST_INT_EN, 0=DATA_RDY_EN)
    // quedan en 0. En Argus no se usa el FIFO ni el modo master I2C del MPU6050.
    ret = writeReg(MPU6050_REG_INT_ENABLE, 0x40);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Detección de movimiento activada (umbral=%d LSB = %dmg)",
             MPU6050_MOTION_THRESHOLD, MPU6050_MOTION_THRESHOLD * 2);
    return ESP_OK;
}

// ─── Instalación de ISR en GPIO27 ────────────────────────────────────────────

/**
 * @brief Configura GPIO27 como entrada y registra la ISR de movimiento.
 *
 * PROPÓSITO:
 *   Conectar el pin físico INT del MPU6050 (conectado a GPIO27 en el PCB)
 *   con el handler de software que se ejecutará cuando el sensor detecte movimiento.
 *   Después de esta función, el sistema puede operar por interrupciones en lugar
 *   de polling activo del sensor.
 *
 * FLUJO LÓGICO:
 *   Configurar GPIO27 (entrada, flanco ascendente, pull-down) →
 *   Instalar servicio ISR global →
 *   Registrar handler para GPIO27
 *
 * NOTAS DE HARDWARE:
 *   - Pull-down en GPIO27: el pin MPU6050 INT es activo alto. Cuando el sensor
 *     no está generando interrupción, el pin está en bajo gracias al pull-down.
 *     Sin el pull-down, el pin flotaría y generaría interrupciones espurias.
 *   - ESP_INTR_FLAG_IRAM: el código de dispatch de la ISR del GPIO se coloca en
 *     IRAM para garantizar que se ejecute incluso durante operaciones de escritura
 *     en flash (que deshabilitan el cache). Sin este flag, una interrupción durante
 *     una escritura flash podría perderse o causar un crash.
 *   - gpio_install_isr_service() puede retornar ESP_ERR_INVALID_STATE si ya fue
 *     llamada anteriormente. Esto es normal si otro componente ya la instaló.
 *     Se ignora este error intencionalmente.
 */
esp_err_t MPU6050Driver::installMotionISR(void (*isrHandler)(void* arg), void* arg) {
    // GPIO27 como entrada digital con detección de flanco ascendente.
    // pull_down_en asegura nivel bajo estable cuando INT está inactivo (sensor quieto).
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_POSEDGE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_MPU6050_INT);  // GPIO27
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;        // Mutuamente exclusivo con pull-down

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    // Instalar el servicio de ISR del GPIO. Si ya fue instalado por otro driver,
    // esta llamada retorna ESP_ERR_INVALID_STATE — se ignora porque el servicio
    // ya está activo y gpio_isr_handler_add funcionará de todas formas.
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Registrar el handler específico para GPIO27. El arg es el semáforo que la ISR
    // dará para despertar a sensorTask.
    ret = gpio_isr_handler_add(PIN_MPU6050_INT, isrHandler, arg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error registrando ISR en GPIO27: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ISR de movimiento registrada en GPIO27");
    return ESP_OK;
}

// ─── Lectura de datos del sensor ──────────────────────────────────────────────

/**
 * @brief Lee los 14 bytes del sensor y convierte a unidades físicas.
 *
 * PROPÓSITO:
 *   Leer la ráfaga de datos de aceleración y giroscopio desde el MPU6050 en
 *   una sola transacción I2C eficiente, reconstruir los valores int16_t big-endian,
 *   y convertirlos a unidades físicas (g y °/s) para que sensorTask pueda
 *   clasificar el movimiento.
 *
 * FLUJO LÓGICO:
 *   readRegs(ACCEL_XOUT_H, buf, 14) →
 *   Reconstrucción big-endian (buf[0]<<8 | buf[1]) para cada valor →
 *   División por escala (LSB → g o °/s) →
 *   checkMotionFlag() → isImpact()
 *
 * MAPA DE BYTES (inicio en ACCEL_XOUT_H = 0x3B):
 *   buf[0..1]  → ACCEL_XOUT_H/L  → accel_x (g)
 *   buf[2..3]  → ACCEL_YOUT_H/L  → accel_y (g)
 *   buf[4..5]  → ACCEL_ZOUT_H/L  → accel_z (g)
 *   buf[6..7]  → TEMP_OUT_H/L    → (descartado — temperatura no usada en Argus)
 *   buf[8..9]  → GYRO_XOUT_H/L   → gyro_x (°/s)
 *   buf[10..11]→ GYRO_YOUT_H/L   → gyro_y (°/s)
 *   buf[12..13]→ GYRO_ZOUT_H/L   → gyro_z (°/s)
 *
 * FORMATO BIG-ENDIAN:
 *   El MPU6050 envía el byte más significativo primero. La reconstrucción es:
 *   int16_t raw = (int16_t)((highByte << 8) | lowByte)
 *   El cast a int16_t es necesario para que el compilador interprete el bit 15
 *   como signo (valores negativos). Sin el cast, sería un uint16_t mal interpretado.
 *
 * @note La lectura de checkMotionFlag() limpia el bit MOT_INT en INT_STATUS.
 *       Si se llama readSensorData() varias veces seguidas, solo la primera
 *       lectura verá el flag de movimiento si la ISR se disparó una sola vez.
 */
esp_err_t MPU6050Driver::readSensorData(SensorData_t& data) {
    // Lectura burst de 14 bytes desde ACCEL_XOUT_H (0x3B).
    // Una sola transacción I2C en lugar de 6 lecturas individuales:
    // reduce la latencia total de ~6×(t_start + t_addr + t_stop) a 1×.
    uint8_t buf[14];
    esp_err_t ret = readRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo datos del sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reconstrucción big-endian de cada valor de 16 bits con signo.
    // El MPU6050 envía el byte alto primero (big-endian), lo opuesto al formato
    // little-endian del ESP32. El shift garantiza la interpretación correcta.
    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6] y buf[7] son TEMP_OUT_H/L — descartados intencionalmente.
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Conversión a unidades físicas usando los factores de la escala configurada.
    // Dividir por ACCEL_SCALE (16384) convierte de LSB a g.
    // Dividir por GYRO_SCALE (131) convierte de LSB a °/s.
    data.accel_x = (float)raw_ax / ACCEL_SCALE;
    data.accel_y = (float)raw_ay / ACCEL_SCALE;
    data.accel_z = (float)raw_az / ACCEL_SCALE;
    data.gyro_x  = (float)raw_gx / GYRO_SCALE;
    data.gyro_y  = (float)raw_gy / GYRO_SCALE;
    data.gyro_z  = (float)raw_gz / GYRO_SCALE;

    // checkMotionFlag() lee INT_STATUS (limpia el flag) y retorna el bit MOT_INT.
    // isImpact() evalúa la magnitud del vector de aceleración.
    // Ambas operaciones se hacen aquí para tener un snapshot coherente del estado.
    data.motion_detected = checkMotionFlag();
    data.impact_detected = isImpact(data);

    return ESP_OK;
}

// ─── Flags de movimiento e impacto ────────────────────────────────────────────

/**
 * @brief Lee INT_STATUS y retorna el estado del flag de movimiento.
 *
 * PROPÓSITO:
 *   Verificar si la interrupción activa en GPIO27 fue generada por el detector
 *   de movimiento (bit 6 de INT_STATUS). La lectura del registro limpia el flag
 *   automáticamente gracias a INT_RD_CLEAR configurado en INT_PIN_CFG.
 *
 * NOTA SOBRE ERRORES I2C:
 *   Si la lectura I2C falla (readReg retorna != ESP_OK), intStatus permanece en 0
 *   y el método retorna false. El error se ignora silenciosamente.
 *   ARQUITECTURA ⚠️: los errores de comunicación en checkMotionFlag() son silenciosos.
 *   Un fallo repetido del bus I2C haría que el sistema creyera que no hay movimiento.
 */
bool MPU6050Driver::checkMotionFlag() {
    uint8_t intStatus = 0;
    // El bit 6 de INT_STATUS = MOT_INT = 1 si el detector de movimiento disparó.
    // La lectura de este registro (o de cualquier otro) limpia el flag porque
    // INT_RD_CLEAR está habilitado en INT_PIN_CFG (0x30).
    readReg(MPU6050_REG_INT_STATUS, intStatus);
    return (intStatus & 0x40) != 0;  // bit 6 = MOT_INT
}

/**
 * @brief Evalúa si la magnitud del vector de aceleración supera 2.5g.
 *
 * PROPÓSITO:
 *   Detectar impactos mecánicos (choques, caídas de la moto) usando la norma
 *   euclidiana del vector (accel_x, accel_y, accel_z). Este cálculo es
 *   invariante a la orientación del sensor.
 *
 * POR QUÉ NORMA EUCLIDIANA:
 *   Si el sensor estuviera perfectamente alineado con el eje de un impacto,
 *   un solo eje sería suficiente. Pero la orientación real del sensor en la moto
 *   varía entre instalaciones. La norma del vector captura la intensidad total
 *   del impacto independientemente de la dirección.
 *
 * NOTA DE FÍSICA:
 *   En reposo: |accel| = sqrt(0² + 0² + 1²) = 1g (solo gravedad).
 *   Frenada fuerte: |accel| ≈ 1.8g.
 *   Choque o caída: |accel| > 2.5g (puede llegar a 5-10g en impactos severos).
 *
 * NOTA DE PRECISIÓN:
 *   sqrtf() usa la implementación de punto flotante de 32 bits. En el ESP32,
 *   sqrtf() tiene latencia de ~5 ciclos con la FPU habilitada (IDF default).
 *   No hay riesgo de overflow: el rango máximo del sensor es ±2g en cada eje,
 *   por lo que la magnitud máxima posible es sqrt(4+4+4) = sqrt(12) ≈ 3.46g.
 *   Esto implica que si el impacto supera 2g en cualquier eje, el sensor satura
 *   y isImpact() puede retornar false negativos para impactos muy fuertes.
 *   ARQUITECTURA ⚠️: ver nota en init() sobre la inconsistencia entre el rango
 *   ±2g del sensor y el umbral de impacto de 2.5g.
 */
bool MPU6050Driver::isImpact(const SensorData_t& data) {
    float mag = sqrtf(data.accel_x * data.accel_x +
                      data.accel_y * data.accel_y +
                      data.accel_z * data.accel_z);
    return mag > MPU6050_IMPACT_G_THRESHOLD;  // > 2.5g = impacto
}

// ─── Comunicación I2C de bajo nivel (nueva API ESP-IDF 5.x/6.x) ──────────────

/**
 * @brief Escribe un byte en un registro del MPU6050.
 *
 * FORMATO DE TRANSACCIÓN I2C:
 *   START → [addr|W (0xD0)] → [reg] → [value] → STOP
 *   i2c_master_transmit() gestiona START, la dirección con bit W, ACKs y STOP.
 *   buf[0] = dirección del registro, buf[1] = valor a escribir.
 */
esp_err_t MPU6050Driver::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    // timeout MPU6050_I2C_TIMEOUT_MS (100ms): si el bus está colgado, retorna
    // ESP_ERR_TIMEOUT en lugar de bloquearse indefinidamente.
    return i2c_master_transmit(devHandle, buf, sizeof(buf), MPU6050_I2C_TIMEOUT_MS);
}

/**
 * @brief Lee un byte de un registro del MPU6050.
 *
 * FORMATO DE TRANSACCIÓN I2C:
 *   START → [addr|W (0xD0)] → [reg] → START → [addr|R (0xD1)] → [value] → STOP
 *   i2c_master_transmit_receive() genera el repeated START automáticamente entre
 *   la fase de escritura (dirección del registro) y la fase de lectura (valor).
 */
esp_err_t MPU6050Driver::readReg(uint8_t reg, uint8_t& value) {
    return i2c_master_transmit_receive(devHandle,
                                       &reg, 1,          // transmit: 1 byte (dirección reg)
                                       &value, 1,        // receive: 1 byte (valor del reg)
                                       MPU6050_I2C_TIMEOUT_MS);
}

/**
 * @brief Lee N bytes consecutivos desde un registro del MPU6050 (lectura burst).
 *
 * FORMATO DE TRANSACCIÓN I2C:
 *   START → [addr|W] → [reg] → START → [addr|R] → [V0][V1]...[VN-1] → STOP
 *   El MPU6050 auto-incrementa la dirección del registro después de cada byte
 *   leído, lo que permite leer múltiples registros contiguos en una transacción.
 *
 * VENTAJA DE LA LECTURA BURST:
 *   14 bytes en una sola transacción vs. 14 transacciones individuales.
 *   Cada transacción I2C tiene overhead de ~20µs (START, addr, ACK, STOP).
 *   La lectura burst reduce la latencia total de ~280µs a ~40µs para los datos del sensor.
 */
esp_err_t MPU6050Driver::readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    return i2c_master_transmit_receive(devHandle,
                                       &reg, 1,          // transmit: 1 byte (dirección inicial)
                                       buf, len,         // receive: N bytes consecutivos
                                       MPU6050_I2C_TIMEOUT_MS);
}


/* ═══════════════════════════════════════════════════════════
   RESUMEN DEL MÓDULO — components/drivers/mpu6050_driver.cpp
   ═══════════════════════════════════════════════════════════

   EXPLICACIÓN PARA HUMANO:
   Este archivo hace hablar al ESP32 con el sensor de movimiento (MPU6050).
   El sensor mide cuánto se mueve, gira e impacta la moto. Para hacerlo funcionar:
   1. Se configura el canal de comunicación (I2C: dos cables, reloj y datos).
   2. Se verificam que el sensor esté conectado (WHO_AM_I).
   3. Se configura el sensor para que genere una señal eléctrica (interrupción)
      cuando detecte movimiento — así el ESP32 no tiene que preguntar cada segundo.
   4. Cuando la interrupción ocurre, se leen los datos del sensor (14 bytes en
      una sola operación eficiente) y se convierten a unidades comprensibles (g).

   PSEUDOCÓDIGO:
   init():
     → crear bus I2C0 (GPIO21/22, 100kHz, pull-ups internos)
     → registrar MPU6050 como dispositivo 0x68
     → verificar WHO_AM_I (0x68/0x70/0x72)
     → despertar sensor (PWR_MGMT_1 = 0x00)
     → configurar: 100Hz muestreo, DLPF 44Hz, ±2g, ±250°/s
     → configureMotionInterrupt(): umbral=15 LSB (30mg), duración=10ms

   readSensorData():
     → i2c_master_transmit_receive (burst 14 bytes desde 0x3B)
     → reconstruir big-endian → dividir por escala → g y °/s
     → checkMotionFlag() + isImpact()

   isImpact():
     → sqrt(ax² + ay² + az²) > 2.5g

   DIAGRAMA MENTAL:
   MPU6050 chip
     → pin INT (activo alto, latch)
          ↓ GPIO27, flanco ascendente
       motionISRHandler (IRAM_ATTR)
          ↓ xSemaphoreGiveFromISR
       sensorTask despierta
          ↓ readSensorData()
       MPU6050Driver → i2c_master_transmit_receive (14 bytes burst)
          ↓
       SensorData_t: {accel_xyz en g, gyro_xyz en °/s, motion_detected, impact_detected}
          ↓
       classifyMovement() → SystemEvent al controlTask

   CONSTANTES CRÍTICAS:
   - ACCEL_SCALE (16384): debe coincidir con ACCEL_CONFIG[4:3]=00b (±2g)
   - GYRO_SCALE (131): debe coincidir con GYRO_CONFIG[4:3]=00b (±250°/s)
   - MPU6050_IMPACT_G_THRESHOLD (2.5g): supera el rango ±2g — ver ARQUITECTURA ⚠️

   RIESGO DE HARDWARE DETECTADO:
   ARQUITECTURA ⚠️: el rango del acelerómetro es ±2g pero el umbral de impacto
   es 2.5g. Los impactos > 2g saturan el sensor a 32767 LSB (±2g máx). La magnitud
   vectorial no puede superar sqrt(3 × 2²) = 3.46g. El umbral de 2.5g sí se puede
   alcanzar con valores saturados (si los 3 ejes saturan simultáneamente la magnitud
   vectorial = 3.46g > 2.5g), pero un impacto de 2.5g en un solo eje podría no
   detectarse si los otros ejes tienen baja aceleración.
   SOLUCIÓN: cambiar ACCEL_CONFIG a ±4g o ±8g para medir impactos reales, y ajustar
   ACCEL_SCALE y el umbral de classifyMovement() en sensor_task.cpp.

   DEUDA TÉCNICA:
   1. checkMotionFlag() ignora el error de retorno de readReg() — mejorar con logging.
   2. isImpact() es static pero necesita ACCEL_SCALE implícitamente (a través de los
      datos ya convertidos). Si la escala cambia, isImpact() no necesita actualizarse.
   3. No hay deinicialización: si el driver necesita liberar el bus I2C (shutdown),
      habría que llamar i2c_master_bus_rm_device() + i2c_del_master_bus().

   ═══════════════════════════════════════════════════════════ */
