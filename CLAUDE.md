# Argus Firmware — ESP32 IDF

## Segundo cerebro
Bóveda Obsidian: `C:\Users\Leonardo\Desktop\Proyecto Argus\Boveda Obsidian\Argus\`
Al retomar una sesión, leer: `_Claude/SEGUNDO_CEREBRO.md` → `_Claude/Pendientes Claude.md` → `_Claude/Contexto Argus.md`

## Stack
- **Framework:** ESP-IDF (C++17)
- **Target:** ESP32
- **Build:** CMake (`idf.py build`)
- **Flash:** `idf.py flash monitor`

## Estructura del proyecto

```
main/
  main.cpp                    ← entry point, inicialización y arranque de tasks

components/
  core/
    state_machine.cpp/.h      ← máquina de estados principal (IDLE/MOVING/ALERT)
    system_events.h           ← definición de eventos del sistema
    system_flags.h            ← flags globales (systemArmed, remoteAlert, planType)
    pin_config.h              ← asignación de GPIOs

  drivers/
    a7670_driver.cpp/.h       ← driver módulo celular 4G+GNSS (AT commands, TCP)
    mpu6050_driver.cpp/.h     ← driver IMU (I2C, acelerómetro, giroscopio)
    mosfet_control.cpp/.h     ← control de MOSFET (bocina, corte de corriente)

  services/
    comm_task.cpp/.h          ← task de comunicaciones (A7670, LoRa, fallback)
    sensor_task.cpp/.h        ← task de lectura de sensores (MPU6050)
    ble_task.cpp/.h           ← task BLE (proximidad, arme/desarme)
    control_task.cpp/.h       ← task de control de actuadores
    event_queue.cpp/.h        ← cola de eventos inter-task (FreeRTOS queue)
    sync_manager.cpp/.h       ← sincronización de estado entre tasks

  utils/
    logger.h                  ← logging estructurado (DEBUG/INFO/WARN/ERROR)
```

## Notas relevantes en la bóveda

| Tema | Nota |
|------|------|
| Máquina de estados y modos | `03-Firmware/Firmware ESP32 — Estados del sistema.md` |
| GPIOs y pines | `02-Hardware/GPIO Assignments.md` |
| MPU6050 y detección | `03-Firmware/Sensors & Detection — MPU6050.md` |
| Umbrales de calibración | `03-Firmware/Detection Thresholds & Calibration.md` |
| Feature flags por plan | `03-Firmware/Feature Flags by Plan.md` |
| Driver A7670 TCP | `04-Comm-Protocols/Case Study - HTTP→TCP.md` |
| Diagnóstico AT commands | `04-Comm-Protocols/Connectivity — Transport & Diagnostics.md` |
| Protocolo del payload | `01-Architecture/Protocolo Argus (frame).md` |
| Detección de jammer | `07-Security/Detección de Jammer.md` |
| Componentes y BOM | `02-Hardware/Components Inventory.md` |
| Energía y batería | `02-Hardware/Power & Energy — Architecture.md` |

## Convenciones de código
- Archivos de implementación: `.cpp`. Headers: `.h`.
- Nombres de tasks: `snake_case` (ej. `comm_task`, `sensor_task`).
- Estados: `STATE_IDLE`, `STATE_MOVING`, `STATE_ALERT` (ver `system_flags.h`).
- Modos: `MODE_IDLE`, `MODE_MONITORING`, `MODE_ALERT`, `MODE_PURSUIT`.
- Flags globales: `systemArmed`, `remoteAlert`, `planType` (FREEMIUM/PREMIUM).
- Logging: usar `logger.h` — nunca `printf` directo.

## Contexto crítico
- La arquitectura es orientada a tasks FreeRTOS con comunicación via `event_queue`.
- `state_machine.cpp` coordina transiciones. Los handlers son **idempotentes**.
- El módulo A7670 usa TCP directo (NO stack HTTP interno del módulo). Ver AT commands en `a7670_driver.cpp`.
- El MPU6050 está en I2C (GPIO 21/22). Ver `pin_config.h` para asignaciones completas.
