# Miniature EV Control System (ESP32, TWAI loopback)

Two-node EV control system on one ESP32: a sensor node (DIM) and a
controller node (VCU), communicating over CAN (TWAI) in self-test loopback
mode. Jumper wire from GPIO 5 (TX) to GPIO 4 (RX), no transceiver needed.

## Demo

[Watch the demo video](media/demo.mp4)

## Features

- Dual redundant pedal potentiometers, cross-checked against each other
- MPU6050 IMU with plausibility check
- Drive state machine: `BOOT` → `IDLE` → `READY` → `DRIVE` → `FAULT`
- Torque output with slew limiting
- Brake/pedal plausibility check (APPS/BPPC-style) with hysteresis
- Fault detection, latching, and persistence across reboot (NVS)
- Fails safe to zero if the CAN connection is lost
- CAN frame logging + Python decoder/plotter

## Wiring

| Signal         | GPIO | Notes |
|----------------|------|-------|
| TWAI TX        | 5    | jumper to GPIO 4 |
| TWAI RX        | 4    | |
| I2C SDA        | 21   | MPU6050, 3V3 only |
| I2C SCL        | 22   | |
| Pot A (APPS1)  | 34   | ADC1 |
| Pot B (APPS2)  | 35   | ADC1 |
| Brake button   | 25   | `INPUT_PULLUP`, pressed = LOW |
| Start button   | 26   | `INPUT_PULLUP` |
| PWM torque out | 18   | servo or LED |
| Buzzer         | 19   | |
| Status LED     | 2    | |

## Folder layout

```
include/          headers
  pins.h
  can_dictionary.h
  project_config.h
  can_bus.h
  mpu6050.h
  fault_store.h
  dim.h
  vcu.h

src/
  main.cpp
  can_bus.cpp
  mpu6050.cpp
  fault_store.cpp
  dim/dim_tasks.cpp
  vcu/vcu_tasks.cpp

tools/
  decode_log.py
  requirements.txt
```

## Instrumentation

Every CAN frame is logged as one line by `canSend()`:

```
CANTX,<millis>,<id_hex>,<ok 0/1>,<byte0>,<byte1>,...,<byte7>
```

Capture and decode:

```
pip install -r tools/requirements.txt
pio device monitor > capture.txt
python tools/decode_log.py capture.txt
```

Live decoded view:

```
python tools/decode_log.py --port COM3 --live
```
