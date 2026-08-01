# Miniature EV Control System (ESP32, TWAI loopback)

A two-node vehicle control system running on a single ESP32: a **DIM**
(Dashboard/Instrument Module, the sensor node) and a **VCU** (Vehicle Control
Unit, the controller node). They communicate over the ESP32's built-in
TWAI/CAN controller in self-test loopback mode. No CAN transceiver hardware
needed, just a jumper wire from TX to RX.

It's a small model of how real vehicle networks are built: separate ECUs
that only exchange information as CAN frames, where the controller never
touches a sensor directly and has to decide, frame by frame, whether to
trust what it's being told.

## Demo

[Watch the demo video](media/demo.mp4) — walkthrough of the arm-to-drive
sequence, fault injection, the timeout/cut-wire test, and NVS fault
persistence across a reboot.

## What it does

- **Reads a redundant pair of pedal potentiometers** and cross-checks them
  against each other before trusting a pedal position at all. A single pot
  reading a plausible-looking value is never enough on its own.
- **Reads an MPU6050 IMU** over I2C and sanity-checks the acceleration
  magnitude against what's physically possible.
- **Runs a drive-enable state machine** (`BOOT` to `IDLE` to `READY` to
  `DRIVE` to `FAULT`) with an arming interlock (brake, start, and released
  pedal) and a timed buzzer warning before torque is allowed to flow.
- **Commands torque with slew limiting**, so a sudden pedal jump or sensor
  glitch can't turn into an instant jolt at the output (servo or LED).
- **Detects, latches, and persists faults.** A serious fault, like a DIM
  that's gone completely silent, stays latched through NVS flash storage
  and survives a full power cycle until deliberately cleared.
- **Fails safe to zero.** If the CAN loopback jumper is disconnected
  mid-drive, torque drops to zero within about 100ms, since the controller
  was never able to read the sensor directly, only through the bus.
- **Logs every CAN frame** transmitted by either node and includes a
  from-scratch Python re-decoder to plot pedal, state, and torque from a
  captured session.

## How the two nodes are separated

Three barriers, from weakest to strongest:

1. **Separate files.** `dim_tasks.cpp` and `vcu_tasks.cpp` share no includes
   beyond `pins.h`, `can_dictionary.h`, `project_config.h`, and `can_bus.h`,
   all of which are constants and plumbing, never live data.
2. **Separate linkage.** Every variable inside each node file is `static`.
   The VCU translation unit cannot name a DIM symbol; this is enforced by
   the linker, not by discipline. The only exported symbols are
   `dimStart()` and `vcuStart()`.
3. **Separate cores.** DIM tasks are pinned to core 1, VCU tasks to core 0.
   They are two CPUs that happen to share a chip, and the only channel
   between them is a byte sequence that physically leaves the TX pin and
   re-enters on RX.

## What the hardware represents

| Component | Stands in for |
|---|---|
| Pot A + Pot B | The redundant accelerator pedal position sensor pair a real EV uses. Two independent pots instead of one, so the controller can catch a single failed sensor rather than blindly trusting it. |
| Brake button | The brake pedal switch. Required to be pressed before the vehicle is allowed to arm. |
| Start button | The vehicle's start/ignition button, held together with the brake to arm the drive sequence. |
| Buzzer | The audible "vehicle is about to move" warning that real EVs sound for a few seconds before torque is enabled. |
| Servo (or LED) | The motor controller's torque output. The servo's angle (or the LED's brightness) is the "how much torque is being commanded right now" signal. |
| MPU6050 IMU | The vehicle's onboard motion sensor. Used here to sanity-check that the reported acceleration is physically plausible, similar to how a real controller cross-checks motion data against what a sensor is claiming. |

## Folder layout

```
platformio.ini          build config: esp32dev (real firmware) + 5 bringup_* envs

include/                headers, the contracts
  pins.h                physical pin map            [shared: both nodes]
  can_dictionary.h      THE PROTOCOL, ids + layouts [shared: both nodes]
  project_config.h      rates, priorities, stacks   [shared: both nodes]
  can_bus.h             TWAI peripheral wrapper     [shared plumbing]
  mpu6050.h             raw I2C accel driver        [shared plumbing]
  fault_store.h         NVS-backed latched faults   [shared plumbing]
  dim.h                 DIM public surface: dimStart()
  vcu.h                 VCU public surface: vcuStart(), DriveState, FaultCode

src/
  main.cpp              board bring-up, starts both node groups
  can_bus.cpp           TWAI driver install / send / receive / CANTX log
  mpu6050.cpp           MPU6050 register-level read/write
  fault_store.cpp       Preferences (NVS) read/write for latched faults
  dim/dim_tasks.cpp     sensor node: pedal, IMU, heartbeat tasks + ISRs
  vcu/vcu_tasks.cpp     controller node: RX task, control task, state
                        machine, torque, fault detection/latching

tools/
  decode_log.py         independent Python re-decoder of the CAN dictionary,
                        plots pedal/state/torque from a capture
  requirements.txt      matplotlib, pyserial
```

## Wiring

| Signal            | GPIO | Notes |
|-------------------|------|-------|
| TWAI TX           | 5    | jumper wire directly to GPIO 4 |
| TWAI RX           | 4    | |
| I2C SDA           | 21   | MPU6050, 3V3 only, not 5V |
| I2C SCL           | 22   | |
| Pot A (APPS1)     | 34   | ADC1, input-only pin. Wiper to 34, ends to 3V3 / GND |
| Pot B (APPS2)     | 35   | ADC1 |
| Brake button      | 25   | button to GND, `INPUT_PULLUP`, pressed = LOW |
| Start button      | 26   | button to GND, `INPUT_PULLUP` |
| PWM torque out    | 18   | servo signal, or LED through ~220R |
| Buzzer            | 19   | |
| Status LED        | 2    | |

## Instrumentation

Every CAN frame either node transmits is logged as one line by `canSend()`
itself (`src/can_bus.cpp`), so neither node needs its own logging code:

```
CANTX,<millis>,<id_hex>,<ok 0/1>,<byte0>,<byte1>,...,<byte7>
```

Capture a session and decode it offline:

```
pip install -r tools/requirements.txt
pio device monitor > capture.txt      # Ctrl+C when done
python tools/decode_log.py capture.txt
```

Or watch it decoded live while testing:

```
python tools/decode_log.py --port COM3 --live
```

## Demonstrated behavior

- **Normal arm-to-drive sequence.** Brake and start with the pedal released
  arms the vehicle. The buzzer sounds for a 2-second warning before torque
  is allowed to follow the pedal.
- **Pedal disagreement fault.** Deliberately mismatching the two pot
  readings forces torque to zero immediately, not gradually.
- **Timeout fail-safe.** Disconnecting the CAN loopback jumper mid-drive
  drops torque to zero within the configured timeout window (about 100ms
  for the pedal channel).
- **Fault latching.** A DIM that goes silent long enough latches its fault
  instead of clearing the moment it reconnects.
- **NVS persistence.** A latched fault survives a full power cycle. The
  VCU boots straight back into `FAULT` instead of forgetting what happened.
- **IMU sanity fault.** Shaking or tilting the board out of its expected
  acceleration range forces torque to zero, and recovers on its own once
  it's still again (this one isn't latched).
- **Fault reporting on the bus.** `VCU_FAULT` frames report the correct
  fault code and stay rate-limited even when several bits flip at once.
- **Cut-wire test.** Arm and drive normally, then pull the jumper
  mid-drive. Torque drops to zero fast enough to look instant, since the
  VCU only ever knew about the sensor through a channel that can be cut.
