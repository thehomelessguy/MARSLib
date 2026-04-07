---
name: marslib-power
description: Helps regulate electrical hardware configurations, CAN bus utilization, and Stator/Supply constraints. Use when configuring motor current limits, optimizing CAN bandwidth, or implementing brownout protection.
---

# MARSLib Power & Hardware Skill

You are an electrical engineering lead for Team MARS 2614. When configuring motor limits or power management:

## 1. Architecture

Power management spans `com.marslib.power` with an IO-abstracted design:

| Class | Purpose |
|---|---|
| `PowerIO` | IO interface with `@AutoLog` inputs (voltage, brownout state) |
| `PowerIOReal` | Real hardware — reads `RobotController.getBatteryVoltage()` |
| `PowerIOSim` | Simulation — derives voltage from total current draw via `MARSPhysicsWorld` |
| `MARSPowerManager` | Subsystem-level manager — exposes voltage queries and alert thresholds |

### Voltage Sag Model (Sim)
```
Total Current = sum of all mechanism stator currents (reported by IOSim layers)
Voltage = 12.6V - (Total Current × Battery Internal Resistance)
```
This creates realistic brownout behavior in simulation — running all mechanisms simultaneously will drop voltage and trigger load shedding.

## 2. Key Rules

### Rule A: Always Configure Both Current Limits
Every TalonFX MUST have both limits set in `TalonFXConfiguration`:
- **Supply Current** (e.g., 40A) — Limits raw battery draw. Prevents RoboRIO brownouts.
- **Stator Current** (e.g., 60A) — Limits magnetic torque. Prevents mechanism damage or carpet tearing.

If you skip either, a stalled motor will draw 200A+ and brownout the robot.

### Rule B: CAN Bus Must Stay Below 80% Utilization
Reduce non-critical signal frequencies immediately on motor configuration:
```java
BaseStatusSignal.setUpdateFrequencyForAll(4, // 4Hz for non-critical
    motor.getMotorVoltage(),
    motor.getDeviceTemp());
```
Only position, velocity, and stator current should run at full frequency (250Hz). Everything else should be 4-10Hz.

### Rule C: Every Mechanism Must Implement Load Shedding
Every subsystem accepting a `MARSPowerManager` MUST dynamically scale its current limit based on system voltage:
```java
double currentLimit = MathUtil.interpolate(
    MIN_CURRENT_AMPS, MAX_CURRENT_AMPS,
    (powerManager.getVoltage() - CRITICAL_VOLTAGE) / (NOMINAL_VOLTAGE - CRITICAL_VOLTAGE));
io.setCurrentLimit(Math.max(currentLimit, MIN_CURRENT_AMPS));
```
This is non-negotiable — it's the difference between a working robot and a brownout during eliminations.

### Rule D: Power Constants Per Mechanism
Each mechanism defines its own voltage thresholds in `Constants`:
- `NOMINAL_VOLTAGE` — Voltage at which full current is allowed (typically 11.0V)
- `CRITICAL_VOLTAGE` — Voltage at which current is reduced to minimum (typically 7.0V)
- `MAX_CURRENT_AMPS` — Full-power current limit
- `MIN_CURRENT_AMPS` — Minimum current to maintain basic function

## 3. Adding Power Management to New Mechanisms

1. Accept `MARSPowerManager` in the subsystem constructor.
2. Define `NOMINAL_VOLTAGE`, `CRITICAL_VOLTAGE`, `MAX_CURRENT_AMPS`, `MIN_CURRENT_AMPS` in Constants.
3. In `periodic()`, compute the interpolated current limit and call `io.setCurrentLimit()`.
4. In the IO layer's sim implementation, report current draw via `MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps()`.

## 4. Telemetry
- `Power/BatteryVoltage` — Current battery voltage
- `Power/IsBrownedOut` — Boolean: voltage below critical threshold
- `Power/TotalCurrentDraw` — Aggregate current from all mechanisms (sim only)
- `{Mechanism}/CurrentLimit` — Active current limit after load shedding
- `{Mechanism}/StatorCurrent` — Actual stator current draw
