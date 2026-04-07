---
name: marslib-mechanisms
description: Helps create new mechanism subsystems (elevators, arms, intakes, shooters) using the MARSLib IO abstraction pattern. Use when scaffolding new mechanisms, writing LinearMechanismIO or RotaryMechanismIO layers, or configuring SysId routines.
---

# MARSLib Mechanisms Skill

You are a mechanism engineer for Team MARS 2614. When creating new physical mechanisms:

## 1. Architecture

MARSLib provides three pre-built IO patterns for common FRC mechanisms:

| IO Pattern | Physics Model | Use For |
|---|---|---|
| `LinearMechanismIO` | Linear actuator (spool + mass) | Elevators, linear slides |
| `RotaryMechanismIO` | Rotational arm (MOI + gravity) | Arms, wrists, turrets |
| `FlywheelIO` | Spinning mass (MOI) | Shooters, intakes, rollers |

Each pattern has three implementations:
- **`*IO.java`** — Interface with `@AutoLog` inputs
- **`*IOSim.java`** — dyn4j physics simulation (registers body to `MARSPhysicsWorld`)
- **`*IOTalonFX.java`** — Real hardware (Phoenix 6 TalonFX)

And one subsystem class that consumes the IO:
- **`MARSElevator.java`** — Accepts `LinearMechanismIO`
- **`MARSArm.java`** — Accepts `RotaryMechanismIO`
- **`MARSIntake.java`** / **`MARSShooter.java`** — Accept `FlywheelIO`

## 2. Key Rules

### Rule A: Subsystems Never Touch Hardware
The subsystem class (e.g., `MARSElevator`) MUST only interact through the IO interface. It calls `io.updateInputs(inputs)` and `Logger.processInputs()` in periodic. It NEVER creates a TalonFX or reads a CAN frame directly.

### Rule B: Feedforward Lives in the Subsystem
PID runs on the motor controller (TalonFX Motion Magic). Feedforward (kS, kG, kV, kA) is computed in the subsystem and applied as an additional voltage offset. Use `LoggedTunableNumber` so gains are tunable in real-time from AdvantageScope.

### Rule C: Power Shedding is Mandatory
Every mechanism subsystem accepts a `MARSPowerManager` in its constructor. In `periodic()`, it continuously scales current limits based on battery voltage:
```java
double currentLimit = MathUtil.interpolate(
    MIN_CURRENT_AMPS, MAX_CURRENT_AMPS,
    (powerManager.getVoltage() - CRITICAL_VOLTAGE) / (NOMINAL_VOLTAGE - CRITICAL_VOLTAGE));
io.setCurrentLimit(currentLimit);
```
If you skip this, the mechanism will brownout the robot during matches.

### Rule D: SysId is Built-In
Every mechanism subsystem exposes `sysIdQuasistatic(direction)` and `sysIdDynamic(direction)` commands. These are pre-wired to the IO layer's `setVoltage()` method for automated feedforward characterization.

## 3. Creating a New Mechanism

To add a new mechanism (e.g., a Climber):

1. **Decide the IO pattern.** Linear (spool-driven) → extend `LinearMechanismIO`. Rotational (arm-like) → extend `RotaryMechanismIO`. Spinning → extend `FlywheelIO`.

2. **Create the subsystem class** `MARSClimber.java`:
   ```java
   public class MARSClimber extends SubsystemBase {
       private final LinearMechanismIO io;
       private final LinearMechanismIOInputsAutoLogged inputs = new LinearMechanismIOInputsAutoLogged();
       private final MARSPowerManager powerManager;
       // constructor, periodic(), setTargetPosition(), getPositionMeters()...
   }
   ```

3. **Add constants** to `Constants.ClimberConstants`:
   - `GEAR_RATIO`, `SPOOL_DIAMETER_METERS`, `SIM_MASS_KG`
   - `NOMINAL_VOLTAGE`, `CRITICAL_VOLTAGE`, `MAX_CURRENT_AMPS`, `MIN_CURRENT_AMPS`

4. **Wire in RobotContainer** with the correct IO implementation:
   ```java
   MARSClimber climber = new MARSClimber(
       Constants.CURRENT_MODE == Mode.SIM
           ? new LinearMechanismIOSim("Climber", ClimberConstants.GEAR_RATIO, ...)
           : new LinearMechanismIOTalonFX(ClimberConstants.MOTOR_ID, ...),
       powerManager);
   ```

5. **Add to `MARSSuperstructure`** if the mechanism has collision interactions with the elevator/arm. Declare safe transitions in the state machine.

6. **Write a test** in `com.marslib.mechanisms.MARSClimberTest` using the `*IOSim` implementation.

7. **Create a skill** using the `marslib-skill-authoring` guide if the mechanism is complex.

## 4. Telemetry (per mechanism)
All mechanisms log under their subsystem name:
- `{Name}/Position` — Current position (meters or radians)
- `{Name}/Velocity` — Current velocity
- `{Name}/AppliedVolts` — Voltage sent to motor
- `{Name}/CurrentAmps` — Stator current draw
- `{Name}/TargetPosition` — Commanded setpoint
- `{Name}/CurrentLimit` — Active current limit after power shedding
