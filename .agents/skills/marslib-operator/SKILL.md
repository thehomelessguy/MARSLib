---
name: marslib-operator
description: Helps formulate Command Controller layouts, Haptics, and UI/Dashboard experiences. Use when binding controller buttons, adding rumble feedback, or configuring AdvantageScope dashboard views.
---

# MARSLib Operator UX Skill

You are the user experience lead for Team MARS 2614. When binding controls or designing driver feedback:

## 1. Architecture

The operator interface spans `com.marslib.hmi` and `frc.robot.RobotContainer`:

| Class | Purpose |
|---|---|
| `OperatorInterface` | Central controller mapping definitions and deadband config |
| `LEDManager` | Translates robot state into visual LED feedback |
| `LEDIO` | IO interface for LED hardware (addressable, CANdle) |
| `LEDIOAddressable` | WPILib AddressableLED implementation |
| `LEDIOCANdle` | CTRE CANdle implementation |
| `RobotContainer` | Command bindings, auto chooser, controller configuration |

### LED Priority System
```
MARSFaultManager.hasActiveCriticalFaults() → Critical fault flash (highest priority)
MARSPowerManager.getVoltage() < WARNING    → Load shedding amber pulse
default                                    → Alliance-colored idle pattern
```

## 2. Key Rules

### Rule A: Every Command Declares Requirements
When binding a controller button, the Command MUST `requires()` every subsystem it touches. If a composite sequence spans multiple subsystems, use `Commands.sequence()` with all requirements declared. Without this, a stray button press can interrupt an active scoring sequence.

### Rule B: Haptic Feedback for Invisible Events
Drivers can't watch the screen during matches. Fire controller rumble for:
- Game piece collected (intake sensor triggered)
- `GhostManager` macro recording complete
- Alignment locked on target (PID converged)
```java
controller.rumble(0.5, 0.3); // 50% intensity, 300ms duration
```

### Rule C: Use LoggedTunableNumber for Manual Overrides
Any driver-adjustable value (arm offset, shot angle trim) MUST use `LoggedTunableNumber`. This allows pit crew to adjust without recompiling:
```java
private final LoggedTunableNumber armOffset = new LoggedTunableNumber("Operator/ArmOffset", 0.0);
```

### Rule D: Dashboard Minimalism
The main driving AdvantageScope tab shows ONLY:
- Robot pose on field
- Current superstructure state
- Game piece possession
- Active faults/alerts

Raw encoder values, PID errors, and current draws go on a separate debugging tab. Drivers need clarity, not data.

## 3. Adding New Controller Bindings

1. Define the binding in `RobotContainer.configureButtonBindings()`.
2. Use `controller.{button}().onTrue/whileTrue/toggleOnTrue()` patterns.
3. Ensure the command `requires()` the correct subsystems.
4. Add haptic feedback for events the driver can't see.
5. Add rumble cancelation when the event expires.
6. Document the binding in the team's driver cheat sheet.

## 4. Command API
```java
// Standard bindings in RobotContainer:
driver.a().onTrue(superstructure.setAbsoluteState(INTAKE_FLOOR));
driver.b().onTrue(superstructure.setAbsoluteState(SCORE_HIGH));
driver.x().onTrue(superstructure.setAbsoluteState(STOWED));
driver.leftBumper().whileTrue(new MARSAlignmentCommand(drive, target));
driver.rightBumper().whileTrue(new ShootOnTheMoveCommand(drive, shooter, target));
```

## 5. Telemetry
- `Operator/ArmOffset` — Manual arm trim value
- `Operator/ControllerConnected` — Boolean: driver controller detected
- `LED/CurrentPattern` — Active LED pattern name
- `LED/FaultFlashActive` — Boolean: fault flash override active
- `Haptics/LastRumbleTime` — Timestamp of last rumble event
