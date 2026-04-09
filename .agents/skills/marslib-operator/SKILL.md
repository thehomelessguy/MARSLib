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
| `TeleopDriveMath` | Pure-function joystick→ChassisSpeeds math (tested separately) |

### Dual Controller Layout
MARSLib uses a **pilot + copilot** controller setup:
- **Port 0** — Drive pilot (translation, rotation, intake, shooting, climb, ghost recording)
- **Port 1** — Copilot (manual feed, fixed scoring, cowl home, climber reverse, swerve stop)

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

### Rule C: Use LoggedTunableNumber for Manual Overrides
Any driver-adjustable value (arm offset, shot angle trim) MUST use `LoggedTunableNumber`. This allows pit crew to adjust without recompiling.

### Rule D: Dashboard Minimalism
The main driving AdvantageScope tab shows ONLY:
- Robot pose on field
- Current superstructure state
- Game piece possession
- Active faults/alerts

Raw encoder values, PID errors, and current draws go on a separate debugging tab.

## 3. Current Button Bindings

### Pilot (Port 0)
| Input | Action |
|---|---|
| Left Stick | Translation (field-relative, slew-limited) |
| Right Stick X | Rotation (with gyro-lock when idle) |
| Left Trigger | Intake deploy + run → STOWED on release |
| Right Trigger | Shoot-On-The-Move + SCORE → STOWED on release |
| A | Slamtake (INTAKE_RUNNING) → STOWED on release |
| B | Stationary SCORE → STOWED on release |
| Left Bumper | UNJAM → STOWED on release |
| Right Bumper | SCORE → STOWED on release |
| DPad Right | Deploy intake (INTAKE_DOWN, no spin) |
| DPad Left | Retract intake (STOWED) |
| DPad Up | Manual climber extend (12V) |
| DPad Down | Manual climber retract (-12V) |
| Back + Start | Ghost recording (records all axes and buttons) |
| Start | Diagnostic hardware check |

### Copilot (Port 1)
| Input | Action |
|---|---|
| Left Trigger | Manual feed (feeder + floor intake at 6V) |
| Right Trigger | Fixed target SCORE → STOWED on release |
| Right Bumper | Fixed target SCORE → STOWED on release |
| Left Bumper | Cowl home position |
| DPad Down | Climber reverse (-12V) |
| X | Emergency swerve stop |

## 4. Adding New Bindings
1. Define the command binding in `RobotContainer` using `controller.{button}().onTrue/whileTrue(...)`.
2. Ensure the command `requires()` every subsystem it actuates.
3. If the action pairs with a release (e.g., intake → stowed), use `.onFalse(superstructure.setAbsoluteState(STOWED))`.
4. Add rumble feedback if the event is invisible to the driver (see Rule B).
5. Update the button map table in this skill's §3 section.
6. Add a test validating the command sequence in `RobotContainerTest` or the relevant subsystem test.

## 5. Command API
```java
// Standard bindings in RobotContainer:
controller.leftTrigger()
    .onTrue(superstructure.setAbsoluteState(SuperstructureState.INTAKE_RUNNING))
    .onFalse(superstructure.setAbsoluteState(SuperstructureState.STOWED));

controller.rightTrigger()
    .whileTrue(new ShootOnTheMoveCommand(drive, vxSupplier, vySupplier))
    .onTrue(superstructure.setAbsoluteState(SuperstructureState.SCORE))
    .onFalse(superstructure.setAbsoluteState(SuperstructureState.STOWED));
```

## 6. Telemetry
- `Teleop/RawJoystickX` — Raw left stick Y axis
- `Teleop/RawJoystickY` — Raw left stick X axis
- `Teleop/RawJoystickOmega` — Raw right stick X axis
- `Teleop/PostDeadband` — Post-deadband values [x, y, omega]
- `Teleop/FieldRelSpeeds` — Field-relative speeds [vx, vy, omega]
- `Teleop/RobotRelSpeeds` — Robot-relative speeds [vx, vy, omega]
- `Teleop/GyroLockActive` — Boolean: heading hold active
- `LED/CurrentPattern` — Active LED pattern name
- `LED/FaultFlashActive` — Boolean: fault flash override active
