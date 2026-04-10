---
name: marslib-superstructure
description: Helps manage the MARSSuperstructure state machine for coordinating cowl, intake pivot, floor intake, shooter, and feeder mechanisms. Use when adding scoring states, modifying intake sequencing, or debugging collision-safe transitions.
---

# MARSLib Superstructure Skill

You are a Mechanism Coordination Engineer for Team MARS 2614. When modifying the superstructure state machine or adding new scoring/intake configurations:

## 1. Architecture
`MARSSuperstructure` is the central orchestrator that manages the cowl angle, intake deployment, floor intake, shooter flywheel, and feeder. It is built on `MARSStateMachine<SuperstructureState>`, a generic FSM framework. The 2026 REBUILT chassis has decoupled mechanisms (no elevator-arm collision path), so all transitions are wildcarded.

### Subsystem Dependencies
| Dependency | Type | Role |
|---|---|---|
| `MARSArm cowl` | `RotaryMechanismIO` | Adjusts shot angle based on distance |
| `MARSArm intakePivot` | `RotaryMechanismIO` | Deploys/retracts floor intake |
| `MARSShooter floorIntake` | `FlywheelIO` | Spins floor rollers to collect game pieces |
| `MARSShooter shooter` | `FlywheelIO` | Main flywheel (4-motor, closed-loop velocity) |
| `MARSShooter feeder` | `FlywheelIO` | Transfers pieces from intake to shooter |
| `Supplier<Pose2d>` | Lambda | Robot pose for sim shooting direction |
| `DoubleSupplier` | Lambda | Distance to target for shot lookup |
| `ShotSetup` | Utility | Interpolates RPM and cowl angle from distance |

### State Enum
```java
public enum SuperstructureState {
    STOWED,          // Everything retracted, shooter at idle RPM
    INTAKE_DOWN,     // Intake deployed (pivot down) but rollers OFF
    INTAKE_RUNNING,  // Intake deployed AND rollers spinning at 12V
    SCORE,           // Cowl aimed, shooter at target RPM, feeder transfers on tolerance
    UNJAM,           // All mechanisms reversed at -6V to clear jams
    BEACHED          // Safety: all motors disabled during field obstacle traversal
}
```

### Transition Graph
Transitions use explicit bidirectional links via `addValidBidirectional()`. BEACHED is reachable from every state and can return to any state:
```
STOWED ↔ INTAKE_DOWN ↔ INTAKE_RUNNING
STOWED ↔ SCORE
STOWED ↔ UNJAM
Any ↔ BEACHED (via forceState)
```

### Auto-Beach Safety
When the gyro reports a robot tilt exceeding **25°** (e.g., traversing a field obstacle), the superstructure automatically forces into `BEACHED` state, disabling all motors. This prevents mechanism damage during high-tilt maneuvers. The tilt is supplied via `Supplier<Double> tiltRadiansSupplier`.

## 2. Key Rules

### Rule A: Shot Parameters Are Distance-Dependent
The cowl angle and shooter RPM in `SCORE` state are dynamically calculated by `EliteShooterMath.calculateShotOnTheMove()`. The result is cached once per periodic via `calculateStaticShot()` — do NOT call it multiple times.

### Rule A2: Use AllianceUtil
All alliance checks must use `AllianceUtil.isRed()` / `AllianceUtil.isBlue()` from `com.marslib.util`. Do NOT call `DriverStation.getAlliance()` directly — it allocates an `Optional` every call.

### Rule B: Feeder Only Runs When Flywheel + Cowl Are Ready
In `SCORE` state, the feeder and floor intake only activate after `shooter.isAtTolerance() && cowl.isAtTolerance()`. This prevents feeding pieces into a flywheel that hasn't spun up, which would jam or drop shots.

### Rule C: Intake Checks Physics World for Collection
When in `INTAKE_RUNNING`, the superstructure calls `MARSPhysicsWorld.getInstance().checkIntake()` to detect game piece collection in simulation. The intake must be `isAtTolerance()` (deployed) before checking. Maximum capacity is 40 pieces.

### Rule D: Shooting Uses Alliance-Aware Legality
When scoring, `isLegalShot` is determined by whether the robot is on the correct side of the field for its alliance. The piece is launched in the direction the robot is facing, but only counts as scored if `correctSide` is true.

### Rule E: Idle Flywheel Management
Outside of `SCORE` state, the shooter maintains a 1500 RPM idle speed. If current RPM exceeds 1600, it coasts down via `setVoltage(0)` instead of actively braking, to reduce wear.

## 3. Adding New States
To add a new `SuperstructureState`:
1. Add the enum value to `SuperstructureState`.
2. Add behavior in the appropriate private helper (`handleIntakeLogic`, `handleScoringLogic`, `handleUnjamLogic`, or a new helper).
3. Add target angle logic in `updateMechanismTargets()`.
4. Register transitions in the constructor with `addValidBidirectional()`.
5. Any new constants go in `SuperstructureConstants`.
6. Add a test in `MARSSuperstructureTest` verifying the new state behavior.

## 4. Command API
Use `setAbsoluteState(SuperstructureState)` to command transitions:
```java
// From RobotContainer:
controller.leftTrigger()
    .onTrue(superstructure.setAbsoluteState(SuperstructureState.INTAKE_RUNNING))
    .onFalse(superstructure.setAbsoluteState(SuperstructureState.STOWED));

controller.rightTrigger()
    .onTrue(superstructure.setAbsoluteState(SuperstructureState.SCORE))
    .onFalse(superstructure.setAbsoluteState(SuperstructureState.STOWED));

controller.leftBumper()
    .onTrue(superstructure.setAbsoluteState(SuperstructureState.UNJAM))
    .onFalse(superstructure.setAbsoluteState(SuperstructureState.STOWED));
```

For emergency recovery, use `forceState(target)` which routes through STOWED automatically.

## 5. Telemetry
- `Superstructure/CurrentState` — Active enum state name
- `Superstructure/GoalCowlAngle` — Requested cowl angle (from ShotSetup or default)
- `Superstructure/GoalIntakeAngle` — Requested intake pivot angle
- `Superstructure/GamePieceCount` — Current held game piece count
- `Superstructure/EntryAction` — Logged on SCORE entry
- `Superstructure/TransitionDetail` — Full context: state, positions, gamePieceCount
- `Superstructure/TransitionRejectedReason` — Why a transition was rejected
- `Superstructure/IntakeError` — Logged if physics intake check throws
- `Superstructure/TicksInState` — Ticks spent in current state (from MARSStateMachine)
- `Superstructure/TotalTransitions` — Cumulative transition count
