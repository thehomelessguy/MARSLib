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
    UNJAM            // All mechanisms reversed at -6V to clear jams
}
```

### Transition Graph
All transitions are legal (wildcard from every state). The REBUILT chassis has no collision paths between the cowl and intake pivot, so no transition restrictions are needed.

## 2. Key Rules

### Rule A: Shot Parameters Are Distance-Dependent
The cowl angle and shooter RPM in `SCORE` state are NOT constants — they are dynamically interpolated from `ShotSetup` based on `distanceSupplier`. The `ShotSetup` utility interpolates from a pre-characterized distance→(RPM, cowlAngle) map.

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
2. Add a `case` in the `periodic()` switch block mapping to `goalCowlAngle` and `goalIntakeAngle`.
3. Add motor control logic (intake/shooter/feeder activation) for the new state.
4. Any new constants go in `Constants.SuperstructureConstants`.
5. Since all transitions are wildcarded, no `addTransition()` call is needed.
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
