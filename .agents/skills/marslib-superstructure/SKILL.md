---
name: marslib-superstructure
description: Helps manage the MARSSuperstructure collision-safe state machine for coordinating elevator, arm, intake, and shooter mechanisms.
---

# MARSLib Superstructure Skill

You are a Mechanism Coordination Engineer for Team MARS 2614. When modifying the superstructure state machine or adding new physical configurations:

## 1. Architecture
`MARSSuperstructure` is the central safety orchestrator that prevents the elevator and arm from colliding. It does NOT directly control motors. Instead, it:
1. Maps a `SuperstructureState` enum to goal positions (elevator height + arm angle).
2. Applies dynamic collision constraint clamping before dispatching to subsystems.
3. Manages intake/shooter activation based on the current state.

## 2. Collision Constraint Rules
Two rules prevent physical damage:

### Rule A: Elevator-Low Arm-Lock
When the elevator is physically below `SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW`, the arm angle is clamped to `SAFE_ARM_ANGLE_RAD_MAX_STOW`. This prevents the arm from extending outward and smashing into the floor or bumpers.

### Rule B: Arm-Extended Elevator-Lock
When the arm is physically extended beyond `SAFE_ARM_ANGLE_RAD_MIN_EXTEND`, the elevator height is clamped to a minimum of `SAFE_ELEVATOR_HEIGHT_METERS_MIN`. This prevents the elevator from crushing downward while the arm is sticking out.

**Key insight:** Both rules read the current *physical* position (not the goal), creating a self-reinforcing safety envelope that prevents damage even if commands arrive simultaneously.

## 3. Adding New States
To add a new `SuperstructureState`:
1. Add the enum value to `SuperstructureState` with a descriptive Javadoc.
2. Add a `case` in the `periodic()` switch block mapping to `goalElevatorHeight` and `goalArmAngle`.
3. All new constants must be added to `Constants.SuperstructureConstants`.
4. Verify the new position doesn't violate the Rule A/B safety envelope by checking:
   - If elevator height < `SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW`, arm angle must be ≤ `SAFE_ARM_ANGLE_RAD_MAX_STOW`.
   - If arm angle > `SAFE_ARM_ANGLE_RAD_MIN_EXTEND`, elevator height must be ≥ `SAFE_ELEVATOR_HEIGHT_METERS_MIN`.

## 4. Intake & Scoring Sequencing
The superstructure also manages intake/scorer activation:
- **INTAKE_FLOOR + no piece**: Runs intake at 12V. In simulation, calls `MARSPhysicsWorld.checkIntake()` to detect game piece collection.
- **SCORE_HIGH**: Runs shooter at 400 rad/s closed-loop. Sets `hasPiece = false`.
- **STOWED / default**: Stops intake and shooter.

## 5. Command API
Use `setAbsoluteState(SuperstructureState)` to command transitions:
```java
// From RobotContainer:
controller.a().onTrue(superstructure.setAbsoluteState(SuperstructureState.INTAKE_FLOOR));
controller.b().onTrue(superstructure.setAbsoluteState(SuperstructureState.SCORE_HIGH));
controller.x().onTrue(superstructure.setAbsoluteState(SuperstructureState.STOWED));
```
This returns an `InstantCommand` that requires the superstructure subsystem, preventing scheduling conflicts.

## 6. Telemetry
The superstructure logs the following keys every loop:
- `Superstructure/HasPiece` — Boolean game piece possession
- `Superstructure/CurrentState` — Active enum state name
- `Superstructure/GoalElevatorHeight` — Requested height before clamping
- `Superstructure/GoalArmAngle` — Requested angle before clamping
- `Superstructure/SafeElevatorHeight` — Height after constraint clamping
- `Superstructure/SafeArmAngle` — Angle after constraint clamping

Comparing Goal vs Safe values in AdvantageScope reveals when the collision system is actively intervening.
