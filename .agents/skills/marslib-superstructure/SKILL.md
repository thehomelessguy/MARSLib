---
name: marslib-superstructure
description: Helps manage the MARSSuperstructure collision-safe state machine for coordinating elevator, arm, intake, and shooter mechanisms.
---

# MARSLib Superstructure Skill

You are a Mechanism Coordination Engineer for Team MARS 2614. When modifying the superstructure state machine or adding new physical configurations:

## 1. Architecture
`MARSSuperstructure` is the central safety orchestrator that prevents the elevator and arm from colliding. It is built on `MARSStateMachine<SuperstructureState>`, a generic FSM framework that:
1. **Validates transitions** — only explicitly declared transitions are allowed. Illegal transitions are rejected and logged.
2. **Maps a `SuperstructureState` enum** to goal positions (elevator height + arm angle).
3. **Applies dynamic collision constraint clamping** before dispatching to subsystems.
4. **Manages intake/shooter activation** based on the current state.
5. **Fires entry/exit actions** on state transitions for side effects (e.g., ejecting a game piece on SCORE_HIGH entry).

### Legal Transitions (Adjacency Graph)
```
STOWED → INTAKE_FLOOR    (via addWildcardFrom)
STOWED → SCORE_HIGH      (via addWildcardFrom)
INTAKE_FLOOR → STOWED
INTAKE_FLOOR → SCORE_HIGH
SCORE_HIGH → STOWED
```
**SCORE_HIGH → INTAKE_FLOOR is ILLEGAL.** The driver must return to STOWED first. This prevents the arm from crashing into the floor when the elevator is still extended.

## 2. MARSStateMachine Framework (`com.marslib.util.MARSStateMachine<S>`)
The generic FSM framework is reusable for any subsystem. Key API:
- `addTransition(from, to)` — declares a legal transition
- `addWildcardFrom(state)` — state can go anywhere
- `addWildcardTo(state)` — state is reachable from anywhere (e.g., EMERGENCY)
- `requestTransition(target)` — attempts a transition; returns false and logs if illegal
- `setEntryAction(state, runnable)` — fires when entering a state
- `setExitAction(state, runnable)` — fires when leaving a state
- `setOnTransition(callback)` — fires on every valid transition
- `update()` — must be called in periodic(); increments tick counter and logs state

**AdvantageKit compatible:** The FSM is pure algorithmic logic. All outputs use `Logger.recordOutput()`. No hardware reads. Fully deterministic in replay.

## 3. Collision Constraint Rules
Two rules prevent physical damage:

### Rule A: Elevator-Low Arm-Lock
When the elevator is physically below `SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW`, the arm angle is clamped to `SAFE_ARM_ANGLE_RAD_MAX_STOW`. This prevents the arm from extending outward and smashing into the floor or bumpers.

### Rule B: Arm-Extended Elevator-Lock
When the arm is physically extended beyond `SAFE_ARM_ANGLE_RAD_MIN_EXTEND`, the elevator height is clamped to a minimum of `SAFE_ELEVATOR_HEIGHT_METERS_MIN`. This prevents the elevator from crushing downward while the arm is sticking out.

**Key insight:** Both rules read the current *physical* position (not the goal), creating a self-reinforcing safety envelope that prevents damage even if commands arrive simultaneously.

## 4. Adding New States
To add a new `SuperstructureState`:
1. Add the enum value to `SuperstructureState` with a descriptive Javadoc.
2. Add a `case` in the `periodic()` switch block mapping to `goalElevatorHeight` and `goalArmAngle`.
3. **Declare legal transitions** in the constructor using `stateMachine.addTransition(from, to)`.
4. All new constants must be added to `Constants.SuperstructureConstants`.
5. Verify the new position doesn't violate the Rule A/B safety envelope by checking:
   - If elevator height < `SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW`, arm angle must be ≤ `SAFE_ARM_ANGLE_RAD_MAX_STOW`.
   - If arm angle > `SAFE_ARM_ANGLE_RAD_MIN_EXTEND`, elevator height must be ≥ `SAFE_ELEVATOR_HEIGHT_METERS_MIN`.
6. Add a test in `MARSSuperstructureTest` verifying the new transition is accepted and illegal paths are rejected.

## 5. Intake & Scoring Sequencing
The superstructure also manages intake/scorer activation:
- **INTAKE_FLOOR + no piece**: Runs intake at 12V. In simulation, calls `MARSPhysicsWorld.checkIntake()` to detect game piece collection.
- **SCORE_HIGH**: Runs shooter at 400 rad/s closed-loop. Entry action sets `hasPiece = false`.
- **STOWED / default**: Stops intake and shooter.

## 6. Command API
Use `setAbsoluteState(SuperstructureState)` to command transitions:
```java
// From RobotContainer:
controller.a().onTrue(superstructure.setAbsoluteState(SuperstructureState.INTAKE_FLOOR));
controller.b().onTrue(superstructure.setAbsoluteState(SuperstructureState.SCORE_HIGH));
controller.x().onTrue(superstructure.setAbsoluteState(SuperstructureState.STOWED));
```
If the transition is illegal, the command is a no-op and the rejection is logged to `Superstructure/RejectedTransition`.

For emergency recovery, use `forceState(target)` which routes through STOWED automatically.

## 7. Telemetry
The superstructure logs the following keys every loop:
- `Superstructure/CurrentState` — Active enum state name
- `Superstructure/TicksInState` — Ticks spent in current state
- `Superstructure/TotalTransitions` — Cumulative transition count
- `Superstructure/Transition` — "FROM→TO" string on state change
- `Superstructure/TransitionTimestamp` — FPGA timestamp of transition
- `Superstructure/TransitionDetail` — Full context: state, positions, hasPiece
- `Superstructure/RejectedTransition` — Logged when an illegal transition is attempted
- `Superstructure/CollisionClamp` — Detailed clamp reason string
- `Superstructure/HasPiece` — Boolean game piece possession
- `Superstructure/GoalElevatorHeight` — Requested height before clamping
- `Superstructure/GoalArmAngle` — Requested angle before clamping
- `Superstructure/SafeElevatorHeight` — Height after constraint clamping
- `Superstructure/SafeArmAngle` — Angle after constraint clamping
- `Superstructure/ArmClamped` — Boolean: arm was clamped this tick
- `Superstructure/ElevatorClamped` — Boolean: elevator was clamped this tick

Comparing Goal vs Safe values in AdvantageScope reveals when the collision system is actively intervening.
