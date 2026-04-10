---
name: marslib-statemachine
description: Helps build validated finite state machines using MARSStateMachine for subsystem coordination. Use when implementing multi-state mechanisms, sequencing logic, or any system that needs transition validation and logging.
---

# MARSLib State Machine Skill

You are a controls engineer for Team MARS 2614. When implementing state-driven logic for subsystems:

## 1. Architecture

`MARSStateMachine<S>` (`com.marslib.util`) is a generic, reusable finite state machine framework. It provides:

- **Transition validation** — only explicitly declared transitions are accepted
- **Illegal transition rejection** — rejected transitions are logged, not silently dropped
- **Entry/exit actions** — per-state side effects (e.g., eject game piece on SCORE entry)
- **Tick counting** — tracks how long the system has been in each state
- **AdvantageKit compatible** — pure algorithmic logic, all outputs via `Logger.recordOutput()`

### Class Signature
```java
public class MARSStateMachine<S extends Enum<S>> {
    public MARSStateMachine(String name, Class<S> stateClass, S initialState);
}
```

## 2. Key API

### Declaring Transitions
```java
// Specific transitions
machine.addTransition(State.IDLE, State.ACTIVE);
machine.addTransition(State.ACTIVE, State.SCORING);

// Wildcard: STOWED can go anywhere
machine.addWildcardFrom(State.STOWED);

// Wildcard: EMERGENCY is reachable from any state
machine.addWildcardTo(State.EMERGENCY);
```

### Requesting Transitions
```java
boolean accepted = machine.requestTransition(State.ACTIVE);
if (!accepted) {
    // Transition was illegal — state unchanged, rejection logged automatically
}
```

### Actions & Callbacks
```java
machine.setEntryAction(State.SCORING, () -> hasPiece = false);
machine.setExitAction(State.IDLE, () -> resetCounters());
machine.setOnTransition((from, to) -> Logger.recordOutput("Detail", from + "→" + to));
```

### Periodic Update (MUST call every loop)
```java
@Override
public void periodic() {
    machine.update();  // Increments tick counter, logs state
    // ... rest of subsystem logic
}
```

### Queries
```java
machine.getState();              // Current state
machine.getPreviousState();      // State before last transition
machine.getTicksInCurrentState(); // Ticks since last transition
machine.getTotalTransitionCount(); // Cumulative valid transitions
machine.isTransitionLegal(target); // Check without committing
```

## 3. Key Rules

### Rule A: Transitions Must Be Declared
Any transition not explicitly added via `addTransition()`, `addWildcardFrom()`, or `addWildcardTo()` will be **rejected**. This is intentional — it forces you to think about every legal path through your state graph.

### Rule B: Self-Transitions Are No-Ops
`requestTransition(currentState)` always returns `true` but does NOT fire entry/exit actions or increment the transition counter. This is safe to call repeatedly.

### Rule C: update() is Mandatory
You MUST call `machine.update()` in your subsystem's `periodic()`. If you skip it, tick counting and state telemetry stop working, and AdvantageScope shows stale data.

### Rule D: AdvantageKit Replay Safety
The FSM contains no hardware reads. All state decisions are deterministic from the sequence of `requestTransition()` calls, which are triggered by Commands (logged by AdvantageKit). This makes the entire state machine fully deterministic in log replay.

### Convenience: Bidirectional Transitions
```java
// Registers both A→B and B→A in one call
machine.addValidBidirectional(State.STOWED, State.SCORE);
```

## 4. Usage in MARSSuperstructure

The superstructure's transition graph uses bidirectional links:
```
STOWED ↔ INTAKE_DOWN      ✅
INTAKE_DOWN ↔ INTAKE_RUNNING ✅
STOWED ↔ SCORE            ✅
STOWED ↔ UNJAM            ✅
Any ↔ BEACHED             ✅ (safety override via forceState)
SCORE → INTAKE_RUNNING    ❌ ILLEGAL (must route through STOWED)
```

See the `marslib-superstructure` skill for collision clamping and beaching details.

## 5. When to Use MARSStateMachine
Use it when a subsystem has 3+ discrete modes needing explicit sequencing, illegal transition prevention, or post-match diagnostic data. Do NOT use it for simple on/off toggles (use a boolean), continuous control loops (use PID), or autonomous sequencing (use PathPlanner/Commands).

## 6. Telemetry (per machine instance)
All keys are prefixed with the `name` passed to the constructor:
- `{name}/CurrentState` — Active enum state name
- `{name}/TicksInState` — Ticks spent in current state
- `{name}/TotalTransitions` — Cumulative valid transition count
- `{name}/Transition` — "FROM→TO" string (only on transition tick)
- `{name}/TransitionTimestamp` — FPGA time of transition
- `{name}/RejectedTransition` — "FROM→TO" string for illegal attempts
- `{name}/RejectedTransitionTimestamp` — FPGA time of rejection
