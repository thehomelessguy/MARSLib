---
name: marslib-testing
description: Helps write and maintain tests for MARSLib. Use this skill when writing JUnit 5 tests for WPILib commands, subsystems, or autonomous behaviors to ensure high-fidelity physical testing with dyn4j over mocked tests.
---

# MARSLib Integrated Testing Skill

You are a test engineer for Team MARS 2614. When writing tests for subsystems, commands, or autonomous paths:

## 1. Architecture

MARSLib uses **Digital Twin** testing — real subsystems with simulated IO, not mocked objects:

| Component | Purpose |
|---|---|
| `*IOSim` classes | Physics-backed simulation of each hardware layer |
| `MARSPhysicsWorld` | Shared dyn4j physics engine for all IOSim instances |
| `CommandScheduler` | WPILib's real scheduler — same as on the robot |
| `SimHooks.stepTiming()` | Advances HAL clock for deterministic timing |
| `DriverStationSim` | Simulates DS heartbeat and mode (auto/teleop/test) |
| `MARSTestHarness` | Centralized reset utility for all static singletons |

### Test Execution Loop
```java
for (int i = 0; i < 150; i++) {
    DriverStationSim.notifyNewData();       // Keep DS alive
    SimHooks.stepTiming(0.02);              // Advance 20ms
    CommandScheduler.getInstance().run();    // Process commands
    MARSPhysicsWorld.getInstance().update(0.02); // Step physics
}
```

## 2. Key Rules

### Rule A: Never Mock Hardware
Do NOT use Mockito to mock IO layers. Instantiate real subsystems with `*IOSim` implementations. Mocks hide physics bugs that only appear under real dynamics (wheel slip, gravity, inertia). The `mockito-core` dependency is intentionally excluded from `build.gradle`.

### Rule B: Use MARSTestHarness for Singleton Resets
The `MARSTestHarness` class (`com.marslib.testing`) centralizes all 7 singleton resets into one call. EVERY test class MUST use it:
```java
import com.marslib.testing.MARSTestHarness;

@BeforeEach
public void setUp() {
    MARSTestHarness.reset();
    // ... construct your subsystems here
}

@AfterEach
public void tearDown() {
    MARSTestHarness.cleanup();
}
```

`MARSTestHarness.reset()` handles:
1. `HAL.initialize()` — WPILib Hardware Abstraction Layer
2. `CommandScheduler` — cancel all commands, unregister all subsystems
3. `MARSPhysicsWorld.resetInstance()` — destroy dyn4j world and bodies
4. `AprilTagVisionIOSim.resetSimulation()` — destroy shared VisionSystemSim
5. `MARSFaultManager.clear()` — clear fault state
6. `Alert.resetAll()` — clear alert groups
7. `DriverStationSim` — set Blue1 alliance, enable, beat heartbeat

Skipping ANY of these causes cross-test contamination: stacked physics bodies, stale alerts, leaked commands.

### Rule C: Beat the DS Heartbeat Continuously
WPILib silently disables the robot if `DriverStationSim.notifyNewData()` isn't called every ~0.5s. In any loop stepping the scheduler, call it EVERY iteration. If you only call it once before the loop, the robot will disable mid-test at tick ~25.

### Rule D: Use @AfterEach for Cleanup
Always call `MARSTestHarness.cleanup()` in `@AfterEach` to prevent WPILib from leaking subsystem references between tests.

## 3. Adding New Tests

1. Create the test in the matching test package (e.g., `com.marslib.mechanisms.MARSClimberTest`).
2. In `@BeforeEach`: call `MARSTestHarness.reset()`, then construct subsystems with `*IOSim`.
3. In `@AfterEach`: call `MARSTestHarness.cleanup()`.
4. For physics tests: use the standard execution loop (Section 1 Architecture).
5. Assert against physical positions, not command states — test what the mechanism actually did.
6. For integration tests spanning multiple subsystems, see `RobotLifecycleTest` as the reference.

## 4. Test Categories

| Type | Example | What It Catches |
|---|---|---|
| Unit | `MARSElevatorTest` | Single-mechanism physics and control |
| Integration | `RobotLifecycleTest` | Multi-subsystem coordination bugs |
| Diagnostics | `MARSDiagnosticCheckTest` | Pre-match sweep validation |
| Math | `KinematicAimingTest`, `ShotSetupTest` | Pure algorithm correctness |
| State Machine | `MARSStateMachineTest` | Transition validation logic |
| Pipeline | `TeleopDrivePipelineTest` | Joystick→ChassisSpeeds math |

## 5. Telemetry
Tests don't emit AdvantageKit telemetry, but you can assert against Logger output keys:
```java
// The state machine logs transitions — verify the key was set
assertEquals(SuperstructureState.SCORE, superstructure.getCurrentState());
assertEquals(1, superstructure.getStateMachine().getTotalTransitionCount());
```

## Reference Implementations
- `MARSSuperstructureTest` — Physics-backed state transition and scoring verification
- `RobotLifecycleTest` — Full auto→teleop→score→stow lifecycle
- `MARSStateMachineTest` — FSM transition validation and rejection
- `MARSAlignmentCommandTest` — PID convergence under physics simulation
- `TeleopDrivePipelineTest` — Joystick→ChassisSpeeds pipeline regression test

---

## 6. AI Execution Workflow: Selective Testing (CRITICAL)

When you (the AI) are modifying code and want to compile/validate your changes:
**Never run the full test suite (`./gradlew build` or `./gradlew test`) blindly unless doing a final distribution audit.**

To save time, **always use selective testing** to run tests only for the specific classes you modified:
```bash
# E.g., if you modified MARSPhysicsWorld.java, run its respective test class:
./gradlew test --tests *MARSPhysicsWorldTest
```
If you change an interface or a deep architecture file, run tests on the handful of core subclasses affected. This keeps iteration cycles fast!
