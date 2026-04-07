---
name: marslib-testing
description: Helps write and maintain tests for MARSLib. Use this skill when writing JUnit 5 tests for WPILib commands, subsystems, or autonomous behaviors to ensure high-fidelity physical testing with dyn4j over mocked tests.
---

# MARSLib Integrated Testing Skill

You are an expert FRC Software Developer for Team MARS 2614. When asked to write tests for `MARSLib` subsystems, commands, or autonomous paths, you MUST strictly adhere to the following **Integrated Physical Simulation Environment** paradigm.

## 1. Avoid Mocking Hardware
- Do **NOT** use `Mockito` to mock the hardware layer or structural subsystems (e.g., `SwerveDrive`) when testing complex Commands or logical controllers.
- Instead, instantiate the real subsystems and inject `[Name]IOSim` instances (e.g., `SwerveModuleIOSim`, `GyroIOSim`).
- We test via "Digital Twins." We assert that the simulation mathematically and physically acts according to realistic constraints.

## 2. Setting Up The Test Environment (BeforeEach)
When testing, multiple JVMs and JUnit runners stack test instances together. Because `dyn4j` and `WPILib` utilize static trackers, you must rigorously clear the testing slate before EVERY test using an `@BeforeEach` hook.

```java
@BeforeEach
public void setUp() {
  // 1. Initialize the WPILib HAL for simulation
  edu.wpi.first.hal.HAL.initialize(500, 0);

  // 2. Mock a heartbeat and ensure the robot thinks it's enabled
  DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
  DriverStationSim.setEnabled(true);
  DriverStationSim.notifyNewData();

  // 3. Clear WPILib's internal scheduler tracking
  CommandScheduler.getInstance().cancelAll();

  // 4. Reset dyn4j physics state so robots don't stack infinitely at (0,0) across tests
  com.marslib.simulation.MARSPhysicsWorld.resetInstance();

  // 5. Initialize your Subsystems here using IOSim models!
}
```

## 3. Running Physics Loops Natively
When evaluating a `Command` (like a path follower or aiming logic) requiring iterative loops:
1. Schedule it in the `CommandScheduler`.
2. Loop over physical ticks manually using `SimHooks.stepTiming`.
3. In every cycle, aggressively beat the `DriverStationSim` heartbeat; otherwise WPILib will stealthily disable the robot mid-execution!

```java
@Test
public void testMySimulatedCommand() {
  CommandScheduler.getInstance().schedule(command);

  // Example: 150 loops = 3.0 seconds at 0.02s per loop
  for (int i = 0; i < 150; i++) {
    // Refresh DS heartbeat to prevent WPILib intrinsic timeouts
    DriverStationSim.notifyNewData();

    // Step HAL timestamp
    edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(frc.robot.Constants.LOOP_PERIOD_SECS);

    // CommandScheduler reads sensors & acts on motors
    CommandScheduler.getInstance().run();

    // Physics engine calculates collision, friction, and wheel translations
    com.marslib.simulation.MARSPhysicsWorld.getInstance().update(frc.robot.Constants.LOOP_PERIOD_SECS);
  }

  // Evaluate the True pose of the robot after dynamic physics simulation constraints!
  assertTrue(swerveDrive.getPose().getX() > 1.0, "Robot should have crossed physical 1-meter threshold.");
}
```

## 4. Handling 3rd Party Configuration Overruns
Some 3rd party vendor libraries (e.g. `AutoBuilder.configure()` in PathPlanner) throw soft/hard errors when reconfigured serially across test limits.
- When configuring libraries inside Subsystems, wrap them in `try/catch` handlers that explicitly ignore exceptions like "already been configured"! This prevents JUnit bulk test failures.

## Reference Implementations
For exact functional examples of FRC-grade digital-twin tests running purely on dyn4j integration physics, review:
- `src/test/java/com/marslib/auto/MARSAlignmentCommandTest.java`
- `src/test/java/com/marslib/auto/ShootOnTheMoveCommandTest.java`

You must follow these rules strictly to prevent static physics bleed, false-timeout disabling, and bulk-suite test failures.

## 5. Test Hygiene & Static State
- **Alert System:** The `Alert` class uses a static `Map` of alert groups. Call `Alert.resetAll()` in your `@BeforeEach` block to prevent alert state from bleeding across tests.
- **Scratch Files:** Do not commit debug utilities (e.g., tag extractors, enum printers) as `@Test` methods. They pollute CI output and confuse students. Store one-off utilities in `/tmp/` or a `scratch/` directory outside the test tree.
- **MARSFaultManager:** This is also static. After tests that trigger faults, call `MARSFaultManager.clearNewCriticalFault()` to prevent downstream test contamination.
