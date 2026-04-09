# Contributing to MARSLib

Welcome to MARSLib, FRC Team 2614's flagship AdvantageKit architecture! We are thrilled to have you writing code for the 2026 REBUILT season.

This repository isn't just standard Java—it is a championship-tier Simulator and Data-Logging framework. Writing code here requires understanding a few core design rules. This guide will walk you through them so your code merges cleanly into the robot.

---

## 1. The "Subsystem IO" Rule

We **never** talk to hardware directly inside a Subsystem (like `SwerveDrive.java` or `Elevator.java`). If you instantiate a `TalonFX` directly inside your Subsystem, you break the simulator!

Instead, we use **Dependency Injection**. Every mechanism must be split into three core files:

### The Interface: `[Mechanism]IO.java`
This defines *what data* the mechanism needs to log and what commands it can receive.
```java
public interface ElevatorIO {
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double currentAmps = 0.0;
    }
    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setVoltage(double volts) {}
}
```

### The Real Hardware: `[Mechanism]IOReal.java`
This is the ONLY place a Motor Controller or Physical Sensor can exist.
```java
public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX motor = new TalonFX(Constants.ElevatorConstants.MOTOR_ID);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = motor.getPosition().getValueAsDouble();
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
```

### The Physics Engine: `[Mechanism]IOSim.java`
This is how we test offline! Wrap it in a Dyn4j physics body or WPILib math model.
```java
public class ElevatorIOSim implements ElevatorIO {
    private double position = 0.0;
    private double simulationVelocity = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Run theoretical time-step math
        position += simulationVelocity * Constants.LOOP_PERIOD_SECS;
        inputs.positionMeters = position;
        inputs.currentAmps = 0.0; // Perfect world!
    }
}
```

### Putting it Together:
Your actual `Elevator.java` subsystem will inject this IO layer inside its constructor. `RobotContainer.java` will decide whether to inject the `Real` or `Sim` version based on whether you are running a deployment or a desktop test.

---

## 2. AdvantageScope Logging

If it isn't logged, it doesn't exist. AdvantageKit makes our codebase completely deterministic.
Almost all your sensor data is handled automatically by the `updateInputs` routine above. However, if you are computing logic (like a target state machine goal, an odometry pose, or an autonomous trajectory), you must log it explicitly using the `Logger`.

```java
// Inside your Subsystem's periodic() loop:
Logger.recordOutput("Elevator/TargetHeight", targetHeightMeters);
Logger.recordOutput("Elevator/AtGoal", Math.abs(current - target) < 0.1);
```
Once logged, open `advantagescope_layout.json` to visualize these outputs natively in the 3D viewer.

---

## 3. Formatting and Linting (Spotless)

We strictly enforce Google Java Style guidelines. If you try to push code with messy indents or unused imports, **GitHub Actions will reject your Pull Request.**

To fix formatting automatically, run this command in your VS Code terminal before pushing:
```bash
# On Windows:
.\gradlew spotlessApply

# On Mac/Linux:
./gradlew spotlessApply
```

> [!TIP]
> Run `./gradlew installGitHooks` in your terminal! It forces your computer to automatically run `spotlessApply` locally every time you type `git commit`.

---

## 4. Submitting a Pull Request

1. **Create a branch** (e.g. `feature/auto-align` or `bugfix/elevator-jitter`).
2. Do not hesitate to use the desktop simulator (`.\gradlew simulateJava`) to verify your logic offline.
3. Once satisfied, execute `.\gradlew build` to ensure no compile errors exist.
4. Stage, commit, and push your branch.
5. Create a Pull Request (PR) against the `master` branch.
6. The CI pipeline will automatically run all JUnit physics-integration tests. If the tests pass and the code is formatted properly, you can request a review from a Robotics Lead.

---

## 5. Writing Tests (Mandatory Hygiene)

All subsystem tests use **physics-backed `IOSim` implementations** and the `dyn4j` engine—never Mockito for mechanism behavior. Every test's `@BeforeEach` **must** include this cleanup block to prevent static state bleed:

```java
@BeforeEach
public void setUp() {
    HAL.initialize(500, 0);
    CommandScheduler.getInstance().cancelAll();
    MARSPhysicsWorld.resetInstance();
    Alert.resetAll();
    MARSFaultManager.clear();
}
```

> [!WARNING]
> Omitting `MARSFaultManager.clear()` will cause phantom critical-fault flags to leak between test methods, causing random assertion failures that only appear when the full suite runs.

---

Happy Coding! 🪐
